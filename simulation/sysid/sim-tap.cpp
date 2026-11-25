// Tap (impulse) vs push (sustained force) on the UU / UD / DU holds, as upright.h runs them.
//   g++ -std=c++17 -O2 simulation/sysid/sim-tap.cpp -o sim-tap && ./sim-tap        (tilt limit x EKF q)
//   ./sim-tap limits                                                               (which hardware limit ends a tap)
// Plant: sysid-fitted model with friction, encoder zero bias, x4 4000-count encoders,
// firmware EKF and innovation gate, TMC ramp slewing to VMAX at AMAX one frame after the command.
// Hold law as in the firmware: UU = LQI (K_q, KI_Q), UD/DU = u = -K x.
// A disturbance is a horizontal force at the tip of link 2, turned into generalised
// joint impulses (Q = J^T F dt). A tap is one 5 ms pulse; a push is the same force
// spread over 0.5 s. Not modelled: encoder edges lost by the ISR on fast whips
// (the guard sees no invalid edges here), belt stretch, step loss.
#include "../../src/double_pendulum/state_estimator.h"
#include "../../src/double_pendulum/rail_guard.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

using est::kPi;
constexpr int CPR = 4000;
constexpr float DT = .005f;
static float AMAX = 12.f, VMAX = .8f, TRAVEL = .30f;
static bool railGuard = true; // limits study switches these off one by one
constexpr float K_q[] = {11.105604f, -147.661261f, 198.038011f, 11.411039f, -4.511831f, 21.653094f};
constexpr float KI_Q = 3.651484f, UU_ILIM = 1.5f;
constexpr float K_w[] = {-3.464102f, -52.422759f, 17.225801f, -4.465170f, -4.803921f, -0.558010f};
constexpr float K_e[] = {-14.832397f, 153.628010f, -244.278154f, -17.451330f, 11.412767f, -27.573131f};
constexpr float D = kPi / 180;
constexpr float L1 = 3.992961656e-04f / 2.475e-03f; // C = l1 d2
constexpr float L2 = .16f;                            // link 2 length (check_model.py nominal)

struct Pose { const char *name; float base, tip; const float *K; bool lqi; };
struct Cfg { float tilt, accelStd; };
struct Out { std::string stop = "HELD"; float peakTilt = 0, peakRateErr = 0, xMax = 0; };

// Joint-space rate change from a horizontal impulse P (N s) at the tip of link 2.
static void impulseRates(const ModelParams &m, const float s[4], float P, float dw[2]) {
    const float q1 = P * L1 * std::cos(s[0]), q2 = P * L2 * std::cos(s[1]);
    const float h12 = m.C * std::cos(s[0] - s[1]), det = m.J1 * m.J2 - h12 * h12;
    dw[0] = (m.J2 * q1 - h12 * q2) / det;
    dw[1] = (m.J1 * q2 - h12 * q1) / det;
}

static Out run(const Pose &pose, const Cfg &cfg, float force, float duration, unsigned seed) {
    const ModelParams truth = kModel;
    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.f, 1.f);
    const float biasBase = .1f * D, biasTip = .2f * D, noise = .1f;
    float s[4] = {pose.base + .002f * gauss(rng), pose.tip + .002f * gauss(rng), 0, 0};
    double x = 0, v = 0;
    float vCmd = 0, vPrev = 0, xi = 0;
    auto counts = [](double a) { return std::floor(a * CPR / (2 * kPi)); };
    float z[2];
    auto measure = [&] {
        const double base = s[0] + biasBase, tip = s[1] + biasTip;
        z[0] = est::wrap(float(counts(base - kPi) * 2 * kPi / CPR) + kPi);
        z[1] = est::wrap(float(counts(tip - base) * 2 * kPi / CPR));
    };
    measure();
    est::AngleEKF kf;
    kf.reset(z[0], est::wrap(z[0] + z[1]));
    kf.accelStd = cfg.accelStd;
    est::ChannelGuard guard[2];
    Out o;
    const float tTap = 3.f, tEnd = tTap + duration + 6.f;
    const unsigned frames = unsigned(tEnd / DT);
    for (unsigned k = 0; k < frames; k++) {
        const float t = k * DT;
        kf.predict((float(v) - vPrev) / DT, DT);
        vPrev = float(v);
        measure();
        for (int ch = 0; ch < 2; ch++) {
            est::processChannel(kf, ch, z[ch], 0, CPR, guard[ch]);
            if (est::channelLost(guard[ch])) { o.stop = "ENCODER_LOST"; return o; }
        }
        const float e1 = est::wrap(kf.s[0] - pose.base), e2 = est::wrap(kf.s[1] - pose.tip);
        if (t >= tTap) {
            o.peakTilt = std::max({o.peakTilt, std::fabs(e1), std::fabs(e2)});
            o.peakRateErr = std::max({o.peakRateErr, std::fabs(kf.s[2] - s[2]), std::fabs(kf.s[3] - s[3])});
        }
        if (std::fabs(e1) > cfg.tilt || std::fabs(e2) > cfg.tilt) { o.stop = "TILT_LIMIT"; return o; }
        if (std::fabs(x) > TRAVEL) { o.stop = "RAMP_POSITION_LIMIT"; return o; }
        if (railGuard && (RailGuard::needsBrake(float(x), float(v)) || RailGuard::needsBrake(float(x), vCmd))) { o.stop = "RAIL_BRAKE"; return o; }
        const float *K = pose.K;
        float u = -(K[1] * e1 + K[2] * e2 + K[3] * float(v) + K[4] * kf.s[2] + K[5] * kf.s[3]) - K[0] * float(x);
        if (pose.lqi) { xi = std::clamp(xi + float(x) * DT, -UU_ILIM / KI_Q, UU_ILIM / KI_Q); u -= KI_Q * xi; }
        vCmd = std::clamp(vCmd + std::clamp(u, -AMAX, AMAX) * DT, -VMAX, VMAX);
        for (int sub = 0; sub < 5; sub++) {
            const double h = DT / 5, ts = t + sub * h;
            if (ts >= tTap && ts < tTap + duration) {
                float dw[2];
                impulseRates(truth, s, force * float(h), dw);
                s[2] += dw[0]; s[3] += dw[1];
            }
            const double dv = std::clamp(double(vCmd) - v, -AMAX * h, AMAX * h);
            s[2] += noise * gauss(rng) * std::sqrt(float(h));
            s[3] += noise * gauss(rng) * std::sqrt(float(h));
            est::rk4(truth, s, float(dv / h), float(h));
            x += v * h + .5 * dv * h; v += dv;
        }
        o.xMax = std::max(o.xMax, float(std::fabs(x)));
        // a real fall: the plant itself is far past saving
        if (std::fabs(est::wrap(s[0] - pose.base)) > 1.f || std::fabs(est::wrap(s[1] - pose.tip)) > 1.f) { o.stop = "FELL"; return o; }
    }
    return o;
}

// Largest impulse (N s) that every seed survives, by bisection over the force.
static float threshold(const Pose &p, const Cfg &c, float duration, int sign, std::string &why) {
    auto ok = [&](float P) {
        for (unsigned sd = 1; sd <= 5; sd++) {
            const Out o = run(p, c, sign * P / duration, duration, sd);
            if (o.stop != "HELD") { why = o.stop; return false; }
        }
        return true;
    };
    float lo = 0, hi = .2f;
    std::string w;
    while (ok(hi) && hi < 5) hi *= 2;
    for (int i = 0; i < 14; i++) { const float mid = .5f * (lo + hi); if (ok(mid)) lo = mid; else hi = mid; }
    ok(hi); why = why.empty() ? "-" : why;
    return lo;
}

static void limitsStudy(const Pose *poses) {
    // Which hardware limit ends a tap once the tilt limit is out of the way (tilt 45 deg, EKF q = 4).
    struct V { const char *name; bool guard; float travel, vmax, amax; } vs[] = {
        {"firmware: rail guard, +-300 mm, 0.8 m/s, 12 m/s^2", true, .30f, .8f, 12},
        {"no rail guard, +-300 mm", false, .30f, .8f, 12},
        {"no rail guard, rail +-1 m", false, 1.f, .8f, 12},
        {"no rail guard, rail +-1 m, 1.6 m/s", false, 1.f, 1.6f, 12},
        {"no rail guard, rail +-1 m, 1.6 m/s, 24 m/s^2", false, 1.f, 1.6f, 24}};
    std::printf("\n== tap 5 ms, tilt limit 45 deg, EKF q=4: largest survived impulse, mN*s | first failure\n");
    for (const auto &v : vs) {
        railGuard = v.guard; TRAVEL = v.travel; VMAX = v.vmax; AMAX = v.amax;
        std::printf("  %-48s", v.name);
        for (int i = 0; i < 3; i++) {
            std::string why;
            const float P = std::min(threshold(poses[i], {45 * D, 4}, .005f, +1, why), threshold(poses[i], {45 * D, 4}, .005f, -1, why));
            std::printf("| %s %5.1f %-19s", poses[i].name, P * 1e3f, why.c_str());
        }
        std::printf("\n");
    }
}

int main(int argc, char **argv) {
    const Pose poses[] = {{"UU", 0, 0, K_q, true}, {"UD", 0, kPi, K_w, false}, {"DU", kPi, 0, K_e, false}};
    if (argc > 1 && std::string(argv[1]) == "limits") { limitsStudy(poses); return 0; }
    const float tilts[] = {.17f, 15 * D, 20 * D, 45 * D};
    const float stds[] = {4, 10, 30};
    for (const auto &p : poses) {
        std::printf("\n== %s hold: largest survived impulse at the tip, N*s (tip rate jump it causes, rad/s) | first failure\n", p.name);
        float dw[2]; const float s0[4] = {p.base, p.tip, 0, 0};
        impulseRates(kModel, s0, 1.f, dw);
        std::printf("   (1 mN*s at the tip changes base/tip rates by %+.3f / %+.3f rad/s)\n", dw[0] * 1e-3f, dw[1] * 1e-3f);
        for (const char *kind : {"tap 5 ms", "push 0.5 s"}) {
            const float dur = kind[0] == 't' ? .005f : .5f;
            std::printf("  %s\n", kind);
            for (float st : stds) {
                std::printf("    EKF q=%2.0f ", st);
                for (float tl : tilts) {
                    std::string why;
                    const float P = std::min(threshold(p, {tl, st}, dur, +1, why), threshold(p, {tl, st}, dur, -1, why));
                    std::printf("| tilt %4.1f deg: %5.1f mN*s (%4.1f rad/s) %-12s", tl / D, P * 1e3f, std::fabs(dw[1]) * P, why.c_str());
                }
                std::printf("\n");
            }
        }
    }
    // Detail at the firmware settings for one tap just past the threshold.
    std::printf("\n== EKF rate error right after a tap (firmware tilt 9.74 deg, 5 seeds max)\n");
    for (const auto &p : poses)
        for (float st : stds) {
            std::string why;
            const float P = threshold(p, {.17f, st}, .005f, +1, why);
            float rate = 0, tilt = 0;
            for (unsigned sd = 1; sd <= 5; sd++) {
                const Out o = run(p, {.17f, st}, .9f * P / .005f, .005f, sd);
                rate = std::max(rate, o.peakRateErr); tilt = std::max(tilt, o.peakTilt);
            }
            std::printf("  %s q=%2.0f: at 90%% of threshold, peak EKF rate error %5.2f rad/s, peak tilt %5.2f deg\n",
                        p.name, st, rate, tilt / D);
        }
}
