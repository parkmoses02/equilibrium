// What happens when x is pressed during a UU / UD / DU hold, and how to make it gentle.
//   g++ -std=c++17 -O2 -I src/double_pendulum simulation/sysid/sim-stop.cpp -o sim-stop && ./sim-stop
//   candidate tables: -DSTOP_TABLE_DIR=... is not used; pass -DUU_DD_FRAMES_HEADER etc.
// Methods after x (pressed after 1 s of hold):
//   off      : motor off at once (firmware before 2026-11-24). The cart is assumed to stop
//              within one frame and stay put; the links fall freely.
//   fall+dd  : motor stays on, the cart is held still while the links fall; once both
//              links are within 30 deg of hanging, the DD damping LQR takes over.
//   descend  : a designed table brings the links to DD under TVLQR (like the transitions),
//              then the DD damping LQR runs until still. Tracking error falls back to DD LQR.
// Plant: sysid-fitted model with friction, x4 4000-count encoders, firmware EKF, TMC ramp.
#include "../../src/double_pendulum/state_estimator.h"
#ifdef UU_DD_FRAMES_HEADER
#include UU_DD_FRAMES_HEADER
#include UD_DD_FRAMES_HEADER
#include DU_DD_FRAMES_HEADER
#else
#include "../../src/double_pendulum/uu_dd_frames.h"
#include "../../src/double_pendulum/ud_dd_frames.h"
#include "../../src/double_pendulum/du_dd_frames.h"
#endif
#include "../../src/double_pendulum/rail_guard.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <cstdlib>

using est::kPi;
constexpr int CPR = 4000;
constexpr float DT = .005f, VMAX = .8f, TRAVEL = .30f, D = kPi / 180;
constexpr float K_q[] = {11.105604f, -147.661261f, 198.038011f, 11.411039f, -4.511831f, 21.653094f};
constexpr float KI_Q = 3.651484f;
constexpr float K_w[] = {-3.464102f, -52.422759f, 17.225801f, -4.465170f, -4.803921f, -0.558010f};
constexpr float K_e[] = {-14.832397f, 153.628010f, -244.278154f, -17.451330f, 11.412767f, -27.573131f};
// DD damping LQR: fitted model, Q=(12,60,60,2,2,2), R=1 (check_model.py dd_gain).
constexpr float K_r[] = {3.464102f, 21.548065f, -9.099465f, 4.209856f, 2.398130f, 0.873836f};
constexpr float SETTLE_RATE = .2f, SETTLE_ANGLE = 2 * D, SETTLE_S = .5f, RUN_S = 12.f;

struct Pose { const char *name; float base, tip; const float *K; const DuFrame *table; unsigned count; };
const Pose poses[] = {
    {"UU", 0, 0, K_q, uuDdFrames, sizeof(uuDdFrames) / sizeof(uuDdFrames[0])},
    {"UD", 0, kPi, K_w, udDdFrames, sizeof(udDdFrames) / sizeof(udDdFrames[0])},
    {"DU", kPi, 0, K_e, duDdFrames, sizeof(duDdFrames) / sizeof(duDdFrames[0])},
};
enum class Method { off, fallDD, descend };
struct Case { const char *name; float J = 1, fc = 1, x0 = 0; };
struct Out { std::string stop = "SETTLED"; float settle = 0, peakW = 0, peakRel = 0, peakA = 0, xMax = 0, rmsAt5 = 0; };

static float refRate(const Pose &p, unsigned i, bool tip) {
    const unsigned lo = i ? i - 1 : 0, hi = i + 1 < p.count ? i + 1 : p.count - 1;
    const float a = tip ? p.table[lo].tip : p.table[lo].base, b = tip ? p.table[hi].tip : p.table[hi].base;
    return remainderf(b - a, 2 * kPi) / ((hi - lo) * DT);
}

static Out run(const Pose &p, Method m, const Case &c, unsigned seed, bool verbose = false) {
    ModelParams truth = kModel;
    truth.J1 *= c.J; truth.J2 *= c.J; truth.C *= c.J; truth.fc1 *= c.fc; truth.fc2 *= c.fc;
    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.f, 1.f);
    float s[4] = {p.base + .005f * gauss(rng), p.tip + .005f * gauss(rng), 0, 0};
    double x = c.x0, v = 0;
    float vCmd = 0, vPrev = 0, amax = 12.f, xi = 0;
    auto counts = [](double a) { return std::floor(a * CPR / (2 * kPi)); };
    auto measure = [&](float z[2]) {
        z[0] = est::wrap(float(counts(s[0] - kPi) * 2 * kPi / CPR) + kPi);
        z[1] = est::wrap(float(counts(double(s[1]) - s[0]) * 2 * kPi / CPR));
    };
    float z[2];
    measure(z);
    est::AngleEKF kf;
    kf.reset(z[0], est::wrap(z[0] + z[1]));
    est::ChannelGuard guard[2];
    enum { HOLD, FALL, DESCEND, DAMP, OFF } phase = HOLD;
    Out o;
    unsigned step = 0;
    float origin = 0, still = 0, tPress = 1.f, sq5 = 0; unsigned n5 = 0;
    for (unsigned k = 0; k * DT < tPress + RUN_S; k++) {
        const float t = k * DT;
        kf.predict(k ? (float(v) - vPrev) / DT : 0.f, DT);
        vPrev = float(v);
        measure(z);
        for (int ch = 0; ch < 2; ch++) est::processChannel(kf, ch, z[ch], 0, CPR, guard[ch]);
        const float t1 = kf.s[0], t2 = kf.s[1], w1 = kf.s[2], w2 = kf.s[3];
        if (phase == HOLD && t >= tPress) {
            if (m == Method::off) { phase = OFF; v = 0; vCmd = 0; }
            else if (m == Method::fallDD) { phase = FALL; vCmd = 0; }
            else { phase = DESCEND; step = 0; origin = float(x); amax = 16.f; }
        }
        if (phase != OFF && phase != HOLD) {
            if (std::fabs(x) > TRAVEL) { o.stop = "RAMP_POSITION_LIMIT"; break; }
            if (RailGuard::needsBrake(float(x), float(v)) || RailGuard::needsBrake(float(x), vCmd)) { o.stop = "RAIL_BRAKE"; break; }
        }
        float u = 0;
        const float eDD1 = est::wrap(t1 - kPi), eDD2 = est::wrap(t2 - kPi);
        auto ddLaw = [&] { return -(K_r[0] * float(x) + K_r[1] * eDD1 + K_r[2] * eDD2 + K_r[3] * float(v) + K_r[4] * w1 + K_r[5] * w2); };
        if (phase == HOLD) {
            const float e1 = est::wrap(t1 - p.base), e2 = est::wrap(t2 - p.tip);
            u = -(p.K[0] * float(x) + p.K[1] * e1 + p.K[2] * e2 + p.K[3] * float(v) + p.K[4] * w1 + p.K[5] * w2);
            if (p.K == K_q) { xi += float(x) * DT; u -= KI_Q * xi; }
        } else if (phase == FALL) {
            u = -float(v) / DT;   // hold the cart still
            if (std::fabs(eDD1) < 30 * D && std::fabs(eDD2) < 30 * D) phase = DAMP;
        } else if (phase == DESCEND) {
            if (step >= p.count) { phase = DAMP; amax = 12.f; }
            else {
                const auto &r = p.table[step];
                const float e[6] = {0, est::wrap(t1 - r.base), est::wrap(t2 - r.tip), float(v) - r.v,
                                    w1 - refRate(p, step, false), w2 - refRate(p, step, true)};
                if (std::fabs(e[1]) > .45f || std::fabs(e[2]) > .45f) { phase = DAMP; amax = 12.f; }   // fall back to damping
                else {
                    u = r.u;
                    for (int j = 1; j < 6; j++) u -= r.k[j] * e[j];
                    u += std::getenv("FULLX") ? -r.k[0] * (float(x) - (origin + r.x)) : std::clamp(-.5f * r.k[0] * (float(x) - (origin + r.x)), -1.f, 1.f);
                    step++;
                }
            }
        }
        if (phase == DAMP) u = ddLaw();
        if (phase == OFF) { vCmd = 0; v = 0; }
        else {
            const float limit = phase == DESCEND ? 16.f : 12.f;
            vCmd = std::clamp(vCmd + std::clamp(u, -limit, limit) * DT, -VMAX, VMAX);
        }
        for (int sub = 0; sub < 5; sub++) {
            const double h = DT / 5;
            double dv = 0;
            if (phase != OFF) dv = std::clamp(double(vCmd) - v, -amax * h, amax * h);
            if (t >= tPress) o.peakA = std::max(o.peakA, float(std::fabs(dv / h)));
            est::rk4(truth, s, float(dv / h), float(h));
            x += v * h + .5 * dv * h; v += dv;
        }
        if (t >= tPress) {
            o.peakW = std::max({o.peakW, std::fabs(s[2]), std::fabs(s[3])});
            o.peakRel = std::max(o.peakRel, std::fabs(s[3] - s[2]));
            o.xMax = std::max(o.xMax, float(std::fabs(x)));
            const bool calm = std::fabs(s[2]) < SETTLE_RATE && std::fabs(s[3]) < SETTLE_RATE &&
                              std::fabs(est::wrap(s[0] - kPi)) < SETTLE_ANGLE && std::fabs(est::wrap(s[1] - kPi)) < SETTLE_ANGLE;
            still = calm ? still + DT : 0;
            if (still >= SETTLE_S && o.settle == 0) o.settle = t - tPress - SETTLE_S;
            if (t - tPress > 4.f && t - tPress < 5.f) { sq5 += s[2] * s[2] + s[3] * s[3]; n5++; }
            if (verbose && k % 40 == 0)
                std::printf("  t=%5.2f %d base %+7.1f tip %+7.1f deg | w %+6.2f %+6.2f | x %+5.0f mm\n", t - tPress, int(phase),
                            est::wrap(s[0] - kPi) / D, est::wrap(s[1] - kPi) / D, s[2], s[3], x * 1e3);
        }
    }
    if (o.stop == "SETTLED" && o.settle == 0) o.stop = "NOT_SETTLED";
    o.rmsAt5 = n5 ? std::sqrt(sq5 / n5) : 0;
    return o;
}

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 2) {
        const Pose &p = poses[std::stoi(argv[1])];
        const Out o = run(p, Method(std::stoi(argv[2])), {"trace"}, 1, true);
        std::printf("%s settle %.2f s\n", o.stop.c_str(), o.settle);
        return 0;
    }
    const struct { const char *name; Method m; } methods[] = {
        {"off (motor off at once)", Method::off}, {"fall + DD damping LQR", Method::fallDD}, {"descend table + DD LQR", Method::descend}};
    const Case cases[] = {{"nominal"}, {"J +5%", 1.05f}, {"J -5%", .95f}, {"Coulomb x1.3", 1, 1.3f}, {"x0 = +60 mm", 1, 1, .06f}};
    for (const auto &p : poses)
        for (const auto &m : methods) {
            std::printf("\n== x during %s hold: %s (10 seeds)\n", p.name, m.name);
            for (const auto &c : cases) {
                float settle = 0, pw = 0, pr = 0, pa = 0, xm = 0, r5 = 0; unsigned ok = 0; std::string fail;
                for (unsigned sd = 1; sd <= 10; sd++) {
                    const Out o = run(p, m.m, c, sd);
                    if (o.stop == "SETTLED") { ok++; settle = std::max(settle, o.settle); } else fail = o.stop;
                    pw = std::max(pw, o.peakW); pr = std::max(pr, o.peakRel); pa = std::max(pa, o.peakA);
                    xm = std::max(xm, o.xMax); r5 = std::max(r5, o.rmsAt5);
                }
                std::printf("  %-14s settled %2u/10 (<= %5.2f s) | peak link rate %5.1f rad/s, elbow %5.1f | rate rms at 4-5 s %5.2f | cart |a| %5.1f m/s^2, |x| %3.0f mm %s\n",
                            c.name, ok, settle, pw, pr, r5, pa, xm * 1e3, fail.c_str());
            }
        }
}
