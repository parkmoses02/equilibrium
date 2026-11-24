// Closed-loop simulation of the DD -> UU swing-up (upright.h, command rq).
//   g++ -std=c++17 -O2 simulation/sysid/sim-uu-swing.cpp -o sim-uu-swing && ./sim-uu-swing
// Same frame order as doubleLoop(): EKF predict with the ramp's velocity change,
// encoder update, swing feedback u = r.u - k.e (e[0]=0: no cart position term),
// vCmd += clamp(u,16)*dt, then the TMC ramp slews to vCmd at AMAX=16 m/s^2 during
// the next frame. Plant: sysid-fitted model with friction, x4 4000-count encoders.
#include "../../src/double_pendulum/state_estimator.h"
// Test a candidate table: -I src/double_pendulum -DUU_FRAMES_HEADER='"path/uu_frames.h"'
#ifdef UU_FRAMES_HEADER
#include UU_FRAMES_HEADER
#else
#include "../../src/double_pendulum/uu_frames.h"
#endif
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

using est::kPi;
constexpr int CPR = 4000;
constexpr float DT = .005f, AMAX = 16.f, VMAX = .8f, TRAVEL = .30f;
constexpr unsigned N = sizeof(uuFrames) / sizeof(uuFrames[0]);

struct Variant {
    const char *name;
    float angleGain = 1, rateGain = 1, velGain = 1; // scale the table's feedback terms
    float posGain = 0;      // + positionFeedback(k0, x - r.x, 1.0) scaled, like DD->UD
    bool friction = true, ekf = true;
    float uLimit = 16;
    float accelStd = 30;
    bool fullX = false;   // designed TVLQR position term (no 0.5 factor, no clamp)
};
// Plant perturbation relative to model_params.h (the EKF keeps the nominal model).
struct Plant { float J1 = 1, J2 = 1, d1 = 1, fc = 1, b = 1; };
struct Out { const char *stop; unsigned step; float ebMax, etMax, dxMax, uJitter, xEnd, catchAngle, catchRate, catchV;
             float holdXMax = 0, holdXEnd = 0, holdTiltRms = 0; };
// Optional UU hold after the catch (upright.h hold branch, mode q), with an
// encoder zero bias (measured upright != true balance point).
struct Hold {
    const float *K = nullptr; float kx = 0, ki = 0, iLimit = 1.5f; float seconds = 0;
    float biasBase = 0, biasTip = 0;
};
static Hold hold;

static float refRate(unsigned i, bool tip) {
    const unsigned lo = i ? i - 1 : 0, hi = i + 1 < N ? i + 1 : N - 1;
    const float a = tip ? uuFrames[lo].tip : uuFrames[lo].base, b = tip ? uuFrames[hi].tip : uuFrames[hi].base;
    return remainderf(b - a, 2 * kPi) / ((hi - lo) * DT);
}

static Out swing(const Variant &var, unsigned seed, float inertia = 1, bool verbose = false, Plant pl = {}) {
    ModelParams truth = kModel;
    if (!var.friction) truth.b1 = truth.b2 = truth.fc1 = truth.fc2 = 0;
    truth.J1 *= inertia; truth.J2 *= inertia; truth.C *= inertia;
    truth.J1 *= pl.J1; truth.J2 *= pl.J2; truth.d1 *= pl.d1;
    truth.fc1 *= pl.fc; truth.fc2 *= pl.fc; truth.b1 *= pl.b; truth.b2 *= pl.b;
    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.f, 1.f);
    float s[4] = {kPi + .003f * gauss(rng), kPi + .003f * gauss(rng), 0, 0};
    double x = 0, v = 0;
    float vCmd = 0, vPrev = 0, uPrev = 0;
    auto counts = [](double a) { return std::floor(a * CPR / (2 * kPi)); };
    auto measure = [&](float z[2]) {
        const double base = s[0] + hold.biasBase, tip = s[1] + hold.biasTip;
        z[0] = est::wrap(float(counts(base - kPi) * 2 * kPi / CPR) + kPi);
        z[1] = est::wrap(float(counts(tip - base) * 2 * kPi / CPR));
    };
    float amax = AMAX;
    double xi = 0, tiltSq = 0;
    const unsigned H = unsigned(hold.seconds / DT);
    float z[2];
    measure(z);
    est::AngleEKF kf;
    kf.reset(z[0], est::wrap(z[0] + z[1]));
    kf.accelStd = var.accelStd;
    est::ChannelGuard guard[2];
    Out o{"COMPLETE", 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double jitter = 0;
    for (unsigned k = 0; k <= N + H; k++) {
        kf.predict(k ? (float(v) - vPrev) / DT : 0.f, DT);
        vPrev = float(v);
        measure(z);
        for (int ch = 0; ch < 2; ch++) est::processChannel(kf, ch, z[ch], 0, CPR, guard[ch]);
        float t1 = kf.s[0], t2 = kf.s[1], w1 = kf.s[2], w2 = kf.s[3];
        if (!var.ekf) { t1 = est::wrap(s[0]); t2 = est::wrap(s[1]); w1 = s[2]; w2 = s[3]; }
        o.step = k;
        if (std::fabs(x) > TRAVEL) { o.stop = "RAMP_POSITION_LIMIT"; break; }
        if (k == N) {
            o.catchAngle = std::max(std::fabs(t1), std::fabs(t2));
            o.catchRate = std::max(std::fabs(w1), std::fabs(w2));
            o.catchV = float(std::fabs(v));
            const bool core = o.catchAngle <= .05f && o.catchV <= .2f && o.catchRate < .5f;
            const bool wide = o.catchAngle <= .10f && o.catchV <= .30f && o.catchRate < .5f;
            o.stop = core || wide ? "CATCH" : "MISSED_UU";
            if (!(core || wide) || !H) break;
            vCmd = float(v); amax = 12.f; xi = 0;   // firmware: vCmd=v; rampLimit(MAX_ACCEL_MPS2)
        }
        if (k >= N) {   // UU hold
            if (std::fabs(t1) > .17f || std::fabs(t2) > .17f) { o.stop = "HOLD_TILT_LIMIT"; break; }
            const float *K = hold.K;
            float u = -(K[1] * t1 + K[2] * t2 + K[3] * float(v) + K[4] * w1 + K[5] * w2);
            if (hold.ki > 0) xi = std::clamp(xi + x * DT, double(-hold.iLimit / hold.ki), double(hold.iLimit / hold.ki));
            u += -hold.kx * float(x) - hold.ki * float(xi);
            vCmd = std::clamp(vCmd + std::clamp(u, -12.f, 12.f) * DT, -VMAX, VMAX);
            o.holdXMax = std::max(o.holdXMax, float(std::fabs(x)));
            const float tilt = std::max(std::fabs(s[0]), std::fabs(s[1]));
            tiltSq += tilt * tilt;
            if (k == N + H) { o.stop = "HELD"; break; }
            for (int sub = 0; sub < 5; sub++) {
                const double h = DT / 5;
                const double dv = std::clamp(double(vCmd) - v, -amax * h, amax * h);
                est::rk4(truth, s, float(dv / h), float(h));
                x += v * h + .5 * dv * h; v += dv;
            }
            continue;
        }
        const auto &r = uuFrames[k];
        const float e[6] = {0, est::wrap(t1 - r.base), est::wrap(t2 - r.tip), float(v) - r.v,
                            w1 - refRate(k, false), w2 - refRate(k, true)};
        if (std::fabs(e[1]) > .45f || std::fabs(e[2]) > .45f) { o.stop = "SWING_TRACKING_ERROR"; break; }
        o.ebMax = std::max(o.ebMax, std::fabs(e[1])); o.etMax = std::max(o.etMax, std::fabs(e[2]));
        o.dxMax = std::max(o.dxMax, float(std::fabs(x - r.x)));
        const float scale[6] = {0, var.angleGain, var.angleGain, var.velGain, var.rateGain, var.rateGain};
        float u = r.u;
        for (int j = 1; j < 6; j++) u -= scale[j] * r.k[j] * e[j];
        if (var.posGain > 0) u += var.fullX ? -r.k[0] * float(x - r.x) : std::clamp(-.5f * var.posGain * r.k[0] * float(x - r.x), -1.f, 1.f);
        if (k) jitter += (u - uPrev) * (u - uPrev);
        uPrev = u;
        vCmd = std::clamp(vCmd + std::clamp(u, -var.uLimit, var.uLimit) * DT, -VMAX, VMAX);
        if (verbose && k % 20 == 0)
            std::printf("  %3u x %+6.0f ref %+6.0f mm | eb %+5.1f et %+5.1f deg | u %+6.2f\n", k, x * 1e3, r.x * 1e3,
                        e[1] * 180 / kPi, e[2] * 180 / kPi, u);
        for (int sub = 0; sub < 5; sub++) {
            const double h = DT / 5;
            const double dv = std::clamp(double(vCmd) - v, -amax * h, amax * h);
            est::rk4(truth, s, float(dv / h), float(h));
            x += v * h + .5 * dv * h; v += dv;
        }
    }
    o.holdXEnd = float(x);
    o.holdTiltRms = H && o.step > N ? float(std::sqrt(tiltSq / (o.step - N))) : 0.f;
    o.uJitter = float(std::sqrt(jitter / std::max(1u, o.step)));
    o.xEnd = float(x);
    return o;
}

static void report(const Variant &var, float inertia = 1, unsigned seeds = 10, Plant pl = {}) {
    unsigned caught = 0, worstStep = N; const char *worst = "CATCH";
    float eb = 0, et = 0, dx = 0, jit = 0;
    for (unsigned sd = 1; sd <= seeds; sd++) {
        const Out o = swing(var, sd, inertia, false, pl);
        if (std::string(o.stop) == "CATCH") caught++;
        else if (o.step < worstStep) { worstStep = o.step; worst = o.stop; }
        eb = std::max(eb, o.ebMax); et = std::max(et, o.etMax); dx = std::max(dx, o.dxMax); jit = std::max(jit, o.uJitter);
    }
    std::printf("%-34s caught %2u/%u | max err base %5.1f tip %5.1f deg | |x-ref| %4.0f mm | du/frame rms %5.2f",
                var.name, caught, seeds, eb * 180 / kPi, et * 180 / kPi, dx * 1e3, jit);
    if (caught < seeds) std::printf(" | first fail %s @%u", worst, worstStep);
    std::printf("\n");
}

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::string(argv[1]) == "hold") {
        // Catch, then hold 20 s. Old: nominal K_q, no x term. New: fitted LQI (R=0.3, qi=4).
        static const float Kold[] = {3.464102f, -116.851324f, 140.474145f, 5.061349f, -4.628502f, 15.246215f};
        static const float Knew[] = {11.105604f, -147.661261f, 198.038011f, 11.411039f, -4.511831f, 21.653094f};
        const struct { const char *name; Hold h; } laws[] = {
            {"old hold (no x term)", {Kold, 0, 0, 1.5f, 20}},
            {"new hold: LQI x + int(x)", {Knew, Knew[0], 3.651484f, 1.5f, 20}}};
        const struct { const char *name; float b, t; Plant p; } cases[] = {
            {"no bias", 0, 0, {}}, {"bias +0.39/+0.17 deg (logged)", .0068f, .003f, {}},
            {"bias x2", .0136f, .006f, {}}, {"bias opposite", -.0068f, -.003f, {}},
            {"bias, J1,J2 +2%", .0068f, .003f, {1.02f, 1.02f}}, {"bias, J1,J2 -2%", .0068f, .003f, {.98f, .98f}},
            {"bias, Coulomb x1.3", .0068f, .003f, {1, 1, 1, 1.3f}}};
        const Variant swingLaw{"", 1, 1, 1, 1};
        for (const auto &l : laws) {
            std::printf("\n-- %s\n", l.name);
            for (const auto &c : cases) {
                hold = l.h; hold.biasBase = c.b; hold.biasTip = c.t;
                unsigned held = 0; float xm = 0, xe = 0, tr = 0; const char *fail = "";
                for (unsigned sd = 1; sd <= 10; sd++) {
                    const Out o = swing(swingLaw, sd, 1, false, c.p);
                    if (std::string(o.stop) == "HELD") { held++; xe = std::max(xe, std::fabs(o.holdXEnd)); }
                    else fail = o.stop;
                    xm = std::max(xm, o.holdXMax); tr = std::max(tr, o.holdTiltRms);
                }
                std::printf("%-32s held 20 s %2u/10 | |x| max %4.0f mm, end %4.0f mm | tilt rms %.2f deg %s\n",
                            c.name, held, xm * 1e3, xe * 1e3, tr * 180 / kPi, fail);
            }
        }
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "robust") {
        Variant fullx{"+ full TVLQR position term", 1, 1, 1, 1}; fullx.fullX = true;
        const Variant laws[] = {{"+ position fb (firmware now)", 1, 1, 1, 1}, fullx};
        const struct { const char *name; Plant p; } plants[] = {
            {"exact", {}}, {"J1 +2%", {1.02f}}, {"J1 -2%", {.98f}}, {"J2 +2%", {1, 1.02f}}, {"J2 -2%", {1, .98f}},
            {"d1 +2%", {1, 1, 1.02f}}, {"d1 -2%", {1, 1, .98f}}, {"Coulomb x0.7", {1, 1, 1, .7f}},
            {"Coulomb x1.3", {1, 1, 1, 1.3f}}, {"viscous x0", {1, 1, 1, 1, 0}}, {"viscous x3", {1, 1, 1, 1, 3}},
            {"J1,J2 +5%", {1.05f, 1.05f}}, {"J1,J2 -5%", {.95f, .95f}}};
        for (const auto &l : laws) {
            std::printf("\n-- %s\n", l.name);
            for (const auto &pp : plants) { Variant v = l; v.name = pp.name; report(v, 1, 10, pp.p); }
        }
        return 0;
    }
    if (argc > 1) {   // trace one run of the firmware law
        Variant v{"firmware"};
        if (std::string(argv[1]) == "ideal") { v.friction = false; v.ekf = false; }
        const Out o = swing(v, 1, 1, true);
        std::printf("%s at step %u\n", o.stop, o.step);
        return 0;
    }
    std::printf("DD->UU, %u frames. Hardware 2026-11-24: tracked to ~140, base err +11 deg by 180, "
                "RAMP_POSITION_LIMIT at 367.\n\n", N);
    std::printf("-- what breaks the firmware law?\n");
    report({"ideal (true state, no friction)", 1, 1, 1, 0, false, false});
    report({"true state + friction", 1, 1, 1, 0, true, false});
    report({"EKF, no friction", 1, 1, 1, 0, false, true});
    report({"firmware-faithful (EKF+friction)", 1, 1, 1, 0, true, true});
    std::printf("\n-- candidate fixes (firmware-faithful plant)\n");
    report({"+ position fb (as DD->UD)", 1, 1, 1, 1});
    report({"rate gains x0.5", 1, .5f, 1, 0});
    report({"rate gains x0.5 + position fb", 1, .5f, 1, 1});
    report({"all gains x0.5 + position fb", .5f, .5f, .5f, 1});
    report({"u limit 8 m/s^2", 1, 1, 1, 0, true, true, 8});
    std::printf("\n-- EKF process noise (accelStd, firmware 30 rad/s^2)\n");
    for (float a : {15.f, 8.f, 4.f, 2.f}) {
        static char names[8][64]; static int used = 0;
        Variant v{"", 1, 1, 1, 0}; v.accelStd = a;
        std::snprintf(names[used], 64, "accelStd %.0f", a); v.name = names[used++]; report(v);
        v.friction = false;
        std::snprintf(names[used], 64, "accelStd %.0f, no friction", a); v.name = names[used++]; report(v);
    }
    std::printf("\n-- robustness of the firmware law, inertia +-5%%\n");
    report({"firmware, J +5%"}, 1.05f);
    report({"firmware, J -5%"}, .95f);
}
