// Manual UD / DU hold (command p) in closed loop, as upright.h runs it.
//   g++ -std=c++17 -O2 simulation/sysid/sim-hold.cpp -o sim-hold && ./sim-hold
// Plant: sysid-fitted model with friction, encoder zero bias, x4 4000-count encoders,
// firmware EKF, TMC ramp slewing to VMAX at AMAX = 12 m/s^2 one frame after the command.
// Compares cart-position handling in the hold law u = -K e:
//   none    : K1..K5 only (firmware manual p before 2026-11-24)
//   half    : + clamp(-0.5 K0 x, +-1.5) (firmware after a swing catch)
//   full    : + K0 x, i.e. the designed LQR u = -K x
#include "../../src/double_pendulum/state_estimator.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <cstdlib>
#include "../../src/double_pendulum/rail_guard.h"

using est::kPi;
constexpr int CPR = 4000;
constexpr float DT = .005f, AMAX = 12.f, VMAX = .8f, TILT = .17f, TRAVEL = .30f;
constexpr float K_w[] = {-3.464102f, -52.422759f, 17.225801f, -4.465170f, -4.803921f, -0.558010f};
constexpr float K_q[] = {11.105604f, -147.661261f, 198.038011f, 11.411039f, -4.511831f, 21.653094f};
constexpr float KI_Q = 3.651484f;
constexpr float K_e[] = {-14.832397f, 153.628010f, -244.278154f, -17.451330f, 11.412767f, -27.573131f};

enum class Law { none, half, full };
struct Pose { const char *name; float base, tip; const float *K; };
struct Case { const char *name; float biasBase = 0, biasTip = 0, x0 = 0, J = 1, fc = 1, noise = 0;
             // catch-start state (errors from the pose, rad and rad/s), post-catch tilt limit, rail guard
             float v0 = 0, e10 = 0, e20 = 0, w10 = 0, w20 = 0, tilt = TILT; bool rail = false; };
struct Out { std::string stop = "HELD"; float t = 0, xMax = 0, xEnd = 0, tiltRms = 0, uJit = 0, sat = 0; };
static float accelStdOverride = 0;   // >0: EKF process noise for the gain study

static Out run(const Pose &pose, Law law, const Case &c, unsigned seed, float seconds = 30) {
    ModelParams truth = kModel;
    truth.J1 *= c.J; truth.J2 *= c.J; truth.C *= c.J; truth.fc1 *= c.fc; truth.fc2 *= c.fc;
    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.f, 1.f);
    const bool caught = c.e10 || c.e20 || c.w10 || c.w20 || c.v0;
    const float jitter = caught ? .002f : .01f;
    float s[4] = {pose.base + c.e10 + jitter * gauss(rng), pose.tip + c.e20 + jitter * gauss(rng), c.w10, c.w20};
    double x = c.x0, v = c.v0;
    float vCmd = c.v0, vPrev = c.v0;
    auto counts = [](double a) { return std::floor(a * CPR / (2 * kPi)); };
    auto measure = [&](float z[2]) {
        const double base = s[0] + c.biasBase, tip = s[1] + c.biasTip;
        z[0] = est::wrap(float(counts(base - kPi) * 2 * kPi / CPR) + kPi);
        z[1] = est::wrap(float(counts(tip - base) * 2 * kPi / CPR));
    };
    float z[2];
    measure(z);
    est::AngleEKF kf;
    kf.reset(z[0], est::wrap(z[0] + z[1]));
    if (accelStdOverride > 0) kf.accelStd = accelStdOverride;
    kf.s[2] = c.w10; kf.s[3] = c.w20;   // after a catch the EKF already tracks the rates
    est::ChannelGuard guard[2];
    Out o;
    double tiltSq = 0, jitSq = 0;
    float xi = 0, uPrev = 0; unsigned nSat = 0;
    const unsigned frames = unsigned(seconds / DT);
    unsigned k = 0;
    for (; k < frames; k++) {
        kf.predict((float(v) - vPrev) / DT, DT);
        vPrev = float(v);
        measure(z);
        for (int ch = 0; ch < 2; ch++) est::processChannel(kf, ch, z[ch], 0, CPR, guard[ch]);
        const float e1 = est::wrap(kf.s[0] - pose.base), e2 = est::wrap(kf.s[1] - pose.tip);
        if (std::fabs(e1) > c.tilt || std::fabs(e2) > c.tilt) { o.stop = "TILT_LIMIT"; break; }
        if (std::fabs(x) > TRAVEL) { o.stop = "RAMP_POSITION_LIMIT"; break; }
        if (c.rail && (RailGuard::needsBrake(float(x), float(v)) || RailGuard::needsBrake(float(x), vCmd))) { o.stop = "RAIL_BRAKE"; break; }
        const float *K = pose.K;
        float u = -(K[1] * e1 + K[2] * e2 + K[3] * float(v) + K[4] * kf.s[2] + K[5] * kf.s[3]);
        if (law == Law::half) u += std::clamp(-.5f * K[0] * float(x), -1.5f, 1.5f);
        if (law == Law::full) u += -K[0] * float(x);
        if (law == Law::full && K == K_q) { xi = std::clamp(xi + float(x) * DT, -1.5f / KI_Q, 1.5f / KI_Q); u -= KI_Q * xi; }
        if (k) jitSq += (u - uPrev) * (u - uPrev);
        uPrev = u; nSat += std::fabs(u) > AMAX;
        vCmd = std::clamp(vCmd + std::clamp(u, -AMAX, AMAX) * DT, -VMAX, VMAX);
        for (int sub = 0; sub < 5; sub++) {
            const double h = DT / 5;
            const double dv = std::clamp(double(vCmd) - v, -AMAX * h, AMAX * h);
            s[2] += c.noise * gauss(rng) * std::sqrt(float(h));
            s[3] += c.noise * gauss(rng) * std::sqrt(float(h));
            est::rk4(truth, s, float(dv / h), float(h));
            x += v * h + .5 * dv * h; v += dv;
        }
        const float tilt = std::max(std::fabs(est::wrap(s[0] - pose.base)), std::fabs(est::wrap(s[1] - pose.tip)));
        tiltSq += tilt * tilt;
        o.xMax = std::max(o.xMax, float(std::fabs(x)));
    }
    o.t = k * DT; o.xEnd = float(x);
    o.uJit = float(std::sqrt(jitSq / std::max(1u, k))); o.sat = float(nSat) / std::max(1u, k); o.tiltRms = float(std::sqrt(tiltSq / std::max(1u, k)));
    return o;
}

static void catchStudy() {
    // Hold right after a swing catch. Starts: the hardware catch states of 2026-11-24 and the corners
    // of the catch limits (angle 0.10 rad, rate < 0.5 rad/s, ramp speed 0.30 m/s), cart offset worst-signed.
    constexpr float D = kPi / 180, DU_TILT = 20 * D;
    const Pose ud{"UD", 0, kPi, K_w}, du{"DU", kPi, 0, K_e};
    const float B1 = .1f * D, B2 = .2f * D;
    struct Named { const Pose *p; Case c; } cases[] = {
        {&ud, {"UD hw: x+172 v+.035 e-1/-1.3 deg", B1, B2, .172f, 1, 1, 0, .035f, -.99f * D, -1.35f * D, .018f, -.037f, TILT, true}},
        {&ud, {"UD hw: x+149 v+.055 e-.8/-1 deg", B1, B2, .149f, 1, 1, 0, .055f, -.81f * D, -.99f * D, .095f, -.023f, TILT, true}},
        {&ud, {"UD corner: x+200 v+.30 e+.10", B1, B2, .20f, 1, 1, 0, .30f, .10f, .10f, .4f, .4f, TILT, true}},
        {&ud, {"UD corner: x+200 v+.30 e-.10", B1, B2, .20f, 1, 1, 0, .30f, -.10f, -.10f, -.4f, -.4f, TILT, true}},
        {&ud, {"UD corner: x-150 v-.30 e+.10/-.10", B1, B2, -.15f, 1, 1, 0, -.30f, .10f, -.10f, .4f, -.4f, TILT, true}},
        {&ud, {"UD hw, J+2% dist .1", B1, B2, .16f, 1.02f, 1, .1f, .045f, -.9f * D, -1.2f * D, .05f, -.03f, TILT, true}},
        {&du, {"DU hw: x+62 v+.235 e-.7/+2.9 deg", B1, B2, .062f, 1, 1, 0, .235f, -.72f * D, 2.88f * D, -.21f, -.36f, DU_TILT, true}},
        {&du, {"DU hw: x+54 v+.243 e-.4/+3.3 deg", B1, B2, .054f, 1, 1, 0, .243f, -.36f * D, 3.33f * D, .12f, .04f, DU_TILT, true}},
        {&du, {"DU UD->DU hw: x+24", B1, B2, .024f, 1, 1, 0, -.015f, .09f * D, .18f * D, .011f, -.001f, DU_TILT, true}},
        {&du, {"DU corner: x+150 v+.30 e+.10", B1, B2, .15f, 1, 1, 0, .30f, .10f, .10f, .4f, .4f, DU_TILT, true}},
        {&du, {"DU corner: x+150 v+.30 e-.10", B1, B2, .15f, 1, 1, 0, .30f, -.10f, -.10f, -.4f, -.4f, DU_TILT, true}},
        {&du, {"DU corner: x-200 v-.30 e+.10/-.10", B1, B2, -.20f, 1, 1, 0, -.30f, .10f, -.10f, .4f, -.4f, DU_TILT, true}},
        {&du, {"DU hw, J-2% dist .1", B1, B2, .06f, .98f, 1, .1f, .24f, -.5f * D, 3.f * D, -.2f, -.3f, DU_TILT, true}},
    };
    const struct { const char *name; Law law; } laws[] = {{"half, clamped (firmware now)", Law::half}, {"full LQR u=-Kx", Law::full}};
    for (const auto &l : laws) {
        std::printf("\n== after catch: %s (20 s, 10 seeds)\n", l.name);
        for (const auto &nc : cases) {
            unsigned held = 0; float xm = 0, xe = 0, tr = 0; std::string fail; float tFail = 1e9f;
            for (unsigned sd = 1; sd <= 10; sd++) {
                const Out o = run(*nc.p, l.law, nc.c, sd, 20);
                if (o.stop == "HELD") { held++; xe = std::max(xe, std::fabs(o.xEnd)); }
                else if (o.t < tFail) { tFail = o.t; fail = o.stop; }
                xm = std::max(xm, o.xMax); tr = std::max(tr, o.tiltRms);
            }
            std::printf("  %-36s held %2u/10 | |x| max %4.0f end %4.0f mm | tilt rms %.2f deg", nc.c.name, held, xm * 1e3, xe * 1e3, tr / D);
            if (held < 10) std::printf(" | first %s @%.2f s", fail.c_str(), tFail);
            std::printf("\n");
        }
    }
}

static void uuCatchStudy() {
    // UU hold (LQI) started from states the current UU catch rule rejects, e.g. the 2026-11-24 22:09 rq miss.
    constexpr float D = kPi / 180;
    const Pose uu{"UU", 0, 0, K_q};
    const float B1 = .1f * D, B2 = .2f * D;
    struct C { Case c; } cs[] = {
        {{"hw miss: e -5.0/-3.7 w .33/.32 v -.25 x -119", B1, B2, -.119f, 1, 1, 0, -.25f, -5.04f * D, -3.69f * D, .33f, .32f, .17f, false}},
        {{"angle .10 rad, w .45 (same sign), v .30 toward", B1, B2, .12f, 1, 1, 0, .30f, .10f, .10f, .45f, .45f, .17f, false}},
        {{"angle .10 rad, w .45 opposite, v -.30", B1, B2, .12f, 1, 1, 0, -.30f, .10f, .10f, -.45f, -.45f, .17f, false}},
        {{"angle .10/-.05, w .45/-.45, v .30, x -150", B1, B2, -.15f, 1, 1, 0, .30f, .10f, -.05f, .45f, -.45f, .17f, false}},
        {{"angle .07 rad, w .3, v .25, x -200 (toward)", B1, B2, -.20f, 1, 1, 0, -.25f, .07f, .07f, .3f, .3f, .17f, false}},
        {{"hw miss, J +2% dist .1", B1, B2, -.119f, 1.02f, 1, .1f, -.25f, -5.04f * D, -3.69f * D, .33f, .32f, .17f, false}},
    };
    std::printf("== UU LQI hold from rejected catch states (20 s, 10 seeds)\n");
    for (const auto &c : cs) {
        unsigned held = 0; float xm = 0, xe = 0, tr = 0; std::string fail; float tFail = 1e9f;
        for (unsigned sd = 1; sd <= 10; sd++) {
            const Out o = run(uu, Law::full, c.c, sd, 20);
            if (o.stop == "HELD") { held++; xe = std::max(xe, std::fabs(o.xEnd)); } else if (o.t < tFail) { tFail = o.t; fail = o.stop; }
            xm = std::max(xm, o.xMax); tr = std::max(tr, o.tiltRms);
        }
        std::printf("  %-48s held %2u/10 | |x| max %4.0f end %4.0f mm | tilt rms %.2f deg", c.c.name, held, xm * 1e3, xe * 1e3, tr / D);
        if (held < 10) std::printf(" | first %s @%.2f s", fail.c_str(), tFail);
        std::printf("\n");
    }
}

static void duGainStudy() {
    // DU hold chatter: EKF rate noise (count steps) times the hold gains -> frame-to-frame accel jumps.
    constexpr float D = kPi / 180;
    static const float Kq1[] = {-3.464102f, 79.481f, -117.51f, -4.684f, 6.379f, -13.33f};      // fitted, Q=(12,60,60,2,2,2), R=1
    static const float Kq2[] = {-7.071f, 98.872f, -152.91f, -8.59f, 7.711f, -17.252f};        // fitted, Q=(50,200,200,10,2,2), R=1
    static const float Kud[] = {-3.464102f, -52.422759f, 17.225801f, -4.465170f, -4.803921f, -0.558010f};
    const struct { const char *name; Pose p; float accelStd; } gs[] = {
        {"UD hold K_w (reference, quiet on hw)", {"UD", 0, kPi, Kud}, 30},
        {"DU K_e (firmware)", {"DU", kPi, 0, K_e}, 30},
        {"DU K_e, EKF accelStd 8", {"DU", kPi, 0, K_e}, 8},
        {"DU fitted Q=(12,60,60,2,2,2)", {"DU", kPi, 0, Kq1}, 30},
        {"DU fitted Q=(50,200,200,10,2,2)", {"DU", kPi, 0, Kq2}, 30},
        {"DU K_e, EKF accelStd 4", {"DU", kPi, 0, K_e}, 4},
        {"DU fitted Q=(12,60,60,2,2,2), accelStd 8", {"DU", kPi, 0, Kq1}, 8},
        {"DU fitted Q=(12,60,60,2,2,2), accelStd 4", {"DU", kPi, 0, Kq1}, 4},
        {"UD K_w, accelStd 8", {"UD", 0, kPi, Kud}, 8},
        {"UU-like check: UD K_w, accelStd 4", {"UD", 0, kPi, Kud}, 4},
    };
    const Case cs[] = {{"bias +0.1/+0.2", .1f * D, .2f * D}, {"bias, J +2% dist .1", .1f * D, .2f * D, 0, 1.02f, 1, .1f},
                       {"start x0 -150 mm", .1f * D, .2f * D, -.15f}, {"bias, Coulomb x1.3", .1f * D, .2f * D, 0, 1, 1.3f}};
    for (const auto &g : gs) {
        accelStdOverride = g.accelStd;
        std::printf("\n== %s\n", g.name);
        for (const auto &c : cs) {
            unsigned held = 0; float jit = 0, sat = 0, tr = 0, xm = 0;
            for (unsigned sd = 1; sd <= 5; sd++) { const Out o = run(g.p, Law::full, c, sd, 20);
                held += o.stop == "HELD"; jit = std::max(jit, o.uJit); sat = std::max(sat, o.sat); tr = std::max(tr, o.tiltRms); xm = std::max(xm, o.xMax); }
            std::printf("  %-22s held %u/5 | accel cmd jump rms %5.2f m/s^2/frame | saturated %4.1f%% | tilt rms %.2f deg | |x| %3.0f mm\n",
                        c.name, held, jit, sat * 100, tr / D, xm * 1e3);
        }
    }
    accelStdOverride = 0;
}

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::string(argv[1]) == "dugain") { duGainStudy(); return 0; }
    if (std::getenv("HOLD4")) accelStdOverride = 4;   // firmware HOLD_EKF_ACCEL_STD
    if (argc > 1 && std::string(argv[1]) == "uucatch") { uuCatchStudy(); return 0; }
    if (argc > 1 && std::string(argv[1]) == "catch") { catchStudy(); return 0; }
    const Pose poses[] = {{"UD", 0, kPi, K_w}, {"DU", kPi, 0, K_e}};
    const struct { const char *name; Law law; } laws[] = {
        {"none (manual p now)", Law::none}, {"half, clamped (after catch)", Law::half}, {"full LQR u=-Kx", Law::full}};
    constexpr float D = kPi / 180;
    const Case cases[] = {
        {"no bias, x0=0"},
        {"bias +0.1/+0.2 deg", .1f * D, .2f * D},
        {"bias -0.4/-0.4 deg", -.4f * D, -.4f * D},
        {"bias +0.4/+0.2 deg, dist 0.1", .4f * D, .2f * D, 0, 1, 1, .1f},
        {"start x0=+80 mm", .1f * D, .2f * D, .08f},
        {"start x0=-150 mm", .1f * D, .2f * D, -.15f},
        {"start x0=+250 mm", .1f * D, .2f * D, .25f},
        {"start x0=-250 mm", .1f * D, .2f * D, -.25f},
        {"bias, J1,J2 +2%", .1f * D, .2f * D, 0, 1.02f},
        {"bias, J1,J2 -2%", .1f * D, .2f * D, 0, .98f},
        {"bias, Coulomb x1.3", .1f * D, .2f * D, 0, 1, 1.3f},
    };
    for (const auto &p : poses)
        for (const auto &l : laws) {
            std::printf("\n== %s hold, %s (30 s, 10 seeds)\n", p.name, l.name);
            for (const auto &c : cases) {
                unsigned held = 0; float xm = 0, xe = 0, tr = 0; std::string fail; float tFail = 1e9f;
                for (unsigned sd = 1; sd <= 10; sd++) {
                    const Out o = run(p, l.law, c, sd);
                    if (o.stop == "HELD") { held++; xe = std::max(xe, std::fabs(o.xEnd)); }
                    else if (o.t < tFail) { tFail = o.t; fail = o.stop; }
                    xm = std::max(xm, o.xMax); tr = std::max(tr, o.tiltRms);
                }
                std::printf("  %-30s held %2u/10 | |x| max %4.0f end %4.0f mm | tilt rms %.2f deg", c.name, held,
                            xm * 1e3, xe * 1e3, tr / D);
                if (held < 10) std::printf(" | first %s @%.1f s", fail.c_str(), tFail);
                std::printf("\n");
            }
        }
}
