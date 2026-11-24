// Closed-loop simulation of the manual UU hold (upright.h, mode q, command p).
//   g++ -std=c++17 -O2 simulation/sysid/sim-uu-hold.cpp -o sim-uu-hold && ./sim-uu-hold
// Plant: fitted model (model_params.h) incl. friction, x4 4000-count encoders,
// a zero-angle bias (encoder upright != true balance point), the firmware EKF,
// and the TMC ramp following VMAX at AMAX = 12 m/s^2 one frame after the command.
// Compares the current hold law (no cart position term) with position feedback
// and with position + integral feedback, which absorbs the angle bias.
#include "../../src/double_pendulum/state_estimator.h"
#include <cmath>
#include <cstdio>
#include <random>
#include <algorithm>
#include <string>

using est::kPi;
constexpr int CPR = 4000;
constexpr float DT = .005f, AMAX = 12.f, VMAX = .8f, TILT_LIMIT = .17f, TRAVEL = .30f;
constexpr float K_q[] = {3.464102f, -116.851324f, 140.474145f, 5.061349f, -4.628502f, 15.246215f};

struct Law { const char *name; float kx, ki; };
// Diagnostic switches (all true = firmware-faithful).
struct Knobs { bool friction = true, quantize = true, ekf = true, delay = true, accelLimit = true; const float *gain = K_q;
                float fcScale = 1, bScale = 1; };
static Knobs knobs;       // u += -kx*x - ki*int(x)
struct Case { float biasBase, biasTip; float inertia; float noise; float seconds; };
struct Out {
    const char *stop; float t, xEnd, xMax, vMean, tiltRms, tiltMax, xRmsLate;
};

static Out simulate(const Law &law, const Case &c, unsigned seed, bool trace = false) {
    ModelParams truth = kModel;
    if (!knobs.friction) truth.b1 = truth.b2 = truth.fc1 = truth.fc2 = 0;
    truth.fc1 *= knobs.fcScale; truth.fc2 *= knobs.fcScale; truth.b1 *= knobs.bScale; truth.b2 *= knobs.bScale;
    truth.J1 *= c.inertia; truth.J2 *= c.inertia; truth.C *= c.inertia;
    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.f, 1.f);
    // True state, absolute angles from up. Starts near the true balance point.
    float s[4] = {.01f * gauss(rng), .01f * gauss(rng), 0.f, 0.f};
    double x = 0, v = 0;           // cart (ramp) position and velocity
    float vCmd = 0;
    auto counts = [](double a) { return knobs.quantize ? std::floor(a * CPR / (2 * kPi)) : a * CPR / (2 * kPi); };
    auto measure = [&](float z[2]) {   // what the firmware's channels report
        const double base = s[0] + c.biasBase, tip = s[1] + c.biasTip;
        z[0] = est::wrap(float(counts(base) * 2 * kPi / CPR));
        z[1] = est::wrap(float(counts(tip - base) * 2 * kPi / CPR));
    };
    float z[2];
    measure(z);
    est::AngleEKF kf;
    kf.reset(z[0], est::wrap(z[0] + z[1]));
    est::ChannelGuard guard[2];
    float vPrev = 0; bool primed = false;
    double xi = 0, tiltSq = 0, xSqLate = 0, vSum = 0;
    float tiltMax = 0, xMax = 0;
    unsigned n = 0, nLate = 0;
    const unsigned frames = unsigned(c.seconds / DT);
    for (unsigned k = 0; k < frames; k++) {
        const float t = k * DT;
        // ---- firmware frame ----
        kf.predict(primed ? (float(v) - vPrev) / DT : 0.f, DT);
        vPrev = float(v); primed = true;
        measure(z);
        for (int ch = 0; ch < 2; ch++) est::processChannel(kf, ch, z[ch], 0, CPR, guard[ch]);
        float t1 = kf.s[0], t2 = kf.s[1], w1 = kf.s[2], w2 = kf.s[3];
        if (!knobs.ekf) { t1 = s[0] + c.biasBase; t2 = s[1] + c.biasTip; w1 = s[2]; w2 = s[3]; }
        if (std::fabs(t1) > TILT_LIMIT || std::fabs(t2) > TILT_LIMIT)
            return {"TILT_LIMIT", t, float(x), xMax, float(vSum / std::max(1u, n)), float(std::sqrt(tiltSq / std::max(1u, n))), tiltMax, 0};
        if (std::fabs(x) > TRAVEL)
            return {"RAMP_POSITION_LIMIT", t, float(x), xMax, float(vSum / std::max(1u, n)), float(std::sqrt(tiltSq / std::max(1u, n))), tiltMax, 0};
        xi += x * DT;
        const float *K = knobs.gain;
        float u = -(K[1] * t1 + K[2] * t2 + K[3] * float(v) + K[4] * w1 + K[5] * w2);
        u += -law.kx * float(x) - law.ki * float(xi);
        const float a = knobs.accelLimit ? std::clamp(u, -AMAX, AMAX) : u;
        vCmd = std::clamp(vCmd + a * DT, -VMAX, VMAX);
        // ---- plant over the next frame: ramp slews to vCmd at AMAX ----
        for (int sub = 0; sub < 5; sub++) {
            const double h = DT / 5;
            double dv = knobs.accelLimit ? std::clamp(double(vCmd) - v, -AMAX * h, AMAX * h) : double(vCmd) - v;
            if (!knobs.delay && sub > 0) dv = 0;
            const float acc = float(dv / h);
            // Unmodelled disturbance: white angular acceleration on both links.
            s[2] += c.noise * gauss(rng) * std::sqrt(float(h));
            s[3] += c.noise * gauss(rng) * std::sqrt(float(h));
            est::rk4(truth, s, acc, float(h));
            x += v * h + .5 * dv * h; v += dv;
        }
        const float tilt = std::max(std::fabs(s[0]), std::fabs(s[1]));
        tiltSq += tilt * tilt; tiltMax = std::max(tiltMax, tilt);
        xMax = std::max(xMax, float(std::fabs(x))); vSum += v; n++;
        if (t > c.seconds - 5) { xSqLate += x * x; nLate++; }
        if (trace && k % 100 == 0)
            std::printf("  t=%5.1f x=%+7.1f mm v=%+6.1f mm/s base=%+5.2f tip=%+5.2f deg (meas %+5.2f %+5.2f)\n", t,
                        x * 1e3, v * 1e3, s[0] * 180 / kPi, s[1] * 180 / kPi, t1 * 180 / kPi, t2 * 180 / kPi);
    }
    return {"none", c.seconds, float(x), xMax, float(vSum / n), float(std::sqrt(tiltSq / n)), tiltMax,
            float(std::sqrt(xSqLate / std::max(1u, nLate)))};
}

static void row(const Law &law, const Case &c, unsigned seeds = 5) {
    unsigned survived = 0; float xMax = 0, xLate = 0, tiltRms = 0, tiltMax = 0, tStop = 1e9f; const char *stop = "none";
    for (unsigned sd = 1; sd <= seeds; sd++) {
        const Out o = simulate(law, c, sd);
        if (o.stop[0] == 'n') { survived++; xLate = std::max(xLate, o.xRmsLate); }
        else if (o.t < tStop) { tStop = o.t; stop = o.stop; }
        xMax = std::max(xMax, o.xMax); tiltRms = std::max(tiltRms, o.tiltRms); tiltMax = std::max(tiltMax, o.tiltMax);
    }
    std::printf("%-26s %u/%u ok  |x|max %5.0f mm  x_rms(last 5 s) %5.1f mm  tilt rms %.2f max %.2f deg",
                law.name, survived, seeds, xMax * 1e3, xLate * 1e3, tiltRms * 180 / kPi, tiltMax * 180 / kPi);
    if (survived < seeds) std::printf("  first stop %s at %.1f s", stop, tStop);
    std::printf("\n");
}

static void diagnose() {
    const Law hold{"current (no x term)", 0, 0};
    const Case quiet{0, 0, 1.f, 0.f, 20.f};
    struct D { const char *name; Knobs k; } ds[] = {
        {"firmware-faithful", {}},
        {"no friction", {false, true, true, true, true}},
        {"no quantisation", {true, false, true, true, true}},
        {"true state (no EKF)", {true, true, false, true, true}},
        {"no accel limit", {true, true, true, true, false}},
        {"no friction, no quant", {false, false, true, true, true}},
        {"true state, no friction", {false, true, false, true, true}},
        {"ideal: true state, no fric/quant", {false, false, false, true, true}},
    };
    std::printf("== diagnosis: no bias, no disturbance (what sustains the ~1 Hz oscillation?)\n");
    for (auto &d : ds) { knobs = d.k; std::printf("%-34s", d.name); row(hold, quiet); }
    knobs = {};
}

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::string(argv[1]) == "diag") { diagnose(); return 0; }
    if (argc > 1 && std::string(argv[1]) == "fric") {
        const Law hold{"current (no x term)", 0, 0};
        const Case quiet{0, 0, 1.f, 0.f, 20.f};
        for (float f : {0.f, .25f, .5f, 1.f, 2.f}) { knobs = {}; knobs.fcScale = f; knobs.bScale = 0;
            std::printf("Coulomb x%.2f, viscous 0  ", f); row(hold, quiet); }
        knobs = {}; knobs.fcScale = 0; std::printf("Coulomb 0, viscous x1     "); row(hold, quiet);
        return 0;
    }
    // gain K0..K5 kx ki from argv: evaluate a candidate design
    if (argc >= 9) {
        static float K[6]; for (int i = 0; i < 6; i++) K[i] = std::stof(argv[1 + i]);
        knobs = {}; knobs.gain = K;
        const Law law{"candidate", std::stof(argv[7]), std::stof(argv[8])};
        const float B1 = .0068f, B2 = .0030f;
        const Case cs[] = {{0, 0, 1, 0, 30}, {B1, B2, 1, 0, 30}, {B1, B2, 1, .1f, 30}, {2 * B1, 2 * B2, 1, .1f, 30},
                           {-B1, -B2, 1, .1f, 30}, {B1, B2, 1.1f, .1f, 30}, {B1, B2, .9f, .1f, 30}};
        const char *names[] = {"no bias quiet", "bias quiet", "bias dist", "2x bias dist", "-bias dist", "J+10% dist", "J-10% dist"};
        for (int i = 0; i < 7; i++) { std::printf("%-14s ", names[i]); row(law, cs[i]); }
        if (argc > 9) { knobs.fcScale = std::stof(argv[9]); std::printf("(Coulomb x%.1f)\n", knobs.fcScale);
            for (int i = 0; i < 7; i++) { std::printf("%-14s ", names[i]); row(law, cs[i]); } }
        return 0;
    }
    const float K0 = K_q[0];
    const Law laws[] = {
        {"current (no x term)", 0, 0},
        {"x fb 0.5*K0 (as guarded)", .5f * K0, 0},
        {"x fb 1.0*K0", K0, 0},
        {"x 0.5*K0 + int 0.5", .5f * K0, .5f},
        {"x 0.5*K0 + int 1.0", .5f * K0, 1.f},
        {"x 1.0*K0 + int 1.0", K0, 1.f},
        {"x 1.0*K0 + int 2.0", K0, 2.f},
        {"x 1.0*K0 + int 4.0", K0, 4.f},
        {"x 1.0*K0 + int 8.0", K0, 8.f},
    };
    const float B1 = .0068f, B2 = .0030f;   // measured hold means from the 2026-11-24 log
    struct Named { const char *name; Case c; } cases[] = {
        {"logged bias (+0.39, +0.17 deg), quiet", {B1, B2, 1.f, 0.f, 30.f}},
        {"logged bias, disturbance 0.3 rad/s^2", {B1, B2, 1.f, .3f, 30.f}},
        {"2x bias (+0.78, +0.34 deg), disturbance", {2 * B1, 2 * B2, 1.f, .3f, 30.f}},
        {"bias opposite sign, disturbance", {-B1, -B2, 1.f, .3f, 30.f}},
        {"logged bias, inertia +10%, disturbance", {B1, B2, 1.1f, .3f, 30.f}},
        {"logged bias, inertia -10%, disturbance", {B1, B2, .9f, .3f, 30.f}},
    };
    for (const auto &c : cases) {
        std::printf("\n== %s\n", c.name);
        for (const auto &l : laws) row(l, c.c);
    }
    std::printf("\n== trace: current law, logged bias, quiet\n");
    simulate(laws[0], cases[0].c, 1, true);
    std::printf("\n== trace: x 1.0*K0 + int 2.0, logged bias, quiet\n");
    simulate(laws[6], cases[0].c, 1, true);
}
