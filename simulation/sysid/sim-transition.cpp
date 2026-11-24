// UD <-> DU transitions as upright.h runs them:
//   hold (LQR u=-Kx) -> armed, still and centred for 1 s -> table + TVLQR -> catch -> hold (LQR u=-Kx)
//   g++ -std=c++17 -O2 -I src/double_pendulum simulation/sysid/sim-transition.cpp -o sim-transition && ./sim-transition
//   candidate tables: -DUD_DU_FRAMES_HEADER='"..."' / -DDU_UD_FRAMES_HEADER='"..."'
// Plant: sysid-fitted model with friction, encoder zero bias, x4 4000-count encoders,
// firmware EKF, TMC ramp slewing to VMAX (AMAX 12 in hold, 16 during the transition).
#include "../../src/double_pendulum/state_estimator.h"
#ifdef UD_DU_FRAMES_HEADER
#include UD_DU_FRAMES_HEADER
#else
#include "../../src/double_pendulum/ud_du_frames.h"
#endif
#ifdef DU_UD_FRAMES_HEADER
#include DU_UD_FRAMES_HEADER
#else
#include "../../src/double_pendulum/du_ud_frames.h"
#endif
#ifdef UU_UD_FRAMES_HEADER
#include UU_UD_FRAMES_HEADER
#else
#include "../../src/double_pendulum/uu_ud_frames.h"
#endif
#ifdef UU_DU_FRAMES_HEADER
#include UU_DU_FRAMES_HEADER
#else
#include "../../src/double_pendulum/uu_du_frames.h"
#endif
#ifdef DU_UU_FRAMES_HEADER
#include DU_UU_FRAMES_HEADER
#else
#include "../../src/double_pendulum/du_uu_frames.h"
#endif
#ifdef UD_UU_FRAMES_HEADER
#include UD_UU_FRAMES_HEADER
#else
#include "../../src/double_pendulum/ud_uu_frames.h"
#endif
#include "../../src/double_pendulum/rail_guard.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

using est::kPi;
constexpr int CPR = 4000;
constexpr float DT = .005f, VMAX = .8f, TRAVEL = .30f, D = kPi / 180;
// Firmware hold gains (upright.h); both holds use the full LQR u = -K x.
constexpr float K_w[] = {-3.464102f, -52.422759f, 17.225801f, -4.465170f, -4.803921f, -0.558010f};
constexpr float K_e[] = {-14.832397f, 153.628010f, -244.278154f, -17.451330f, 11.412767f, -27.573131f};
constexpr float K_q[] = {11.105604f, -147.661261f, 198.038011f, 11.411039f, -4.511831f, 21.653094f};
constexpr float KI_Q = 3.651484f;   // UU hold is LQI
// Readiness and catch (upright.h TRANSITION_READY_* and catch limits).
static float READY_ANGLE = 1.f * D, READY_RATE = .2f;   // runtime: the UU study varies them
static float READY_SPEED = .05f;
static bool GATE = false;   // after the window: start at an instant with |e| <= 1 deg and |v| <= .05
constexpr float READY_X = .05f, READY_S = 1.f;
constexpr float RATE_TAU_S = .1f, REQUEST_S = 2.f, READY_TIMEOUT_S = 10.f, END_HOLD_S = 10.f;
constexpr float CATCH_ANGLE = .10f, CATCH_RATE = .5f, CATCH_SPEED = .30f;

struct Pose { const char *name; float base, tip; const float *K; float tilt; };
const Pose UD{"UD", 0, kPi, K_w, .17f}, DU{"DU", kPi, 0, K_e, 20 * D}, UU{"UU", 0, 0, K_q, .17f};
// Readiness window per source hold: UD/DU sit still; UU limit-cycles (about +-2.5 deg, +-0.13 m/s)
// from pivot Coulomb friction, so it gets a wider window (upright.h UU_READY_*).
struct Route { const char *name; const Pose &from, &to; const DuFrame *frames; unsigned count; float rAngle = 1, rRate = .2f, rSpeed = .05f; };
const Route routes[] = {
    {"UD->DU", UD, DU, udDuFrames, sizeof(udDuFrames) / sizeof(udDuFrames[0])},
    {"DU->UD", DU, UD, duUdFrames, sizeof(duUdFrames) / sizeof(duUdFrames[0])},
    {"UU->UD", UU, UD, uuUdFrames, sizeof(uuUdFrames) / sizeof(uuUdFrames[0]), 4, .5f, .2f},
    {"UU->DU", UU, DU, uuDuFrames, sizeof(uuDuFrames) / sizeof(uuDuFrames[0]), 4, .5f, .2f},
    {"DU->UU", DU, UU, duUuFrames, sizeof(duUuFrames) / sizeof(duUuFrames[0])},
    {"UD->UU", UD, UU, udUuFrames, sizeof(udUuFrames) / sizeof(udUuFrames[0])},
};

struct Case { const char *name; float biasBase = 0, biasTip = 0, J = 1, fc = 1, b = 1, d1 = 1, noise = 0, startTilt = 0; };
struct Out { std::string stop; float readyAt = 0, ebMax = 0, etMax = 0, xMax = 0, endTiltRms = 0; };

static float refRate(const Route &r, unsigned i, bool tip) {
    const unsigned lo = i ? i - 1 : 0, hi = i + 1 < r.count ? i + 1 : r.count - 1;
    const float a = tip ? r.frames[lo].tip : r.frames[lo].base, b = tip ? r.frames[hi].tip : r.frames[hi].base;
    return remainderf(b - a, 2 * kPi) / ((hi - lo) * DT);
}
static float holdLaw(const Pose &p, float e1, float e2, float v, float w1, float w2, float x) {
    const float *K = p.K;
    return -(K[0] * x + K[1] * e1 + K[2] * e2 + K[3] * v + K[4] * w1 + K[5] * w2);
}

static Out run(const Route &route, const Case &c, unsigned seed, bool verbose = false, bool perRoute = true) {
    if (perRoute) { READY_ANGLE = route.rAngle * D; READY_RATE = route.rRate; READY_SPEED = route.rSpeed; }
    ModelParams truth = kModel;
    truth.J1 *= c.J; truth.J2 *= c.J; truth.C *= c.J; truth.d1 *= c.d1;
    truth.fc1 *= c.fc; truth.fc2 *= c.fc; truth.b1 *= c.b; truth.b2 *= c.b;
    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0.f, 1.f);
    const Pose &A = route.from, &B = route.to;
    float s[4] = {A.base + .01f * gauss(rng), A.tip + .01f * gauss(rng), 0, 0};
    double x = .01 * gauss(rng), v = 0;
    float vCmd = 0, vPrev = 0, amax = 12.f;
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
    est::ChannelGuard guard[2];
    enum { HOLD_FROM, SWING, HOLD_TO } phase = HOLD_FROM;
    Out o;
    float stable = 0, origin = 0, tiltSq = 0, wf1 = 0, wf2 = 0, xi = 0;
    unsigned step = 0, endFrames = 0;
    for (unsigned k = 0;; k++) {
        const float t = k * DT;
        // Firmware: EKF process noise 30 rad/s^2 during swings, HOLD_EKF_ACCEL_STD (4) in holds.
        kf.accelStd = phase == SWING ? 30.f : (std::getenv("HOLD30") ? 30.f : 4.f);
        kf.predict(k ? (float(v) - vPrev) / DT : 0.f, DT);
        vPrev = float(v);
        measure(z);
        for (int ch = 0; ch < 2; ch++) est::processChannel(kf, ch, z[ch], 0, CPR, guard[ch]);
        const float t1 = kf.s[0], t2 = kf.s[1], w1 = kf.s[2], w2 = kf.s[3];
        wf1 += DT / RATE_TAU_S * (w1 - wf1); wf2 += DT / RATE_TAU_S * (w2 - wf2);
        o.xMax = std::max(o.xMax, float(std::fabs(x)));
        if (std::fabs(x) > TRAVEL) { o.stop = "RAMP_POSITION_LIMIT"; break; }
        if (RailGuard::needsBrake(float(x), float(v)) || RailGuard::needsBrake(float(x), vCmd)) { o.stop = "RAIL_BRAKE"; break; }
        float u = 0;
        if (phase == HOLD_FROM) {
            const float e1 = est::wrap(t1 - A.base), e2 = est::wrap(t2 - A.tip);
            if (std::fabs(e1) > A.tilt || std::fabs(e2) > A.tilt) { o.stop = std::string(A.name) + "_TILT_LIMIT"; break; }
            u = holdLaw(A, e1, e2, float(v), w1, w2, float(x));
            if (A.K == K_q) { xi = std::clamp(xi + float(x) * DT, -1.5f / KI_Q, 1.5f / KI_Q); u -= KI_Q * xi; }
            if (verbose && k % 40 == 0)
                std::printf("  %s t=%5.2f e %+5.2f %+5.2f deg | w_lp %+5.2f %+5.2f | v %+6.3f x %+5.0f mm\n", A.name, t,
                            e1 / D, e2 / D, wf1, wf2, v, x * 1e3);
            if (t >= REQUEST_S) {
                const bool ok = std::fabs(e1) <= READY_ANGLE && std::fabs(e2) <= READY_ANGLE && std::fabs(wf1) <= READY_RATE &&
                                std::fabs(wf2) <= READY_RATE && std::fabs(v) <= READY_SPEED && std::fabs(x) <= READY_X;
                stable = ok ? stable + DT : 0;
                const bool gateOk = !GATE || (std::fabs(e1) <= 1 * D && std::fabs(e2) <= 1 * D && std::fabs(v) <= .05f);
                if (stable >= READY_S && gateOk) { phase = SWING; step = 0; origin = float(x); amax = 16.f; o.readyAt = t - REQUEST_S; }
                else if (t - REQUEST_S > READY_TIMEOUT_S) { o.stop = "NOT_READY"; break; }
            }
        } else if (phase == SWING) {
            if (step == route.count) {
                const float e1 = est::wrap(t1 - B.base), e2 = est::wrap(t2 - B.tip);
                bool caught = std::fabs(e1) <= CATCH_ANGLE && std::fabs(e2) <= CATCH_ANGLE && std::fabs(wf1) < CATCH_RATE &&
                              std::fabs(wf2) < CATCH_RATE && std::fabs(v) <= CATCH_SPEED;
                if (&B == &UU && std::getenv("STRICT_UU_CATCH")) {   // DD->UU uuCatchReady (DU->UU uses the transition limits)
                    const float ang = std::max(std::fabs(e1), std::fabs(e2)), rate = std::max(std::fabs(wf1), std::fabs(wf2));
                    const float spd = float(std::fabs(v));
                    caught = (ang <= .05f && spd <= .20f) ? rate < .5f
                           : rate <= .05f && std::fabs(e1 - e2) <= .01f && ang / .10f + spd / .30f <= 1.f;
                }
                if (!caught) {
                    o.stop = std::string("MISSED_") + B.name;
                    if (std::getenv("MISS")) std::printf("  miss: e %+.2f %+.2f deg w_lp %+.2f %+.2f v %+.3f\n", e1 / D, e2 / D, wf1, wf2, v);
                    break;
                }
                phase = HOLD_TO; vCmd = float(v); amax = 12.f; xi = 0;   // firmware clears the UU integral at catch
            } else {
                const auto &r = route.frames[step];
                const float e[6] = {0, est::wrap(t1 - r.base), est::wrap(t2 - r.tip), float(v) - r.v,
                                    w1 - refRate(route, step, false), w2 - refRate(route, step, true)};
                if (std::fabs(e[1]) > .45f || std::fabs(e[2]) > .45f) { o.stop = "SWING_TRACKING_ERROR"; break; }
                o.ebMax = std::max(o.ebMax, std::fabs(e[1])); o.etMax = std::max(o.etMax, std::fabs(e[2]));
                u = r.u;
                for (int j = 1; j < 6; j++) u -= r.k[j] * e[j];
                u += std::clamp(-.5f * r.k[0] * (float(x) - (origin + r.x)), -1.f, 1.f);   // positionFeedback
                if (verbose && step % 40 == 0)
                    std::printf("  %3u x %+5.0f ref %+5.0f mm | eb %+5.1f et %+5.1f deg | u %+6.2f\n", step, x * 1e3,
                                (origin + r.x) * 1e3, e[1] / D, e[2] / D, u);
                step++;
            }
        }
        if (phase == HOLD_TO) {
            const float e1 = est::wrap(t1 - B.base), e2 = est::wrap(t2 - B.tip);
            if (std::fabs(e1) > B.tilt || std::fabs(e2) > B.tilt) { o.stop = std::string(B.name) + "_TILT_LIMIT"; break; }
            u = holdLaw(B, e1, e2, float(v), w1, w2, float(x));
            if (B.K == K_q) { xi = std::clamp(xi + float(x) * DT, -1.5f / KI_Q, 1.5f / KI_Q); u -= KI_Q * xi; }
            const float tilt = std::max(std::fabs(est::wrap(s[0] - B.base)), std::fabs(est::wrap(s[1] - B.tip)));
            tiltSq += tilt * tilt;
            if (++endFrames > END_HOLD_S / DT) { o.stop = "HELD"; break; }
        }
        const float limit = phase == SWING ? 16.f : 12.f;
        vCmd = std::clamp(vCmd + std::clamp(u, -limit, limit) * DT, -VMAX, VMAX);
        for (int sub = 0; sub < 5; sub++) {
            const double h = DT / 5;
            const double dv = std::clamp(double(vCmd) - v, -amax * h, amax * h);
            s[2] += c.noise * gauss(rng) * std::sqrt(float(h));
            s[3] += c.noise * gauss(rng) * std::sqrt(float(h));
            est::rk4(truth, s, float(dv / h), float(h));
            x += v * h + .5 * dv * h; v += dv;
        }
    }
    o.endTiltRms = endFrames ? std::sqrt(tiltSq / endFrames) : 0;
    return o;
}

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc > 1 && std::string(argv[1]) == "uuready") {
        // UU hold limit-cycles 1-2 deg (pivot Coulomb friction): which readiness limits let UU->UD start?
        const Case cs[] = {{"bias +0.1/+0.2 deg", .1f * D, .2f * D}, {"bias, J +2%", .1f * D, .2f * D, 1.02f},
                           {"bias, J -2%", .1f * D, .2f * D, .98f}, {"bias, Coulomb x1.3", .1f * D, .2f * D, 1, 1.3f},
                           {"bias, disturbance 0.05", .1f * D, .2f * D, 1, 1, 1, 1, .05f}};
        const struct { float ang, rate, speed; bool gate; } opts[] = {{3, .5f, .15f, false}, {3, .5f, .15f, true}, {4, .5f, .2f, false}, {4, .5f, .2f, true}};
        for (const auto &op : opts) {
            READY_ANGLE = op.ang * D; READY_RATE = op.rate; READY_SPEED = op.speed; GATE = op.gate;
            std::printf("\n== UU->UD, window: angle %.0f deg, rate_lp %.1f, speed %.2f for 1 s%s\n", op.ang, op.rate, op.speed,
                        op.gate ? ", then start at a calm instant" : "");
            for (const auto &c : cs) {
                unsigned held = 0; float ready = 0, xm = 0; std::string fail;
                for (unsigned sd = 1; sd <= 10; sd++) { const Out o = run(routes[2], c, sd, false, false);
                    if (o.stop == "HELD") held++; else fail = o.stop; ready = std::max(ready, o.readyAt); xm = std::max(xm, o.xMax); }
                std::printf("  %-24s held %2u/10 | ready <= %.2f s | |x|max %3.0f mm %s\n", c.name, held, ready, xm * 1e3, fail.c_str());
            }
        }
        return 0;
    }
    if (argc > 1) {   // trace: ud-du, du-ud or uu-ud
        const std::string a = argv[1];
        const Route &r = a == "du-ud" ? routes[1] : a == "uu-ud" ? routes[2] : a == "uu-du" ? routes[3] : a == "du-uu" ? routes[4] : a == "ud-uu" ? routes[5] : routes[0];
        const Out o = run(r, {"trace", .1f * D, .2f * D}, 1, true);
        std::printf("%s %s (ready after %.2f s)\n", r.name, o.stop.c_str(), o.readyAt);
        return 0;
    }
    // Hardware UD holds (2026-11-24) gave mean errors of about -0.03 / -0.09 deg.
    const float B1 = .1f * D, B2 = .2f * D;
    const Case cases[] = {
        {"exact, no bias"},
        {"bias +0.1/+0.2 deg", B1, B2},
        {"bias -0.3/-0.3 deg", -.3f * D, -.3f * D},
        {"bias, disturbance 0.05", B1, B2, 1, 1, 1, 1, .05f},
        {"bias, J1,J2 +2%", B1, B2, 1.02f},
        {"bias, J1,J2 -2%", B1, B2, .98f},
        {"bias, d1 +2%", B1, B2, 1, 1, 1, 1.02f},
        {"bias, d1 -2%", B1, B2, 1, 1, 1, .98f},
        {"bias, Coulomb x0.7", B1, B2, 1, .7f},
        {"bias, Coulomb x1.3", B1, B2, 1, 1.3f},
        {"bias, viscous x3", B1, B2, 1, 1, 3},
        {"bias, J1,J2 +5%", B1, B2, 1.05f},
        {"bias, J1,J2 -5%", B1, B2, .95f},
    };
    for (const auto &r : routes) {
        std::printf("\n== %s: hold -> ready 1 s -> %u frames -> catch -> hold %.0f s (10 seeds)\n", r.name, r.count, END_HOLD_S);
        for (const auto &c : cases) {
            unsigned held = 0; float ready = 0, eb = 0, et = 0, xm = 0, tilt = 0; std::string fail;
            for (unsigned sd = 1; sd <= 10; sd++) {
                const Out o = run(r, c, sd);
                if (o.stop == "HELD") held++; else fail = o.stop;
                ready = std::max(ready, o.readyAt); eb = std::max(eb, o.ebMax); et = std::max(et, o.etMax);
                xm = std::max(xm, o.xMax); tilt = std::max(tilt, o.endTiltRms);
            }
            std::printf("  %-26s held %2u/10 | ready <= %.2f s | swing err %4.1f/%4.1f deg | |x|max %3.0f mm | end tilt rms %.2f deg %s\n",
                        c.name, held, ready, eb / D, et / D, xm * 1e3, tilt / D, fail.c_str());
        }
    }
}
