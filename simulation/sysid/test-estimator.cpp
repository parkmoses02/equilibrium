// Host test for src/double_pendulum/state_estimator.h.
//   g++ -std=c++17 -O2 simulation/sysid/test-estimator.cpp -o test-estimator && ./test-estimator
// The "true" plant has mismatched inertias and friction the EKF does not know.
// Encoders are quantised x4 counters. Injected faults mimic the ISR:
//   slip   = the shaft outran the ISR: +/-2 counts lost in the direction of motion
//   glitch = both lines spike and return: two invalid edges, no net count
#include "../../src/double_pendulum/state_estimator.h"
#include "../../src/double_pendulum/du_frames.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

using est::kPi;
constexpr int CPR = 4000;
constexpr float DT = .005f;

struct Faults { float slip = 0, glitch = 0; bool deadElbow = false; bool verbose = false; };
struct Result {
    double kfRate, oldRate, kfAngle;
    long residualCounts;
    unsigned fixed, unresolved, events, lost;
    float worstMiss;
};

static Result run(const Faults &fault, unsigned seed) {
    ModelParams truth = kModel;
    truth.J1 *= 1.06f; truth.J2 *= .95f; truth.d1 *= 1.03f;
    truth.b1 = 2e-4f; truth.b2 = 1e-4f; truth.fc1 = 3e-4f;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> uni(0.f, 1.f);

    float s[4] = {kPi + .02f, kPi + .01f, 0.f, 0.f}; // unwrapped truth
    // Encoder channel 0 counts base - pi, channel 1 counts tip - base (z taken at true DD).
    long missed[2] = {0, 0}, offsets[2] = {0, 0};
    uint32_t bad[2] = {0, 0};
    auto counted = [&](int ch, double angle) { return long(std::floor(angle * CPR / (2 * kPi))) - missed[ch]; };
    auto reading = [&](int ch, double angle) {
        return est::wrap(float((counted(ch, angle) - offsets[ch]) * 2 * kPi / CPR));
    };
    auto channels = [&](const float *x, float z[2]) {
        z[0] = est::wrap(reading(0, x[0] - kPi) + kPi);
        z[1] = reading(1, x[1] - x[0]);
    };
    float z[2];
    channels(s, z);
    est::AngleEKF kf;
    kf.reset(z[0], z[0] + z[1]);
    est::ChannelGuard guard[2];

    float oldW[2] = {0, 0}, oldPrev[2] = {kf.s[0], kf.s[1]};
    double eRate = 0, eOld = 0, eAngle = 0;
    unsigned n = 0, lost = 0, events = 0;
    float worst = 0;
    const unsigned duCount = sizeof(duFrames) / sizeof(duFrames[0]);
    double prev[2] = {s[0] - kPi, s[1] - s[0]};
    for (unsigned frame = 0; frame < 2400; frame++) {
        // Aggressive input: the DD->DU feedforward, then a chirp through the modes.
        float u;
        if (frame < duCount) u = duFrames[frame].u;
        else {
            const float t = (frame - duCount) * DT, f = .4f + .25f * t;
            u = 3.f * sinf(2 * kPi * (.4f * t + .125f * t * t)) * (f < 4.f);
        }
        for (int k = 0; k < 5; k++) est::rk4(truth, s, u, DT / 5);

        const double now[2] = {s[0] - kPi, s[1] - s[0]};
        for (int ch = 0; ch < 2; ch++) {
            const double moved = (now[ch] - prev[ch]) * CPR / (2 * kPi);
            prev[ch] = now[ch];
            if (fault.deadElbow && ch == 1 && frame > 400) { bad[ch]++; continue; }
            if (frame > 2380) continue; // let the last event resolve before scoring
            if (std::fabs(moved) >= 2 && uni(rng) < fault.slip) { missed[ch] += moved > 0 ? 2 : -2; bad[ch]++; events++; }
            if (uni(rng) < fault.glitch) { bad[ch] += 2; events++; }
        }
        channels(s, z);

        kf.predict(u, DT);
        for (int ch = 0; ch < 2; ch++) {
            const unsigned unresolvedBefore = guard[ch].unresolved;
            guard[ch].worstMiss = 0;
            const auto step = est::processChannel(kf, ch, z[ch], bad[ch], CPR, guard[ch]);
            offsets[ch] -= step.countShift;
            worst = std::max(worst, guard[ch].worstMiss);
            if (fault.verbose && guard[ch].unresolved != unresolvedBefore)
                std::printf("  unresolved: seed %u frame %u ch%d miss %.2f counts\n", seed, frame, ch, guard[ch].worstMiss);
            if (est::channelLost(guard[ch])) lost++;
        }
        if (lost) break;

        // Previous firmware estimate: IIR on differenced raw (uncorrected) angles.
        const float raw[2] = {z[0], est::wrap(z[0] + z[1])};
        for (int j = 0; j < 2; j++) {
            oldW[j] = .6f * oldW[j] + .4f * est::wrap(raw[j] - oldPrev[j]) / DT;
            oldPrev[j] = raw[j];
            eRate += std::pow(kf.s[2 + j] - s[2 + j], 2);
            eOld += std::pow(oldW[j] - s[2 + j], 2);
            eAngle += std::pow(est::wrap(kf.s[j] - est::wrap(s[j])), 2);
        }
        n += 2;
    }
    // Remaining count error: what the corrected reading still misses.
    const long residual = std::labs(missed[0] + offsets[0]) + std::labs(missed[1] + offsets[1]);
    return {std::sqrt(eRate / n), std::sqrt(eOld / n), std::sqrt(eAngle / n), residual,
            guard[0].fixed + guard[1].fixed, guard[0].unresolved + guard[1].unresolved, events, lost, worst};
}

static void campaign(const char *name, Faults fault, unsigned runs, bool expectClean) {
    unsigned fixed = 0, unresolved = 0, events = 0, wrong = 0, silent = 0, lost = 0, drift = 0;
    float worst = 0;
    double rate = 0, old = 0;
    for (unsigned seed = 1; seed <= runs; seed++) {
        const auto r = run(fault, seed);
        fixed += r.fixed; unresolved += r.unresolved; events += r.events; lost += r.lost != 0;
        wrong += r.residualCounts != 0 && !r.lost; // a lost run stops the motor anyway
        silent += r.residualCounts != 0 && r.unresolved == 0 && !r.lost;
        worst = std::max(worst, r.worstMiss);
        rate += r.kfRate / runs; old += r.oldRate / runs;
        if (!r.residualCounts && !r.lost && r.kfAngle >= 3e-3) {
            std::printf("  seed %u: angle RMS %.2f mrad with no count error (rate RMS %.3f)\n", seed,
                        1e3 * r.kfAngle, r.kfRate);
            drift++;
        }
    }
    std::printf("%-8s %2u runs, %4u injected: %4u resolved, %2u flagged, %u runs with count error (%u unflagged), "
                "%u lost; worst step miss %.2f counts; rate RMS %.3f rad/s (old %.3f)\n",
                name, runs, events, fixed, unresolved, wrong, silent, lost, worst, rate, old);
    assert(silent == 0 && drift == 0); // a wrong re-alignment must always be flagged
    if (expectClean) assert(!lost && !wrong && unresolved * 20 <= fixed);
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const auto clean = run({}, 1);
    std::printf("clean    rate RMS EKF %.3f rad/s vs old filter %.3f; angle RMS %.2f mrad\n",
                clean.kfRate, clean.oldRate, 1e3 * clean.kfAngle);
    assert(!clean.lost && clean.residualCounts == 0);
    assert(clean.kfRate < .5 * clean.oldRate);

    // "Occasional": roughly one fault per channel per second of fast motion.
    campaign("typical", {.005f, .002f}, 40, true);
    // Stress: ~5 faults per channel per second; events overlap and may hit limits.
    campaign("stress", {.02f, .005f}, 40, false);

    const auto dead = run({0, 0, true}, 1);
    std::printf("dead     elbow encoder invalid every frame -> lost = %s\n", dead.lost ? "yes" : "no");
    assert(dead.lost);
    std::puts("PASS");
}
