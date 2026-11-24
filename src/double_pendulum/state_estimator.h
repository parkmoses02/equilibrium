// Pendulum-angle EKF and encoder-slip recovery. Plain C++ so that the host test
// (simulation/sysid/test-estimator.cpp) exercises exactly this code.
//
// State: absolute base/tip angles from up and their rates. Input: cart
// acceleration over the last frame, taken from consecutive ramp velocities.
// Measurements: channel 0 = base angle (base HEDS), channel 1 = tip - base
// (elbow HEDS), so each encoder can be skipped independently.
#pragma once
#include <math.h>
#include <stdint.h>
#include <algorithm>
#include "model_params.h"

namespace est {
constexpr float kPi = 3.14159265358979f;
inline float wrap(float a) { return remainderf(a, 2.f * kPi); }

inline void derivative(const ModelParams &p, const float s[4], float u, float d[4]) {
    const float a = s[0], b = s[1], w = s[2], z = s[3];
    const float h = p.C * cosf(a - b), sn = p.C * sinf(a - b), det = p.J1 * p.J2 - h * h;
    const float rel = z - w;
    // Elbow friction acts equally and oppositely on the two links.
    const float elbow = p.b2 * rel + p.fc2 * tanhf(rel / p.coulombWidth);
    const float f = p.d1 * (p.g * sinf(a) - u * cosf(a)) - sn * z * z + elbow
                    - p.b1 * w - p.fc1 * tanhf(w / p.coulombWidth);
    const float k = p.d2 * (p.g * sinf(b) - u * cosf(b)) + sn * w * w - elbow;
    d[0] = w;
    d[1] = z;
    d[2] = (p.J2 * f - h * k) / det;
    d[3] = (p.J1 * k - h * f) / det;
}

inline void rk4(const ModelParams &p, float s[4], float u, float dt) {
    float k1[4], k2[4], k3[4], k4[4], t[4];
    derivative(p, s, u, k1);
    for (int i = 0; i < 4; i++) t[i] = s[i] + .5f * dt * k1[i];
    derivative(p, t, u, k2);
    for (int i = 0; i < 4; i++) t[i] = s[i] + .5f * dt * k2[i];
    derivative(p, t, u, k3);
    for (int i = 0; i < 4; i++) t[i] = s[i] + dt * k3[i];
    derivative(p, t, u, k4);
    for (int i = 0; i < 4; i++) s[i] += dt / 6.f * (k1[i] + 2.f * k2[i] + 2.f * k3[i] + k4[i]);
}

struct AngleEKF {
    ModelParams p = kModel;
    float accelStd = 30.f;   // rad/s^2 the model does not explain (tune from sysid residuals)
    float measStd = 1.0e-3f; // rad: x4 quantisation (0.45 mrad) plus snapshot jitter
    float s[4]{};
    float P[4][4]{};
    float clock = 0.f; // s since reset; time base for slip fits

    void reset(float base, float tip, float rateStd = .05f) {
        s[0] = wrap(base); s[1] = wrap(tip); s[2] = s[3] = 0.f;
        clock = 0.f;
        for (auto &row : P) std::fill(row, row + 4, 0.f);
        P[0][0] = P[1][1] = measStd * measStd;
        P[2][2] = P[3][3] = rateStd * rateStd;
    }

    void predict(float u, float dt) {
        if (!(dt > 0.f)) return;
        clock += dt;
        // Continuous Jacobian by central differences, then F = I + A dt + (A dt)^2 / 2.
        float A[4][4];
        for (int j = 0; j < 4; j++) {
            const float eps = j < 2 ? 1e-3f : 1e-2f;
            float hi[4], lo[4], dh[4], dl[4];
            std::copy(s, s + 4, hi); std::copy(s, s + 4, lo);
            hi[j] += eps; lo[j] -= eps;
            derivative(p, hi, u, dh); derivative(p, lo, u, dl);
            for (int i = 0; i < 4; i++) A[i][j] = (dh[i] - dl[i]) / (2.f * eps);
        }
        float F[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                float a2 = 0.f;
                for (int k = 0; k < 4; k++) a2 += A[i][k] * A[k][j];
                F[i][j] = (i == j) + A[i][j] * dt + .5f * a2 * dt * dt;
            }
        const int steps = std::max(1, int(ceilf(dt / .005f)));
        for (int n = 0; n < steps; n++) rk4(p, s, u, dt / steps);
        s[0] = wrap(s[0]); s[1] = wrap(s[1]);

        float FP[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                float v = 0.f;
                for (int k = 0; k < 4; k++) v += F[i][k] * P[k][j];
                FP[i][j] = v;
            }
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                float v = 0.f;
                for (int k = 0; k < 4; k++) v += FP[i][k] * F[j][k];
                P[i][j] = v;
            }
        // Each link gets an independent white angular-acceleration disturbance.
        const float q = accelStd * accelStd;
        for (int j = 0; j < 2; j++) {
            P[j][j] += q * dt * dt * dt / 3.f;
            P[j][j + 2] += q * dt * dt / 2.f;
            P[j + 2][j] += q * dt * dt / 2.f;
            P[j + 2][j + 2] += q * dt;
        }
    }

    float expected(int ch) const { return ch == 0 ? s[0] : wrap(s[1] - s[0]); }

    // Scalar measurement update with an already wrapped innovation.
    void update(int ch, float innovation) {
        const float h[4] = {ch ? -1.f : 1.f, ch ? 1.f : 0.f, 0.f, 0.f};
        float Ph[4];
        for (int i = 0; i < 4; i++) {
            float v = 0.f;
            for (int j = 0; j < 4; j++) v += P[i][j] * h[j];
            Ph[i] = v;
        }
        float S = measStd * measStd;
        for (int i = 0; i < 4; i++) S += h[i] * Ph[i];
        for (int i = 0; i < 4; i++) s[i] += Ph[i] / S * innovation;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) P[i][j] -= Ph[i] * Ph[j] / S;
        for (int i = 0; i < 4; i++)
            for (int j = i + 1; j < 4; j++) P[i][j] = P[j][i] = .5f * (P[i][j] + P[j][i]);
        s[0] = wrap(s[0]); s[1] = wrap(s[1]);
    }
};

// ---- Encoder slip recovery --------------------------------------------------
// An invalid x4 transition (both lines changed between ISR samples) hides
// exactly +/-2 counts, so n invalid edges in one frame hide 2m counts with
// m in {-n, -n+2, ..., n}: the candidates are 4 counts apart.
// From the first invalid edge the channel runs on EKF prediction alone. After
// kPostFrames clean frames, a cubic in time plus one step per invalid frame is
// fitted by least squares to the raw readings around the event (encoder data
// only, so model error cannot bias it). Each step is rounded to its allowed
// set, the count offset is re-aligned and measurement updates resume.
// Above kSlipRate an edge is lost because the shaft outran the ISR, so hidden
// counts can only lie along the motion (a glitch pair still nets zero); the
// candidates are restricted to that side, which settles fast whips where a
// cubic over 40 ms cannot follow the curvature.
constexpr float kInnovationGate = .05f;    // rad in one clean frame
constexpr unsigned kPreFrames = 4, kPostFrames = 4;
constexpr int kPolyTerms = 4;              // cubic
constexpr float kSlipRate = .6f;           // rad/s: 2 counts per frame, ~3x EKF rate noise
constexpr unsigned kMaxSlipFrames = 3;     // frames with invalid edges in one event
constexpr unsigned kMaxPendingFrames = 12; // prediction-only frames in one event (60 ms)
constexpr unsigned kMaxRejectFrames = 3;   // consecutive gated measurements
constexpr unsigned kHistory = kPreFrames + kMaxPendingFrames + 1;
constexpr float kStepTolerance = 1.5f;     // counts between fitted step and chosen candidate (wrong only beyond 2)

struct ChannelGuard {
    uint32_t seenBad = 0;
    // Raw readings (rad), EKF clock and predicted channel rate, newest last.
    float z[kHistory]{}, t[kHistory]{}, w[kHistory]{};
    uint8_t bad[kHistory]{};
    unsigned stored = 0;
    unsigned pendingFrames = 0, cleanAfter = 0, slipFrames = 0, rejectStreak = 0;
    bool lost = false;
    unsigned fixed = 0, unresolved = 0, rejected = 0;
    float worstMiss = 0.f; // largest |fitted - chosen| step in counts, for logging

    void push(float zv, float tv, float wv, uint8_t b) {
        if (stored == kHistory) {
            std::copy(z + 1, z + kHistory, z);
            std::copy(t + 1, t + kHistory, t);
            std::copy(w + 1, w + kHistory, w);
            std::copy(bad + 1, bad + kHistory, bad);
        } else ++stored;
        z[stored - 1] = zv; t[stored - 1] = tv; w[stored - 1] = wv; bad[stored - 1] = b;
    }
};
enum class Reading : uint8_t { measured, predicted, rejected };
struct ChannelStep { Reading reading; int32_t countShift; };

// Small dense solve (n <= 8), partial pivoting. False if singular.
inline bool solveSmall(double A[8][8], double b[8], int n) {
    for (int c = 0; c < n; c++) {
        int piv = c;
        for (int r = c + 1; r < n; r++) if (fabs(A[r][c]) > fabs(A[piv][c])) piv = r;
        if (fabs(A[piv][c]) < 1e-12) return false;
        if (piv != c) { for (int k = 0; k < n; k++) std::swap(A[c][k], A[piv][k]); std::swap(b[c], b[piv]); }
        for (int r = c + 1; r < n; r++) {
            const double f = A[r][c] / A[c][c];
            for (int k = c; k < n; k++) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int c = n - 1; c >= 0; c--) {
        for (int k = c + 1; k < n; k++) b[c] -= A[c][k] * b[k];
        b[c] /= A[c][c];
    }
    return true;
}

// Fit the pending event; returns the total count correction and whether every
// step landed within kStepTolerance of an allowed value. History is corrected
// in place so that later fits see consistent readings.
inline long resolveSlips(ChannelGuard &g, int cpr, bool &sure) {
    const unsigned window = std::min(g.stored, kPreFrames + g.pendingFrames);
    const unsigned first = g.stored - window, pendingStart = g.stored - g.pendingFrames;
    unsigned stepAt[kMaxSlipFrames];
    int steps = 0;
    for (unsigned i = std::max(first, pendingStart); i < g.stored; i++)
        if (g.bad[i] && steps < int(kMaxSlipFrames)) stepAt[steps++] = i;
    const int unknowns = kPolyTerms + steps;
    sure = false;
    if (int(window) < unknowns + 2) return 0;
    const double countsPerRad = cpr / (2.0 * kPi);
    double A[8][8] = {}, b[8] = {};
    double y = 0;
    for (unsigned i = first; i < g.stored; i++) {
        if (i > first) y += double(wrap(g.z[i] - g.z[i - 1])) * countsPerRad;
        const double tau = (double(g.t[i]) - double(g.t[pendingStart])) / .02;
        double phi[8] = {1};
        for (int j = 1; j < kPolyTerms; j++) phi[j] = phi[j - 1] * tau;
        for (int j = 0; j < steps; j++) phi[kPolyTerms + j] = i >= stepAt[j];
        for (int r = 0; r < unknowns; r++) {
            b[r] += phi[r] * y;
            for (int c = 0; c < unknowns; c++) A[r][c] += phi[r] * phi[c];
        }
    }
    if (!solveSmall(A, b, unknowns)) return 0;
    sure = true;
    long total = 0;
    for (int j = 0; j < steps; j++) {
        // The reading dropped by the hidden counts, so the correction is -step.
        const double hidden = -b[kPolyTerms + j];
        const long n = std::min<long>(g.bad[stepAt[j]], 8);
        const float rate = g.w[stepAt[j]];
        const long along = fabsf(rate) > kSlipRate ? ((rate > 0) == (cpr > 0) ? 1 : -1) : 0;
        long best = 0;
        double miss = 1e9;
        for (long m = -n; m <= n; m += 2)
            if (m * along >= 0 && fabs(hidden - 2.0 * m) < miss) { miss = fabs(hidden - 2.0 * m); best = 2 * m; }
        sure = sure && miss <= kStepTolerance;
        g.worstMiss = std::max(g.worstMiss, float(miss));
        total += best;
        for (unsigned i = stepAt[j]; i < g.stored; i++) g.z[i] = wrap(g.z[i] + float(best / countsPerRad));
    }
    return total;
}

// z: raw channel angle (rad) with the current offset. badTotal: cumulative
// invalid-edge count of that encoder. countShift is added to the reading from
// now on, i.e. the caller does offset -= countShift.
inline ChannelStep processChannel(AngleEKF &kf, int ch, float z, uint32_t badTotal, int cpr,
                                  ChannelGuard &g) {
    const uint32_t bad = badTotal - g.seenBad;
    g.seenBad = badTotal;
    g.push(z, kf.clock, ch == 0 ? kf.s[2] : kf.s[3] - kf.s[2], uint8_t(std::min<uint32_t>(bad, 255u)));
    long shift = 0;
    if (bad || g.pendingFrames) {
        ++g.pendingFrames;
        if (bad) { g.cleanAfter = 0; ++g.slipFrames; }
        else ++g.cleanAfter;
        if (g.slipFrames > kMaxSlipFrames || g.pendingFrames > kMaxPendingFrames) g.lost = true;
        if (g.lost || g.cleanAfter < kPostFrames) return {Reading::predicted, 0};
        bool sure;
        shift = resolveSlips(g, cpr, sure);
        ++(sure ? g.fixed : g.unresolved);
        g.pendingFrames = g.cleanAfter = g.slipFrames = 0;
        z = wrap(z + float(shift) * 2.f * kPi / float(cpr));
    }
    const float innovation = wrap(z - kf.expected(ch));
    if (fabsf(innovation) > kInnovationGate) {
        ++g.rejected;
        if (++g.rejectStreak > kMaxRejectFrames) g.lost = true;
        return {Reading::rejected, int32_t(shift)};
    }
    g.rejectStreak = 0;
    kf.update(ch, innovation);
    return {Reading::measured, int32_t(shift)};
}

inline bool channelLost(const ChannelGuard &g) { return g.lost; }
} // namespace est
