#pragma once
// System-identification recorder for the Rev.L double pendulum (env double_sysid).
// Records 200 Hz samples to RAM, then dumps CSV for simulation/sysid/fit.py.
//   z        hang DD still, cart centred, motor OFF: save angle and cart origin
//   f        free decay, motor stays OFF: swing the links by hand, release, then f
//   c / c1-9 chirp: cart sweeps 0.3 -> 5 Hz around z while the links hang
//            (accel amplitude 0.4 m/s^2 per digit, default c = c3)
//   d        dump the last recording as SYSID_CSV
//
// Free-decay (f) recipes. Angles are measured from hanging (DD). base = link 1
// (on the cart), tip = link 2. Hold the links at the start pose, let go
// without pushing, then send f straight away (fit.py drops the first 0.5 s).
// Do one run of each, with d after each run:
//   ("in line" = elbow straight, relative 0: tip points the same way as base,
//    not hanging vertically)
//   f1 base +60 deg, tip in line (tip also +60 abs)    slow mode, large swing (J1, d1, b1)
//   f2 base +15 deg, tip in line (tip also +15 abs)    small swing: viscous vs Coulomb friction
//   f3 base +30 deg, tip bent back to -30 deg abs      fast mode: J2 and C (rel. angle -60 deg)
//            (elbow folded the opposite way)
//   f4 base hanging (0), tip +45 deg                   elbow only (J2, b2, fc2)
// Keep every angle under about 70 deg: the model holds at large angles, but the tip
// can whip over the top. With the motor OFF, XACTUAL does not see real cart motion
// and fit.py assumes the cart is still, so clamp the cart or hold it firmly by hand
// during f (a freewheeling cart gets pushed by the swing and biases the fit).
//   h        encoder counters;  x  stop now (motor OFF)
#include <algorithm>
#include "encoder_config.h"
#include "revl_drive.h"
#include "rail_guard.h"

namespace sysid {
struct Sample { uint32_t us; float x, v, vcmd, base, tip; uint8_t flags; };
enum Flag : uint8_t { BAD1 = 1, BAD2 = 2, DRIVING = 4, BRAKING = 8, MOTOR_ON = 16 };
constexpr unsigned CAPACITY = 4400;             // 22 s at 5 ms
constexpr uint32_t FRAME_US = 5000, LATE_US = 20000;
constexpr float FREE_S = 20.f;
constexpr float CHIRP_S = 16.f, FADE_S = 1.5f, F0 = .3f, F1 = 5.f, TAIL_S = 4.f;
constexpr float X_MAX = .08f;                   // cart amplitude cap around z
constexpr float KP_X = 4.f;                     // 1/s, keeps the cart centred on the reference
constexpr float ACCEL_LIMIT = 16.f, SPEED_LIMIT = .8f;
constexpr float START_X_M = .03f;               // chirp needs the cart near z
constexpr float START_TILT_RAD = .10f, ANGLE_ABORT_RAD = 1.4f; // from hanging
constexpr float TRAVEL_LIMIT_M = .30f;

enum class Run : uint8_t { idle, freeDecay, chirp, braking, tail };
// One 123 KB block does not fit the ESP32 heap's largest free region, so
// the buffer is split into chunks allocated separately.
constexpr unsigned CHUNKS = 4, CHUNK = CAPACITY / CHUNKS;
static_assert(CHUNK * CHUNKS == CAPACITY, "CAPACITY must split evenly");
static Sample *chunk[CHUNKS] = {};
static bool bufferOk = false;
static Sample &at(unsigned i) { return chunk[i / CHUNK][i % CHUNK]; }
static unsigned used = 0;
static Run run = Run::idle;
static char kind = '-';
static float amp = 0, vCmd = 0;
static bool referenced = false;
static uint32_t cartZero, tick, start, phaseStart;
static long offsets[2];
static uint32_t badSeen[2];
static const char *reason = "";

static float cartX() { return static_cast<int32_t>(readRegister(REG_XACTUAL) - cartZero) / SCALE; }
static float hangingError(float a) { return fabsf(remainderf(a - PI, 2 * PI)); }

static void motorOff() {
    digitalWrite(PIN_EN, HIGH);
    writeRegister(REG_VMAX, 0);
    writeRegister(REG_RAMPMODE, 1);
}
static void finish(const char *why) {
    motorOff();
    run = Run::idle;
    reason = why;
    Serial.printf("SYSID_DONE kind=%c samples=%u (%.1f s) reason=%s. 'd' dumps CSV.\n", kind, used,
                  used * FRAME_US * 1e-6f, why);
    if (kind == 'c') Serial.println("Cart may be off centre: centre it, hang DD still, then z before the next c.");
}
static void beginBraking(const char *why) {
    reason = why;
    rampLimit(8.f, true);
    writeRegister(REG_VMAX, 0);
    vCmd = 0;
    run = Run::braking;
    phaseStart = micros();
}

static void zero() {
    referenced = false;
    const auto id = readRegister(REG_IOIN);
    if (!safeChip(id, readRegister(REG_DRV_STATUS)) || !(id & IOIN_DRV_ENN) || (readRegister(REG_VACTUAL) & 0xffffff)) {
        Serial.println("BLOCK z: need disabled driver and stationary ramp.");
        return;
    }
    writeRegister(REG_GSTAT, 1);
    cartZero = readRegister(REG_XACTUAL);
    const auto frame = readEncoderFrame();
    for (int i = 0; i < 2; i++) { offsets[i] = frame.pos[i]; badSeen[i] = frame.bad[i]; }
    referenced = true;
    Serial.println("Saved DD angles and cart origin (0 mm).");
}

static void begin(char k, float a) {
    if (!referenced) { Serial.println("BLOCK: hang DD still with the cart centred, then z."); return; }
    if (!bufferOk) { Serial.println("BLOCK: no sample buffer (malloc failed at boot)."); return; }
    if (k == 'c') {
        float base, tip;
        absoluteAnglesFrom(readEncoderFrame(), offsets, base, tip);
        if (hangingError(base) > START_TILT_RAD || hangingError(tip) > START_TILT_RAD) {
            Serial.printf("BLOCK c: links not hanging still (base %+.1f, tip %+.1f deg from DD).\n",
                          degrees(remainderf(base - PI, 2 * PI)), degrees(remainderf(tip - PI, 2 * PI)));
            return;
        }
        if (!prepareRevl(false)) { motorOff(); Serial.println("BLOCK c: driver preflight failed."); return; }
        const float x = cartX();
        if (fabsf(x) > START_X_M) {
            Serial.printf("BLOCK c: cart %+.1f mm from z (limit %.0f mm). Centre it, hang DD, then z.\n",
                          x * 1e3f, START_X_M * 1e3f);
            return;
        }
        rampLimit(ACCEL_LIMIT, true);
        writeRegister(REG_RAMPMODE, 1);
        writeRegister(REG_VMAX, 0);
        digitalWrite(PIN_EN, LOW);
        run = Run::chirp;
        Serial.printf("CHIRP %.1f m/s^2, %.1f->%.1f Hz over %.0f s, then %.0f s hold. x stops.\n", a, F0, F1, CHIRP_S, TAIL_S);
    } else {
        motorOff();
        run = Run::freeDecay;
        Serial.printf("FREE DECAY %.0f s, motor OFF. Do not touch the links.\n", FREE_S);
    }
    kind = k;
    amp = a;
    used = 0;
    vCmd = 0;
    const auto frame = readEncoderFrame();
    for (int i = 0; i < 2; i++) badSeen[i] = frame.bad[i];
    tick = start = phaseStart = micros();
}

static void dump() {
    if (!used) { Serial.println("Nothing recorded."); return; }
    Serial.printf("SYSID_CSV_BEGIN kind=%c amp=%.2f frame_us=%lu base_cpr=%d rel_cpr=%d base_channel=%d reason=%s\n",
                  kind, amp, (unsigned long)FRAME_US, BASE_CPR, REL_CPR, BASE_CHANNEL, reason);
    Serial.println("t_us,x_m,v_m_s,vcmd_m_s,base_rad,tip_rad,flags");
    for (unsigned i = 0; i < used; i++) {
        const auto &s = at(i);
        Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%u\n", (unsigned long)s.us, s.x, s.v, s.vcmd, s.base, s.tip,
                      unsigned(s.flags));
        if ((i & 63) == 63) delay(1); // let the USB bridge keep up
    }
    Serial.println("SYSID_CSV_END");
}

// Cart reference: log chirp in acceleration, position-capped, with fades.
static void reference(float t, float &xRef, float &vRef) {
    const float ratio = F1 / F0, f = F0 * powf(ratio, t / CHIRP_S);
    const float phase = 2 * PI * F0 * CHIRP_S / logf(ratio) * (powf(ratio, t / CHIRP_S) - 1);
    const float fade = std::max(0.f, std::min(1.f, std::min(t, CHIRP_S - t) / FADE_S));
    const float w = 2 * PI * f, X = fade * std::min(X_MAX, amp / (w * w));
    xRef = X * sinf(phase);
    vRef = X * w * cosf(phase);
}

static void step() {
    const uint32_t now = micros(), us = now - tick;
    if (run == Run::idle || us < FRAME_US) return;
    tick = now;
    const bool driving = run == Run::chirp || run == Run::braking || run == Run::tail;
    if (driving && us > LATE_US) { finish("LOOP_DEADLINE"); return; }
    const float t = (now - start) * 1e-6f;
    const float x = cartX();
    int32_t vr;
    const float v = rampVelocity(vr);
    const auto frame = readEncoderFrame();
    float base, tip;
    absoluteAnglesFrom(frame, offsets, base, tip);
    uint8_t flags = 0;
    for (int i = 0; i < 2; i++) {
        if (frame.bad[i] != badSeen[i]) flags |= i ? BAD2 : BAD1;
        badSeen[i] = frame.bad[i];
    }
    if (run == Run::chirp) flags |= DRIVING;
    if (run == Run::braking) flags |= BRAKING;
    if (driving) flags |= MOTOR_ON;
    if (used >= CAPACITY) { finish(driving ? "BUFFER_FULL_MOTOR_STOPPED" : "BUFFER_FULL"); return; }
    at(used++) = {now - start, x, v, vCmd, base, tip, flags};

    if (run == Run::freeDecay) {
        if (t >= FREE_S) finish("COMPLETE");
        return;
    }
    if (driving) {
        const auto id = readRegister(REG_IOIN), drv = readRegister(REG_DRV_STATUS);
        if (!safeChip(id, drv) || (id & IOIN_DRV_ENN) || (readRegister(REG_GSTAT) & 7)) { finish("DRIVER_FAULT"); return; }
        if (fabsf(x) > TRAVEL_LIMIT_M) { finish("RAMP_POSITION_LIMIT"); return; }
    }
    if (run == Run::chirp) {
        if (hangingError(base) > ANGLE_ABORT_RAD || hangingError(tip) > ANGLE_ABORT_RAD) { beginBraking("ANGLE_LIMIT"); return; }
        if (RailGuard::needsBrake(x, v) || RailGuard::needsBrake(x, vCmd)) { beginBraking("RAIL_GUARD"); return; }
        if (t >= CHIRP_S) { beginBraking("COMPLETE"); return; }
        float xRef, vRef;
        reference(t, xRef, vRef);
        vCmd = constrain(vRef + KP_X * (xRef - x), -SPEED_LIMIT, SPEED_LIMIT);
        commandVelocity(vCmd);
    } else if (run == Run::braking) {
        if (vr == 0) { run = Run::tail; phaseStart = now; }
        else if (now - phaseStart > 500000u) finish("BRAKE_TIMEOUT");
    } else if (run == Run::tail) {
        // Motor holds the cart still; the links ring down with no input.
        if (now - phaseStart >= uint32_t(TAIL_S * 1e6f)) finish(reason);
    }
}

static void command(char c, char arg) {
    if (c == 'z' && !arg) zero();
    else if (c == 'f' && !arg) begin('f', 0);
    else if (c == 'c' && (!arg || (arg >= '1' && arg <= '9'))) begin('c', .4f * ((arg ? arg : '3') - '0'));
    else if (c == 'd' && !arg) dump();
    else if (c == 'h' && !arg) {
        const auto f = readEncoderFrame();
        Serial.printf("ENC raw=%ld,%ld invalid HEDS1=%u HEDS2=%u\n", (long)f.pos[0], (long)f.pos[1], f.bad[0], f.bad[1]);
    } else Serial.println("Commands: z, f, c / c1-c9, d, h, x");
}
} // namespace sysid

static void sysidSetup() {
    sysid::bufferOk = true;
    for (auto &c : sysid::chunk) {
        c = static_cast<sysid::Sample *>(malloc(sizeof(sysid::Sample) * sysid::CHUNK));
        sysid::bufferOk &= c != nullptr;
    }
    setupEncoders();
    Serial.printf("SYSID recorder: %u samples x %u B (%s). Free heap %u B, largest block %u B. Motor OFF.\n",
                  sysid::CAPACITY, unsigned(sizeof(sysid::Sample)), sysid::bufferOk ? "ok" : "MALLOC FAILED",
                  unsigned(ESP.getFreeHeap()), unsigned(ESP.getMaxAllocHeap()));
    Serial.println("z = save DD hanging + cart origin | f = free decay (motor OFF) | c, c1..c9 = chirp | d = dump | x = stop");
}

static void sysidLoop() {
    static char cmd[2];
    static unsigned len = 0;
    static bool discard = false;
    if (Serial.available()) {
        const char ch = Serial.read();
        if (ch == 'x') {
            len = 0; discard = true;
            if (sysid::run != sysid::Run::idle) sysid::finish("MANUAL_STOP");
            else sysid::motorOff();
            return;
        }
        if (ch == '\r' || ch == '\n') {
            if (sysid::run == sysid::Run::idle && !discard && len) sysid::command(cmd[0], len == 2 ? cmd[1] : 0);
            len = 0; discard = false;
        } else if (sysid::run != sysid::Run::idle) discard = true;
        else if (!discard) {
            if (len < 2) cmd[len++] = ch;
            else discard = true;
        }
    }
    sysid::step();
}
