// =============================================================================
// robot_all_in_one_case2.cpp   (Teensy 4.x, main controller)
//
// CASE 2: a dedicated DSP Teensy owns the microphones and does the heavy DSP.
// The main Teensy sends commands over a serial link and polls for results, so
// every update() is short (long-in-wait) and subsystems can run together.
//
// Layers:   Sequencer  ->  Superstructure  ->  Subsystems (Speaker, Mic proxy, Chassis)
//
// What changed from the single-Teensy version (robot_all_in_one.cpp):
//
//   Subsystems
//     * Each declares Traits {blocksCpu, needsRealtime, abortable}.
//     * Mic became MicProxy: talks to the DSP board; update() takes microseconds.
//     * Chassis keeps a short yaw history so a measurement can be matched to the
//       heading at the moment it was taken (needed when listening while turning).
//
//   Superstructure
//     * No global state. EACH SUBSYSTEM has its own state: IDLE / ACTIVE / SETTLING.
//     * requestGroup() starts several steps atomically (all or none) and says WHY
//       it refused: BUSY / SETTLING (ask again) or CONFLICT / INVALID (table error).
//     * What may overlap comes from a conflict rule: an acoustic table plus a rule
//       derived from the traits. The old "one at a time" is the special case where
//       everything conflicts.
//     * Settle timers are per subsystem and only start on subsystems that conflict
//       with the one that just released.
//     * Results are per subsystem (pollResult), plus abort() for one branch.
//
//   Sequencer
//     * A step is a group of size 1. Consecutive rows joined with PAR_WITH_NEXT form
//       a parallel group with an ALL / ANY / DEADLINE join policy.
//
// Not compiled. Hardware and DSP pieces are stubs marked TODO.
// Arduino IDE: rename to .ino. PlatformIO: keep .cpp.
//
// TODO list (search for "TODO"):
//   1. Gyro driver            -> readGyroZ_dps()
//   2. Motor pins + driver    -> cfg::L_PWM etc., Chassis::setMotors()
//   3. Speaker output         -> spkStartChirp / spkStartAfsk / spkStop
//   4. DSP board firmware     -> must speak the frame protocol in namespace dsp
//   5. Command table          -> cmdToStep()
//   6. Calibrate              -> turn gains, drive speed, settle times, mic spacing
//   7. Decide                 -> MOTOR_NOISE_MASKS_CHIRP (can the DSP hear through motor noise?)
// =============================================================================
#include <Arduino.h>
#include <math.h>
#include <string.h>

#if __has_include(<Watchdog_t4.h>)
  #include <Watchdog_t4.h>
  #define HAVE_WDT 1
  static WDT_T4<WDT1> g_wdt;
#else
  #define HAVE_WDT 0
#endif

static inline void feedWatchdog() {
#if HAVE_WDT
  g_wdt.feed();
#endif
}

// ----------------------------- build options ---------------------------------
#define ROBOT_A 1
#define ROBOT_B 2
#define ROBOT_ROLE ROBOT_A          // change to ROBOT_B for the other robot

#ifndef A_USE_SWEEP
#define A_USE_SWEEP 1               // 1: Robot A listens WHILE turning (parallel group). 0: flat table
#endif
#ifndef MOTOR_NOISE_MASKS_CHIRP
#define MOTOR_NOISE_MASKS_CHIRP 0   // 1: Mic and Chassis may NOT run together
#endif
#define LOG_STATS  1                // print per-primitive timing when each one finishes
#define LOG_STATES 0                // print every subsystem's state once per second

// ============================ configuration ==================================
namespace cfg {
  // motor driver pins (PWM + DIR style). TODO: match your wiring.
  // Put pull-downs on the PWM pins so motors stay off if the watchdog resets the chip.
  constexpr uint8_t L_PWM = 2, L_DIR = 3, R_PWM = 4, R_DIR = 5;

  // gyro
  constexpr uint32_t GYRO_PERIOD_US = 2000;     // read gyro at 500 Hz
  constexpr uint32_t GYRO_MAX_DT_US = 20000;    // longer gap: don't integrate

  // gyro-closed-loop turn
  constexpr float   TURN_KP          = 3.0f;    // PWM counts per degree of error
  constexpr int     TURN_PWM_MAX     = 200;
  constexpr int     TURN_PWM_MIN     = 70;      // overcome stiction
  constexpr float   TURN_TOL_DEG     = 2.0f;
  constexpr float   TURN_RATE_OK_DPS = 5.0f;    // must be nearly still to finish
  constexpr uint8_t TURN_HOLD_TICKS  = 20;      // consecutive loop passes in tolerance and still

  // timer fallback if gyro is dead (calibrate on your floor + battery)
  constexpr int     OPENLOOP_TURN_PWM = 140;
  constexpr float   OPENLOOP_TURN_DPS = 120.0f;

  // drive (timer-based with gyro heading hold)
  constexpr int     DRIVE_PWM      = 150;
  constexpr float   DRIVE_FT_PER_S = 1.0f;      // calibrate
  constexpr float   HOLD_KP        = 2.0f;

  // settle (quiet) time after a subsystem releases; applies only to subsystems that conflict with it
  constexpr uint32_t SETTLE_CHASSIS_MS   = 300; // motor noise + vibration
  constexpr uint32_t SETTLE_SPEAKER_MS   = 150; // speaker ring-down
  constexpr uint32_t SEND_CHIRP_GRACE_MS = 200; // deadline margin for SEND_CHIRP

  // acoustics
  constexpr float SPEED_OF_SOUND_MPS = 343.0f;
  constexpr float MIC_SPACING_M      = 0.10f;   // TODO: your mic baseline
  constexpr float BEARING_AGREE_DEG  = 30.0f;

  // DSP link
  constexpr uint32_t DSP_BAUD                 = 921600;
  constexpr uint32_t DSP_HEARTBEAT_TIMEOUT_MS = 500;

  // timing budget: every update() should be short now
  constexpr uint32_t UPDATE_BUDGET_US = 1000;

  // misc
  constexpr uint8_t  MAX_RETRIES   = 3;
  constexpr uint32_t WDT_TIMEOUT_S = 5;
}

// ================================ types ======================================
enum class Prim : uint8_t {
  WAIT, LISTEN_CHIRP, COMPUTE_TDOA, COMPUTE_BEARING, TURN, DRIVE,
  SEND_CHIRP, SEND_AFSK, LISTEN_AFSK, EXEC_RECEIVED_CMD, COUNT
};
static const char* PRIM_NAME[] = {
  "WAIT", "LISTEN_CHIRP", "COMPUTE_TDOA", "COMPUTE_BEARING", "TURN", "DRIVE",
  "SEND_CHIRP", "SEND_AFSK", "LISTEN_AFSK", "EXEC_RECEIVED_CMD"
};

enum class Result : uint8_t { RUNNING, SUCCESS, TIMEOUT, FAIL, ABORTED };
static const char* RESULT_NAME[] = { "RUNNING", "SUCCESS", "TIMEOUT", "FAIL", "ABORTED" };

enum class OnFail : uint8_t { ABORT, RETRY, SKIP, GOTO };
enum class Join   : uint8_t { ALL, ANY, DEADLINE };     // how a parallel group finishes
enum : uint8_t { PAR_WITH_NEXT = 1, IS_DEADLINE = 2 };  // Step::flags

// Subsystems as bit values so a set of them is a mask. SUB_INT = superstructure-internal
// work (WAIT, COMPUTE_BEARING): a pseudo-subsystem so every primitive is handled the same way.
enum Sub : uint8_t { SUB_NONE = 0, SUB_SPK = 1, SUB_MIC = 2, SUB_CHS = 4, SUB_INT = 8 };
static const char* SUB_NAME[] = { "SPK", "MIC", "CHS", "INT" };
static inline int slotOf(Sub s) { return s == SUB_SPK ? 0 : s == SUB_MIC ? 1 : s == SUB_CHS ? 2 : 3; }
static inline Sub subOfSlot(int i) { return (Sub)(1 << i); }

// AFSK command ids (shared by both robots)
enum Cmd : uint8_t {
  CMD_NONE = 0, CMD_STOP_CHIRP, CMD_LED_OFF, CMD_SEND_ACK, CMD_ACK,
  CMD_TURN_LEFT, CMD_TURN_RIGHT, CMD_TURN_180, CMD_FORWARD
};

struct Step {
  Prim     prim;
  float    param;        // TURN: degrees (NAN = use ctx.bearing) | DRIVE: feet |
                         // SEND_AFSK: command id | COMPUTE_TDOA: 1 or 2 (which measurement)
  uint32_t timeout_ms;   // limit for most prims; DURATION for WAIT and SEND_CHIRP
  OnFail   onFail;       // policy when the step/group fails
  uint8_t  gotoIdx;      // used when onFail == GOTO
  uint8_t  flags = 0;    // PAR_WITH_NEXT: runs in parallel with the next row; IS_DEADLINE
  Join     join  = Join::ALL;   // read from the LAST row of a group
};

struct Context {         // "blackboard": data passed between steps. Parallel branches must
  float   theta1, theta2;      // write DIFFERENT fields.
  float   heading1, heading2;  // yaw (deg) when theta1/theta2 were measured
  float   bearing;
  uint8_t rxCmd;
  uint8_t retries;
};

static Sub subFor(Prim p) {
  switch (p) {
    case Prim::SEND_CHIRP: case Prim::SEND_AFSK:                               return SUB_SPK;
    case Prim::LISTEN_CHIRP: case Prim::COMPUTE_TDOA: case Prim::LISTEN_AFSK: return SUB_MIC;
    case Prim::TURN: case Prim::DRIVE:                                         return SUB_CHS;
    default:                                                                   return SUB_INT;
  }
}

static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

static uint32_t settleMsAfter(Sub s) {
  return s == SUB_CHS ? cfg::SETTLE_CHASSIS_MS : s == SUB_SPK ? cfg::SETTLE_SPEAKER_MS : 0;
}
static uint32_t graceMs(const Step& s) { return s.prim == Prim::SEND_CHIRP ? cfg::SEND_CHIRP_GRACE_MS : 0; }

// ============================ subsystem interface ============================
struct Traits {
  bool blocksCpu;      // update() can run much longer than a loop pass (long-in-time on THIS CPU)
  bool needsRealtime;  // must be serviced at a steady rate (e.g. gyro control loop)
  bool abortable;      // abort() takes effect immediately
};

class Subsystem {
public:
  virtual ~Subsystem() {}
  virtual Traits traits() const = 0;
  virtual void   start(const Step& s, Context* ctx) = 0;   // accept a goal, return immediately
  virtual Result update() = 0;                             // RUNNING until finished
  virtual void   abort() = 0;                              // stop what you are doing NOW
  virtual bool   isIdle() const = 0;
  virtual void   forceIdle() = 0;                          // hardware safe + state = IDLE
};

// ============================================================================
// SPEAKER subsystem   (SEND_CHIRP, SEND_AFSK)
// Output plays in the background (Audio library / DMA); update() checks the clock.
// ============================================================================
static void     spkBegin()                 { /* TODO: init audio output */ }
static void     spkStartChirp(uint32_t ms) { /* TODO: start chirp of ms duration */ }
static uint32_t spkStartAfsk(uint8_t cmd)  { /* TODO: modulate + start frame */ return 500; /* est. ms */ }
static void     spkStop()                  { /* TODO: silence the output */ }

class Speaker : public Subsystem {
public:
  void begin() { spkBegin(); }
  Traits traits() const override { return { /*blocksCpu*/false, /*needsRealtime*/false, /*abortable*/true }; }

  void start(const Step& s, Context*) override {
    uint32_t now = millis();
    if (s.prim == Prim::SEND_CHIRP) { spkStartChirp(s.timeout_ms); endMs_ = now + s.timeout_ms; }
    else                            { endMs_ = now + spkStartAfsk((uint8_t)s.param); }
    busy_ = true;
  }
  Result update() override {
    if (!busy_) return Result::SUCCESS;
    if ((int32_t)(millis() - endMs_) >= 0) { spkStop(); return Result::SUCCESS; }
    return Result::RUNNING;
  }
  void abort() override        { forceIdle(); }
  bool isIdle() const override { return !busy_; }
  void forceIdle() override    { spkStop(); busy_ = false; }

private:
  bool busy_ = false; uint32_t endMs_ = 0;
};

// ============================================================================
// DSP LINK   (main Teensy <-> DSP Teensy over a UART)
//
// Frame:  0xA5 | type | len | payload[len] | crc8   (crc8 poly 0x07 over type,len,payload)
//
//   main -> DSP   0x01 LISTEN_CHIRP  {u32 timeout_ms}
//                 0x02 LISTEN_AFSK   {u32 timeout_ms}
//                 0x03 ABORT         {}
//   DSP -> main   0x81 CHIRP_RESULT  {float tdoa_sec, u32 arrival_dsp_ms}
//                 0x82 AFSK_RESULT   {u8 cmd}
//                 0x83 TIMEOUT       {}
//                 0x84 HEARTBEAT     {u32 dsp_ms}      (send every ~100 ms)
//
// Clock mapping: the DSP stamps arrivals with ITS clock. Heartbeats let us estimate
// the offset to our millis(). TODO: refine (e.g. keep the minimum offset over a window).
// ============================================================================
#define DSP_LINK Serial1

namespace dsp {
  constexpr uint8_t SOF = 0xA5;
  enum : uint8_t { TX_LISTEN_CHIRP = 0x01, TX_LISTEN_AFSK = 0x02, TX_ABORT = 0x03,
                   RX_CHIRP = 0x81, RX_AFSK = 0x82, RX_TIMEOUT = 0x83, RX_HEARTBEAT = 0x84 };

  struct Msg { uint8_t type; float tdoaSec; uint32_t arrivalMs; uint8_t cmd; };   // arrivalMs is LOCAL time

  struct Parser { uint8_t st = 0, type = 0, len = 0, n = 0, crc = 0, buf[16]; };

  static Parser   g_parser;
  static Msg      g_mail;                 // single-slot mailbox: newest result wins
  static bool     g_mailValid = false;
  static uint32_t g_lastHbMs = 0;
  static int32_t  g_clockOffset = 0;      // local_ms = dsp_ms + offset

  static uint8_t crc8(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
  }

  static void begin() { DSP_LINK.begin(cfg::DSP_BAUD); g_lastHbMs = millis(); }   // 500 ms grace for boot

  static void send(uint8_t type, const void* payload, uint8_t len) {
    uint8_t crc = crc8(crc8(0, type), len);
    DSP_LINK.write(SOF); DSP_LINK.write(type); DSP_LINK.write(len);
    const uint8_t* p = (const uint8_t*)payload;
    for (uint8_t i = 0; i < len; i++) { DSP_LINK.write(p[i]); crc = crc8(crc, p[i]); }
    DSP_LINK.write(crc);
  }

  // Returns true when a complete CRC-valid frame is in p.
  static bool feed(Parser& p, uint8_t b) {
    switch (p.st) {
      case 0: if (b == SOF) { p.st = 1; p.crc = 0; } break;
      case 1: p.type = b; p.crc = crc8(p.crc, b); p.st = 2; break;
      case 2: if (b > sizeof(p.buf)) { p.st = 0; break; }
              p.len = b; p.n = 0; p.crc = crc8(p.crc, b); p.st = (b == 0) ? 4 : 3; break;
      case 3: p.buf[p.n++] = b; p.crc = crc8(p.crc, b); if (p.n == p.len) p.st = 4; break;
      case 4: p.st = 0; return b == p.crc;
    }
    return false;
  }

  static void handle(const Parser& p) {
    uint32_t now = millis();
    switch (p.type) {
      case RX_HEARTBEAT:
        if (p.len == 4) { uint32_t t; memcpy(&t, p.buf, 4); g_clockOffset = (int32_t)(now - t); g_lastHbMs = now; }
        break;
      case RX_CHIRP:
        if (p.len == 8) {
          float tdoa; uint32_t t; memcpy(&tdoa, p.buf, 4); memcpy(&t, p.buf + 4, 4);
          g_mail = { RX_CHIRP, tdoa, (uint32_t)(t + (uint32_t)g_clockOffset), 0 }; g_mailValid = true;
        }
        break;
      case RX_AFSK:
        if (p.len == 1) { g_mail = { RX_AFSK, 0, now, p.buf[0] }; g_mailValid = true; }
        break;
      case RX_TIMEOUT:
        g_mail = { RX_TIMEOUT, 0, now, 0 }; g_mailValid = true;
        break;
      default: break;
    }
  }

  // Called every loop pass: drains the UART, tracks heartbeats, fills the mailbox. Cheap.
  static void pump() {
    while (DSP_LINK.available()) if (feed(g_parser, (uint8_t)DSP_LINK.read())) handle(g_parser);
  }
  static bool take(Msg& m)  { if (!g_mailValid) return false; m = g_mail; g_mailValid = false; return true; }
  static void dropMail()    { g_mailValid = false; }                 // forget stale results
  static bool alive()       { return (millis() - g_lastHbMs) < cfg::DSP_HEARTBEAT_TIMEOUT_MS; }
}

// ============================================================================
// MIC subsystem (proxy for the DSP board)   (LISTEN_CHIRP, COMPUTE_TDOA, LISTEN_AFSK)
// update() only checks the mailbox: microseconds, no matter how long the DSP works.
// ============================================================================
static float tdoaToAngleDeg(float dtSec) {
  float s = cfg::SPEED_OF_SOUND_MPS * dtSec / cfg::MIC_SPACING_M;
  if (s >  1.0f) s =  1.0f;
  if (s < -1.0f) s = -1.0f;
  return asinf(s) * 180.0f / (float)M_PI;       // TODO: confirm sign vs gyro (CCW +)
}

class MicProxy : public Subsystem {
public:
  void begin() { dsp::begin(); }
  Traits traits() const override { return { /*blocksCpu*/false, false, /*abortable*/true }; }
  uint32_t lastArrivalMs() const { return last_.arrivalMs; }     // when the chirp reached the mics (local clock)

  void start(const Step& s, Context* ctx) override {
    ctx_ = ctx; param_ = s.param; prim_ = s.prim; busy_ = true;
    uint32_t t = s.timeout_ms;
    switch (prim_) {
      case Prim::LISTEN_CHIRP: dsp::dropMail(); dsp::send(dsp::TX_LISTEN_CHIRP, &t, 4); break;
      case Prim::LISTEN_AFSK:  dsp::dropMail(); dsp::send(dsp::TX_LISTEN_AFSK,  &t, 4); break;
      default: break;                              // COMPUTE_TDOA: local math on the stored result
    }
  }

  Result update() override {
    if (!busy_) return Result::FAIL;

    if (prim_ == Prim::COMPUTE_TDOA) {             // DSP already sent the TDOA with the chirp result
      if (!haveChirp_) return Result::FAIL;
      float deg = tdoaToAngleDeg(last_.tdoaSec);
      if ((int)param_ == 1) ctx_->theta1 = deg; else ctx_->theta2 = deg;
      return Result::SUCCESS;
    }

    if (!dsp::alive()) return Result::FAIL;        // DSP board died or went silent
    dsp::Msg m;
    if (dsp::take(m)) {
      if (prim_ == Prim::LISTEN_CHIRP && m.type == dsp::RX_CHIRP) { last_ = m; haveChirp_ = true; return Result::SUCCESS; }
      if (prim_ == Prim::LISTEN_AFSK  && m.type == dsp::RX_AFSK)  { ctx_->rxCmd = m.cmd; return Result::SUCCESS; }
      if (m.type == dsp::RX_TIMEOUT) return Result::TIMEOUT;
    }
    return Result::RUNNING;
  }

  void abort() override {
    if (busy_ && (prim_ == Prim::LISTEN_CHIRP || prim_ == Prim::LISTEN_AFSK)) dsp::send(dsp::TX_ABORT, nullptr, 0);
    busy_ = false;
  }
  bool isIdle() const override { return !busy_; }
  void forceIdle() override    { busy_ = false; }

private:
  Context* ctx_ = nullptr; Prim prim_ = Prim::LISTEN_CHIRP; float param_ = 0;
  dsp::Msg last_{}; bool haveChirp_ = false, busy_ = false;
};

// ============================================================================
// CHASSIS subsystem   (TURN, DRIVE)
// Yaw convention: degrees, CCW (left) positive, wrapped to (-180, 180].
// Needs a steady service rate (needsRealtime). Keeps a yaw history.
// ============================================================================
// TODO: replace with your gyro driver. Return true + deg/s (CCW positive) on success.
static bool readGyroZ_dps(float& dps) { return false; }

class Chassis : public Subsystem {
public:
  Traits traits() const override { return { false, /*needsRealtime*/true, true }; }

  void begin() {
    pinMode(cfg::L_PWM, OUTPUT); pinMode(cfg::L_DIR, OUTPUT);
    pinMode(cfg::R_PWM, OUTPUT); pinMode(cfg::R_DIR, OUTPUT);
    analogWriteFrequency(cfg::L_PWM, 20000);
    analogWriteFrequency(cfg::R_PWM, 20000);
    setMotors(0, 0);

    float sum = 0, v; int n = 0;                   // gyro bias: robot must be still ~0.5 s at boot
    for (int i = 0; i < 500; i++) { if (readGyroZ_dps(v)) { sum += v; n++; } delay(1); }
    gyroOk_ = (n > 400);
    bias_   = gyroOk_ ? sum / n : 0;
    yaw_    = 0;
    lastUs_ = micros();
  }

  // Called every loop pass (rate-limited inside). A sensor, not an "active" subsystem.
  void tickHeading() {
    uint32_t nowUs = micros();
    uint32_t dtUs  = nowUs - lastUs_;
    if (dtUs < cfg::GYRO_PERIOD_US) return;
    lastUs_ = nowUs;
    if (dtUs > cfg::GYRO_MAX_DT_US) return;        // long gap: skip rather than integrate

    float raw;
    if (!readGyroZ_dps(raw)) { if (++badReads_ > 50) gyroOk_ = false; return; }
    badReads_ = 0; gyroOk_ = true;

    if (mode_ == STOPPED && fabsf(raw - bias_) < 1.0f) bias_ += 0.001f * (raw - bias_);
    rate_ = raw - bias_;
    yaw_  = wrap180(yaw_ + rate_ * (dtUs * 1e-6f));

    histYaw_[histHead_] = yaw_; histMs_[histHead_] = millis();     // yaw history
    histHead_ = (histHead_ + 1) % kHist; if (histCount_ < kHist) histCount_++;
  }

  float yawDeg() const { return yaw_; }
  float yawAt(uint32_t ms) const {                 // yaw at (or just before) time ms
    for (uint16_t k = 0; k < histCount_; k++) {
      uint16_t i = (histHead_ + kHist - 1 - k) % kHist;            // newest first
      if ((int32_t)(histMs_[i] - ms) <= 0) return histYaw_[i];
    }
    return yaw_;                                   // older than the history: fall back to now
  }

  void start(const Step& s, Context*) override {
    uint32_t now = millis();
    holdTicks_ = 0;
    if (s.prim == Prim::TURN) {
      float deg = wrap180(s.param);                // + = left, - = right
      useGyro_ = gyroOk_;
      if (useGyro_) {
        targetYaw_ = wrap180(yaw_ + deg);
      } else {                                     // timer fallback
        pendingDeg_  = deg;
        openLoopDir_ = (deg >= 0) ? 1 : -1;
        endMs_ = now + (uint32_t)(fabsf(deg) / cfg::OPENLOOP_TURN_DPS * 1000.0f);
      }
      mode_ = TURNING;
    } else {                                       // DRIVE, param = feet
      holdYaw_ = yaw_;
      endMs_   = now + (uint32_t)(s.param / cfg::DRIVE_FT_PER_S * 1000.0f);
      mode_    = DRIVING;
    }
  }

  Result update() override {
    uint32_t now = millis();
    switch (mode_) {
      case TURNING: return updateTurn(now);
      case DRIVING: return updateDrive(now);
      default:      return Result::SUCCESS;
    }
  }

  void abort() override        { forceIdle(); }
  bool isIdle() const override { return mode_ == STOPPED; }
  void forceIdle() override    { setMotors(0, 0); mode_ = STOPPED; }

private:
  enum Mode : uint8_t { STOPPED, TURNING, DRIVING };

  Result updateTurn(uint32_t now) {
    if (!useGyro_) {                               // open-loop timed turn, dead-reckon heading
      if ((int32_t)(now - endMs_) >= 0) {
        setMotors(0, 0); yaw_ = wrap180(yaw_ + pendingDeg_); mode_ = STOPPED;
        return Result::SUCCESS;
      }
      int p = cfg::OPENLOOP_TURN_PWM * openLoopDir_;
      setMotors(-p, +p);
      return Result::RUNNING;
    }

    float err   = wrap180(targetYaw_ - yaw_);      // + = still need to turn left
    bool  inTol = fabsf(err) < cfg::TURN_TOL_DEG;
    bool  still = fabsf(rate_) < cfg::TURN_RATE_OK_DPS;

    if (inTol) {
      setMotors(0, 0);
      if (still && ++holdTicks_ >= cfg::TURN_HOLD_TICKS) { mode_ = STOPPED; return Result::SUCCESS; }
      if (!still) holdTicks_ = 0;
      return Result::RUNNING;
    }

    holdTicks_ = 0;
    float cmd = cfg::TURN_KP * err;                // P control, slows near the target
    if (cmd >  cfg::TURN_PWM_MAX) cmd =  cfg::TURN_PWM_MAX;
    if (cmd < -cfg::TURN_PWM_MAX) cmd = -cfg::TURN_PWM_MAX;
    if (fabsf(cmd) < cfg::TURN_PWM_MIN) cmd = copysignf(cfg::TURN_PWM_MIN, err);
    setMotors(-(int)cmd, +(int)cmd);               // err>0: left wheel back, right forward
    return Result::RUNNING;                        // overshoot fixes itself: err flips sign
  }

  Result updateDrive(uint32_t now) {
    if ((int32_t)(now - endMs_) >= 0) { setMotors(0, 0); mode_ = STOPPED; return Result::SUCCESS; }
    float corr = gyroOk_ ? cfg::HOLD_KP * wrap180(holdYaw_ - yaw_) : 0.0f;   // + = veer left
    setMotors((int)(cfg::DRIVE_PWM - corr), (int)(cfg::DRIVE_PWM + corr));
    return Result::RUNNING;
  }

  void setMotors(int left, int right) {            // -255..255. TODO: match your driver
    auto drive = [](uint8_t pwmPin, uint8_t dirPin, int v) {
      if (v >  255) v =  255;
      if (v < -255) v = -255;
      digitalWriteFast(dirPin, v >= 0 ? HIGH : LOW);
      analogWrite(pwmPin, abs(v));
    };
    drive(cfg::L_PWM, cfg::L_DIR, left);
    drive(cfg::R_PWM, cfg::R_DIR, right);
  }

  static constexpr uint16_t kHist = 512;           // ~1 s of history at 500 Hz
  Mode     mode_ = STOPPED;
  float    yaw_ = 0, bias_ = 0, rate_ = 0, targetYaw_ = 0, pendingDeg_ = 0, holdYaw_ = 0;
  uint32_t lastUs_ = 0, endMs_ = 0;
  uint8_t  badReads_ = 0, holdTicks_ = 0;
  int8_t   openLoopDir_ = 1;
  bool     gyroOk_ = false, useGyro_ = false;
  float    histYaw_[kHist]; uint32_t histMs_[kHist]; uint16_t histHead_ = 0, histCount_ = 0;
};

// ============================================================================
// SUPERSTRUCTURE
//
// There is NO global state. Each subsystem has its own state:
//
//   IDLE       free to accept a request
//   ACTIVE     running a step
//   SETTLING   refuses requests until its timer expires (started when a CONFLICTING
//              subsystem releases; it never starts on the subsystem that just released)
//
//   IDLE --requestGroup accepted--> ACTIVE --finish--> IDLE
//   IDLE --conflicting subsystem finished--> SETTLING --timer expires--> IDLE
//
// "The robot's state" is the combination, e.g. MIC=ACTIVE CHS=ACTIVE.
// ============================================================================
enum class SlotState : uint8_t { IDLE, ACTIVE, SETTLING };
static const char* SLOT_STATE_NAME[] = { "IDLE", "ACTIVE", "SETTLING" };

enum class Refusal : uint8_t { NONE, BUSY, SETTLING, CONFLICT, INVALID };

// Acoustic conflicts (data): which subsystems cannot run together. MUST stay symmetric.
static const uint8_t ACOUSTIC[3] = {
  SUB_MIC,                                                                    // SPK: speaker deafens the mics
  (uint8_t)(SUB_SPK | (MOTOR_NOISE_MASKS_CHIRP ? SUB_CHS : 0)),               // MIC
  (uint8_t)(MOTOR_NOISE_MASKS_CHIRP ? SUB_MIC : 0)                            // CHS
};

// Map a decoded AFSK command to a concrete step. TODO: extend for your command set.
// Must be IDEMPOTENT: it can be called again when a request is refused and retried.
// Commands with no subsystem are done here and returned as WAIT(0).
static bool cmdToStep(uint8_t cmd, Step& out) {
  switch (cmd) {
    case CMD_TURN_LEFT:  out = { Prim::TURN,  90.0f, 3000, OnFail::ABORT, 0 }; return true;
    case CMD_TURN_RIGHT: out = { Prim::TURN, -90.0f, 3000, OnFail::ABORT, 0 }; return true;
    case CMD_TURN_180:   out = { Prim::TURN, 180.0f, 4000, OnFail::ABORT, 0 }; return true;
    case CMD_FORWARD:    out = { Prim::DRIVE,  1.0f, 4000, OnFail::ABORT, 0 }; return true;
    case CMD_SEND_ACK:   out = { Prim::SEND_AFSK, (float)CMD_ACK, 1500, OnFail::ABORT, 0 }; return true;
    case CMD_LED_OFF:    digitalWrite(LED_BUILTIN, LOW);
                         out = { Prim::WAIT, 0, 0, OnFail::ABORT, 0 }; return true;
    case CMD_STOP_CHIRP: out = { Prim::WAIT, 0, 0, OnFail::ABORT, 0 }; return true;  // Robot B: ending its
                                                                                     // sequence stops chirping
    default: return false;
  }
}

struct PrimStats { uint32_t count = 0, maxUpdateUs = 0, maxTotalMs = 0; };

class Superstructure {
public:
  static constexpr uint8_t kMaxGroup = 4;

  void begin() {
    sub_[0] = &speaker_; sub_[1] = &mic_; sub_[2] = &chassis_; sub_[3] = nullptr;   // slot 3 = SUB_INT
    speaker_.begin(); mic_.begin(); chassis_.begin();
  }

  // ---------------------------------------------------------------------------
  // requestGroup(): the ONLY way to start work. ATOMIC: starts all n steps or none.
  // outSubs[k] tells the caller which subsystem runs step k (after resolution).
  //   NONE      accepted
  //   BUSY      a needed subsystem, or one that conflicts with it, is active: ask again
  //   SETTLING  a needed subsystem is still settling: ask again
  //   CONFLICT  these steps can never run together (table error): give up
  //   INVALID   a step could not be resolved (e.g. unknown received command): give up
  // ---------------------------------------------------------------------------
  Refusal requestGroup(const Step* in, uint8_t n, Context* ctx, Sub* outSubs) {
    if (n == 0 || n > kMaxGroup) return Refusal::INVALID;
    Step s[kMaxGroup]; Sub need[kMaxGroup]; uint8_t groupMask = 0;

    for (uint8_t k = 0; k < n; k++) {                          // 1. resolve and validate the group itself
      s[k] = in[k];
      if (!resolveStep(s[k], ctx, chassis_.yawDeg())) return Refusal::INVALID;
      need[k] = subFor(s[k].prim);
      if (groupMask & need[k]) return Refusal::CONFLICT;       // two branches want the same subsystem
      groupMask |= need[k];
    }
    for (uint8_t k = 0; k < n; k++)
      if (conflictsOf(need[k]) & groupMask) return Refusal::CONFLICT;   // can never run together

    uint32_t now = millis();                                   // 2. transient reasons
    uint8_t active = activeMask();
    for (uint8_t k = 0; k < n; k++) {
      Sub x = need[k]; Slot& sl = slots_[slotOf(x)];
      if (active & x)                 return Refusal::BUSY;
      if (conflictsOf(x) & active)    return Refusal::BUSY;
      if (sl.state == SlotState::SETTLING) {
        if ((int32_t)(now - sl.settleUntil) < 0) return Refusal::SETTLING;
        sl.state = SlotState::IDLE;                            // timer already expired
      }
      if (x != SUB_INT && !sub_[slotOf(x)]->isIdle()) return Refusal::BUSY;   // invariant check
    }

    for (uint8_t k = 0; k < n; k++) {                          // 3. start everything, atomically
      Sub x = need[k]; Slot& sl = slots_[slotOf(x)];
      sl.state = SlotState::ACTIVE;
      sl.step = s[k]; sl.ctx = ctx; sl.startMs = now;
      sl.deadlineMs = now + s[k].timeout_ms + graceMs(s[k]);
      sl.hasResult = false; sl.result = Result::RUNNING;
      if (x != SUB_INT) sub_[slotOf(x)]->start(s[k], ctx);
      outSubs[k] = x;
    }
    return Refusal::NONE;
  }

  // ---------------------------------------------------------------------------
  // update(): call every loop pass. Every subsystem's update() is short here.
  // Non-blocking subsystems run first, blocking ones last (none today; this keeps
  // the design valid if you ever swap in a Mic that does its own DSP).
  // ---------------------------------------------------------------------------
  void update() {
    dsp::pump();                                               // drain the DSP link
    chassis_.tickHeading();                                    // heading integration always runs

    uint32_t now = millis();
    for (int i = 0; i < 4; i++)                                // SETTLING -> IDLE when timers expire
      if (slots_[i].state == SlotState::SETTLING && (int32_t)(now - slots_[i].settleUntil) >= 0)
        slots_[i].state = SlotState::IDLE;

    for (int pass = 0; pass < 2; pass++)
      for (int i = 0; i < 4; i++) {
        if (slots_[i].state != SlotState::ACTIVE) continue;
        bool blocking = sub_[i] && sub_[i]->traits().blocksCpu;
        if (blocking == (pass == 1)) service(subOfSlot(i));
      }
  }

  Result pollResult(Sub s) {                                   // one-shot, per subsystem
    Slot& sl = slots_[slotOf(s)];
    if (!sl.hasResult) return Result::RUNNING;
    sl.hasResult = false;
    return sl.result;
  }

  void abort(Sub s) {                                          // cancel ONE running branch
    if (slots_[slotOf(s)].state == SlotState::ACTIVE) finish(s, Result::ABORTED);
    slots_[slotOf(s)].hasResult = false;                       // caller does not want this result
  }

  void emergencyStop() {
    chassis_.forceIdle();                                      // motors off, no matter what
    for (int i = 0; i < 4; i++)
      if (slots_[i].state == SlotState::ACTIVE) finish(subOfSlot(i), Result::ABORTED);
  }

  uint8_t   activeMask() const {
    uint8_t m = 0;
    for (int i = 0; i < 4; i++) if (slots_[i].state == SlotState::ACTIVE) m |= (uint8_t)(1 << i);
    return m;
  }
  SlotState slotState(Sub s) const { return slots_[slotOf(s)].state; }

  void dumpStates() const {
    Serial.printf("[sup] SPK=%s MIC=%s CHS=%s INT=%s  mask=0x%02X\n",
                  SLOT_STATE_NAME[(int)slots_[0].state], SLOT_STATE_NAME[(int)slots_[1].state],
                  SLOT_STATE_NAME[(int)slots_[2].state], SLOT_STATE_NAME[(int)slots_[3].state], activeMask());
  }

private:
  struct Slot {
    SlotState state = SlotState::IDLE;
    Step      step{};
    Context*  ctx = nullptr;
    uint32_t  startMs = 0, deadlineMs = 0, settleUntil = 0;
    Result    result = Result::RUNNING;
    bool      hasResult = false;
  };

  // What can NOT run together with s: acoustic table + a rule derived from the traits.
  uint8_t conflictsOf(Sub s) const {
    if (s == SUB_INT || s == SUB_NONE) return 0;
    int si = slotOf(s);
    uint8_t c = ACOUSTIC[si];
    Traits t = sub_[si]->traits();
    for (int i = 0; i < 3; i++) {
      if (i == si) continue;
      Traits u = sub_[i]->traits();
      if ((t.blocksCpu && u.needsRealtime) || (u.blocksCpu && t.needsRealtime)) c |= (uint8_t)subOfSlot(i);
    }
    return c;
  }

  void service(Sub s) {
    Slot& sl = slots_[slotOf(s)];
    Result r;
    if (s == SUB_INT) {
      r = runInternal(sl.step, sl.startMs, *sl.ctx);
    } else {
      uint32_t t0 = micros();
      r = sub_[slotOf(s)]->update();
      uint32_t us = micros() - t0;
      PrimStats& st = stats_[(int)sl.step.prim];
      if (us > st.maxUpdateUs) {
        st.maxUpdateUs = us;
#if LOG_STATS
        if (us > cfg::UPDATE_BUDGET_US)
          Serial.printf("[sup] WARN %s update() took %luus (budget %luus): is it still long-in-time?\n",
                        PRIM_NAME[(int)sl.step.prim], (unsigned long)us, (unsigned long)cfg::UPDATE_BUDGET_US);
#endif
      }
      if (r == Result::RUNNING && (int32_t)(millis() - sl.deadlineMs) >= 0) r = Result::TIMEOUT;   // backstop
    }

    if (r == Result::SUCCESS && sl.step.prim == Prim::COMPUTE_TDOA) {
      float y = chassis_.yawAt(mic_.lastArrivalMs());          // heading when the chirp ARRIVED
      if ((int)sl.step.param == 1) sl.ctx->heading1 = y; else sl.ctx->heading2 = y;
    }
    if (r != Result::RUNNING) finish(s, r);
  }

  // The single release path: success, timeout, failure, abort and e-stop all end here.
  void finish(Sub s, Result r) {
    Slot& sl = slots_[slotOf(s)];
    uint32_t now = millis();

    PrimStats& st = stats_[(int)sl.step.prim];                 // timing stats
    uint32_t total = now - sl.startMs;
    st.count++;
    if (total > st.maxTotalMs) st.maxTotalMs = total;
#if LOG_STATS
    Serial.printf("[sup] %-3s %-17s %-8s total=%lums  max_total=%lums  max_update=%luus\n",
                  SUB_NAME[slotOf(s)], PRIM_NAME[(int)sl.step.prim], RESULT_NAME[(int)r],
                  (unsigned long)total, (unsigned long)st.maxTotalMs, (unsigned long)st.maxUpdateUs);
#endif

    if (s != SUB_INT) {
      Subsystem* sub = sub_[slotOf(s)];
      if (r != Result::SUCCESS) sub->abort();                  // make hardware safe on any abnormal end
      sub->forceIdle();                                        // never free the slot until truly idle
    }
    sl.state = SlotState::IDLE;                                // the finished subsystem is NOT settling
    sl.result = r; sl.hasResult = true;

    uint32_t ms = settleMsAfter(s);                            // settle ONLY the subsystems that conflict with s
    if (ms) {
      uint8_t cf = conflictsOf(s); uint32_t until = now + ms;
      for (int i = 0; i < 3; i++) {
        if (!(cf & (1 << i))) continue;
        Slot& q = slots_[i];
        if (q.state == SlotState::ACTIVE) continue;
        if (q.state != SlotState::SETTLING || (int32_t)(until - q.settleUntil) > 0) q.settleUntil = until;
        q.state = SlotState::SETTLING;
      }
    }
  }

  // Turn "data" steps into concrete ones at the moment they run.
  bool resolveStep(Step& s, Context* c, float yawNow) {
    if (s.prim == Prim::EXEC_RECEIVED_CMD && !cmdToStep(c->rxCmd, s)) return false;
    if (s.prim == Prim::TURN && isnan(s.param)) s.param = wrap180(c->bearing - yawNow);
    return true;
  }

  // Combine two TDOA angles taken at different headings to remove the front/back ambiguity.
  // theta = source angle from robot forward axis, CCW +, in [-90, 90]; mirror = 180 - theta.
  bool computeBearing(Context& c) {
    float a[2] = { c.heading1 + c.theta1, c.heading1 + (180.0f - c.theta1) };
    float b[2] = { c.heading2 + c.theta2, c.heading2 + (180.0f - c.theta2) };
    float best = 1e9f; int bi = 0, bj = 0;
    for (int i = 0; i < 2; i++)
      for (int j = 0; j < 2; j++) {
        float d = fabsf(wrap180(a[i] - b[j]));
        if (d < best) { best = d; bi = i; bj = j; }
      }
    if (best > cfg::BEARING_AGREE_DEG) return false;           // measurements inconsistent
    c.bearing = wrap180(a[bi] + 0.5f * wrap180(b[bj] - a[bi])); // circular mean
    return true;
  }

  // Work done by the superstructure itself (SUB_INT).
  Result runInternal(const Step& s, uint32_t startMs, Context& c) {
    if (s.prim == Prim::WAIT)                                  // timeout_ms is the wait duration
      return ((int32_t)(millis() - (startMs + s.timeout_ms)) >= 0) ? Result::SUCCESS : Result::RUNNING;
    if (s.prim == Prim::COMPUTE_BEARING)
      return computeBearing(c) ? Result::SUCCESS : Result::FAIL;
    return Result::FAIL;
  }

  // Subsystems are private: nothing outside can call chassis.drive() / speaker.send() directly.
  Speaker    speaker_;
  MicProxy   mic_;
  Chassis    chassis_;
  Subsystem* sub_[4] = {};
  Slot       slots_[4];
  PrimStats  stats_[(int)Prim::COUNT];
};

// ============================================================================
// SEQUENCER   (runs a step table through the superstructure, with parallel groups)
//
// A group = consecutive rows where every row except the last has PAR_WITH_NEXT.
// The join policy and the failure policy are read from the LAST row of the group.
//
//   ALL       group finishes when every branch has finished
//   ANY       group finishes when the first branch finishes (the rest are aborted)
//   DEADLINE  group finishes when the IS_DEADLINE branch finishes (the rest are aborted)
//
// Any non-success result aborts the other branches and fails the group.
// States: IDLE, REQUEST, WAIT, DONE, FAILED
// ============================================================================
class Sequencer {
public:
  enum State : uint8_t { IDLE, REQUEST, WAIT, DONE, FAILED };
  static constexpr uint8_t kMaxGroup = Superstructure::kMaxGroup;

  explicit Sequencer(Superstructure& s) : sup_(s) {}

  void load(const Step* steps, uint8_t n, Context* ctx) { steps_ = steps; n_ = n; ctx_ = ctx; }
  void start() { i_ = 0; ctx_->retries = 0; state_ = REQUEST; reported_ = false; }
  void abort() { sup_.emergencyStop(); state_ = FAILED; }
  State state() const { return state_; }

  void update() {
    switch (state_) {
      case REQUEST: {
        Group g = groupAt(i_);
        Refusal why = sup_.requestGroup(&steps_[g.first], g.count, ctx_, subs_);
        if (why == Refusal::NONE) {
          uint32_t maxT = 0;
          for (uint8_t k = 0; k < g.count; k++) {
            done_[k] = false; res_[k] = Result::RUNNING;
            if (steps_[g.first + k].timeout_ms > maxT) maxT = steps_[g.first + k].timeout_ms;
          }
          groupDeadlineMs_ = millis() + maxT + 500;            // group-level timeout
          state_ = WAIT;
        } else if (why == Refusal::CONFLICT || why == Refusal::INVALID) {
          Serial.printf("[seq] step %u refused: %s\n", i_, why == Refusal::CONFLICT ? "CONFLICT" : "INVALID");
          state_ = FAILED;                                     // table error: fail fast
        }                                                      // BUSY / SETTLING: ask again next pass
        break;
      }

      case WAIT: {
        Group g = groupAt(i_);
        const Step& last = steps_[g.first + g.count - 1];
        bool allDone = true, anyDone = false, deadlineDone = false, bad = false, aborted = false;

        for (uint8_t k = 0; k < g.count; k++) {
          if (!done_[k]) {
            Result r = sup_.pollResult(subs_[k]);
            if (r != Result::RUNNING) { done_[k] = true; res_[k] = r; }
          }
          if (done_[k]) {
            anyDone = true;
            if (res_[k] == Result::ABORTED) aborted = true;
            if (res_[k] != Result::SUCCESS) bad = true;
            if (steps_[g.first + k].flags & IS_DEADLINE) deadlineDone = true;
          } else {
            allDone = false;
          }
        }
        if ((int32_t)(millis() - groupDeadlineMs_) >= 0) bad = true;   // group-level timeout

        bool finished = bad ||
          (last.join == Join::ALL      && allDone)      ||
          (last.join == Join::ANY      && anyDone)      ||
          (last.join == Join::DEADLINE && deadlineDone);
        if (!finished) break;

        for (uint8_t k = 0; k < g.count; k++)                  // cancel whatever is still running
          if (!done_[k]) sup_.abort(subs_[k]);

        if (aborted) { state_ = FAILED; break; }               // e-stop: never retry
        finishGroup(!bad, g, last);
        break;
      }

      case DONE:
      case FAILED:
        if (!reported_) {
          Serial.printf("[seq] %s at step %u\n", state_ == DONE ? "DONE" : "FAILED", i_);
          reported_ = true;
        }
        break;

      default: break;
    }
  }

private:
  struct Group { uint8_t first, count; };

  Group groupAt(uint8_t i) const {
    uint8_t j = i;
    while ((steps_[j].flags & PAR_WITH_NEXT) && j + 1 < n_) j++;
    return { i, (uint8_t)(j - i + 1) };
  }

  void finishGroup(bool ok, const Group& g, const Step& last) {
    if (ok) { ctx_->retries = 0; advanceTo(g.first + g.count); return; }
    switch (last.onFail) {
      case OnFail::GOTO:  ctx_->retries = 0; i_ = last.gotoIdx; state_ = REQUEST; break;
      case OnFail::RETRY: state_ = (++ctx_->retries < cfg::MAX_RETRIES) ? REQUEST : FAILED; break;   // restarts the whole group
      case OnFail::SKIP:  advanceTo(g.first + g.count); break;
      default:            state_ = FAILED; break;
    }
  }
  void advanceTo(uint8_t next) { i_ = next; state_ = (i_ >= n_) ? DONE : REQUEST; }

  Superstructure& sup_;
  const Step* steps_ = nullptr;
  uint8_t  n_ = 0, i_ = 0;
  Context* ctx_ = nullptr;
  State    state_ = IDLE;
  bool     reported_ = false;
  Sub      subs_[kMaxGroup];
  bool     done_[kMaxGroup];
  Result   res_[kMaxGroup];
  uint32_t groupDeadlineMs_ = 0;
};

// ============================================================================
// SEQUENCES
// Step = { prim, param, timeout_ms, onFail, gotoIdx [, flags, join] }
// Rows that omit flags/join are ordinary single steps (a group of size 1).
// ============================================================================
#if ROBOT_ROLE == ROBOT_A

#if A_USE_SWEEP
// Robot A, "sweep and listen": rotate while listening and stop as soon as the chirp is heard.
// LISTEN_CHIRP is the deadline branch, so the group ends when listening ends and the turn is
// cancelled. The heading is taken from the yaw history at the chirp's ARRIVAL time, so it is
// correct even though the robot kept turning.
// CAVEAT: turn slowly enough that the heading barely changes during one chirp (0.7 s),
// otherwise the TDOA angle is smeared. TODO: add a turn-speed limit for sweeps.
static const Step SEQ[] = {
  { Prim::LISTEN_CHIRP,    0,    12000, OnFail::ABORT, 0, PAR_WITH_NEXT | IS_DEADLINE },        // 0 ┐ one group
  { Prim::TURN,            360,  12000, OnFail::ABORT, 0, 0, Join::DEADLINE },                  // 1 ┘ join read here
  { Prim::COMPUTE_TDOA,    1,      500, OnFail::ABORT, 0 },                                     // 2  theta1, heading1
  { Prim::TURN,            90,    3000, OnFail::ABORT, 0 },                                     // 3
  { Prim::WAIT,            0,      300, OnFail::ABORT, 0 },                                     // 4  let vibration die down
  { Prim::LISTEN_CHIRP,    0,    10000, OnFail::ABORT, 0 },                                     // 5
  { Prim::COMPUTE_TDOA,    2,      500, OnFail::ABORT, 0 },                                     // 6  theta2, heading2
  { Prim::COMPUTE_BEARING, 0,       10, OnFail::ABORT, 0 },                                     // 7
  { Prim::TURN,            NAN,   3000, OnFail::ABORT, 0 },                                     // 8  NAN = use ctx.bearing
  { Prim::DRIVE,           1.0f,  4000, OnFail::ABORT, 0 },                                     // 9  feet
  { Prim::SEND_AFSK,       (float)CMD_STOP_CHIRP, 1500, OnFail::RETRY, 0 },                     // 10
};
#else
// Robot A, flat table: identical to the single-Teensy version. It runs unchanged here.
static const Step SEQ[] = {
  { Prim::LISTEN_CHIRP,    0,    10000, OnFail::ABORT, 0 },   // 0
  { Prim::COMPUTE_TDOA,    1,      500, OnFail::ABORT, 0 },   // 1  -> theta1, heading1
  { Prim::TURN,            90,    3000, OnFail::ABORT, 0 },   // 2
  { Prim::WAIT,            0,      300, OnFail::ABORT, 0 },   // 3  settle
  { Prim::LISTEN_CHIRP,    0,    10000, OnFail::ABORT, 0 },   // 4
  { Prim::COMPUTE_TDOA,    2,      500, OnFail::ABORT, 0 },   // 5  -> theta2, heading2
  { Prim::COMPUTE_BEARING, 0,       10, OnFail::ABORT, 0 },   // 6
  { Prim::TURN,            NAN,   3000, OnFail::ABORT, 0 },   // 7
  { Prim::DRIVE,           1.0f,  4000, OnFail::ABORT, 0 },   // 8
  { Prim::SEND_AFSK,       (float)CMD_STOP_CHIRP, 1500, OnFail::RETRY, 0 },   // 9
};
#endif

#else
// Robot B: chirp 0.7 s, listen for AFSK 2.3 s; nothing heard -> repeat; heard -> do it, DONE.
// The speaker and the mic still conflict acoustically, so these two cannot overlap.
static const Step SEQ[] = {
  { Prim::SEND_CHIRP,        0,  700, OnFail::ABORT, 0 },     // 0  duration = 700 ms
  { Prim::LISTEN_AFSK,       0, 2300, OnFail::GOTO,  0 },     // 1  timeout -> back to step 0
  { Prim::EXEC_RECEIVED_CMD, 0, 5000, OnFail::ABORT, 0 },     // 2  then DONE
};
#endif

// ============================================================================
// MAIN
// ============================================================================
static Superstructure sup;
static Sequencer      seq(sup);
static Context        ctx{};

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  sup.begin();                                     // init pins, gyro calibration, audio out, DSP link
  seq.load(SEQ, sizeof(SEQ) / sizeof(SEQ[0]), &ctx);

#if HAVE_WDT
  WDT_timings_t wdtCfg;
  wdtCfg.timeout = cfg::WDT_TIMEOUT_S;             // reset if nothing feeds it for this long
  g_wdt.begin(wdtCfg);
#endif

  seq.start();
}

void loop() {
  feedWatchdog();
  sup.update();     // every subsystem advances; each update() is short
  seq.update();     // decides the next group

#if LOG_STATES
  static uint32_t lastDump = 0;
  if (millis() - lastDump >= 1000) { lastDump = millis(); sup.dumpStates(); }
#endif

  // Other command sources go through the same gate. Example hooks:
  // if (digitalRead(ESTOP_PIN) == LOW) { seq.abort(); }          // e-stop button
  // (An AFSK STOP can now arrive while the robot is moving: decode it and call seq.abort().)
}
