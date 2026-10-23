// =============================================================================
// robot_all_in_one.cpp   (Teensy 4.x)
//
// Layers:   Sequencer  ->  Superstructure  ->  Subsystems (Speaker, Mic, Chassis)
//
//   * Sequencer      : "what comes next?"   step table, context, failure policy
//   * Superstructure : "is it safe now?"    single request() gate, single release
//                      path, settle timer, deadline watchdog, per-primitive timing
//   * Subsystems     : do the actual work. Hardware/DSP bits are STUBS (marked TODO).
//
// Design choices (from the design notes):
//   * Each primitive takes the time it needs. Mic::update() may run its whole
//     DSP pipeline inside one call (no forced slicing).
//   * Deadlines are enforced INSIDE long loops, and a hardware watchdog is fed
//     from those loops. Worst-case time per primitive is measured and logged.
//
// Arduino IDE: rename this file to robot_all_in_one.ino.  PlatformIO: keep .cpp.
// Optional: install the "Watchdog_t4" library; it is auto-detected below.
//
// TODO list (search for "TODO"):
//   1. Gyro driver            -> readGyroZ_dps()
//   2. Motor pins + driver    -> cfg::L_PWM etc., Chassis::setMotors()
//   3. Speaker output         -> spkStartChirp / spkStartAfsk / spkStop
//   4. Mic DSP                -> micFlush / micGetBlock / roughCorrelation / mixFFT /
//                                fineCorrelation / micTdoaSeconds / afskDecodeFrame
//   5. Command table          -> cmdToStep()
//   6. Calibrate              -> turn gains, drive speed, settle times, mic spacing
// =============================================================================



//#include <Arduino.h>
#include <math.h>

#if __has_include(<Watchdog_t4.h>)
  #include <Watchdog_t4.h>
  #define HAVE_WDT 1
  static WDT_T4<WDT1> g_wdt;
#else
  #define HAVE_WDT 0
#endif



//  code for https://www.onlinegdb.com/online_c++_compiler

#define DMAMEM 
#include <chrono>
#include <cstdint>

inline uint32_t millis()
{
    static auto start =
        std::chrono::steady_clock::now();

    auto now = std::chrono::steady_clock::now();

    return (uint32_t)std::chrono::duration_cast<
        std::chrono::milliseconds>(now - start).count();
}

#define INPUT         0
#define OUTPUT        1
#define INPUT_PULLUP  2

#define HIGH          1
#define LOW           0

inline void pinMode(int pin, int mode)
{
    // do nothing
}

inline void digitalWrite(int pin, int value)
{
    // do nothing
}

inline int digitalRead(int pin)
{
    return LOW;
}


inline void analogWriteFrequency(int pin, int frequency)
{
    // do nothing
}
inline void delay(unsigned long ms)
{
    // do nothing
}
inline uint32_t micros()
{
    return 0;
}
inline void digitalWriteFast(int pin, int value)
{
    digitalWrite(pin, value);
}
inline void analogWrite(int pin, int value)
{
    // do nothing
}

#define LED_BUILTIN 0 

#include <cstdio>











// ----------------------------- robot role ------------------------------------
#define ROBOT_A 1
#define ROBOT_B 2
#define ROBOT_ROLE ROBOT_A          // change to ROBOT_B for the other robot

#define LOG_STATS 1                 // print per-primitive timing when each one finishes

// ============================ configuration ==================================
namespace cfg {
  // motor driver pins (PWM + DIR style). TODO: match your wiring.
  // Put pull-downs on the PWM pins so motors stay off if the watchdog resets the chip.
  constexpr uint8_t L_PWM = 2, L_DIR = 3, R_PWM = 4, R_DIR = 5;

  // gyro
  constexpr uint32_t GYRO_PERIOD_US = 2000;     // read gyro at 500 Hz
  constexpr uint32_t GYRO_MAX_DT_US = 20000;    // longer gap (e.g. mic DSP ran): don't integrate

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

  // settle (quiet) time after a subsystem releases the floor
  constexpr uint32_t SETTLE_CHASSIS_MS = 300;   // motor noise + vibration
  constexpr uint32_t SETTLE_SPEAKER_MS = 150;   // speaker ring-down
  constexpr uint32_t SEND_CHIRP_GRACE_MS = 200; // deadline margin for SEND_CHIRP

  // acoustics
  constexpr float SPEED_OF_SOUND_MPS = 343.0f;
  constexpr float MIC_SPACING_M      = 0.10f;   // TODO: your mic baseline
  constexpr float BEARING_AGREE_DEG  = 30.0f;

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

enum class Result : uint8_t { RUNNING, SUCCESS, TIMEOUT, FAIL, REJECTED, ABORTED };
static const char* RESULT_NAME[] = { "RUNNING", "SUCCESS", "TIMEOUT", "FAIL", "REJECTED", "ABORTED" };

enum class OnFail : uint8_t { ABORT, RETRY, SKIP, GOTO };

enum class Owner : uint8_t { NONE = 0, SPEAKER, MIC, CHASSIS, COUNT };

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
  OnFail   onFail;       // policy when result != SUCCESS
  uint8_t  gotoIdx;      // used when onFail == GOTO
};

struct Context {         // "blackboard": data passed between steps
  float   theta1, theta2;      // TDOA angles (deg, from robot forward axis, CCW +)
  float   heading1, heading2;  // yaw (deg) when theta1/theta2 were measured
  float   bearing;             // world bearing to source (deg)
  uint8_t rxCmd;               // last decoded AFSK command
  uint8_t retries;
};

static inline int idx(Owner o) { return (int)o; }

static inline void feedWatchdog() {
#if HAVE_WDT
  g_wdt.feed();
#endif
}

static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// ============================ subsystem interface ============================
class Subsystem {
public:
  virtual ~Subsystem() {}
  virtual void   start(const Step& s, Context* ctx) = 0;
  virtual Result update() = 0;        // RUNNING until finished. MAY take as long as it needs.
  virtual void   abort() = 0;         // stop what you are doing NOW
  virtual bool   isIdle() const = 0;
  virtual void   forceIdle() = 0;     // hardware safe + state = IDLE
};

// ============================================================================
// SPEAKER subsystem   (SEND_CHIRP, SEND_AFSK)
// Output is assumed to play in the background (Audio library / DMA), so update()
// just checks whether the transmission has finished.
// ============================================================================
static void     spkBegin()                   { /* TODO: init audio output */ }
static void     spkStartChirp(uint32_t ms)   { /* TODO: start chirp of ms duration */ }
static uint32_t spkStartAfsk(uint8_t cmd)    { /* TODO: modulate + start frame */ return 500; /* est. ms */ }
static void     spkStop()                    { /* TODO: silence the output */ }

class Speaker : public Subsystem {
public:
  void begin() { spkBegin(); }

  void start(const Step& s, Context*) override {
    uint32_t now = millis();
    if (s.prim == Prim::SEND_CHIRP) {
      spkStartChirp(s.timeout_ms);                 // timeout_ms = chirp duration
      endMs_ = now + s.timeout_ms;
      st_ = CHIRP;
    } else {                                       // SEND_AFSK
      endMs_ = now + spkStartAfsk((uint8_t)s.param);
      st_ = AFSK;
    }
  }

  Result update() override {
    if (st_ == IDLE) return Result::SUCCESS;
    if ((int32_t)(millis() - endMs_) >= 0) {       // transmission time elapsed
      spkStop();
      return Result::SUCCESS;
    }
    return Result::RUNNING;
  }

  void abort() override      { forceIdle(); }
  bool isIdle() const override { return st_ == IDLE; }
  void forceIdle() override  { spkStop(); st_ = IDLE; }

private:
  enum St : uint8_t { IDLE, CHIRP, AFSK } st_ = IDLE;
  uint32_t endMs_ = 0;
};

// ============================================================================
// MIC subsystem   (LISTEN_CHIRP, COMPUTE_TDOA, LISTEN_AFSK)
// Long DSP is allowed inside update(): the loops below check their own deadline
// and feed the hardware watchdog.
// ============================================================================
constexpr int CAP_LEN = 4096;
DMAMEM static float g_capL[CAP_LEN];   // TODO: replace with your existing capture buffers
DMAMEM static float g_capR[CAP_LEN];

// ---- DSP stubs: fill these in with your dechirp / FSK demod code -----------
static void micBegin()          { /* TODO: init ADC/DMA/Audio input */ }
static void micFlush()          { /* TODO: discard stale audio so old samples are never decoded */ }
static bool micGetBlock()       { /* TODO: true when a new block of samples is ready */ return true; }
static bool roughCorrelation()  { /* TODO: stage 1, cheap check on newest block; true = candidate */ return true; }
static bool mixFFT()            { /* TODO: stage 2, dechirp mix + FFT; true = peak found */ return true; }
static bool fineCorrelation()   { /* TODO: stage 3, confirm chirp; fill g_capL/g_capR + arrival indices */ return true; }
static bool micTdoaSeconds(float& dt) { /* TODO: cross-correlate capL/capR; dt>0 means left mic hears it first */ return true; }
static bool afskDecodeFrame(uint8_t& cmd) { /* TODO: quad demod + frame/CRC check; true = valid frame */ return true; }

class Mic : public Subsystem {
public:
  enum Stage : uint8_t { WAITING, ROUGH, FFT, FINE };   // diagnostics only
  Stage stage() const { return stage_; }

  void begin() { micBegin(); }

  void start(const Step& s, Context* ctx) override {
    ctx_ = ctx;
    param_ = s.param;
    deadlineMs_ = millis() + s.timeout_ms;
    stage_ = WAITING;
    switch (s.prim) {
      case Prim::LISTEN_CHIRP: st_ = LISTEN_CHIRP; break;
      case Prim::COMPUTE_TDOA: st_ = TDOA;         break;
      case Prim::LISTEN_AFSK:  st_ = LISTEN_AFSK;  break;
      default:                 st_ = IDLE;         break;
    }
  }

  // Runs the whole primitive; returns when done, failed, or the deadline passes.
  Result update() override {
    switch (st_) {
      case LISTEN_CHIRP: return listenChirp();
      case TDOA:         return computeTdoa();
      case LISTEN_AFSK:  return listenAfsk();
      default:           return Result::FAIL;
    }
  }

  // Note: a blocking update() cannot be interrupted from outside. Abort only
  // resets state. The deadline checks and watchdog protect against hangs.
  void abort() override      { st_ = IDLE; }
  bool isIdle() const override { return st_ == IDLE; }
  void forceIdle() override  { st_ = IDLE; }

private:
  bool deadlinePassed() const { return (int32_t)(millis() - deadlineMs_) >= 0; }

  Result listenChirp() {
    micFlush();
    while (!deadlinePassed()) {
      feedWatchdog();
      stage_ = WAITING;
      if (!micGetBlock())      continue;   // nothing new yet
      stage_ = ROUGH;
      if (!roughCorrelation()) continue;   // cheap reject
      stage_ = FFT;
      if (!mixFFT())           continue;
      stage_ = FINE;
      if (!fineCorrelation())  continue;
      return Result::SUCCESS;              // capture buffers now hold the chirp
    }
    return Result::TIMEOUT;
  }

  Result computeTdoa() {
    float dt;
    if (!micTdoaSeconds(dt)) return Result::FAIL;
    float s = cfg::SPEED_OF_SOUND_MPS * dt / cfg::MIC_SPACING_M;
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    float deg = asinf(s) * 180.0f / (float)M_PI;   // TODO: confirm sign vs gyro (CCW +)
    if ((int)param_ == 1) ctx_->theta1 = deg;
    else                  ctx_->theta2 = deg;
    return Result::SUCCESS;
  }

  Result listenAfsk() {
    micFlush();
    while (!deadlinePassed()) {
      feedWatchdog();
      uint8_t cmd;
      if (afskDecodeFrame(cmd)) {
        ctx_->rxCmd = cmd;
        return Result::SUCCESS;
      }
    }
    return Result::TIMEOUT;
  }

  enum St : uint8_t { IDLE, LISTEN_CHIRP, TDOA, LISTEN_AFSK } st_ = IDLE;
  Stage    stage_ = WAITING;
  Context* ctx_ = nullptr;
  float    param_ = 0;
  uint32_t deadlineMs_ = 0;
};

// ============================================================================
// CHASSIS subsystem   (TURN, DRIVE)
// Yaw convention: degrees, CCW (left) positive, wrapped to (-180, 180].
// Non-blocking: update() is called every loop pass and does one control step.
// ============================================================================
// TODO: replace with your gyro driver. Return true + deg/s (CCW positive) on success.
static bool readGyroZ_dps(float& dps) { return false; }

class Chassis : public Subsystem {
public:
  void begin() {
    pinMode(cfg::L_PWM, OUTPUT); pinMode(cfg::L_DIR, OUTPUT);
    pinMode(cfg::R_PWM, OUTPUT); pinMode(cfg::R_DIR, OUTPUT);
    analogWriteFrequency(cfg::L_PWM, 20000);
    analogWriteFrequency(cfg::R_PWM, 20000);
    setMotors(0, 0);

    // Gyro bias calibration: robot must be still for ~0.5 s at boot.
    float sum = 0, v; int n = 0;
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
    if (!readGyroZ_dps(raw)) {
      if (++badReads_ > 50) gyroOk_ = false;       // sustained failure -> timer fallback
      return;
    }
    badReads_ = 0; gyroOk_ = true;

    if (mode_ == STOPPED && fabsf(raw - bias_) < 1.0f)   // slow bias tracking while still
      bias_ += 0.001f * (raw - bias_);

    rate_ = raw - bias_;
    yaw_  = wrap180(yaw_ + rate_ * (dtUs * 1e-6f));
  }

  float yawDeg() const { return yaw_; }

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
        setMotors(0, 0);
        yaw_ = wrap180(yaw_ + pendingDeg_);
        mode_ = STOPPED;
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

  Mode     mode_ = STOPPED;
  float    yaw_ = 0, bias_ = 0, rate_ = 0;
  uint32_t lastUs_ = 0;
  uint8_t  badReads_ = 0;
  bool     gyroOk_ = false;
  bool     useGyro_ = false;
  float    targetYaw_ = 0, pendingDeg_ = 0, holdYaw_ = 0;
  int8_t   openLoopDir_ = 1;
  uint8_t  holdTicks_ = 0;
  uint32_t endMs_ = 0;
};

// ============================================================================
// SUPERSTRUCTURE
//
// States:
//   IDLE              nothing running, free to accept a request
//   SUBSYSTEM_ACTIVE  one subsystem owns the floor (Speaker, Mic or Chassis)
//   INTERNAL_ACTIVE   superstructure-only step (WAIT, COMPUTE_BEARING)
//   SETTLING          quiet period after a release. A request for a DIFFERENT
//                     subsystem is refused until it ends; owner-less steps and
//                     the same subsystem are allowed.
//
//   IDLE --request--> SUBSYSTEM_ACTIVE | INTERNAL_ACTIVE --finish--> SETTLING | IDLE
//   SETTLING --timer expires--> IDLE
// ============================================================================
enum class SupState : uint8_t { IDLE, SUBSYSTEM_ACTIVE, INTERNAL_ACTIVE, SETTLING };

// Which subsystem does a primitive need? NONE = superstructure-only work.
static Owner ownerFor(Prim p) {
  switch (p) {
    case Prim::SEND_CHIRP:
    case Prim::SEND_AFSK:     return Owner::SPEAKER;
    case Prim::LISTEN_CHIRP:
    case Prim::COMPUTE_TDOA:
    case Prim::LISTEN_AFSK:   return Owner::MIC;
    case Prim::TURN:
    case Prim::DRIVE:         return Owner::CHASSIS;
    default:                  return Owner::NONE;   // WAIT, COMPUTE_BEARING, EXEC_RECEIVED_CMD
  }
}

static uint32_t settleMsAfter(Owner o) {
  switch (o) {
    case Owner::CHASSIS: return cfg::SETTLE_CHASSIS_MS;
    case Owner::SPEAKER: return cfg::SETTLE_SPEAKER_MS;
    default:             return 0;
  }
}

// Map a decoded AFSK command to a concrete step. TODO: extend for your command set.
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
  void begin() {
    sub_[idx(Owner::SPEAKER)] = &speaker_;
    sub_[idx(Owner::MIC)]     = &mic_;
    sub_[idx(Owner::CHASSIS)] = &chassis_;
    speaker_.begin(); mic_.begin(); chassis_.begin();
  }

  // ---------------------------------------------------------------------------
  // request(): the ONLY way to start work. false = "not now, ask again".
  // true = accepted; the outcome arrives through takeResult().
  // ---------------------------------------------------------------------------
  bool request(const Step& in, Context* ctx) {
    if (state_ == SupState::SUBSYSTEM_ACTIVE || state_ == SupState::INTERNAL_ACTIVE)
      return false;                                // one thing at a time, owner-less steps included

    Step s = in;
    if (!resolve(s, ctx)) {                        // e.g. unknown received command
      ctx_ = ctx; active_ = in; owner_ = Owner::NONE;
      startMs_ = millis(); state_ = SupState::INTERNAL_ACTIVE;
      finish(Result::FAIL);
      return true;                                 // accepted; result = FAIL
    }

    Owner need = ownerFor(s.prim);
    if (need != Owner::NONE) {
      if (settling() && need != lastOwner_) return false;   // quiet period for a different subsystem
      if (!allSubsystemsIdle()) return false;               // invariant check; should never fail
    }

    active_  = s;
    ctx_     = ctx;
    owner_   = need;
    startMs_ = millis();
    uint32_t grace = (s.prim == Prim::SEND_CHIRP) ? cfg::SEND_CHIRP_GRACE_MS : 0;
    deadlineMs_ = startMs_ + s.timeout_ms + grace;
    lastResult_ = Result::RUNNING;

    if (need == Owner::NONE) {
      state_ = SupState::INTERNAL_ACTIVE;
    } else {
      state_ = SupState::SUBSYSTEM_ACTIVE;
      sub_[idx(need)]->start(s, ctx);
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  // update(): call every loop pass. A subsystem's update() may take a long time
  // (Mic DSP); the loops inside it enforce their own deadline and feed the watchdog.
  // ---------------------------------------------------------------------------
  void update() {
    chassis_.tickHeading();                        // heading integration always runs

    switch (state_) {
      case SupState::IDLE: return;
      case SupState::SETTLING:
        if (!settling()) { settleActive_ = false; state_ = SupState::IDLE; }
        return;
      default: break;                              // an active state: fall through to work
    }

    uint32_t now = millis();
    Result r = Result::RUNNING;

    switch (active_.prim) {
      case Prim::WAIT:                             // timeout_ms is the wait duration
        r = ((int32_t)(now - (startMs_ + active_.timeout_ms)) >= 0) ? Result::SUCCESS : Result::RUNNING;
        break;

      case Prim::COMPUTE_BEARING:                  // pure math, finishes immediately
        r = computeBearing(*ctx_) ? Result::SUCCESS : Result::FAIL;
        break;

      default: {                                   // subsystem-owned primitive
        uint32_t t0 = micros();
        r = sub_[idx(owner_)]->update();
        uint32_t us = micros() - t0;
        PrimStats& st = stats_[(int)active_.prim];
        if (us > st.maxUpdateUs) st.maxUpdateUs = us;      // worst-case single update() call

        if (r == Result::RUNNING && (int32_t)(millis() - deadlineMs_) >= 0)
          r = Result::TIMEOUT;                     // backstop deadline for every primitive
        break;
      }
    }

    if (r == Result::SUCCESS && active_.prim == Prim::COMPUTE_TDOA) {
      // Mic stored theta1/theta2. Stamp the heading it was measured at.
      if ((int)active_.param == 1) ctx_->heading1 = chassis_.yawDeg();
      else                         ctx_->heading2 = chassis_.yawDeg();
    }

    if (r != Result::RUNNING) finish(r);
  }

  void emergencyStop() {
    chassis_.forceIdle();                          // motors off no matter what
    if (state_ == SupState::SUBSYSTEM_ACTIVE || state_ == SupState::INTERNAL_ACTIVE)
      finish(Result::ABORTED);
  }

  Result   takeResult() { Result r = lastResult_; lastResult_ = Result::RUNNING; return r; }  // one-shot
  SupState state() const { return state_; }

private:
  bool settling() const { return settleActive_ && (int32_t)(millis() - settleUntil_) < 0; }

  bool allSubsystemsIdle() const {
    for (int i = 1; i < (int)Owner::COUNT; i++)
      if (sub_[i] && !sub_[i]->isIdle()) return false;
    return true;
  }

  // The single release path: success, timeout, failure and e-stop all end here.
  void finish(Result r) {
    uint32_t now = millis();

    PrimStats& st = stats_[(int)active_.prim];     // timing stats
    uint32_t total = now - startMs_;
    st.count++;
    if (total > st.maxTotalMs) st.maxTotalMs = total;
#if LOG_STATS
    printf("[sup] %-17s %-8s total=%lums  max_total=%lums  max_update=%luus\n",
                  PRIM_NAME[(int)active_.prim], RESULT_NAME[(int)r],
                  (unsigned long)total, (unsigned long)st.maxTotalMs, (unsigned long)st.maxUpdateUs);
#endif

    if (owner_ != Owner::NONE) {
      Subsystem* sub = sub_[idx(owner_)];
      if (r != Result::SUCCESS) sub->abort();      // make hardware safe on any abnormal end
      sub->forceIdle();                            // never free the floor until truly idle
      uint32_t ms = settleMsAfter(owner_);
      settleActive_ = (ms > 0);
      if (ms) settleUntil_ = now + ms;
      lastOwner_ = owner_;
    }
    owner_      = Owner::NONE;
    lastResult_ = r;
    state_      = settling() ? SupState::SETTLING : SupState::IDLE;
  }

  // Turn "data" steps into concrete ones at the moment they run.
  bool resolve(Step& s, Context* ctx) {
    if (s.prim == Prim::EXEC_RECEIVED_CMD) {
      if (!cmdToStep(ctx->rxCmd, s)) return false;
    }
    if (s.prim == Prim::TURN && isnan(s.param))
      s.param = wrap180(ctx->bearing - chassis_.yawDeg());   // relative turn to the world bearing
    return true;
  }

  // Combine two TDOA angles taken at different headings to remove the front/back ambiguity.
  // theta = source angle from robot forward axis, CCW +, in [-90, 90]; mirror = 180 - theta.
  // Flip the sign of theta if your TDOA convention disagrees with the gyro's.
  bool computeBearing(Context& c) {
    float a[2] = { c.heading1 + c.theta1, c.heading1 + (180.0f - c.theta1) };
    float b[2] = { c.heading2 + c.theta2, c.heading2 + (180.0f - c.theta2) };
    float best = 1e9f; int bi = 0, bj = 0;
    for (int i = 0; i < 2; i++)
      for (int j = 0; j < 2; j++) {
        float d = fabsf(wrap180(a[i] - b[j]));
        if (d < best) { best = d; bi = i; bj = j; }
      }
    if (best > cfg::BEARING_AGREE_DEG) return false;          // measurements inconsistent
    c.bearing = wrap180(a[bi] + 0.5f * wrap180(b[bj] - a[bi])); // circular mean
    return true;
  }

  // Subsystems are private: nothing outside can call chassis.drive() / speaker.send() directly.
  Speaker    speaker_;
  Mic        mic_;
  Chassis    chassis_;
  Subsystem* sub_[(int)Owner::COUNT] = {};

  SupState state_ = SupState::IDLE;
  Owner    owner_ = Owner::NONE, lastOwner_ = Owner::NONE;
  Step     active_{};
  Context* ctx_ = nullptr;
  Result   lastResult_ = Result::RUNNING;
  uint32_t startMs_ = 0, deadlineMs_ = 0, settleUntil_ = 0;
  bool     settleActive_ = false;
  PrimStats stats_[(int)Prim::COUNT];
};

// ============================================================================
// SEQUENCER   (runs a step table through the superstructure)
// States: IDLE, REQUEST, WAIT, DONE, FAILED
// ============================================================================
class Sequencer {
public:
  enum State : uint8_t { IDLE, REQUEST, WAIT, DONE, FAILED };

  explicit Sequencer(Superstructure& s) : sup_(s) {}

  void load(const Step* steps, uint8_t n, Context* ctx) { steps_ = steps; n_ = n; ctx_ = ctx; }
  void start() { i_ = 0; ctx_->retries = 0; state_ = REQUEST; reported_ = false; }
  void abort() { sup_.emergencyStop(); state_ = FAILED; }
  State state() const { return state_; }

  void update() {
    switch (state_) {
      case REQUEST:
        if (sup_.request(steps_[i_], ctx_)) state_ = WAIT;   // may be refused: just ask again next pass
        break;

      case WAIT: {
        Result r = sup_.takeResult();
        if (r == Result::RUNNING) break;
        if (r == Result::SUCCESS) { advance(); break; }
        if (r == Result::ABORTED) { state_ = FAILED; break; }

        const Step& s = steps_[i_];                          // TIMEOUT / FAIL: apply the step's policy
        switch (s.onFail) {
          case OnFail::GOTO:  ctx_->retries = 0; i_ = s.gotoIdx; state_ = REQUEST; break;
          case OnFail::RETRY: state_ = (++ctx_->retries < cfg::MAX_RETRIES) ? REQUEST : FAILED; break;
          case OnFail::SKIP:  advance(); break;
          default:            state_ = FAILED; break;
        }
        break;
      }

      case DONE:
      case FAILED:
        if (!reported_) {
          printf("[seq] %s at step %u\n", state_ == DONE ? "DONE" : "FAILED", i_);
          reported_ = true;
        }
        break;

      default: break;
    }
  }

private:
  void advance() {
    ctx_->retries = 0;
    if (++i_ >= n_) state_ = DONE; else state_ = REQUEST;
  }

  Superstructure& sup_;
  const Step* steps_ = nullptr;
  uint8_t  n_ = 0, i_ = 0;
  Context* ctx_ = nullptr;
  State    state_ = IDLE;
  bool     reported_ = false;
};

// ============================================================================
// SEQUENCES
// Step = { prim, param, timeout_ms, onFail, gotoIdx }
// ============================================================================
#if ROBOT_ROLE == ROBOT_A
// Robot A: hear chirp -> TDOA -> turn 90 -> hear chirp -> TDOA -> bearing ->
//          face source -> drive 1 ft -> send AFSK "stop chirp"
static const Step SEQ[] = {
  { Prim::LISTEN_CHIRP,    0,    100000000, OnFail::ABORT, 0 },   // 0
  { Prim::COMPUTE_TDOA,    1,      500, OnFail::ABORT, 0 },   // 1  -> theta1, heading1
  { Prim::TURN,            90,    3000, OnFail::ABORT, 0 },   // 2
  { Prim::WAIT,            0,      300, OnFail::ABORT, 0 },   // 3  settle (overlaps the superstructure settle)
  { Prim::LISTEN_CHIRP,    0,    10000, OnFail::ABORT, 0 },   // 4
  { Prim::COMPUTE_TDOA,    2,      500, OnFail::ABORT, 0 },   // 5  -> theta2, heading2
  { Prim::COMPUTE_BEARING, 0,       10, OnFail::ABORT, 0 },   // 6  -> bearing
  { Prim::TURN,            NAN,   3000, OnFail::ABORT, 0 },   // 7  NAN = use ctx.bearing
  { Prim::DRIVE,           1.0f,  4000, OnFail::ABORT, 0 },   // 8  feet
  { Prim::SEND_AFSK,       (float)CMD_STOP_CHIRP, 1500, OnFail::RETRY, 0 },  // 9
};
#else
// Robot B: chirp 0.7 s, listen for AFSK 2.3 s; nothing heard -> repeat; heard -> do it, DONE
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

// -----------------------------------------------------------------------------
// Manual Serial commands. All actual work still goes through sup.request().
// This version has ONE active subsystem at a time; there is no requestGroup().
// -----------------------------------------------------------------------------
static bool manualPending = false;
static bool manualInFlight = false;
static Step manualStep{};

// 't' = turn left 90 degrees; 'd' = drive forward 1 foot; 'x' = emergency stop.
static bool userCharToStep(char ch, Step& out) {
  switch (ch) {
    case 't':
    case 'T':
      out = { Prim::TURN, 90.0f, 3000, OnFail::ABORT, 0 };
      return true;
    case 'd':
    case 'D':
      out = { Prim::DRIVE, 1.0f, 4000, OnFail::ABORT, 0 };
      return true;
    default:
      return false;
  }
}

// A manual motion command preempts the autonomous sequence, then goes through
// the Superstructure gate. Refused requests are retried by serviceUserCommands().
static void onUserCommand(char ch) {
  if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') return;

  if (ch == 'x' || ch == 'X' || ch == '!') {
    manualPending = false;
    manualInFlight = false;
    seq.abort();  // invokes sup.emergencyStop(); chassis motors are commanded off
    printf("[user] EMERGENCY STOP: sequence aborted, motors commanded off");
    return;
  }

  Step requested{};
  if (!userCharToStep(ch, requested)) {
    if (ch == '?' || ch == 'h' || ch == 'H') {
      printf("Commands: t=turn left 90 deg, d=drive forward 1 ft, x=emergency stop, ?=help");
    } else {
      printf("[user] Unknown command '%c'. Type ? for help.\n", ch);
    }
    return;
  }

  // The sequencer and manual control share sup.takeResult(), so stop the
  // sequencer before manual control takes ownership of that result channel.
  seq.abort();
  manualStep = requested;
  manualPending = true;       // newest motion command replaces any pending one
  manualInFlight = false;     // any previous manual action was preempted
  printf("[user] Queued manual command: %s\n",
                requested.prim == Prim::TURN ? "turn left 90 degrees" : "drive forward 1 foot");
}

// Call after sup.update(). A request can be refused during settling; retry it.
// Since seq is FAILED after manual takeover, it will not consume manual results.
static void serviceUserCommands() {
  if (manualPending && !manualInFlight) {
    if (sup.request(manualStep, &ctx)) {
      manualPending = false;
      manualInFlight = true;
      printf("[user] Accepted by Superstructure: %s\n",
                    manualStep.prim == Prim::TURN ? "TURN" : "DRIVE");
    }
  }

  if (manualInFlight) {
    Result r = sup.takeResult();
    if (r != Result::RUNNING) {
      printf("[user] Manual command finished: %s\n", RESULT_NAME[(int)r]);
      manualInFlight = false;
    }
  }
}

void setup() {
  //Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  sup.begin();                                     // init pins, gyro calibration, audio in/out
  seq.load(SEQ, sizeof(SEQ) / sizeof(SEQ[0]), &ctx);

#if HAVE_WDT
  WDT_timings_t wdtCfg;
  wdtCfg.timeout = cfg::WDT_TIMEOUT_S;             // reset if nothing feeds it for this long
  g_wdt.begin(wdtCfg);
#endif

  seq.start();
  printf("Robot ready. Serial commands: t=left 90 deg, d=forward 1 ft, x=stop, ?=help");
}

void loop() {
  feedWatchdog();

  // 1) Read user input first so manual commands can preempt autonomous work.
  //while (Serial.available() > 0) {
  //  onUserCommand((char)Serial.read());
  //}

  // 2) Advance the currently active primitive / settling timer.
  sup.update();

  // 3) Start/retry manual work and collect its result through the same gate.
  serviceUserCommands();

  // 4) Advance the autonomous sequence. After manual takeover it stays FAILED.
  seq.update();

  // Other input sources should follow the same pattern: decode -> sup.request()
  // for work, or sup.emergencyStop() for an emergency.
}


int main() {
    setup();
    while(1) {
        loop();
    }
}
