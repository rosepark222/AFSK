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
// MERGE STATUS
//   [x] afskDecodeFrame   -> merged: the standalone AFSK RX sketch lives in namespace afsk
//   [ ] spkStartChirp / spkStartAfsk / spkStop   (files 1 and 2)
//   [ ] roughCorrelation / mixFFT / fineCorrelation   (file 4)
//
// TODO list (search for "TODO"):
//   1. Gyro driver            -> readGyroZ_dps()
//   2. Motor pins + driver    -> cfg::L_PWM etc., Chassis::setMotors()
//   3. Speaker output         -> spkStartChirp / spkStartAfsk / spkStop
//   4. Mic DSP (chirp)        -> micFlush / micGetBlock / roughCorrelation / mixFFT /
//                                fineCorrelation / micTdoaSeconds
//   5. Command table          -> cmdToStep()
//   6. Calibrate              -> turn gains, drive speed, settle times, mic spacing
// =============================================================================
#include <Arduino.h>
#include <Audio.h>
#include <arm_math.h>
#include <math.h>
#include <string.h>

#if __has_include(<Watchdog_t4.h>)
  #include <Watchdog_t4.h>
  #define HAVE_WDT 1
  static WDT_T4<WDT1> g_wdt;
#else
  #define HAVE_WDT 0
#endif



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

  // audio library memory blocks (shared by everything that uses the Audio library).
  // TODO: raise this when the speaker output and the dechirp queue are merged.
  constexpr int AUDIO_MEMORY_BLOCKS = 16;

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
// AFSK RECEIVER   (merged from the standalone "FSK Packet RX" test sketch)
//
// Frame: [4-byte preamble][start sync][size][payload][CRC16][end sync]
//
// How the standalone sketch maps onto the framework:
//   setup()  : AudioMemory() -> setup() below;  queue1.begin() -> afsk::start()
//              detector / parser init           -> afsk::reset() (called by start())
//   loop()   : the body                         -> afsk::decodeFrame()
//   prints   : only with AFSK_DEBUG 1 (verbose, as in the standalone sketch)
//
// The queue is started at the beginning of a listen and stopped at the end, so it
// never holds audio buffers while nobody reads them.
// ============================================================================
#define AFSK_DEBUG 0   // 1: per-byte / per-event prints, like the standalone sketch
#if AFSK_DEBUG
  #define AFSK_LOG(...) Serial.printf(__VA_ARGS__)
#else
  #define AFSK_LOG(...) ((void)0)
#endif

// Teensy Audio objects must be globals.
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;                         // AFSK queue
AudioConnection   patchCord1(i2s_in, 0, queue1, 0);
// TODO (file 4): the dechirp gets its own AudioRecordQueue and AudioConnection off i2s_in.

namespace afsk {

  // FSK tones (audible range for bench testing; use 15000 / 17000 for ultrasonic)
  constexpr float FREQ_MARK   = 6000.0f;
  constexpr float FREQ_SPACE  = 8000.0f;
  constexpr int   BAUD_RATE   = 100;
  constexpr float SAMPLE_RATE = 44100.0f;

  // Envelope LPF for the I/Q detector
  constexpr float LPF_FC = 2.0f * BAUD_RATE;
  static const float LPF_ALPHA = 1.0f - expf(-2.0f * PI * LPF_FC / SAMPLE_RATE);

  // Packet bytes
  static const uint8_t PREAMBLE_BYTE = 0x55;
  static const uint8_t START_SYNC    = 0x7E;
  static const uint8_t END_SYNC      = 0x7F;
  static const uint8_t PREAMBLE_LEN  = 4;
  static const uint8_t MAX_PAYLOAD   = 254;
  static const uint32_t FRAME_TIMEOUT_MS = 5000;

  // ─── debug counters ───────────────────────────────────────────────────────
  static uint32_t debugBitCount = 0;
  static uint32_t debugByteCount = 0;
  static uint32_t debugBlockCount = 0;
  static float    debugAvgMark = 0;
  static float    debugAvgSpace = 0;

  // ─── I/Q tone detector ────────────────────────────────────────────────────
  struct IQDetector {
    float phaseInc;
    float phase;
    float lpf_I;
    float lpf_Q;

    void init(float freq) {
      phaseInc = 2.0f * PI * freq / SAMPLE_RATE;
      phase = 0.0f;
      lpf_I = 0.0f;
      lpf_Q = 0.0f;
    }

    float process(float x) {
      float c = cosf(phase);
      float s = sinf(phase);

      float i_raw = x * c;
      float q_raw = x * s;

      lpf_I += LPF_ALPHA * (i_raw - lpf_I);
      lpf_Q += LPF_ALPHA * (q_raw - lpf_Q);

      phase += phaseInc;
      if (phase >= 2.0f * PI) phase -= 2.0f * PI;

      return lpf_I * lpf_I + lpf_Q * lpf_Q;
    }
  };

  static IQDetector markDet, spaceDet;

  // ─── clock recovery ───────────────────────────────────────────────────────
  constexpr int SAMPLES_PER_BIT = (int)(SAMPLE_RATE / BAUD_RATE);
  static int clockCounter = 0;
  static int lastSoft = -1;

  static int clockRecovery(int softBit) {
    int committed = -1;

    if (softBit != -1 && lastSoft != -1 && softBit != lastSoft) {
      int mid = SAMPLES_PER_BIT / 2;
      if (clockCounter < mid) {
        clockCounter += 2;
      } else {
        clockCounter -= 2;
      }
    }

    if (softBit != -1) lastSoft = softBit;

    clockCounter++;
    if (clockCounter >= SAMPLES_PER_BIT) {
      clockCounter = 0;
      if (lastSoft != -1) committed = lastSoft;
    }

    return committed;
  }

  // ─── CRC ──────────────────────────────────────────────────────────────────
  static uint16_t crc16_ccitt_false_update(uint16_t crc, uint8_t data) {
    crc ^= ((uint16_t)data) << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
    return crc;
  }

  static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
      crc = crc16_ccitt_false_update(crc, data[i]);
    }
    return crc;
  }

  // ─── bit-to-byte assembly ─────────────────────────────────────────────────
  struct BitAssembler {
    uint8_t currentByte = 0;
    uint8_t bitCount = 0;

    bool pushBit(int bit, uint8_t &outByte) {
      currentByte = (currentByte << 1) | (bit & 0x01);
      bitCount++;

      if (bitCount == 8) {
        outByte = currentByte;
        currentByte = 0;
        bitCount = 0;
        return true;
      }
      return false;
    }

    void reset() {
      currentByte = 0;
      bitCount = 0;
    }
  };

  static BitAssembler assembler;

  // ─── packet state machine ─────────────────────────────────────────────────
  enum RxState {
    RX_SEARCH_PREAMBLE,
    RX_SEARCH_SYNC,
    RX_READ_SIZE,
    RX_READ_PAYLOAD,
    RX_READ_CRC_HI,
    RX_READ_CRC_LO,
    RX_READ_END_SYNC
  };

#if AFSK_DEBUG
  static const char* stateNames[] = {
    "PREAMBLE", "SYNC", "SIZE", "PAYLOAD", "CRC_HI", "CRC_LO", "END_SYNC"
  };
#endif

  static RxState  rxState = RX_SEARCH_PREAMBLE;
  static uint8_t  preambleMatch = 0;
  static uint8_t  sizeByte = 0;
  static uint8_t  payload[MAX_PAYLOAD];
  static uint8_t  payloadIndex = 0;
  static uint16_t rxCrc = 0;
  static uint8_t  rxCrcHi = 0;
  static uint32_t frameStartMs = 0;

  // Sliding 8-bit window of the most recent committed bits (MSB oldest, LSB newest).
  // Used to detect START_SYNC on any bit boundary.
  static uint8_t recentBits = 0;

  // Preamble (0x55 repeated) detection at bit level: count consecutive alternating bits.
  static int lastCommittedBit = -1;
  static int altBitCount = 0;
  constexpr int PREAMBLE_BITS_REQUIRED = PREAMBLE_LEN * 8;

  // Result of the last accepted frame (a copy: the parser buffer is reused).
  static bool    frameReady = false;
  static uint8_t rxPayload[MAX_PAYLOAD];
  static uint8_t rxSize = 0;

  static void resetFrameParser() {
    rxState = RX_SEARCH_PREAMBLE;
    preambleMatch = 0;
    sizeByte = 0;
    payloadIndex = 0;
    rxCrc = 0;
    rxCrcHi = 0;
    assembler.reset();
    recentBits = 0;             // reset sliding window
    lastCommittedBit = -1;
    altBitCount = 0;
    frameStartMs = millis();
    AFSK_LOG("[RESET] State -> %s, frameStartMs=%lu\n", stateNames[rxState], (unsigned long)frameStartMs);
  }

  static void rejectFrame(const char* reason) {
    AFSK_LOG("FRAME_REJECT: %s (State: %s, elapsed: %lu ms, bytes: %lu)\n",
             reason, stateNames[rxState], (unsigned long)(millis() - frameStartMs),
             (unsigned long)debugByteCount);
    (void)reason;
    resetFrameParser();
  }

  static void acceptFrame() {
    uint8_t crcInput[1 + MAX_PAYLOAD];
    crcInput[0] = sizeByte;
    for (uint8_t i = 0; i < sizeByte; i++) {
      crcInput[1 + i] = payload[i];
    }

    uint16_t calc = crc16_ccitt_false(crcInput, 1 + sizeByte);

    if (calc != rxCrc) {
      rejectFrame("CRC_FAIL");
      return;
    }

#if AFSK_DEBUG
    Serial.print("FRAME_OK size=");
    Serial.print(sizeByte);
    Serial.print(" payload=\"");
    for (uint8_t i = 0; i < sizeByte; i++) {
      char c = (char)payload[i];
      if (c >= 32 && c <= 126) Serial.print(c);
      else Serial.print('.');
    }
    Serial.println("\"");
#endif

    // Merge change: instead of printing and blinking (delay(80) would stall the
    // audio queue), hand the frame to decodeFrame().
    rxSize = sizeByte;
    memcpy(rxPayload, payload, sizeByte);
    frameReady = true;
    resetFrameParser();
  }

  static void processByte(uint8_t b) {
    // Merge change: the frame timeout only applies once a preamble was found.
    // While just searching, silence must not generate a TIMEOUT reject every 5 s.
    if (rxState != RX_SEARCH_PREAMBLE && millis() - frameStartMs > FRAME_TIMEOUT_MS) {
      rejectFrame("TIMEOUT");
      return;
    }

    debugByteCount++;

    AFSK_LOG("[BYTE %lu] 0x%02X (%3d) | State: %s", (unsigned long)debugByteCount, b, b, stateNames[rxState]);

    switch (rxState) {
      case RX_SEARCH_PREAMBLE:
        if (b == PREAMBLE_BYTE) {
          preambleMatch++;
          AFSK_LOG(" | Preamble match %d/%d", preambleMatch, PREAMBLE_LEN);
          if (preambleMatch >= PREAMBLE_LEN) {
            rxState = RX_SEARCH_SYNC;
            AFSK_LOG(" -> Moving to SYNC");
          }
        } else {
          if (preambleMatch > 0) {
            AFSK_LOG(" | Preamble reset (was %d)", preambleMatch);
          }
          preambleMatch = 0;
        }
        break;

      case RX_SEARCH_SYNC:
        if (b == START_SYNC) {
          rxState = RX_READ_SIZE;
          AFSK_LOG(" | START_SYNC found -> SIZE");
        } else if (b != PREAMBLE_BYTE) {
          preambleMatch = 0;
          rxState = RX_SEARCH_PREAMBLE;
          AFSK_LOG(" | Not sync, back to PREAMBLE");
        }
        break;

      case RX_READ_SIZE:
        sizeByte = b;
        if (sizeByte > MAX_PAYLOAD) {
          AFSK_LOG(" | BAD_SIZE");
          rejectFrame("BAD_SIZE");
          return;
        }
        payloadIndex = 0;
        if (sizeByte == 0) {
          rxState = RX_READ_CRC_HI;
          AFSK_LOG(" | Size=0 -> CRC_HI");
        } else {
          rxState = RX_READ_PAYLOAD;
          AFSK_LOG(" | Size=%d -> PAYLOAD", sizeByte);
        }
        break;

      case RX_READ_PAYLOAD:
        payload[payloadIndex++] = b;
        AFSK_LOG(" | Payload[%d]", payloadIndex - 1);
        if (payloadIndex >= sizeByte) {
          rxState = RX_READ_CRC_HI;
          AFSK_LOG(" -> CRC_HI");
        }
        break;

      case RX_READ_CRC_HI:
        rxCrcHi = b;
        rxState = RX_READ_CRC_LO;
        AFSK_LOG(" | CRC_HI=0x%02X -> CRC_LO", rxCrcHi);
        break;

      case RX_READ_CRC_LO:
        rxCrc = ((uint16_t)rxCrcHi << 8) | b;
        rxState = RX_READ_END_SYNC;
        AFSK_LOG(" | CRC_LO, full CRC=0x%04X -> END_SYNC", rxCrc);
        break;

      case RX_READ_END_SYNC:
        if (b == END_SYNC) {
          AFSK_LOG(" | END_SYNC found -> ACCEPT");
          acceptFrame();
        } else {
          AFSK_LOG(" | Expected END_SYNC 0x%02X, got 0x%02X", END_SYNC, b);
          rejectFrame("MISSING_END_SYNC");
        }
        break;
    }

    AFSK_LOG("\n");
  }

  // ─── public interface used by the Mic subsystem ───────────────────────────

  // Fresh decoder state for a new listen (the old setup() init).
  static void reset() {
    markDet.init(FREQ_MARK);
    spaceDet.init(FREQ_SPACE);
    clockCounter = 0;
    lastSoft = -1;
    frameReady = false;
    resetFrameParser();
  }

  // Start a listen: drop stale audio, start collecting, fresh decoder.
  static void start() {
    queue1.clear();
    queue1.begin();
    reset();
  }

  // End a listen: stop collecting and release buffered audio blocks.
  static void stop() {
    queue1.end();
    queue1.clear();
  }

  // The old loop() body. Drains the audio that has arrived; returns true when a
  // CRC-valid frame with a payload was accepted. cmd = first payload byte.
  static bool decodeFrame(uint8_t& cmd) {
    while (queue1.available()) {
      debugBlockCount++;
      int16_t* block = queue1.readBuffer();
      float totalMark = 0, totalSpace = 0;

      for (int i = 0; i < 128; i++) {
        float x = block[i] / 32768.0f;

        float em = markDet.process(x);
        float es = spaceDet.process(x);
        totalMark += em;
        totalSpace += es;
        int softBit = (em > es) ? 1 : 0;

        int committed = clockRecovery(softBit);
        if (committed != -1) {
          debugBitCount++;

          // Update sliding 8-bit window (MSB oldest, LSB newest)
          recentBits = (uint8_t)(((recentBits << 1) | (committed & 0x01)) & 0xFF);

          // Bit-level alternating-run detection for preamble (0x55 repeated).
          if (lastCommittedBit == -1) {
            lastCommittedBit = committed;
            altBitCount = 1;
          } else {
            if (committed != lastCommittedBit) {
              altBitCount++;
              lastCommittedBit = committed;
            } else {
              // same bit twice breaks alternation; start new count from this bit
              altBitCount = 1;
              lastCommittedBit = committed;
            }
          }

          // Declare PREAMBLE when enough alternating bits were seen AND the
          // current 8-bit window equals PREAMBLE_BYTE (0x55).
          if (rxState == RX_SEARCH_PREAMBLE && altBitCount >= PREAMBLE_BITS_REQUIRED && recentBits == PREAMBLE_BYTE) {
            AFSK_LOG("[SLIDING] PREAMBLE detected (altBits=%d, recentBits=0x%02X) -> SYNC\n", altBitCount, recentBits);
            rxState = RX_SEARCH_SYNC;
            preambleMatch = PREAMBLE_LEN; // effectively matched the preamble
            assembler.reset();            // align assembler so next bits build START_SYNC
            recentBits = 0;               // start fresh for START_SYNC detection
            frameStartMs = millis();      // mark frame start time here
            continue;                     // this bit belongs to the preamble
          }

          // Sliding START_SYNC detection on every committed bit while searching for sync.
          if (rxState == RX_SEARCH_SYNC && recentBits == START_SYNC) {
            AFSK_LOG("[SLIDING] START_SYNC detected (recentBits=0x%02X) -> SIZE\n", recentBits);
            rxState = RX_READ_SIZE;
            assembler.reset();            // align assembler so subsequent bits build the size byte
            preambleMatch = 0;
            continue;                     // this bit was part of START_SYNC
          }

          uint8_t byteVal;
          if (assembler.pushBit(committed, byteVal)) {
            processByte(byteVal);
          }
        }
      }

      debugAvgMark = totalMark / 128.0f;
      debugAvgSpace = totalSpace / 128.0f;

      queue1.freeBuffer();

      if (frameReady) {
        frameReady = false;
        if (rxSize > 0) {
          cmd = rxPayload[0];             // first payload byte = command id
          return true;
        }
      }
    }
    return false;
  }

#if AFSK_DEBUG
  // Optional: call from the serial console code to see decoder counters.
  static void printStats() {
    Serial.printf("[afsk] blocks=%lu bits=%lu bytes=%lu avgMark=%.3f avgSpace=%.3f state=%s\n",
                  (unsigned long)debugBlockCount, (unsigned long)debugBitCount,
                  (unsigned long)debugByteCount, debugAvgMark, debugAvgSpace, stateNames[rxState]);
  }
#endif

}  // namespace afsk

// ============================================================================
// MIC subsystem   (LISTEN_CHIRP, COMPUTE_TDOA, LISTEN_AFSK)
// Long DSP is allowed inside update(): the loops below check their own deadline
// and feed the hardware watchdog.
// ============================================================================
constexpr int CAP_LEN = 4096;
DMAMEM static float g_capL[CAP_LEN];   // TODO: replace with your existing capture buffers
DMAMEM static float g_capR[CAP_LEN];

// ---- DSP stubs: fill these in with your dechirp code -----------------------
static void micBegin()          { /* TODO (file 4): init the dechirp's audio queue / buffers */ }
static void micFlush()          { /* TODO (file 4): discard stale dechirp audio so old samples are never decoded */ }
static bool micGetBlock()       { /* TODO: true when a new block of samples is ready */ return false; }
static bool dechirp()           {   /*
   * TODO: Complete the chirp detection pipeline:
   *   1. Rough correlation: cheaply reject non-chirp blocks.
   *   2. Mix + FFT: dechirp the signal and locate the peak.
   *   3. Fine correlation: confirm the chirp and capture samples
   *      into g_capL/g_capR, including arrival indices.
   *
   * Return true only when a valid chirp has been detected.
   */ return false; }
static bool micTdoaSeconds(float& dt) { /* TODO: cross-correlate capL/capR; dt>0 means left mic hears it first */ return false; }

// AFSK: merged. The receiver lives in namespace afsk above.
static bool afskDecodeFrame(uint8_t& cmd) { return afsk::decodeFrame(cmd); }

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
  // resets state (and makes sure the AFSK queue is not left running).
  void abort() override      { afsk::stop(); st_ = IDLE; }
  bool isIdle() const override { return st_ == IDLE; }
  void forceIdle() override  { afsk::stop(); st_ = IDLE; }

private:
  bool deadlinePassed() const { return (int32_t)(millis() - deadlineMs_) >= 0; }

  Result listenChirp() {
    micFlush();
  
    while (!deadlinePassed()) {
      feedWatchdog();
      stage_ = WAITING;
      if (!micGetBlock()) continue;
      stage_ = ROUGH;  // Diagnostics: dechirp pipeline active
      if (!dechirp()) continue;
      return Result::SUCCESS;  // Capture buffers now hold the chirp
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
    afsk::start();                         // flush stale audio, start queue, fresh decoder
    Result r = Result::TIMEOUT;
    while (!deadlinePassed()) {
      feedWatchdog();
      uint8_t cmd;
      if (afskDecodeFrame(cmd)) {
        ctx_->rxCmd = cmd;
        r = Result::SUCCESS;
        break;
      }
    }
    afsk::stop();                          // stop collecting audio nobody reads
    return r;
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
    Serial.printf("[sup] %-17s %-8s total=%lums  max_total=%lums  max_update=%luus\n",
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
          Serial.printf("[seq] %s at step %u\n", state_ == DONE ? "DONE" : "FAILED", i_);
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
  { Prim::LISTEN_CHIRP,    0,    10000, OnFail::ABORT, 0 },   // 0
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
    Serial.println("[user] EMERGENCY STOP: sequence aborted, motors commanded off");
    return;
  }

  Step requested{};
  if (!userCharToStep(ch, requested)) {
    if (ch == '?' || ch == 'h' || ch == 'H') {
      Serial.println("Commands: t=turn left 90 deg, d=drive forward 1 ft, x=emergency stop, ?=help");
    } else {
      Serial.printf("[user] Unknown command '%c'. Type ? for help.\n", ch);
    }
    return;
  }

  // The sequencer and manual control share sup.takeResult(), so stop the
  // sequencer before manual control takes ownership of that result channel.
  seq.abort();
  manualStep = requested;
  manualPending = true;       // newest motion command replaces any pending one
  manualInFlight = false;     // any previous manual action was preempted
  Serial.printf("[user] Queued manual command: %s\n",
                requested.prim == Prim::TURN ? "turn left 90 degrees" : "drive forward 1 foot");
}

// Call after sup.update(). A request can be refused during settling; retry it.
// Since seq is FAILED after manual takeover, it will not consume manual results.
static void serviceUserCommands() {
  if (manualPending && !manualInFlight) {
    if (sup.request(manualStep, &ctx)) {
      manualPending = false;
      manualInFlight = true;
      Serial.printf("[user] Accepted by Superstructure: %s\n",
                    manualStep.prim == Prim::TURN ? "TURN" : "DRIVE");
    }
  }

  if (manualInFlight) {
    Result r = sup.takeResult();
    if (r != Result::RUNNING) {
      Serial.printf("[user] Manual command finished: %s\n", RESULT_NAME[(int)r]);
      manualInFlight = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  AudioMemory(cfg::AUDIO_MEMORY_BLOCKS);           // once, before any Audio object is used

  sup.begin();                                     // init pins, gyro calibration, audio in/out
  seq.load(SEQ, sizeof(SEQ) / sizeof(SEQ[0]), &ctx);

#if HAVE_WDT
  WDT_timings_t wdtCfg;
  wdtCfg.timeout = cfg::WDT_TIMEOUT_S;             // reset if nothing feeds it for this long
  g_wdt.begin(wdtCfg);
#endif

  seq.start();
  Serial.println("Robot ready. Serial commands: t=left 90 deg, d=forward 1 ft, x=stop, ?=help");
}

void loop() {
  feedWatchdog();

  // 1) Read user input first so manual commands can preempt autonomous work.
  while (Serial.available() > 0) {
    onUserCommand((char)Serial.read());
  }

  // 2) Advance the currently active primitive / settling timer.
  sup.update();

  // 3) Start/retry manual work and collect its result through the same gate.
  serviceUserCommands();

  // 4) Advance the autonomous sequence. After manual takeover it stays FAILED.
  seq.update();

  // Other input sources should follow the same pattern: decode -> sup.request()
  // for work, or sup.emergencyStop() for an emergency.
}
