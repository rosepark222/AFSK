// ============================================================
// TDOA Detector — Dual SPH0645LM4H I2S Microphones on Teensy 4.1
// Mark: 6000 Hz | Space: 8000 Hz | Baud: 100
// Detects preamble (0x55 alternating pattern) on both channels
// Measures time delay of arrival (TDOA) between mics
// Output: Direction in degrees (-90 to +90)
//   0° = sound in center (equidistant from both mics)
//   +90° = sound at LEFT mic
//   -90° = sound at RIGHT mic
// ============================================================

#include <Audio.h>
#include <arm_math.h>

// ── Audio graph ─────────────────────────────────────────────
AudioInputI2S     i2s_in;
AudioRecordQueue  queue_left;
AudioRecordQueue  queue_right;

// Left channel (I2S input channel 0)
AudioConnection   c1(i2s_in, 0, queue_left,  0);

// Right channel (I2S input channel 1)
AudioConnection   c2(i2s_in, 1, queue_right, 0);

// ── Config ───────────────────────────────────────────────────
static const float SAMPLE_RATE  = 44100.0f;
static const float MARK_HZ      = 6000.0f;
static const float SPACE_HZ     = 8000.0f;
static const int   BLOCK_SIZE   = 128;

static const uint8_t PREAMBLE_BYTE = 0x55;  // 01010101 — alternating pattern
static const uint8_t PREAMBLE_LEN  = 4;
static const int PREAMBLE_BITS_REQUIRED = PREAMBLE_LEN * 8;  // 32 bits of alternation

static const int MAX_TDOA_SAMPLES = 13;  // Max direction range: -13 to +13 samples
static const float MAX_ANGLE = 90.0f;    // Max angle: ±90 degrees

// ── Time window validation ──────────────────────────────────
// For 4-inch (10cm) mic spacing, max realistic TDOA is ~9.5 samples (~2.2ms)
// We use 5 milliseconds as safety threshold for same-event detection
static const uint32_t MAX_TDOA_TIME_WINDOW_US = 5000;  // microseconds (~2.3ms for 4-inch spacing)

// ── I/Q tone detector ────────────────────────────────────────
struct IQDetector {
  float phaseInc;
  float phase;
  float lpf_I;
  float lpf_Q;

  void init(float freq) {
    phaseInc = 2.0f * M_PI * freq / SAMPLE_RATE;
    phase = 0.0f;
    lpf_I = 0.0f;
    lpf_Q = 0.0f;
  }

  float process(float x) {
    float c = cosf(phase);
    float s = sinf(phase);

    float i_raw = x * c;
    float q_raw = x * s;

    // LPF for envelope
    float lpf_alpha = 1.0f - expf(-2.0f * M_PI * (2 * 100) / SAMPLE_RATE);  // 2 * BAUD_RATE
    lpf_I += lpf_alpha * (i_raw - lpf_I);
    lpf_Q += lpf_alpha * (q_raw - lpf_Q);

    phase += phaseInc;
    if (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;

    return lpf_I * lpf_I + lpf_Q * lpf_Q;
  }
};

IQDetector markDet_L, spaceDet_L;
IQDetector markDet_R, spaceDet_R;

// ── Clock recovery (per channel) ────────────────────────────
struct ClockRecovery {
  const int SAMPLES_PER_BIT = (int)(SAMPLE_RATE / 100);  // BAUD_RATE = 100
  int clockCounter = 0;
  int lastSoft = -1;

  int process(int softBit) {
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

  void reset() {
    clockCounter = 0;
    lastSoft = -1;
  }
};

ClockRecovery clockRec_L, clockRec_R;

// ── Preamble detection state (per channel) ──────────────────
struct PreambleDetector {
  int lastCommittedBit = -1;
  int altBitCount = 0;
  uint8_t recentBits = 0;
  uint32_t preambleDetectedTimeUs = 0;  // Wall-clock timestamp (microseconds) when preamble was detected
  bool preambleFound = false;

  void reset() {
    lastCommittedBit = -1;
    altBitCount = 0;
    recentBits = 0;
    preambleDetectedTimeUs = 0;
    preambleFound = false;
  }

  // Returns true if preamble just detected in this call
  // wallClockTimeUs: Wall-clock time in microseconds (same reference for both channels)
  bool processBit(int committed, uint32_t wallClockTimeUs) {
    if (preambleFound) return false;  // Already found, don't detect again

    // Update sliding 8-bit window (MSB oldest, LSB newest)
    recentBits = (uint8_t)(((recentBits << 1) | (committed & 0x01)) & 0xFF);

    // Track alternating bit run
    if (lastCommittedBit == -1) {
      lastCommittedBit = committed;
      altBitCount = 1;
    } else {
      if (committed != lastCommittedBit) {
        altBitCount++;
        lastCommittedBit = committed;
      } else {
        // Same bit twice breaks alternation
        altBitCount = 1;
        lastCommittedBit = committed;
      }
    }

    // Preamble detected: enough alternating bits + window matches 0x55
    if (altBitCount >= PREAMBLE_BITS_REQUIRED && recentBits == PREAMBLE_BYTE) {
      preambleDetectedTimeUs = wallClockTimeUs;  // Store wall-clock timestamp
      preambleFound = true;
      return true;
    }

    return false;
  }
};

PreambleDetector preambleDet_L, preambleDet_R;

// ── Global state ─────────────────────────────────────────────
uint32_t blockCount = 0;

// ── Helper function to convert time delay to degrees ──────
// For 4-inch (10cm) spacing: max delay ≈ 0.296 ms (290 µs) = ±90°
float timeDeltaToDegrees(int32_t timeDelta_us) {
  // Sound speed: 343 m/s = 343e-6 m/µs
  // 4 inches = 0.1016 m
  // Max delay: 0.1016 / 343e-6 ≈ 296 µs
  static const float MAX_TIME_DELTA_US = 300.0f;  // µs, approximately ±90°
  return (float)timeDelta_us * (MAX_ANGLE / MAX_TIME_DELTA_US);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  AudioMemory(40);
  queue_left.begin();
  queue_right.begin();

  markDet_L.init(MARK_HZ);
  spaceDet_L.init(SPACE_HZ);
  markDet_R.init(MARK_HZ);
  spaceDet_R.init(SPACE_HZ);

  Serial.println("\n========================================");
  Serial.println(" TDOA Detector — Dual Mic");
  Serial.println(" Teensy 4.1 + 2x SPH0645LM4H I2S Mics");
  Serial.println("========================================");
  Serial.printf("MARK=%d Hz, SPACE=%d Hz\n", (int)MARK_HZ, (int)SPACE_HZ);
  Serial.printf("Sample Rate=%d Hz\n", (int)SAMPLE_RATE);
  Serial.printf("Time window: ±%d µs (for 4-inch spacing)\n", MAX_TDOA_TIME_WINDOW_US);
  Serial.println("Direction range: ±90° (based on sound arrival time)\n");
  Serial.println("Waiting for preamble...\n");
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
  bool left_ready = queue_left.available();
  bool right_ready = queue_right.available();

  if (!left_ready || !right_ready) return;  // Wait for both buffers

  blockCount++;
  int16_t* block_L = queue_left.readBuffer();
  int16_t* block_R = queue_right.readBuffer();

  // Process each sample in the block
  for (int i = 0; i < BLOCK_SIZE; i++) {
    // ── WALL-CLOCK TIME REFERENCE ──
    // Get current microseconds (same for both channels at this iteration)
    uint32_t wallClockTimeUs = micros();

    // ── LEFT CHANNEL ──
    float x_L = block_L[i] / 32768.0f;
    float em_L = markDet_L.process(x_L);
    float es_L = spaceDet_L.process(x_L);
    int softBit_L = (em_L > es_L) ? 1 : 0;
    int committed_L = clockRec_L.process(softBit_L);

    if (committed_L != -1) {
      // Pass wall-clock timestamp (same reference for both channels)
      bool preamble_L_detected = preambleDet_L.processBit(committed_L, wallClockTimeUs);
      if (preamble_L_detected) {
        Serial.printf("[LEFT]  Preamble detected at time: %lu µs\n", preambleDet_L.preambleDetectedTimeUs);
      }
    }

    // ── RIGHT CHANNEL ──
    float x_R = block_R[i] / 32768.0f;
    float em_R = markDet_R.process(x_R);
    float es_R = spaceDet_R.process(x_R);
    int softBit_R = (em_R > es_R) ? 1 : 0;
    int committed_R = clockRec_R.process(softBit_R);

    if (committed_R != -1) {
      // Pass SAME wall-clock timestamp reference
      bool preamble_R_detected = preambleDet_R.processBit(committed_R, wallClockTimeUs);
      if (preamble_R_detected) {
        Serial.printf("[RIGHT] Preamble detected at time: %lu µs\n", preambleDet_R.preambleDetectedTimeUs);
      }
    }

    // ── Check if both preambles detected, calculate TDOA ──
    if (preambleDet_L.preambleFound && preambleDet_R.preambleFound) {
      // ── TIME WINDOW VALIDATION ──
      // Check if detections are within realistic time window for 4-inch mic spacing
      int32_t time_delta_us = (int32_t)(preambleDet_L.preambleDetectedTimeUs - preambleDet_R.preambleDetectedTimeUs);
      uint32_t abs_time_delta = (uint32_t)abs(time_delta_us);
      
      if (abs_time_delta > MAX_TDOA_TIME_WINDOW_US) {
        // Detections are too far apart in time — they're from different events
        Serial.printf("[REJECT] Time difference too large: %lu µs (threshold: %lu µs)\n", 
                      abs_time_delta, MAX_TDOA_TIME_WINDOW_US);
        Serial.println("         Likely two separate transmissions, not TDOA\n");
        
        // Reset and wait for next detection
        preambleDet_L.reset();
        preambleDet_R.reset();
        clockRec_L.reset();
        clockRec_R.reset();
      } else {
        // Valid TDOA detection
        // Direction = LEFT_time - RIGHT_time
        // Positive = sound arrived at LEFT mic first (source on left)
        // Negative = sound arrived at RIGHT mic first (source on right)
        int32_t time_delta = (int32_t)(preambleDet_L.preambleDetectedTimeUs - preambleDet_R.preambleDetectedTimeUs);
        
        // Convert to degrees
        float direction_degrees = timeDeltaToDegrees(time_delta);
        
        // Clamp to ±90°
        if (direction_degrees > MAX_ANGLE) direction_degrees = MAX_ANGLE;
        if (direction_degrees < -MAX_ANGLE) direction_degrees = -MAX_ANGLE;
        
        Serial.println("\n========================================");
        Serial.printf("PREAMBLE DETECTED (Wall-Clock TDOA):\n");
        Serial.printf("  LEFT  detection time:   %lu µs\n", preambleDet_L.preambleDetectedTimeUs);
        Serial.printf("  RIGHT detection time:   %lu µs\n", preambleDet_R.preambleDetectedTimeUs);
        Serial.printf("  Time difference:        %ld µs\n", time_delta);
        Serial.printf("  DIRECTION: %.1f degrees\n", direction_degrees);
        Serial.printf("  (Positive = LEFT mic first, Negative = RIGHT mic first)\n");
        Serial.println("========================================\n");

        // Reset for next packet
        preambleDet_L.reset();
        preambleDet_R.reset();
        clockRec_L.reset();
        clockRec_R.reset();
      }
    }
  }

  queue_left.freeBuffer();
  queue_right.freeBuffer();
}
