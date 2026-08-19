#include <Audio.h>
#include <arm_math.h>

// ────────────────────────────────────────────────────────────────────
// FSK Packet RX
// Frame format:
//   [4-byte preamble][1-byte start sync][1-byte size][payload][2-byte CRC][1-byte end sync]
// RX continuously demodulates and performs clock recovery.
// ────────────────────────────────────────────────────────────────────

enum DemodMethod { DEMOD_GOERTZEL_IIR, DEMOD_FFT_SLIDING };
const DemodMethod DEMOD_SELECT = DEMOD_GOERTZEL_IIR;

// FSK tones
//#define FREQ_MARK   15000.0f
//#define FREQ_SPACE  17000.0f

//audiable range
#define FREQ_MARK   6000.0f
#define FREQ_SPACE  8000.0f

#define BAUD_RATE   100
#define SAMPLE_RATE 44100.0f

// Envelope LPF for I/Q detector
#define LPF_FC      (2 * BAUD_RATE)
const float LPF_ALPHA = 1.0f - expf(-2.0f * PI * LPF_FC / SAMPLE_RATE);

// Audio input
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;
AudioConnection   patchCord1(i2s_in, 0, queue1, 0);

// Packet bytes
static const uint8_t PREAMBLE_BYTE = 0x55;
static const uint8_t START_SYNC    = 0x7E;
static const uint8_t END_SYNC      = 0x7F;
static const uint8_t PREAMBLE_LEN  = 4;
static const uint8_t MAX_PAYLOAD   = 254;

// ─── DEBUG COUNTERS ───────────────────────────────────────────────────────
uint32_t debugBitCount = 0;
uint32_t debugByteCount = 0;
uint32_t debugBlockCount = 0;
uint32_t lastDebugPrintMs = 0;
float debugAvgMark = 0;
float debugAvgSpace = 0;

// ─── I/Q tone detector ───────────────────────────────────────────────────────
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

IQDetector markDet, spaceDet;

// ─── Clock recovery ────────────────────────────────────────────────────────
const int SAMPLES_PER_BIT = (int)(SAMPLE_RATE / BAUD_RATE);
int clockCounter = 0;
int lastSoft = -1;

int clockRecovery(int softBit) {
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

// ─── CRC ─────────────────────────────────────────────────────────────
uint16_t crc16_ccitt_false_update(uint16_t crc, uint8_t data) {
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

uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc = crc16_ccitt_false_update(crc, data[i]);
  }
  return crc;
}

// ─── Bit-to-byte assembly ────────────────────────────────────────────────────
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

BitAssembler assembler;

// ─── Packet state machine ────────────────────────────────────────────────────
enum RxState {
  RX_SEARCH_PREAMBLE,
  RX_SEARCH_SYNC,
  RX_READ_SIZE,
  RX_READ_PAYLOAD,
  RX_READ_CRC_HI,
  RX_READ_CRC_LO,
  RX_READ_END_SYNC
};

const char* stateNames[] = {
  "PREAMBLE",
  "SYNC",
  "SIZE",
  "PAYLOAD",
  "CRC_HI",
  "CRC_LO",
  "END_SYNC"
};

RxState rxState = RX_SEARCH_PREAMBLE;
uint8_t preambleMatch = 0;
uint8_t sizeByte = 0;
uint8_t payload[MAX_PAYLOAD];
uint8_t payloadIndex = 0;
uint16_t rxCrc = 0;
uint8_t rxCrcHi = 0;
uint32_t frameStartMs = 0;

static const uint32_t FRAME_TIMEOUT_MS = 5000;  // the packet timeout 

// Sliding 8-bit window of most recent committed bits (MSB oldest, LSB newest).
// Used to detect START_SYNC on any bit boundary.
uint8_t recentBits = 0;

// For preamble (0x55 repeated) detection at bit level:
// track last committed bit and count of consecutive alternating bits seen.
int lastCommittedBit = -1;
int altBitCount = 0;
const int PREAMBLE_BITS_REQUIRED = PREAMBLE_LEN * 8;

void resetFrameParser() {
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
  Serial.printf("[RESET] State -> %s, frameStartMs=%lu\n", stateNames[rxState], frameStartMs);
}

void rejectFrame(const char* reason) {
  uint32_t elapsed = millis() - frameStartMs;
  Serial.printf("FRAME_REJECT: %s (State: %s, elapsed: %lu ms, bytes: %lu)\n", 
    reason, stateNames[rxState], elapsed, debugByteCount);
  resetFrameParser();
}

void blinkPacketOK() {
  digitalWriteFast(LED_BUILTIN, HIGH);
  delay(80);
  digitalWriteFast(LED_BUILTIN, LOW);
}

void acceptFrame() {
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

  Serial.print("FRAME_OK size=");
  Serial.print(sizeByte);
  Serial.print(" payload=\"");
  for (uint8_t i = 0; i < sizeByte; i++) {
    char c = (char)payload[i];
    if (c >= 32 && c <= 126) Serial.print(c);
    else Serial.print('.');
  }
  Serial.println("\"");

  blinkPacketOK();
  resetFrameParser();
}

void processByte(uint8_t b) {
  if (millis() - frameStartMs > FRAME_TIMEOUT_MS) {
    rejectFrame("TIMEOUT");
    return;
  }

  debugByteCount++;
  
  // Debug: Print every byte received
  Serial.printf("[BYTE %lu] 0x%02X (%3d) | State: %s", debugByteCount, b, b, stateNames[rxState]);

  switch (rxState) {
    case RX_SEARCH_PREAMBLE:
      if (b == PREAMBLE_BYTE) {
        preambleMatch++;
        Serial.printf(" | Preamble match %d/%d", preambleMatch, PREAMBLE_LEN);
        if (preambleMatch >= PREAMBLE_LEN) {
          rxState = RX_SEARCH_SYNC;
          Serial.printf(" -> Moving to SYNC");
        }
      } else {
        if (preambleMatch > 0) {
          Serial.printf(" | Preamble reset (was %d)", preambleMatch);
        }
        preambleMatch = 0;
      }
      break;

    case RX_SEARCH_SYNC:
      if (b == START_SYNC) {
        rxState = RX_READ_SIZE;
        Serial.printf(" | START_SYNC found -> SIZE");
      } else if (b != PREAMBLE_BYTE) {
        preambleMatch = 0;
        rxState = RX_SEARCH_PREAMBLE;
        Serial.printf(" | Not sync, back to PREAMBLE");
      }
      break;

    case RX_READ_SIZE:
      sizeByte = b;
      if (sizeByte > MAX_PAYLOAD) {
        Serial.printf(" | BAD_SIZE");
        rejectFrame("BAD_SIZE");
        return;
      }
      payloadIndex = 0;
      if (sizeByte == 0) {
        rxState = RX_READ_CRC_HI;
        Serial.printf(" | Size=0 -> CRC_HI");
      } else {
        rxState = RX_READ_PAYLOAD;
        Serial.printf(" | Size=%d -> PAYLOAD", sizeByte);
      }
      break;

    case RX_READ_PAYLOAD:
      payload[payloadIndex++] = b;
      Serial.printf(" | Payload[%d]", payloadIndex - 1);
      if (payloadIndex >= sizeByte) {
        rxState = RX_READ_CRC_HI;
        Serial.printf(" -> CRC_HI");
      }
      break;

    case RX_READ_CRC_HI:
      rxCrcHi = b;
      rxState = RX_READ_CRC_LO;
      Serial.printf(" | CRC_HI=0x%02X -> CRC_LO", rxCrcHi);
      break;

    case RX_READ_CRC_LO:
      rxCrc = ((uint16_t)rxCrcHi << 8) | b;
      rxState = RX_READ_END_SYNC;
      Serial.printf(" | CRC_LO, full CRC=0x%04X -> END_SYNC", rxCrc);
      break;

    case RX_READ_END_SYNC:
      if (b == END_SYNC) {
        Serial.printf(" | END_SYNC found -> ACCEPT");
        acceptFrame();
      } else {
        Serial.printf(" | Expected END_SYNC 0x%02X, got 0x%02X", END_SYNC, b);
        rejectFrame("MISSING_END_SYNC");
      }
      break;
  }
  
  Serial.println();
}

// ─── Setup/Loop ─────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  AudioMemory(16);
  queue1.begin();

  markDet.init(FREQ_MARK);
  spaceDet.init(FREQ_SPACE);

  resetFrameParser();

  Serial.println("\n=== FSK Packet RX ready ===");
  Serial.printf("FREQ_MARK=%d Hz, FREQ_SPACE=%d Hz\n", (int)FREQ_MARK, (int)FREQ_SPACE);
  Serial.printf("BAUD_RATE=%d, SAMPLE_RATE=%d\n", BAUD_RATE, (int)SAMPLE_RATE);
  Serial.printf("SAMPLES_PER_BIT=%d\n", SAMPLES_PER_BIT);
  Serial.printf("FRAME_TIMEOUT_MS=%d\n", FRAME_TIMEOUT_MS);
  Serial.println("=== Waiting for frames... ===\n");
}

void loop() {
  if (queue1.available()) {
    debugBlockCount++;
    int16_t* block = queue1.readBuffer();
    float totalMark = 0, totalSpace = 0;

    for (int i = 0; i < 128; i++) {
      float x = block[i] / 32768.0f;
      int softBit = -1;

      if (DEMOD_SELECT == DEMOD_GOERTZEL_IIR) {
        float em = markDet.process(x);
        float es = spaceDet.process(x);
        totalMark += em;
        totalSpace += es;
        softBit = (em > es) ? 1 : 0;
      }

      int committed = clockRecovery(softBit);
      if (committed != -1) {
        debugBitCount++;

        // Update sliding 8-bit window (MSB oldest, LSB newest)
        recentBits = (uint8_t)(((recentBits << 1) | (committed & 0x01)) & 0xFF);

        // --- Bit-level alternating-run detection for preamble (0x55 repeated) ---
        // Track alternation of bits: 0x55 is an alternating pattern (01010101).
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

        // Only declare PREAMBLE when:
        //  - we've seen enough alternating bits, AND
        //  - the current 8-bit sliding window equals PREAMBLE_BYTE (0x55)
        if (rxState == RX_SEARCH_PREAMBLE && altBitCount >= PREAMBLE_BITS_REQUIRED && recentBits == PREAMBLE_BYTE) {
          Serial.printf("[SLIDING] PREAMBLE detected (altBits=%d, recentBits=0x%02X) -> SYNC\n", altBitCount, recentBits);
          rxState = RX_SEARCH_SYNC;
          preambleMatch = PREAMBLE_LEN; // indicate we've effectively matched the preamble
          assembler.reset();            // align assembler so next bits build START_SYNC
          recentBits = 0;               // start fresh for START_SYNC detection
          frameStartMs = millis();      // mark frame start time here
          // Do NOT push this bit into assembler because it belongs to the preamble
          continue;
        }

        // Sliding START_SYNC detection: check on every committed bit while searching for sync.
        // If detected, align assembler so next bits form SIZE byte.
        if (rxState == RX_SEARCH_SYNC && recentBits == START_SYNC) {
          Serial.printf("[SLIDING] START_SYNC detected (recentBits=0x%02X) -> SIZE\n", recentBits);
          rxState = RX_READ_SIZE;
          assembler.reset();   // align assembler so subsequent bits build the size byte
          // Reset preambleMatch so future preamble failures are handled normally
          preambleMatch = 0;
          // Do NOT push this committed bit into assembler because it was part of START_SYNC
          continue;
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

    // Print stats every 2 seconds
    uint32_t now = millis();
    if (now - lastDebugPrintMs >= 2000) {
      lastDebugPrintMs = now;
      Serial.printf("\n[STATS] Blocks: %lu | Bits: %lu | Bytes: %lu | AvgMark: %.3f | AvgSpace: %.3f | State: %s\n\n",
        debugBlockCount, debugBitCount, debugByteCount, debugAvgMark, debugAvgSpace, stateNames[rxState]);
    }
  }
}