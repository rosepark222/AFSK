#include <Audio.h>
#include <arm_math.h>

// ─────────────────────────────────────────────────────────────────────────────
// FSK Packet RX
// Frame format:
//   [4-byte preamble][1-byte start sync][1-byte size][payload][2-byte CRC][1-byte end sync]
// RX continuously demodulates and performs clock recovery.
// ─────────────────────────────────────────────────────────────────────────────

enum DemodMethod { DEMOD_GOERTZEL_IIR, DEMOD_FFT_SLIDING };
const DemodMethod DEMOD_SELECT = DEMOD_GOERTZEL_IIR;

// FSK tones
#define FREQ_MARK   15000.0f
#define FREQ_SPACE  17000.0f

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

// ─── Clock recovery ──────────────────────────────────────────────────────────
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

// ─── CRC ─────────────────────────────────────────────────────────────────────
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

RxState rxState = RX_SEARCH_PREAMBLE;
uint8_t preambleMatch = 0;
uint8_t sizeByte = 0;
uint8_t payload[MAX_PAYLOAD];
uint8_t payloadIndex = 0;
uint16_t rxCrc = 0;
uint8_t rxCrcHi = 0;
uint32_t frameStartMs = 0;

static const uint32_t FRAME_TIMEOUT_MS = 500;

void resetFrameParser() {
  rxState = RX_SEARCH_PREAMBLE;
  preambleMatch = 0;
  sizeByte = 0;
  payloadIndex = 0;
  rxCrc = 0;
  rxCrcHi = 0;
  assembler.reset();
  frameStartMs = millis();
}

void rejectFrame(const char* reason) {
  Serial.print("FRAME_REJECT: ");
  Serial.println(reason);
  resetFrameParser();
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

  resetFrameParser();
}

void processByte(uint8_t b) {
  if (millis() - frameStartMs > FRAME_TIMEOUT_MS) {
    rejectFrame("TIMEOUT");
    return;
  }

  switch (rxState) {
    case RX_SEARCH_PREAMBLE:
      if (b == PREAMBLE_BYTE) {
        preambleMatch++;
        if (preambleMatch >= PREAMBLE_LEN) {
          rxState = RX_SEARCH_SYNC;
        }
      } else {
        preambleMatch = 0;
      }
      break;

    case RX_SEARCH_SYNC:
      if (b == START_SYNC) {
        rxState = RX_READ_SIZE;
      } else if (b != PREAMBLE_BYTE) {
        preambleMatch = 0;
        rxState = RX_SEARCH_PREAMBLE;
      }
      break;

    case RX_READ_SIZE:
      sizeByte = b;
      if (sizeByte > MAX_PAYLOAD) {
        rejectFrame("BAD_SIZE");
        return;
      }
      payloadIndex = 0;
      if (sizeByte == 0) {
        rxState = RX_READ_CRC_HI;
      } else {
        rxState = RX_READ_PAYLOAD;
      }
      break;

    case RX_READ_PAYLOAD:
      payload[payloadIndex++] = b;
      if (payloadIndex >= sizeByte) {
        rxState = RX_READ_CRC_HI;
      }
      break;

    case RX_READ_CRC_HI:
      rxCrcHi = b;
      rxState = RX_READ_CRC_LO;
      break;

    case RX_READ_CRC_LO:
      rxCrc = ((uint16_t)rxCrcHi << 8) | b;
      rxState = RX_READ_END_SYNC;
      break;

    case RX_READ_END_SYNC:
      if (b == END_SYNC) {
        acceptFrame();
      } else {
        rejectFrame("MISSING_END_SYNC");
      }
      break;
  }
}

// ─── Setup/Loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  AudioMemory(16);
  queue1.begin();

  markDet.init(FREQ_MARK);
  spaceDet.init(FREQ_SPACE);

  resetFrameParser();

  Serial.println("FSK Packet RX ready");
}

void loop() {
  if (queue1.available()) {
    int16_t* block = queue1.readBuffer();

    for (int i = 0; i < 128; i++) {
      float x = block[i] / 32768.0f;
      int softBit = -1;

      if (DEMOD_SELECT == DEMOD_GOERTZEL_IIR) {
        float em = markDet.process(x);
        float es = spaceDet.process(x);
        softBit = (em > es) ? 1 : 0;
      }

      int committed = clockRecovery(softBit);
      if (committed != -1) {
        uint8_t byteVal;
        if (assembler.pushBit(committed, byteVal)) {
          processByte(byteVal);
        }
      }
    }

    queue1.freeBuffer();
  }
}