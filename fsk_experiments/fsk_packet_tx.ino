#include <Audio.h>

// ─────────────────────────────────────────────────────────────────────────────
// FSK Packet TX
// Frame format:
//   [4-byte preamble][1-byte start sync][1-byte size][payload][2-byte CRC][1-byte end sync]
// ─────────────────────────────────────────────────────────────────────────────

AudioSynthWaveform   fskOsc;
AudioOutputI2S       i2s1;
AudioConnection      patchCord1(fskOsc, 0, i2s1, 0);
AudioConnection      patchCord2(fskOsc, 0, i2s1, 1);

// FSK tones
//#define FREQ_MARK   15000
//#define FREQ_SPACE  17000

//audible tones
#define FREQ_MARK   6000
#define FREQ_SPACE  8000

// Bit rate
#define BAUD_RATE   100
#define BIT_PERIOD_US (1000000UL / BAUD_RATE)

// Packet bytes
static const uint8_t PREAMBLE_BYTE = 0x55;
static const uint8_t START_SYNC    = 0x7E;
static const uint8_t END_SYNC      = 0x7F;
static const uint8_t PREAMBLE_LEN  = 4;
static const uint8_t MAX_PAYLOAD   = 254;

// Example payload
const uint8_t demoPayload[] = {
  'H', 'e', 'l', 'l', 'o', ',', ' ', 'F', 'S', 'K'
};
const uint8_t demoPayloadLen = sizeof(demoPayload);

uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void sendBit(uint8_t bit) {
  fskOsc.frequency(bit ? FREQ_MARK : FREQ_SPACE);
  delayMicroseconds(BIT_PERIOD_US);
}

void sendByte(uint8_t value) {
  for (int i = 7; i >= 0; i--) {
    sendBit((value >> i) & 0x01);
  }
}

void sendPreamble() {
  for (uint8_t i = 0; i < PREAMBLE_LEN; i++) {
    sendByte(PREAMBLE_BYTE);
  }
}

void sendFrame(const uint8_t* payload, uint8_t len) {
  if (len > MAX_PAYLOAD) return;

  uint8_t crcInput[1 + MAX_PAYLOAD];
  crcInput[0] = len;
  for (uint8_t i = 0; i < len; i++) {
    crcInput[1 + i] = payload[i];
  }

  uint16_t crc = crc16_ccitt_false(crcInput, 1 + len);

  // Print frame start with timestamp
  uint32_t t0 = millis();
  Serial.printf("FRAME_START: TX size=%d start=%lu ms\n", len, t0);

  sendPreamble();
  sendByte(START_SYNC);
  sendByte(len);

  for (uint8_t i = 0; i < len; i++) {
    sendByte(payload[i]);
  }

  sendByte((crc >> 8) & 0xFF);
  sendByte(crc & 0xFF);
  sendByte(END_SYNC);

  // Print frame end with elapsed time
  uint32_t elapsed = millis() - t0;
  Serial.printf("FRAME_END: TX size=%d elapsed=%lu ms\n", len, elapsed);
}

void setup() {
  AudioMemory(16);
  fskOsc.begin(WAVEFORM_SINE);
  fskOsc.amplitude(0.1f);

  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("FSK Packet TX ready");
}

void loop() {
  sendFrame(demoPayload, demoPayloadLen);
  delay(1000);
}