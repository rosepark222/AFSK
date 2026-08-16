#include <Audio.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>

// Audio Objects
AudioSynthWaveform   fskOsc;
AudioOutputI2S       i2s1;
AudioConnection      patchCord1(fskOsc, 0, i2s1, 0);
AudioConnection      patchCord2(fskOsc, 0, i2s1, 1);

// Bell 202 Standards
//#define FREQ_MARK  1200  // Binary 1
//#define FREQ_SPACE 2200  // Binary 0

//#define FREQ_MARK  16000  // Binary 1
//#define FREQ_SPACE 20000  // Binary 0

//#define FREQ_MARK  6000  // Binary 1
//#define FREQ_SPACE 8000  // Binary 0

#define FREQ_MARK  15000  // Binary 1
#define FREQ_SPACE 17000  // Binary 0

#define BAUD_RATE   100 // 1200  // Bits per second
#define BIT_PERIOD (1000000 / BAUD_RATE)

// CSV dump parameters
#define DUMP_DURATION_MS 30
#define SAMPLE_RATE 44100  // samples per second for CSV (use 44100 for easy import)
#define CSV_FILENAME "/fsk_dump.csv"

void setup() {
  AudioMemory(10);
  fskOsc.begin(WAVEFORM_SINE);
  fskOsc.amplitude(0.5);

  Serial.begin(9600);
  while (!Serial && millis() < 2000) ; // wait a short while for serial on some boards
  Serial.println("Bell 202 FSK: Sending 01010101 Pattern");

  // Check SD card setup and dump 30 ms of generated wave
  if (!initSD()) {
    Serial.println("SD card initialization failed — skipping CSV dump.");
  } else {
    if (dumpWaveToSD()) {
      Serial.println("CSV dump complete: " CSV_FILENAME);
    } else {
      Serial.println("CSV dump failed.");
    }
  }
}

uint32_t nextBitTime = 0;


void sendBit(int bit) {
  if (nextBitTime == 0) {
    nextBitTime = micros();
    return;
  }

  if (bit == 1) fskOsc.frequency(FREQ_MARK);
  else          fskOsc.frequency(FREQ_SPACE);

  nextBitTime += BIT_PERIOD;
  while (micros() < nextBitTime);
}


void loop() {
  int bitCount = 4;

  for (int i = 0; i < bitCount*8; i++) {
    int currentBit = i % 2; // Alternates 0, 1, 0, 1...
    sendBit(currentBit);
  }
}


// ----- SD + CSV dumping helpers -----

bool initSD() {
  Serial.print("Initializing SD card...");
#ifdef BUILTIN_SDCARD
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(" failed (BUILTIN_SDCARD)");
    return false;
  }
#else
  if (!SD.begin()) {
    Serial.println(" failed");
    return false;
  }
#endif
  Serial.println(" ok");
  return true;
}

bool dumpWaveToSD() {
  // Open file for writing (overwrite if exists)
  if (SD.exists(CSV_FILENAME)) {
    SD.remove(CSV_FILENAME);
  }
  File csv = SD.open(CSV_FILENAME, FILE_WRITE);
  if (!csv) {
    Serial.println("Failed to open CSV file for writing");
    return false;
  }

  // Write header
  csv.println("sample_index,time_ms,sample");

  const unsigned long totalSamples = (unsigned long)SAMPLE_RATE * DUMP_DURATION_MS / 1000UL;
  const float amp = 0.5f; // must match fskOsc amplitude
  const float samplePeriodUs = 1000000.0f / (float)SAMPLE_RATE; // microseconds per sample

  // Start time at t=0, we'll assume the transmitted bit pattern starts with MARK (1)
  // This emulates the same frequencies set to fskOsc during real transmit
  for (unsigned long n = 0; n < totalSamples; ++n) {
    float t_us = n * samplePeriodUs; // time since start in microseconds
    unsigned long bitIndex = (unsigned long)(t_us / (float)BIT_PERIOD);
    int bit = (bitIndex % 2 == 0) ? 1 : 0; // start with MARK on bitIndex==0
    float freq = (bit == 1) ? (float)FREQ_MARK : (float)FREQ_SPACE;

    float t_s = t_us / 1e6f;
    float sample = amp * sinf(2.0f * M_PI * freq * t_s);

    // Write CSV line: index, time_ms, sample_value
    float time_ms = t_us / 1000.0f;
    csv.print(n);
    csv.print(',');
    csv.print(time_ms, 6);
    csv.print(',');
    csv.println(sample, 6);

    // Occasionally yield to avoid watchdog issues on some platforms
    if ((n & 0x1FF) == 0) {
      csv.flush();
      delay(0);
    }
  }

  csv.close();
  return true;
}
