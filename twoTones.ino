
#include <Audio.h>

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

void setup() {
  AudioMemory(10);
  fskOsc.begin(WAVEFORM_SINE);
  fskOsc.amplitude(0.5);
  
  Serial.begin(9600);
  Serial.println("Bell 202 FSK: Sending 01010101 Pattern");
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
