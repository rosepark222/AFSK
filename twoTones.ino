
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

#define FREQ_MARK  6000  // Binary 1
#define FREQ_SPACE 8000  // Binary 0

#define BAUD_RATE   300 // 1200  // Bits per second
#define BIT_PERIOD (1000000 / BAUD_RATE)  

void setup() {
  AudioMemory(10);
  fskOsc.begin(WAVEFORM_SINE);
  fskOsc.amplitude(0.1);
  
  Serial.begin(9600);
  Serial.println("Bell 202 FSK: Sending 01010101 Pattern");
}

void sendBit(int bit) {
  // Switch frequency based on bit
  if (bit == 1) {
    fskOsc.frequency(FREQ_MARK);
  } else {
    fskOsc.frequency(FREQ_SPACE);
  }
  
  // Wait for the duration of exactly one bit
  delayMicroseconds(BIT_PERIOD);
}

void loop() {
  // Send a 10-bit pattern of alternating 0s and 1s
  for (int i = 0; i < 10; i++) {
    int currentBit = i % 2; // Alternates 0, 1, 0, 1...
    sendBit(currentBit);
  }
  
  // Optional: Long Mark (1) between bursts to stabilize receiver
  //fskOsc.frequency(FREQ_MARK);
  //delay(100); 
}

// #include <Audio.h>

// AudioSynthWaveform osc;
// AudioEffectEnvelope env;
// AudioOutputI2S i2s1;

// AudioConnection patchCord1(osc, env);
// AudioConnection patchCord2(env, 0, i2s1, 0);
// AudioConnection patchCord3(env, 0, i2s1, 1);

// // Notes
// #define E7 2637
// #define C7 2093
// #define G7 3136
// #define G6 1568
// #define E6 1319
// #define A6 1760
// #define B6 1976
// #define AS6 1865
// #define F7 2794
// #define D7 2349

// void setup() {
//   AudioMemory(12);

//   osc.begin(WAVEFORM_SQUARE); // classic game sound
//   osc.amplitude(0.1);

//   env.attack(2);
//   env.decay(80);
//   env.sustain(0.0);
//   env.release(20);
// }



// void play(int freq, int duration) {
//   if (freq > 0) {
//     osc.frequency(freq);
//     env.noteOn();
//   }

//   delay(duration);

//   env.noteOff();
//   delay(duration * 0.3);
// }

// void loop() {

//   // Main theme (iconic opening)
//   play(E7, 150);
//   play(E7, 150);
//   delay(150);

//   play(E7, 150);
//   delay(150);

//   play(C7, 150);
//   play(E7, 150);
//   delay(150);

//   play(G7, 300);
//   delay(300);

//   play(G6, 300);
//   delay(300);

//   // Phrase 2
//   play(C7, 200);
//   delay(150);

//   play(G6, 200);
//   delay(150);

//   play(E6, 200);
//   delay(150);

//   play(A6, 150);
//   play(B6, 150);
//   play(AS6, 150);
//   play(A6, 150);

//   play(G6, 200);
//   play(E7, 200);
//   play(G7, 200);
//   play(A7, 150);

//   play(F7, 150);
//   play(G7, 150);

//   delay(300);

//   play(E7, 150);
//   play(C7, 150);
//   play(D7, 150);
//   play(B6, 300);

//   delay(2000); // loop pause
// }



// // #include <Audio.h>

// // AudioSynthWaveform osc;
// // AudioEffectEnvelope env;
// // AudioOutputI2S i2s1;

// // AudioConnection patchCord1(osc, env);
// // AudioConnection patchCord2(env, 0, i2s1, 0);
// // AudioConnection patchCord3(env, 0, i2s1, 1);

// // // Notes
// // #define E7 2637
// // #define C7 2093
// // #define G7 3136
// // #define G6 1568
// // #define E6 1319
// // #define A6 1760
// // #define B6 1976
// // #define AS6 1865
// // #define F7 2794
// // #define D7 2349
// // #define A7 3520


// // void setup_better() {
// //   Serial.begin(115200);

// //   AudioMemory(12);

// //   osc.begin(WAVEFORM_SQUARE);
// //   osc.amplitude(0.5);

// //   env.attack(2);
// //   env.decay(80);
// //   env.sustain(0.0);
// //   env.release(20);

// //   delay(1000); // give Serial time to start
// // }

// // void play_better(int freq, const char* name, int duration) {
// //   if (freq > 0) {
// //     Serial.print("Playing: ");
// //     Serial.print(name);
// //     Serial.print(" (");
// //     Serial.print(freq);
// //     Serial.println(" Hz)");

// //     osc.frequency(freq);
// //     env.noteOn();
// //   } else {
// //     Serial.println("Rest");
// //   }

// //   delay(duration);

// //   env.noteOff();
// //   delay(duration * 0.3);
// // }

// // void loop_better() {

// //   play_better(E7, "E7", 150);
// //   play_better(E7, "E7", 150);
// //   delay(150);

// //   play_better(E7, "E7", 150);
// //   delay(150);

// //   play_better(C7, "C7", 150);
// //   play_better(E7, "E7", 150);
// //   delay(150);

// //   play_better(G7, "G7", 300);
// //   delay(300);

// //   play_better(G6, "G6", 300);
// //   delay(300);

// //   play_better(C7, "C7", 200);
// //   delay(150);

// //   play_better(G6, "G6", 200);
// //   delay(150);

// //   play_better(E6, "E6", 200);
// //   delay(150);

// //   play_better(A6, "A6", 150);
// //   play_better(B6, "B6", 150);
// //   play_better(AS6, "A#6", 150);
// //   play_better(A6, "A6", 150);

// //   play_better(G6, "G6", 200);
// //   play_better(E7, "E7", 200);
// //   play_better(G7, "G7", 200);
// //   play_better(A7, "A7", 150);

// //   play_better(F7, "F7", 150);
// //   play_better(G7, "G7", 150);

// //   delay(300);

// //   play_better(E7, "E7", 150);
// //   play_better(C7, "C7", 150);
// //   play_better(D7, "D7", 150);
// //   play_better(B6, "B6", 300);

// //   delay(2000);
// // }