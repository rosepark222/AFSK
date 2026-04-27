

/*

TODO: fix Cumulative Timing Error (or cumulative drift)
Zero Drift: The IntervalTimer uses dedicated hardware clocks. It will trigger your ISR at the exact microsecond requested, regardless of code execution jitter.
Non-Blocking: Your loop() is now empty and runs at full speed. You can read sensors, update a screen, or process Serial commands while the FSK burst plays in the background.
Stability: By removing the while(micros() < nextBitTime) loop, you eliminate the risk of the CPU hanging if the timing logic overflows or gets stuck. 
 
#include <Audio.h>
#include <IntervalTimer.h>

IntervalTimer bitTimer;
volatile bool isTransmitting = false;
volatile int currentBitIndex = 0;
const int TOTAL_BITS = 64; // bitCount * 8

void sendBitISR() {
  if (currentBitIndex < TOTAL_BITS) {
    // Alternating 0 and 1 for testing; replace with your data array
    int bit = currentBitIndex % 2; 
    
    if (bit == 1) fskOsc.frequency(FREQ_MARK);
    else fskOsc.frequency(FREQ_SPACE);
    
    currentBitIndex++;
  } else {
    // Burst finished: cleanup
    fskOsc.amplitude(0.0);
    isTransmitting = false;
    bitTimer.end(); // Stop the timer until next burst
  }
}

void startBurst() {
  if (isTransmitting) return; // Don't start if already running
  
  currentBitIndex = 0;
  isTransmitting = true;
  fskOsc.amplitude(0.5);
  
  // Start the timer to call sendBitISR every BIT_PERIOD (in microseconds)
  bitTimer.begin(sendBitISR, BIT_PERIOD);
  bitTimer.priority(200); // Lower priority than Audio Library to prevent clicks
}

elapsedMillis sinceLastBurst;

void loop() {
  // Trigger a burst every 5 seconds without blocking
  if (sinceLastBurst >= 5000) {
    sinceLastBurst = 0;
    Serial.println("Starting burst...");
    startBurst();
  }

  // You can now run other code here freely!
}

Do you need help storing your data bits in an array so the ISR can read them instead of just alternating 0 and 1?
*/
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

#define BAUD_RATE   10 // 1200  // Bits per second
#define BIT_PERIOD (1000000 / BAUD_RATE)  

void setup() {
  AudioMemory(10);
  fskOsc.begin(WAVEFORM_SINE);
  fskOsc.amplitude(0.1);
  
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

void sendBit_buggyprint(int bit) {
  if (nextBitTime == 0) {
    nextBitTime = micros();
    return;  // first call just initializes — don't send yet
  }

  // How late are we arriving at this bit's deadline?
  int32_t lag = (int32_t)(micros() - nextBitTime);  // should be 0..few us

  if (bit == 1) fskOsc.frequency(FREQ_MARK);
  else          fskOsc.frequency(FREQ_SPACE);

  nextBitTime += BIT_PERIOD;
  while (micros() < nextBitTime);

  // Print lag at entry — not drift at exit
  static int diagCount = 0;
  if (++diagCount % 100 == 0) {
    Serial.printf("entry lag: %+d us\n", lag);
  }
}

void sendBit_new_idea(int bit) {
  // First call: initialize the timeline
  if (nextBitTime == 0) nextBitTime = micros();

  // Switch frequency immediately
  if (bit == 1) {
    fskOsc.frequency(FREQ_MARK);
  } else {
    fskOsc.frequency(FREQ_SPACE);
  }

  // Wait until the absolute deadline — not a relative delay
  nextBitTime += BIT_PERIOD;
  while (micros() < nextBitTime);
}

void sendBit_buggyone(int bit) {
  // Switch frequency based on bit
  if (bit == 1) {
    fskOsc.frequency(FREQ_MARK);
     //fskOsc.frequency(FREQ_SPACE);

  } else {
    //    fskOsc.frequency(FREQ_MARK);

    fskOsc.frequency(FREQ_SPACE);
  }
  
  // Wait for the duration of exactly one bit
  delayMicroseconds(BIT_PERIOD);
}

int bitCount = 4; // 3 bytes
void loop_loop() {
  // Get current time in milliseconds
  unsigned long currentTime = millis();
  
  // Print the time
  Serial.print("Current Time (ms): ");
  Serial.println(currentTime);
  
  // Delay for 5000 milliseconds (5 seconds)
  delay(5000);
}

void loop_5secdelay() {
  unsigned long currentTime = millis();
  Serial.print("Current Time (ms): ");
  Serial.println(currentTime);

  fskOsc.amplitude(0.1);
  nextBitTime = 0;           // ← reset before every burst
  for (int i = 0; i < bitCount*8; i++) {
    int currentBit = i % 2; // Alternates 0, 1, 0, 1...
    sendBit(currentBit);
  }
  
  // Optional: Long Mark (1) between bursts to stabilize receiver
  //fskOsc.frequency(FREQ_MARK);
  fskOsc.amplitude(0.0);  // turn off
  delay(5000); // 5 second
}


void loop() {
 
  for (int i = 0; i < bitCount*8; i++) {
    int currentBit = i % 2; // Alternates 0, 1, 0, 1...
    sendBit(currentBit);
  }
  
  // Optional: Long Mark (1) between bursts to stabilize receiver
  //fskOsc.frequency(FREQ_MARK);
  //fskOsc.amplitude(0.0);  // turn off
  //delay(5000); // 5 second
}
