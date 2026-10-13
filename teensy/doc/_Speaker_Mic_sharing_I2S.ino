
#include <Audio.h>
#include <math.h>

// ─── Audio Objects ─────────────────────────────────────────
AudioPlayQueue     queue;
AudioOutputI2S     i2s1;

AudioInputI2S      mic;
AudioAnalyzePeak   peakLeft;
AudioAnalyzePeak   peakRight;

AudioConnection    patchCord1(queue, 0, i2s1, 0);
AudioConnection    patchCord2(queue, 0, i2s1, 1);

AudioConnection    patchCord3(mic, 0, peakLeft, 0);
AudioConnection    patchCord4(mic, 1, peakRight, 0);

// ─── Settings ──────────────────────────────────────────────
const float SAMPLE_RATE = 44100.0f;
const float TONE_FREQ   = 440.0f;
const int   BLOCK_SIZE  = AUDIO_BLOCK_SAMPLES;

bool speakerMode = false;
uint32_t phase = 0;

void setup() {
  Serial.begin(115200);
  AudioMemory(20);

  Serial.println("Teensy I2S test");
  Serial.println("s = speaker tone");
  Serial.println("m = microphone peak monitor");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 's') {
      speakerMode = true;
      Serial.println("Speaker mode");
    } else if (c == 'm') {
      speakerMode = false;
      Serial.println("Microphone mode");
    }
  }

  if (speakerMode) {
    int16_t *buffer = queue.getBuffer();

    if (buffer != nullptr) {
      for (int i = 0; i < BLOCK_SIZE; i++) {
        float sample = sinf(
          2.0f * PI * TONE_FREQ * phase / SAMPLE_RATE
        );

        buffer[i] = (int16_t)(sample * 5000.0f);

        phase++;
        if (phase >= (uint32_t)SAMPLE_RATE)
          phase = 0;
      }

      queue.playBuffer();
    }
  } else {
    if (peakLeft.available() && peakRight.available()) {
      Serial.print("Left peak: ");
      Serial.print(peakLeft.read(), 4);
      Serial.print("  Right peak: ");
      Serial.println(peakRight.read(), 4);
    }
  }
}
