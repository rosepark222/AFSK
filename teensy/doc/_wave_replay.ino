#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// GUItool: Define the audio pipeline
AudioPlaySdWav           playWav1;       // SD Card WAV player object
AudioOutputI2S           i2s_output;     // Outputs digital I2S audio

// Route the mono/stereo file data to the I2S output blocks
AudioConnection          patchCord1(playWav1, 0, i2s_output, 0); // Left channel
AudioConnection          patchCord2(playWav1, 1, i2s_output, 1); // Right channel

// Use built-in SD card slot for Teensy 3.5/3.6/4.1
#define SDCARD_CS_PIN    BUILTIN_SDCARD

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 3000); // Wait briefly for Serial Monitor

  // Allocate processing memory blocks for reading the file
  AudioMemory(20);

  // Initialize the built-in SD card slot
  if (!(SD.begin(SDCARD_CS_PIN))) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  Serial.println("SD Card Initialized successfully.");
}

void loop() {
  // If the file is not currently playing, start playing it in a continuous loop
  if (playWav1.isPlaying() == false) {
    Serial.println("Playing RECORD.WAV...");
    playWav1.play("RECORD.WAV"); // Starts streaming the file
    delay(25); // Give the player library a quick moment to open the file
  }
}
