#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MTP_Teensy.h>


// GUItool: Audio pipeline routing
AudioInputI2S            i2s_mic;        // Your SPH0645 Microphone
AudioRecordQueue         queue1;         // Collects raw audio memory blocks
AudioConnection          patchCord1(i2s_mic, 0, queue1, 0); // Pipe Mic Left channel to queue

// Configuration for built-in SD card slot (Teensy 3.5, 3.6, 4.1)
#define SDCARD_CS_PIN    BUILTIN_SDCARD

File wavFile;
unsigned long recordStartTime;
const unsigned long recordDuration = 5000; // Record for 5000 ms (5 seconds)
bool isRecording = false;

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 3000); // Wait briefly for Serial Monitor

  // Audio connections require memory allocation. 
  // Queues need plenty of blocks to handle occasional SD card write delays.
  AudioMemory(60); 
  
  MTP.begin();
  MTP.addFilesystem(SD, "Teensy SD");

  // Initialize SD Card
  if (!(SD.begin(SDCARD_CS_PIN))) {
    Serial.println("Unable to access the SD card");
    while (1) delay(500);
  }

  Serial.println("SD Card Initialized.");
  startRecording("RECORD.WAV");
}

void loop() {
  if (isRecording) {
    continueRecording();

    // Check if 5 seconds have passed to automatically stop
    if (millis() - recordStartTime >= recordDuration) {
      stopRecording();
    }
  } else {
    MTP.loop(); 
  }
}

// -------------------------------------------------------------
// Functions to manage WAV File Creation
// -------------------------------------------------------------

void startRecording(const char* filename) {
  if (SD.exists(filename)) {
    SD.remove(filename); // Clear old file if it exists
  }

  wavFile = SD.open(filename, FILE_WRITE);
  if (!wavFile) {
    Serial.println("Failed to open file for writing.");
    return;
  }

  Serial.print("Recording to ");
  Serial.println(filename);

  // 1. Write a placeholder 44-byte WAV Header. 
  // We will overwrite this with the correct size values once recording stops.
  byte header[44] = {0};
  wavFile.write(header, 44);

  // 2. Clear out any leftover audio blocks and start the queue
  queue1.clear();
  queue1.begin();
  
  recordStartTime = millis();
  isRecording = true;
}

void continueRecording() {
  // Check if the audio queue has an available block of raw sound data (256 bytes)
  if (queue1.available() >= 1) {
    byte buffer[256];
    // Copy data out of the audio library memory into our local buffer
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer(); // Instantly free the block back to the library

    // Write raw PCM data block to the SD card
    wavFile.write(buffer, 256);
  }
}

void stopRecording() {
  queue1.end(); // Stop collecting audio
  isRecording = false;

  // Flush any remaining data hanging out in the queue
  while (queue1.available() > 0) {
    byte buffer[256];
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    wavFile.write(buffer, 256);
  }

  // Calculate final audio sizes
  uint32_t fileSize = wavFile.size();
  uint32_t dataSize = fileSize - 44;

  // 3. Rewind to the beginning of the file and write the standard 44-byte WAV Header
  wavFile.seek(0);
  writeWavHeader(fileSize, dataSize);

  wavFile.close(); // Save and close
  Serial.println("Recording stopped. File saved cleanly!");
}

// Standard PCM 44.1kHz, 16-bit, Mono WAV Header generator
void writeWavHeader(uint32_t fileSize, uint32_t dataSize) {
  uint32_t sampleRate = 44100;
  uint32_t byteRate = sampleRate * 1 * 2; // SampleRate * NumChannels * (BitsPerSample/8)

  byte header[44] = {
    'R', 'I', 'F', 'F',
    (byte)(fileSize - 8), (byte)((fileSize - 8) >> 8), (byte)((fileSize - 8) >> 16), (byte)((fileSize - 8) >> 24),
    'W', 'A', 'V', 'E',
    'f', 'm', 't', ' ',
    16, 0, 0, 0,                         // Subchunk1Size (16 for PCM)
    1, 0,                                // AudioFormat (1 for PCM uncompressed)
    1, 0,                                // NumChannels (1 = Mono)
    (byte)(sampleRate), (byte)(sampleRate >> 8), (byte)(sampleRate >> 16), (byte)(sampleRate >> 24),
    (byte)(byteRate), (byte)(byteRate >> 8), (byte)(byteRate >> 16), (byte)(byteRate >> 24),
    2, 0,                                // BlockAlign (NumChannels * BitsPerSample/8)
    16, 0,                               // BitsPerSample (16-bit)
    'd', 'a', 't', 'a',
    (byte)(dataSize), (byte)(dataSize >> 8), (byte)(dataSize >> 16), (byte)(dataSize >> 24)
  };

  wavFile.write(header, 44);
}
