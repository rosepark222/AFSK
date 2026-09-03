#include <SD.h>
#include <MTP_Teensy.h>

bool isDSP = true; // Mode flag

void setup() {
  Serial.begin(9600);
  SD.begin(BUILTIN_SDCARD);
  
  MTP.begin();
  MTP.addFilesystem(SD, "Teensy SD");
  
  // Pin setup for a simple "Stop/Download" button
  pinMode(0, INPUT_PULLUP); 
}

void loop() {
  if (isDSP) {
    // ----------------------------------------------------
    // MODE 1: REGULAR WORK (LOGGING DATA)
    // ----------------------------------------------------
    // Execute your high-speed logging, audio tasks, etc.
    logAudioDataToSD(); 

    // Explicitly DO NOT call MTP.loop() here so the PC doesn't 
    // interrupt the critical timing of your SD card writes.

    // If button on Pin 0 is pressed, stop logging and switch to download mode
    if (digitalRead(0) == LOW) {
      isDSP = false;
      Serial.println("Logging stopped. USB MTP Mode activated.");
      delay(500); // Debounce
    }
    
  } else {
    // ----------------------------------------------------
    // MODE 2: DOWNLOAD MODE (TALKING TO PC)
    // ----------------------------------------------------
    // Now pass total control to the USB interface so you can 
    // drag and drop the files onto your PC.
    MTP.loop(); 

    // Optional: Press button again to lock back into logging mode
    if (digitalRead(0) == LOW) {
      isDSP = true;
      Serial.println("Returning to Logging Mode.");
      delay(500);
    }
  }
}

void logAudioDataToSD() {
  // Your normal audio saving code here
  Serial.println("Logging Mode.");
  delay(2);

}

