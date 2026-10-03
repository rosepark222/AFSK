/*
C:\Users\rosep\AppData\Local\Temp\.arduinoIDE-unsaved202693-20660-406t86.bzqti\sketch_oct3c\sketch_oct3c.ino: In function 'void setup()':
C:\Users\rosep\AppData\Local\Temp\.arduinoIDE-unsaved202693-20660-406t86.bzqti\sketch_oct3c\sketch_oct3c.ino:10:3: error: 'MTP' was not declared in this scope
   10 |   MTP.begin();
      |   ^~~
C:\Users\rosep\AppData\Local\Temp\.arduinoIDE-unsaved202693-20660-406t86.bzqti\sketch_oct3c\sketch_oct3c.ino: In function 'void loop()':
C:\Users\rosep\AppData\Local\Temp\.arduinoIDE-unsaved202693-20660-406t86.bzqti\sketch_oct3c\sketch_oct3c.ino:41:5: error: 'MTP' was not declared in this scope
   41 |     MTP.loop();
      |     ^~~
Multiple libraries were found for "SD.h"
  Used: C:\Users\rosep\AppData\Local\Arduino15\packages\teensy\hardware\avr\1.62.0\libraries\SD
  Not used: C:\Users\rosep\AppData\Local\Arduino15\libraries\SD
exit status 1

Compilation error: 'MTP' was not declared in this scope



The error is almost certainly caused by Tools → USB Type no longer being set to an MTP-capable option. The MTP object is only provided when the Teensy USB configuration includes MTP support; including <MTP_Teensy.h> alone does not guarantee that MTP exists. Teensy’s official example also requires selecting an MTP USB type. (github.com)

Fix
In Arduino IDE, select:

Tools → Board → Teensy 4.1
Tools → USB Type → MTP Disk (Experimental)

*/

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

