/*

---------------------------------------------------------------------------------------------------------------------------------

what is the slot 2 in 
AudioConnection waveToMixerL(playWav1, 0, spkMixerL, 2);

In your Teensy Audio library setup, slot 2 refers to the third input channel of the 4-channel audio mixer object (spkMixerL or spkMixerR).

Because mixer input channels are indexed starting at 0, the number 2 corresponds to input index 2:

Slot 0: Connected to fskOsc (the AFSK tone generator).

Slot 1: Connected to chirpTxQueue (the chirp generator).

Slot 2 (waveToMixerL / waveToMixerR): Connected to your wave player (playWav1).

Slot 3: Currently unused (left open).

Why use slot 2?
By plugging the wave player into slot 2, you allow the Teensy Audio library to mix multiple audio sources together digitally before sending the final mixed stream out to your I2S audio output (i2s_out). This means your robot can seamlessly layer or switch between playing back .wave sound files, transmitting AFSK data tones, and broadcasting acoustic chirps without needing separate hardware audio lines!


---------------------------------------------------------------------------------------------------------------------------------

what happen all slots are used? do I need to create new mixer? 

In your Teensy Audio library architecture, **yes, you would need to add a new mixer** if you run out of input slots on your existing `AudioMixer4` objects.

An `AudioMixer4` has a hard limit of **4 input channels (slots 0 to 3)**. Because your current speaker pipeline (`spkMixerL` and `spkMixerR`) is already using all 4 slots:

* **Slot 0:** `fskOsc` (AFSK tones)
* **Slot 1:** `chirpTxQueue` (Chirps)
* **Slot 2:** `playWav1` (WAV audio player)
* **Slot 3:** Currently open / available (wait, looking at your code, slot 3 is actually unassigned, so you currently have **1 slot left**!).

---

### What happens if you use up slot 3 and need more?

If you add yet another audio source later (like a second sound player, a sound effect generator, or background noise) and all 4 slots across your primary mixers are taken, you cannot plug a 5th audio connection directly into `spkMixerL`. The Audio library will throw a compile error or ignore the connection because inputs are strictly bound to indices 0–3.

### How to expand using a "Sub-Mixer" pattern

Instead of redesigning your entire master audio chain, the standard solution in the Teensy Audio ecosystem is to **cascade mixers** (daisy-chaining them).

For example, if you run out of slots, you can introduce a secondary auxiliary mixer (`spkAuxMixerL`):

1. **Create an auxiliary mixer:**
```cpp
AudioMixer4 spkAuxMixerL;
AudioMixer4 spkAuxMixerR;

```


2. **Plug your extra audio sources into the auxiliary mixer:**
```cpp
AudioPlaySdWav extraSoundPlayer;
AudioConnection extraToAuxL(extraSoundPlayer, 0, spkAuxMixerL, 0);
AudioConnection extraToAuxR(extraSoundPlayer, 1, spkAuxMixerR, 0);

```


3. **Route the output of the auxiliary mixer into the last available slot (Slot 3) of your main master mixer:**
```cpp
AudioConnection auxToMainL(spkAuxMixerL, 0, spkMixerL, 3);
AudioConnection auxToMainR(spkAuxMixerR, 0, spkMixerR, 3);

```



This lets you combine multiple additional audio streams together before feeding them into your final main output stage!

---------------------------------------------------------------------------------------------------------------------------------

what is the purpose of using mixer? I do not intend to mix fsk, chirp and wave. 

If you **do not** intend to mix multiple audio sources simultaneously (meaning your robot will never play a chirp, an AFSK tone, and a wave file at the exact same time), you technically **do not need** a mixer object in your audio pipeline.

Here is the purpose of using a mixer, and how you can bypass it if your application is strictly sequential:

---

### 1. The Core Purpose of a Mixer in Teensy Audio

In the Teensy Audio Library, the mixer (`AudioMixer4`) serves two main purposes:

* **Combining Signals:** It adds multiple active audio streams together (e.g., blending a background sound effect with a voice line or a communication tone).
* **Acting as a Routing Gateway (Multiplexer / Switch):** Even if you only play one sound at a time, routing your various players (`fskOsc`, `chirpTxQueue`, `playWav1`) into a single mixer allows them all to share a single fixed path out to your physical output (`i2s_out`) without you having to constantly patch and unpatch audio connections in code.

---

### 2. What happens if you remove the mixer?

If you only ever play **one audio source at a time** (for example, playing a WAV file during state A, sending an AFSK packet during state B, and playing a chirp during state C), you can simplify your hardware wiring by connecting your audio sources **directly** to the output.

However, the Teensy Audio Library has a strict rule: **An input channel on an output object (like `i2s_out`) can only have one connection attached to it.**

* **If you have multiple sources:** If you wire `fskOsc` to `i2s_out` channel 0, and then try to wire `playWav1` to `i2s_out` channel 0 as well, the compiler or audio library will conflict because multiple streams are trying to drive the exact same input slot.
* **The Solution without a Mixer:** If you want to drop the mixer entirely, you would need to dynamically connect and disconnect patch cords in code, or use a simpler structure where everything funnels through a single unified audio generation buffer.

---

### Recommendation for your Robot

Even if you don't mix them *at the exact same time*, **keeping the mixer is usually recommended** for this architecture because:

1. It avoids runtime audio routing bugs.
2. It allows seamless transitions (e.g., if a wave file finishes just as a chirp needs to start, the mixer handles the handoff cleanly without audio tearing).
3. It keeps your setup code clean, as you defined the channels once at boot and never have to rewrite patch cords later.

Would you prefer to simplify the audio graph and remove the mixers, or does keeping them as a unified gateway make sense for your cleanup phases?


---------------------------------------------------------------------------------------------------------
 
*/

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// 1. Define the Audio components
AudioPlaySdWav           playWav1;       // Source 1: SD Card WAV player
AudioSynthWaveform       toneGen;        // Source 2: Tone synthesizer (e.g., beep/alarm)
AudioMixer4              mixerL;         // Left channel master mixer (4 input slots)
AudioMixer4              mixerR;         // Right channel master mixer (4 input slots)
AudioOutputI2S           i2sOut;         // Hardware I2S audio output DAC

// 2. Route both audio sources into the master mixers
// Slot 0 of mixer: WAV Player (Left & Right channels)
AudioConnection          patchWavL(playWav1, 0, mixerL, 0);
AudioConnection          patchWavR(playWav1, 1, mixerR, 0);

// Slot 1 of mixer: Synthesized Tone generator (duplicating mono tone to both sides)
AudioConnection          patchToneL(toneGen, 0, mixerL, 1);
AudioConnection          patchToneR(toneGen, 0, mixerR, 1);

// 3. Route the mixed output to the physical I2S output DAC
AudioConnection          patchOutL(mixerL, 0, i2sOut, 0);
AudioConnection          patchOutR(mixerR, 0, i2sOut, 1);

// SD Card chip select pin for Teensy 4.1
#define SDCARD_CS_PIN    BUILTIN_SDCARD

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 3000);

  // Allocate memory blocks for the audio library engine
  AudioMemory(20);

  // Initialize SD Card
  if (!(SD.begin(SDCARD_CS_PIN))) {
    Serial.println("Unable to access the SD card!");
    while (true) { delay(500); }
  }
  Serial.println("SD Card Initialized successfully.");

  // Configure the Sine Wave generator (e.g., a quiet 440 Hz beep)
  toneGen.begin(WAVEFORM_SINE);
  toneGen.frequency(440.0f);   // Concert A pitch
  toneGen.amplitude(0.1f);     // Keep it low so it doesn't overpower the WAV file

  // Set the gains on the Mixer (Slots 0 and 1)
  // Gain scale is typically 0.0 to 1.0 (or higher if boosting)
  mixerL.gain(0, 1.0f);        // Full volume for WAV player (Left)
  mixerL.gain(1, 0.5f);        // Half volume for background tone (Left)
  
  mixerR.gain(0, 1.0f);        // Full volume for WAV player (Right)
  mixerR.gain(1, 0.5f);        // Half volume for background tone (Right)

  // Unused slots set to 0 gain
  mixerL.gain(2, 0); mixerL.gain(3, 0);
  mixerR.gain(2, 0); mixerR.gain(3, 0);

  Serial.println("Audio pipeline ready. Starting mixed playback...");
}

void loop() {
  // If the WAV file isn't playing, restart it so we can hear the mix loop
  if (!playWav1.isPlaying()) {
    Serial.println("Playing RECORD.WAV with background tone...");
    playWav1.play("B/yes.WAV");
    delay(25); // Brief stabilization delay for file opening
  }
}
