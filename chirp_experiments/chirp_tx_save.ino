#include <Audio.h>
#include <arm_math.h>
#include <SD.h>

// ─── Audio Objects ─────────────────────────────────────────
AudioPlayQueue     queue;
AudioOutputI2S     i2s1;
AudioConnection    patchCord1(queue, 0, i2s1, 0);
AudioConnection    patchCord2(queue, 0, i2s1, 1);

// ─── Parameters ────────────────────────────────────────────
#define SAMPLE_RATE     44100
#define CHIRP_DURATION  1.0f
#define N_SAMPLES       (int)(SAMPLE_RATE * CHIRP_DURATION)

#define F_START 300.0f
#define F_END   1500.0f

#define ID 5
#define NUM_SLOTS 16

#define AMPLITUDE 0.8f

// ─── Buffer ────────────────────────────────────────────────
int16_t chirp_buffer[N_SAMPLES];

// ─── Generate CLEAN Cyclic Chirp ──────────────────────────
void generateChirp()
{
    float bandwidth  = F_END - F_START;
    float slot_width = bandwidth / NUM_SLOTS;
    float f0         = F_START + ID * slot_width;
    float k          = bandwidth / CHIRP_DURATION;
    float phase      = 0.0f;

    for (int n = 0; n < N_SAMPLES; n++) {
        float t    = (float)n / SAMPLE_RATE;
        float freq = f0 + k * t;

        if (freq > F_END) freq -= bandwidth;

        phase += 2.0f * PI * freq / SAMPLE_RATE;
        if (phase > 2 * PI) phase -= 2 * PI;

        float s   = sinf(phase);
        float w   = 0.5f * (1.0f - cosf(2 * PI * n / (N_SAMPLES - 1)));
        float out = AMPLITUDE * s * w;

        chirp_buffer[n] = (int16_t)(out * 32767.0f);
    }
}

// ─── Save chirp buffer to SD as text (one float per line) ──
void saveChirpToSD()
{
    Serial.println("Saving chirp to SD...");

    if (SD.exists("tx_chirp.txt")) SD.remove("tx_chirp.txt");

    File f = SD.open("tx_chirp.txt", FILE_WRITE);
    if (!f) {
        Serial.println("ERROR: cannot open tx_chirp.txt");
        return;
    }

    for (int n = 0; n < N_SAMPLES; n++) {
        // Convert back to float [-1, 1] for easy plotting
        float v = chirp_buffer[n] / 32767.0f;
        f.println(v, 6);
    }

    f.close();
    Serial.print("Saved ");
    Serial.print(N_SAMPLES);
    Serial.println(" samples to tx_chirp.txt");
}

// ─── Setup ─────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    AudioMemory(100);

    generateChirp();

    // Save to SD
    Serial.println("Initializing SD...");
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("ERROR: SD init failed.");
        while (1) {}
    }
    Serial.println("SD OK.");

    saveChirpToSD();

    Serial.println("DONE");
}

// ─── Loop ──────────────────────────────────────────────────
void loop()
{
    // Nothing — just save once and stop
}