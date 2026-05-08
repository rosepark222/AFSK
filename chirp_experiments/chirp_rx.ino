#include <Audio.h>
#include <SD.h>

// ================= AUDIO SETUP ====================
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;
AudioConnection   patchCord1(i2s_in, 0, queue1, 0);

// ================= PARAMETERS =====================
#define SAMPLE_RATE     44100
#define CHIRP_DURATION  0.1f
#define CHIRP_SAMPLES   (int)(SAMPLE_RATE * CHIRP_DURATION)  // 4410

#define F_START   300.0f
#define F_END    1500.0f
#define NUM_SLOTS   16

#define CAPTURE_SECONDS  2
#define CAPTURE_SAMPLES  (SAMPLE_RATE * CAPTURE_SECONDS)     // 88200

// ================= BUFFERS ========================
float ref_chirp[CHIRP_SAMPLES];

int   capture_idx  = 0;
bool  capture_done = false;

// ================= REFERENCE GENERATION ===========
void generateReference(int ref_id = 0)
{
    float bandwidth  = F_END - F_START;
    float slot_width = bandwidth / NUM_SLOTS;
    float f0         = F_START + ref_id * slot_width;
    float k          = bandwidth / CHIRP_DURATION;
    float phase      = 0.0f;

    for (int n = 0; n < CHIRP_SAMPLES; n++) {
        float freq = f0 + k * ((float)n / SAMPLE_RATE);
        if (freq > F_END) freq -= bandwidth;

        phase += 2.0f * PI * freq / SAMPLE_RATE;
        if (phase > 2.0f * PI) phase -= 2.0f * PI;

        float s = sinf(phase);
        float w = 0.5f * (1.0f - cosf(2.0f * PI * n / (CHIRP_SAMPLES - 1)));
        ref_chirp[n] = s * w;
    }
}

// ================= SETUP ==========================
void setup()
{
    Serial.begin(9600);
    while (!Serial) { delay(10); }

    // --- SD init ---
    Serial.println("Initializing SD card...");
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("ERROR: SD init failed. Halting.");
        while (1) {}
    }
    Serial.println("SD card OK.");

    // --- Write reference chirp first (no audio needed) ---
    generateReference(0);

    if (SD.exists("ref_chirp.txt")) SD.remove("ref_chirp.txt");
    File refFile = SD.open("ref_chirp.txt", FILE_WRITE);
    if (!refFile) {
        Serial.println("ERROR: Cannot open ref_chirp.txt. Halting.");
        while (1) {}
    }
    for (int i = 0; i < CHIRP_SAMPLES; i++) {
        refFile.println(ref_chirp[i], 6);
    }
    refFile.close();
    Serial.println("ref_chirp.txt written.");

    // --- Prepare rx file ---
    if (SD.exists("rx_chirp.txt")) SD.remove("rx_chirp.txt");

    // --- Audio init ---
    AudioMemory(160);
    queue1.begin();

    Serial.println("Capturing 2 seconds of audio...");
}

// ================= LOOP ===========================
void loop()
{
    if (capture_done) return;

    // Open file in append mode each loop to keep writes incremental
    File rxFile = SD.open("rx_chirp.txt", FILE_WRITE);
    if (!rxFile) {
        Serial.println("ERROR: Cannot open rx_chirp.txt. Halting.");
        while (1) {}
    }

    while (queue1.available() && capture_idx < CAPTURE_SAMPLES) {
        int16_t *data = queue1.readBuffer();

        for (int i = 0; i < AUDIO_BLOCK_SAMPLES && capture_idx < CAPTURE_SAMPLES; i++) {
            float v = data[i] / 32768.0f;
            rxFile.println(v, 6);
            capture_idx++;
        }

        queue1.freeBuffer();
    }

    rxFile.close();

    // Progress report every ~0.5 s
    static int last_pct = -1;
    int pct = (capture_idx * 100) / CAPTURE_SAMPLES;
    if (pct / 10 != last_pct / 10) {
        last_pct = pct;
        Serial.print("Capture: ");
        Serial.print(pct);
        Serial.println("%");
    }

    // Done?
    if (capture_idx >= CAPTURE_SAMPLES) {
        capture_done = true;
        Serial.println("DONE");
    }
}