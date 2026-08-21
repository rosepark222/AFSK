// ============================================================
// AFSK SNR Meter — Dual SPH0645LM4H I2S Microphones on Teensy 4.1
// Mark: 6000 Hz | Space: 8000 Hz | Baud: 10
// Goertzel-based tone detection — works reliably at high freq
// Noise floor printed on every measurement line
// DUAL MIC: Left (Ch 0) & Right (Ch 1) processed independently
// ============================================================

#include <Audio.h>
#include <Wire.h>

// ── Audio graph ─────────────────────────────────────────────
AudioInputI2S         i2s_in;
AudioAnalyzeRMS       rms_total_left;
AudioAnalyzeRMS       rms_total_right;
AudioRecordQueue      queue_left;
AudioRecordQueue      queue_right;

// Left channel (I2S input channel 0)
AudioConnection c1(i2s_in, 0, rms_total_left,  0);
AudioConnection c2(i2s_in, 0, queue_left,      0);

// Right channel (I2S input channel 1)
AudioConnection c3(i2s_in, 1, rms_total_right, 0);
AudioConnection c4(i2s_in, 1, queue_right,     0);

// ── Config ───────────────────────────────────────────────────
static const float SAMPLE_RATE  = 44100.0f;
//static const float MARK_HZ      = 6000.0f;
//static const float SPACE_HZ     = 8000.0f;

static const float MARK_HZ      = 15000.0f;
static const float SPACE_HZ     = 17000.0f;

static const int   BLOCK_SIZE   = 128;
static const int   CALIBRATE_MS = 2000;
static const int   REPORT_MS    = 200;

// ── State ────────────────────────────────────────────────────
float noiseFloorPower_L = 0.0f;
float noiseFloorPower_R = 0.0f;
bool  calibrated        = false;
char  noiseFloorStr_L[40] = "";
char  noiseFloorStr_R[40] = "";

// ── Goertzel ─────────────────────────────────────────────────
// Returns normalized power at targetHz for the given sample block.
// Works at any frequency up to Nyquist (22050 Hz at 44100 sample rate).
float goertzel(int16_t* samples, int N, float targetHz) {
    float k     = (float)N * targetHz / SAMPLE_RATE;
    float omega = 2.0f * M_PI * k / (float)N;
    float coeff = 2.0f * cosf(omega);
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;

    for (int i = 0; i < N; i++) {
        float x = samples[i] / 32768.0f;   // normalize int16 → -1..+1
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    float power = (s2 * s2) + (s1 * s1) - coeff * s1 * s2;
    return power / (float)N;               // normalize by block size
}

// ── Helpers ──────────────────────────────────────────────────
// Power ratio → dB  (use 10x because inputs are already power, not amplitude)
float toDb(float signal, float noise) {
    if (noise < 1e-12f || signal < 1e-12f) return -99.0f;
    return 10.0f * log10f(signal / noise);
}

void printBar(float fraction, int width = 24) {
    fraction = constrain(fraction, 0.0f, 1.0f);
    int filled = (int)(fraction * width);
    Serial.print('[');
    for (int i = 0; i < width; i++)
        Serial.print(i < filled ? '=' : ' ');
    Serial.print(']');
}

// ── Calibration ──────────────────────────────────────────────
void calibrate() {
    Serial.println("========================================");
    Serial.println(" AFSK SNR Meter — Dual Mic (15kHz/17kHz)");
    Serial.println(" Teensy 4.1 + 2x SPH0645LM4H I2S Mics");
    Serial.println("========================================");
    Serial.println();
    Serial.println("CALIBRATION: Keep channels SILENT for 2s...");

    float    accum_L = 0.0f, accum_R = 0.0f;
    int      count_L = 0, count_R = 0;
    uint32_t t       = millis();

    while (millis() - t < CALIBRATE_MS) {
        // Process LEFT channel
        if (queue_left.available()) {
            int16_t* block = queue_left.readBuffer();
            float mp = goertzel(block, BLOCK_SIZE, MARK_HZ);
            float sp = goertzel(block, BLOCK_SIZE, SPACE_HZ);
            accum_L += mp + sp;
            count_L++;
            queue_left.freeBuffer();
        }
        
        // Process RIGHT channel
        if (queue_right.available()) {
            int16_t* block = queue_right.readBuffer();
            float mp = goertzel(block, BLOCK_SIZE, MARK_HZ);
            float sp = goertzel(block, BLOCK_SIZE, SPACE_HZ);
            accum_R += mp + sp;
            count_R++;
            queue_right.freeBuffer();
        }
        
        // Progress dot every ~200ms
        static uint32_t lastDot = 0;
        if (millis() - lastDot > 200) {
            Serial.print('.');
            lastDot = millis();
        }
    }

    noiseFloorPower_L = (count_L > 0) ? (accum_L / count_L) : 1e-12f;
    noiseFloorPower_R = (count_R > 0) ? (accum_R / count_R) : 1e-12f;
    calibrated        = true;

    // Pre-format noise floor strings — printed on every line from now on
    snprintf(noiseFloorStr_L, sizeof(noiseFloorStr_L),
             "NF_L=%.3e", noiseFloorPower_L);
    snprintf(noiseFloorStr_R, sizeof(noiseFloorStr_R),
             "NF_R=%.3e", noiseFloorPower_R);

    float nf_dB_L = 10.0f * log10f(max(noiseFloorPower_L, 1e-12f));
    float nf_dB_R = 10.0f * log10f(max(noiseFloorPower_R, 1e-12f));

    Serial.println();
    Serial.println();
    Serial.println("── Calibration Result ──────────────────");
    Serial.printf( "   LEFT Mic           \n");
    Serial.printf( "     Noise floor power : %.6e\n", noiseFloorPower_L);
    Serial.printf( "     Noise floor        : %.1f dB\n", nf_dB_L);
    Serial.printf( "     Samples averaged   : %d\n", count_L);
    Serial.println();
    Serial.printf( "   RIGHT Mic          \n");
    Serial.printf( "     Noise floor power : %.6e\n", noiseFloorPower_R);
    Serial.printf( "     Noise floor        : %.1f dB\n", nf_dB_R);
    Serial.printf( "     Samples averaged   : %d\n", count_R);
    Serial.println("────────────────────────────────────────");
    Serial.println();
    Serial.println("NOW TRANSMIT: 01010101... at 10 baud");
    Serial.println("  Mark  = 15000 Hz | Space = 17000 Hz");
    Serial.println("  100ms per bit");
    Serial.println();

    // Column headers
    Serial.printf("%-5s %-11s %-11s %-11s %-16s  %s\n",
                  "CH",
                  "Mark Pwr",
                  "Space Pwr",
                  "SNR (dB)",
                  "Noise Floor",
                  "Bar (0–40dB)");
    Serial.println("---------------------------------------------------------------------");
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    AudioMemory(40);  // Increased from 20 for dual queues
    queue_left.begin();
    queue_right.begin();

    calibrate();
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
    if (!calibrated) return;

    // ── Process LEFT channel ──────────────────────────────────
    if (queue_left.available()) {
        int16_t* block = queue_left.readBuffer();
        float markPow   = goertzel(block, BLOCK_SIZE, MARK_HZ);
        float spacePow  = goertzel(block, BLOCK_SIZE, SPACE_HZ);
        queue_left.freeBuffer();

        float signalPow = max(markPow, spacePow);
        float snr       = toDb(signalPow, noiseFloorPower_L);

        // Rate-limit output
        static uint32_t lastReport_L = 0;
        if (millis() - lastReport_L >= REPORT_MS) {
            lastReport_L = millis();

            // Determine which tone is dominant
            const char* label;
            if      (markPow  > spacePow * 2.0f) label = "MARK ";
            else if (spacePow > markPow  * 2.0f) label = "SPACE";
            else                                  label = "TRANS";

            const char* quality;
            if      (snr >= 25.0f) quality = "EXCELLENT";
            else if (snr >= 20.0f) quality = "GOOD     ";
            else if (snr >= 15.0f) quality = "FAIR     ";
            else if (snr >= 10.0f) quality = "MARGINAL ";
            else if (snr >=  6.0f) quality = "POOR     ";
            else                   quality = "UNUSABLE ";

            Serial.printf("L    %-11.6f %-11.6f %+8.1f dB  %-16s  %-10s ",
                        markPow, spacePow, snr, noiseFloorStr_L, quality);
            printBar(snr / 40.0f);
            Serial.println();
        }
    }

    // ── Process RIGHT channel ─────────────────────────────────
    if (queue_right.available()) {
        int16_t* block = queue_right.readBuffer();
        float markPow   = goertzel(block, BLOCK_SIZE, MARK_HZ);
        float spacePow  = goertzel(block, BLOCK_SIZE, SPACE_HZ);
        queue_right.freeBuffer();

        float signalPow = max(markPow, spacePow);
        float snr       = toDb(signalPow, noiseFloorPower_R);

        // Rate-limit output
        static uint32_t lastReport_R = 0;
        if (millis() - lastReport_R >= REPORT_MS) {
            lastReport_R = millis();

            // Determine which tone is dominant
            const char* label;
            if      (markPow  > spacePow * 2.0f) label = "MARK ";
            else if (spacePow > markPow  * 2.0f) label = "SPACE";
            else                                  label = "TRANS";

            const char* quality;
            if      (snr >= 25.0f) quality = "EXCELLENT";
            else if (snr >= 20.0f) quality = "GOOD     ";
            else if (snr >= 15.0f) quality = "FAIR     ";
            else if (snr >= 10.0f) quality = "MARGINAL ";
            else if (snr >=  6.0f) quality = "POOR     ";
            else                   quality = "UNUSABLE ";

            Serial.printf("R    %-11.6f %-11.6f %+8.1f dB  %-16s  %-10s ",
                        markPow, spacePow, snr, noiseFloorStr_R, quality);
            printBar(snr / 40.0f);
            Serial.println();
        }
    }
}
