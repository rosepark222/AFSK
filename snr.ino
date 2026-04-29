// ============================================================
// AFSK SNR Meter — SPH0645LM4H I2S Microphone on Teensy 4.1
// Mark: 6000 Hz | Space: 8000 Hz | Baud: 10
// Goertzel-based tone detection — works reliably at high freq
// Noise floor printed on every measurement line
// ============================================================

/*

========================================
 AFSK SNR Meter — 6000/8000 Hz Goertzel
 Teensy 4.1 + SPH0645LM4H I2S Mic
========================================

CALIBRATION: Keep channel SILENT for 2s...
..........

── Calibration Result ──────────────────
   Noise floor power : 3.124000e-07
   Noise floor        : -65.1 dB
   Samples averaged   : 68
────────────────────────────────────────

NOW TRANSMIT: 01010101... at 10 baud
  Mark  = 6000 Hz | Space = 8000 Hz
  100ms per bit

Tone    Mark Pwr    Space Pwr   SNR (dB)   Noise Floor       Bar (0–40dB)
----------------------------------------------------------------------
MARK    0.048821    0.000103    +48.3 dB   NF=3.124e-07      [=====================   ]
SPACE   0.000091    0.041204    +47.1 dB   NF=3.124e-07      [====================    ]
TRANS   0.021100    0.019800    +39.4 dB   NF=3.124e-07      [==================      ]
MARK    0.048710    0.000098    +48.4 dB   NF=3.124e-07      [=====================   ]
  
*/

#include <Audio.h>
#include <Wire.h>

// ── Audio graph ─────────────────────────────────────────────
AudioInputI2S         i2s_in;
AudioAnalyzeRMS       rms_total;
AudioRecordQueue      queue;

AudioConnection c1(i2s_in, 0, rms_total, 0);
AudioConnection c2(i2s_in, 0, queue,     0);

// ── Config ───────────────────────────────────────────────────
static const float SAMPLE_RATE  = 44100.0f;
static const float MARK_HZ      = 6000.0f;
static const float SPACE_HZ     = 8000.0f;
static const int   BLOCK_SIZE   = 128;
static const int   CALIBRATE_MS = 2000;
static const int   REPORT_MS    = 200;

// ── State ────────────────────────────────────────────────────
float noiseFloorPower   = 0.0f;
bool  calibrated        = false;
char  noiseFloorStr[40] = "";    // pre-formatted once, reused every line

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
    Serial.println(" AFSK SNR Meter — 6000/8000 Hz Goertzel");
    Serial.println(" Teensy 4.1 + SPH0645LM4H I2S Mic");
    Serial.println("========================================");
    Serial.println();
    Serial.println("CALIBRATION: Keep channel SILENT for 2s...");

    float    accum = 0.0f;
    int      count = 0;
    uint32_t t     = millis();

    while (millis() - t < CALIBRATE_MS) {
        if (queue.available()) {
            int16_t* block = queue.readBuffer();
            float mp = goertzel(block, BLOCK_SIZE, MARK_HZ);
            float sp = goertzel(block, BLOCK_SIZE, SPACE_HZ);
            accum += mp + sp;
            count++;
            queue.freeBuffer();
        }
        // Progress dot every ~200ms
        static uint32_t lastDot = 0;
        if (millis() - lastDot > 200) {
            Serial.print('.');
            lastDot = millis();
        }
    }

    noiseFloorPower = (count > 0) ? (accum / count) : 1e-12f;
    calibrated      = true;

    // Pre-format noise floor string — printed on every line from now on
    snprintf(noiseFloorStr, sizeof(noiseFloorStr),
             "NF=%.3e", noiseFloorPower);

    float nf_dB = 10.0f * log10f(max(noiseFloorPower, 1e-12f));

    Serial.println();
    Serial.println();
    Serial.println("── Calibration Result ──────────────────");
    Serial.printf( "   Noise floor power : %.6e\n", noiseFloorPower);
    Serial.printf( "   Noise floor        : %.1f dB\n", nf_dB);
    Serial.printf( "   Samples averaged   : %d\n", count);
    Serial.println("────────────────────────────────────────");
    Serial.println();
    Serial.println("NOW TRANSMIT: 01010101... at 10 baud");
    Serial.println("  Mark  = 6000 Hz | Space = 8000 Hz");
    Serial.println("  100ms per bit");
    Serial.println();

    // Column header
    Serial.printf("%-7s %-11s %-11s %-11s %-16s  %s\n",
                  "Tone",
                  "Mark Pwr",
                  "Space Pwr",
                  "SNR (dB)",
                  "Noise Floor",
                  "Bar (0–40dB)");
    Serial.println("----------------------------------------------------------------------");
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    AudioMemory(20);
    queue.begin();

    calibrate();
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
    if (!calibrated)        return;
    if (!queue.available()) return;

    // Read one DMA block of raw I2S samples
    int16_t* block  = queue.readBuffer();
    float markPow   = goertzel(block, BLOCK_SIZE, MARK_HZ);
    float spacePow  = goertzel(block, BLOCK_SIZE, SPACE_HZ);
    queue.freeBuffer();

    float signalPow = max(markPow, spacePow);
    float snr       = toDb(signalPow, noiseFloorPower);

    // Rate-limit output
    static uint32_t lastReport = 0;
    if (millis() - lastReport < REPORT_MS) return;
    lastReport = millis();

    // Determine which tone is dominant
    const char* label;
    if      (markPow  > spacePow * 2.0f) label = "MARK ";
    else if (spacePow > markPow  * 2.0f) label = "SPACE";
    else                                  label = "TRANS";   // transitioning

    // Print measurement — noise floor on every line
    Serial.printf("%-7s %-11.6f %-11.6f %+8.1f dB  %-16s  ",
                  label,
                  markPow,
                  spacePow,
                  snr,
                  noiseFloorStr);     // <-- always visible
    printBar(snr / 40.0f);            // full bar = 40 dB SNR
    Serial.println();

    // Warnings
    if (markPow < 1e-9f && spacePow < 1e-9f && snr < -90.0f) {
        Serial.println("  *** WARNING: No signal detected — "
                       "check mic wiring or transmitter ***");
    } else if (snr < 6.0f) {
        Serial.println("  *** WARNING: SNR < 6 dB — "
                       "decoding will be unreliable ***");
    } else if (snr < 12.0f) {
        Serial.println("  *** CAUTION: SNR < 12 dB — "
                       "marginal decoding quality ***");
    }
}
