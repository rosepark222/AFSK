#include <Audio.h>
#include <arm_math.h>

// ================= AUDIO OBJECTS ==================
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;
AudioConnection   patchCord1(i2s_in, 0, queue1, 0);

// ================= PARAMETERS =====================
#define SAMPLE_RATE     44100
#define CHIRP_DURATION  1.0f
#define N_CHIRP         (int)(SAMPLE_RATE * CHIRP_DURATION)  // 44100
#define ID              5
#define F_START    300.0f
#define F_END     1500.0f
#define NUM_SLOTS    16

#define THRESHOLD      0.01f
#define MIN_GAP_BLOCKS 344    // one chirp duration = 44100/128 blocks

int blocks_since_detect = 0;
// ================= BUFFERS ========================
DMAMEM float circ_buf[N_CHIRP];
DMAMEM float ref_chirp[N_CHIRP];

float ref_energy = 0.0f;
int   circ_head  = 0;
int   samples_seen = 0;

// ================= GENERATE REFERENCE =============
void generateReference(int ref_id = 0)
{
    float bandwidth  = F_END - F_START;
    float slot_width = bandwidth / NUM_SLOTS;
    float f0         = F_START + ref_id * slot_width;
    float k          = bandwidth / CHIRP_DURATION;
    float phase      = 0.0f;
    ref_energy       = 0.0f;

    for (int n = 0; n < N_CHIRP; n++) {
        float freq = f0 + k * ((float)n / SAMPLE_RATE);
        if (freq > F_END) freq -= (F_END - F_START);

        phase += 2.0f * PI * freq / SAMPLE_RATE;
        if (phase > 2.0f * PI) phase -= 2.0f * PI;

        float s = sinf(phase);
        float w = 0.5f * (1.0f - cosf(2.0f * PI * n / (N_CHIRP - 1)));
        ref_chirp[n]  = s * w;
        ref_energy   += ref_chirp[n] * ref_chirp[n];
    }

    // // Reverse in-place for matched filter
    // for (int n = 0; n < N_CHIRP / 2; n++) {
    //     float tmp                  = ref_chirp[n];
    //     ref_chirp[n]               = ref_chirp[N_CHIRP - 1 - n];
    //     ref_chirp[N_CHIRP - 1 - n] = tmp;
    // }
}

// ================= CORRELATE ======================
// One dot product against the full circular buffer — called once per block
inline float correlateNow_normalized_only_by_rx_energy()
{
    float acc = 0.0f;
    for (int n = 0; n < N_CHIRP; n++) {
        // newest sample = circ_head-1, oldest = circ_head
        // newest should multiply ref_chirp[N_CHIRP-1] (last ref sample)
        // oldest should multiply ref_chirp[0] (first ref sample)
        int idx = (circ_head - 1 - n + N_CHIRP) % N_CHIRP;  // newest first
        acc += circ_buf[idx] * ref_chirp[N_CHIRP - 1 - n];   // ref also newest first
    }
    return acc / ref_energy;
}

inline float correlateNow()
{
    float acc      = 0.0f;
    float rx_energy = 0.0f;

    for (int n = 0; n < N_CHIRP; n++) {
        int idx = (circ_head - 1 - n + N_CHIRP) % N_CHIRP;
        float rx = circ_buf[idx];
        acc      += rx * ref_chirp[N_CHIRP - 1 - n];
        rx_energy += rx * rx;
    }

    float denom = sqrtf(rx_energy * ref_energy);
    if (denom < 1e-10f) return 0.0f;
    return acc / denom;
}
// ================= SETUP ==========================
void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    AudioMemory(160);
    queue1.begin();

    generateReference(ID);
    memset(circ_buf, 0, sizeof(circ_buf));


    // Self-test: correlate reference against itself — must return 1.000000
    memset(circ_buf, 0, sizeof(circ_buf));
    circ_head = 0;

    // Fill circ_buf with forward reference (what TX sends)
    float bandwidth = F_END - F_START;
    float f0        = F_START;
    float k         = bandwidth / CHIRP_DURATION;
    float phase     = 0.0f;
    for (int n = 0; n < N_CHIRP; n++) {
        float freq = f0 + k * ((float)n / SAMPLE_RATE);
        if (freq > F_END) freq -= bandwidth;
        phase += 2.0f * PI * freq / SAMPLE_RATE;
        if (phase > 2.0f * PI) phase -= 2.0f * PI;
        float s = sinf(phase);
        float w = 0.5f * (1.0f - cosf(2.0f * PI * n / (N_CHIRP - 1)));
        circ_buf[n] = s * w;
    }

    circ_head = 0;

    Serial.print("circ_buf[100]=");    Serial.println(circ_buf[100], 6);
    Serial.print("circ_buf[N-101]=");  Serial.println(circ_buf[N_CHIRP-101], 6);
    Serial.print("ref_chirp[100]=");   Serial.println(ref_chirp[100], 6);
    Serial.print("ref_chirp[N-101]="); Serial.println(ref_chirp[N_CHIRP-101], 6);
    float self_corr = correlateNow();
    Serial.print("Self-test (expect 1.000000): ");
    Serial.println(self_corr, 6);

    // Clean up for real operation
    memset(circ_buf, 0, sizeof(circ_buf));
    circ_head    = 0;
    samples_seen = 0;

    //delay(50000);

}
void ledBlink()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
}
// ================= LOOP ===========================
void loop()
{
    while (queue1.available()) {

        int16_t *block = queue1.readBuffer();
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            circ_buf[circ_head] = block[i] / 32768.0f;
            circ_head = (circ_head + 1) % N_CHIRP;
            samples_seen++;
        }
        queue1.freeBuffer();

        if (samples_seen < N_CHIRP) continue;

        float corr  = correlateNow();
        float abs_c = fabsf(corr);

// in loop(), after computing corr:
blocks_since_detect++;
if (abs_c > THRESHOLD && blocks_since_detect > MIN_GAP_BLOCKS) {
    blocks_since_detect = 0;
    ledBlink();
    Serial.println(">>> CHIRP DETECTED");
}
        // Track raw audio peak this block
        float raw_peak = 0.0f;
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            float v = fabsf(circ_buf[(circ_head - AUDIO_BLOCK_SAMPLES + i + N_CHIRP) % N_CHIRP]);
            if (v > raw_peak) raw_peak = v;
        }

        const char *marker = "";
        if      (abs_c > 0.01f)    marker = "####";
        else if (abs_c > 0.001f)   marker = "###";
        else if (abs_c > 0.0001f)  marker = "##";
        else if (abs_c > 0.00001f) marker = "#";

        char buf[48];
        snprintf(buf, sizeof(buf), "%10.6f  %-4s  raw=%8.6f", corr, marker, raw_peak);
        Serial.println(buf);
    }
}