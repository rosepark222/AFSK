#include <Audio.h>
#include <arm_math.h>

// ================= AUDIO OBJECTS ==================
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;
//AudioConnection   patchCord1(i2s_in, 0, queue1, 0);
AudioFilterBiquad  dcBlocker;
AudioConnection    patchCord1(i2s_in, 0, dcBlocker, 0);
AudioConnection    patchCord2(dcBlocker, 0, queue1,  0);  // replace direct connection

// ================= PARAMETERS =====================
#define SAMPLE_RATE     44100
#define CHIRP_DURATION  1.0f
#define N_CHIRP         (int)(SAMPLE_RATE * CHIRP_DURATION)  // 44100
#define ID              5
#define F_START         300.0f
#define F_END           1500.0f
#define NUM_SLOTS       16

#define THRESHOLD        0.01f   // coarse threshold to trigger fine search
#define MIN_GAP_BLOCKS   344     // one chirp duration in blocks

// ================= BUFFERS ========================
DMAMEM float circ_buf[N_CHIRP];
DMAMEM float ref_chirp[N_CHIRP];

float ref_energy     = 0.0f;
int   circ_head      = 0;
int   samples_seen   = 0;
int   blocks_since_detect = 0;

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
}

// ================= CORRELATE AT ARBITRARY HEAD ====
// head = the index of the NEXT write position (oldest sample)
// offset allows fine search: shift head by -offset samples
inline float correlateAt(int head)
{
    float acc       = 0.0f;
    float rx_energy = 0.0f;

    for (int n = 0; n < N_CHIRP; n++) {
        int   idx = (head - 1 - n + N_CHIRP) % N_CHIRP;
        float rx  = circ_buf[idx];
        acc       += rx * ref_chirp[N_CHIRP - 1 - n];
        rx_energy += rx * rx;
    }

    float denom = sqrtf(rx_energy * ref_energy);
    if (denom < 1e-10f) return 0.0f;
    return acc / denom;
}

// ================= FINE SEARCH ====================
// Called when coarse pass exceeds threshold.
// Tries all 128 sample offsets within current block,
// returns the best correlation and its offset.
float fineSearch(int &best_offset)
{
    float best_corr = 0.0f;
    best_offset     = 0;

    for (int offset = 0; offset < AUDIO_BLOCK_SAMPLES; offset++) {
        // shift circ_head back by offset to check earlier alignments
        int head = (circ_head - offset + N_CHIRP) % N_CHIRP;
        float c  = fabsf(correlateAt(head));
        if (c > best_corr) {
            best_corr   = c;
            best_offset = offset;
        }
    }
    return best_corr;
}

// ================= LED ============================
void ledBlink()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
}

// ================= SETUP ==========================
void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    AudioMemory(160);
    dcBlocker.setHighpass(0, 20, 0.707);  // 20Hz highpass removes DC
    queue1.begin();

    generateReference(ID);
    memset(circ_buf, 0, sizeof(circ_buf));

    // ---- Self-test ----
    float bandwidth = F_END - F_START;
    float f0        = F_START + ID * (bandwidth / NUM_SLOTS);
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
    float self_corr = correlateAt(0);
    Serial.print("Self-test (expect 1.000000): ");
    Serial.println(self_corr, 6);

    memset(circ_buf, 0, sizeof(circ_buf));
    circ_head    = 0;
    samples_seen = 0;

    Serial.println("RX ready.");
}

// ================= LOOP ===========================
void loop()
{
    while (queue1.available()) {

        // 1. Drain block into circular buffer
        int16_t *block = queue1.readBuffer();
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            circ_buf[circ_head] = block[i] / 32768.0f;
            circ_head = (circ_head + 1) % N_CHIRP;
            samples_seen++;
        }
        queue1.freeBuffer();

        if (samples_seen < N_CHIRP) continue;

        // 2. Coarse correlation — one per block
        float coarse = fabsf(correlateAt(circ_head));
        blocks_since_detect++;

        // 3. Print coarse value every block
        const char *marker = "";
        if      (coarse > 0.1f)   marker = "####";
        else if (coarse > 0.01f)  marker = "###";
        else if (coarse > 0.001f) marker = "##";
        else if (coarse > 0.0001f)marker = "#";

        if(coarse > 0.1f) {
          char buf[48];
          snprintf(buf, sizeof(buf), "coarse=%8.6f  %-4s", coarse, marker);
          Serial.println(buf);
        }

        // 4. Fine search — only when coarse exceeds threshold
        if (coarse > THRESHOLD && blocks_since_detect > MIN_GAP_BLOCKS) {

            int   best_offset;
            float fine = fineSearch(best_offset);

            Serial.print("  >>> FINE peak=");
            Serial.print(fine, 6);
            Serial.print("  offset=");
            Serial.println(best_offset);

            if (fine > THRESHOLD) {
                blocks_since_detect = 0;
                ledBlink();
                Serial.println("  >>> CHIRP DETECTED");
            }
        }
    }
}