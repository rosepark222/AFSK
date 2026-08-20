#include <Audio.h>
#include <arm_math.h>

// ================= AUDIO OBJECTS ==================
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;
AudioConnection   patchCord1(i2s_in, 0, queue1, 0);

// ================= TX-MATCHED PARAMETERS ==========
#define SAMPLE_RATE     44100
#define CHIRP_DURATION  0.08f
#define N_CHIRP         (int)(SAMPLE_RATE * CHIRP_DURATION)  // 3528

#define F_START    300.0f
#define F_END     1500.0f
#define NUM_SLOTS    16

// ================= DETECTION PARAMETERS ===========
#define THRESHOLD_ABS    0.005f   // absolute correlation peak
#define THRESHOLD_PSR    1.5f     // PSR — chirp gives ~2.0, tune down if missed
#define PSR_EXCLUDE      10       // samples either side of peak excluded from sidelobe stats
#define MIN_GAP_SAMPLES  (N_CHIRP / 2)

// ================= BUFFERS ========================
DMAMEM float circ_buf[N_CHIRP];
DMAMEM float ref_chirp[N_CHIRP];
DMAMEM float corr_buf[AUDIO_BLOCK_SAMPLES];

float ref_energy   = 0.0f;
int   circ_head    = 0;
int   samples_seen = 0;
int   global_pos   = 0;
int   last_detect  = -(MIN_GAP_SAMPLES + 1);

// ================= GENERATE REFERENCE =============
void generateReference(int ref_id = 0)
{
    float bandwidth  = F_END - F_START;
    float slot_width = bandwidth / NUM_SLOTS;
    float f0         = F_START + ref_id * slot_width;
    float k          = bandwidth / CHIRP_DURATION;
    float phase      = 0.0f;
    ref_energy       = 0.0f;

    float tmp[N_CHIRP];
    for (int n = 0; n < N_CHIRP; n++) {
        float freq = f0 + k * ((float)n / SAMPLE_RATE);
        if (freq > F_END) freq -= (F_END - F_START);

        phase += 2.0f * PI * freq / SAMPLE_RATE;
        if (phase > 2.0f * PI) phase -= 2.0f * PI;

        float s = sinf(phase);
        float w = 0.5f * (1.0f - cosf(2.0f * PI * n / (N_CHIRP - 1)));
        tmp[n]      = s * w;
        ref_energy += tmp[n] * tmp[n];
    }

    for (int n = 0; n < N_CHIRP; n++)
        ref_chirp[n] = tmp[N_CHIRP - 1 - n];

    Serial.print("N_CHIRP=");    Serial.println(N_CHIRP);
    Serial.print("ref_energy="); Serial.println(ref_energy, 4);
}

// ================= CORRELATE AT ONE HEAD POSITION =
inline float correlateAt(int head)
{
    float acc1 = 0.0f, acc2 = 0.0f;
    int seg1 = N_CHIRP - head;

    arm_dot_prod_f32(&circ_buf[head], &ref_chirp[0],     seg1,  &acc1);
    if (head > 0)
        arm_dot_prod_f32(&circ_buf[0], &ref_chirp[seg1], head,  &acc2);

    return (acc1 + acc2) / ref_energy;
}

// ================= PSR CALCULATION ================
float computePSR(float *buf, int len, int peak_i, float peak_val)
{
    float sum   = 0.0f;
    float sumsq = 0.0f;
    int   count = 0;

    for (int i = 0; i < len; i++) {
        if (abs(i - peak_i) <= PSR_EXCLUDE) continue;
        float v  = buf[i];
        sum   += v;
        sumsq += v * v;
        count++;
    }

    if (count < 2) return 0.0f;

    float mean = sum / count;
    float var  = (sumsq / count) - (mean * mean);
    if (var < 1e-12f) var = 1e-12f;

    return (peak_val - mean) / sqrtf(var);
}

// ================= LED BLINK (non-blocking) =======
// Instead of delay(), use a timer so audio queue keeps draining
#define LED_BLINK_MS  80

unsigned long led_on_time  = 0;
bool          led_is_on    = false;

void ledUpdate()
{
    if (led_is_on && (millis() - led_on_time >= LED_BLINK_MS)) {
        digitalWrite(LED_BUILTIN, LOW);
        led_is_on = false;
    }
}

void ledBlink()
{
    digitalWrite(LED_BUILTIN, HIGH);
    led_on_time = millis();
    led_is_on   = true;
}

// ================= DETECTION CALLBACK =============
void onDetect(int pos, float peak, float psr)
{
    // ledBlink();   // non-blocking
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);

    float time_s = (float)pos / SAMPLE_RATE;
    Serial.print(">>> CHIRP DETECTED  t=");
    Serial.print(time_s, 4);
    Serial.print(" s   peak=");
    Serial.print(peak, 6);
    Serial.print("   PSR=");
    Serial.println(psr, 2);
}

// ================= SETUP ==========================
void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    AudioMemory(160);
    queue1.begin();

    generateReference(0);
    memset(circ_buf,  0, sizeof(circ_buf));
    memset(corr_buf,  0, sizeof(corr_buf));

    Serial.println("RX started (abs + PSR gates).");
    Serial.print("THRESHOLD_ABS="); Serial.println(THRESHOLD_ABS, 6);
    Serial.print("THRESHOLD_PSR="); Serial.println(THRESHOLD_PSR, 2);
}

// ================= LOOP ===========================
void loop()
{
    ledUpdate();   // turn LED off after blink duration, without delay()

    while (queue1.available()) {

        // ---- 1. Drain audio into circular buffer ----
        int16_t *block = queue1.readBuffer();
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            circ_buf[circ_head] = block[i] / 32768.0f;
            circ_head = (circ_head + 1) % N_CHIRP;
            samples_seen++;
            global_pos++;
        }
        queue1.freeBuffer();

        if (samples_seen < N_CHIRP) continue;

        // ---- 2. Compute correlation at all 128 positions in this block ----
        float block_peak   = 0.0f;
        int   block_peak_i = 0;

        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            int   head_i = (circ_head - AUDIO_BLOCK_SAMPLES + i + N_CHIRP) % N_CHIRP;
            float c      = correlateAt(head_i);
            corr_buf[i]  = c;

            float abs_c = fabsf(c);
            if (abs_c > block_peak) {
                block_peak   = abs_c;
                block_peak_i = i;
            }
        }

        // ---- 3. Debug print every 20 blocks (~58 ms) ----
        static int dbg = 0;
        if (++dbg % 20 == 0) {
            Serial.print("block_peak=");
            Serial.print(block_peak, 6);
            if (block_peak >= THRESHOLD_ABS) {
                float psr_dbg = computePSR(corr_buf, AUDIO_BLOCK_SAMPLES,
                                           block_peak_i,
                                           corr_buf[block_peak_i]);
                Serial.print("   PSR=");
                Serial.print(psr_dbg, 2);
            }
            Serial.println();
        }

        // ---- 4. Gate 1: absolute threshold ----
        if (block_peak < THRESHOLD_ABS) continue;

        // ---- 5. Gate 2: PSR ----
        float psr = computePSR(corr_buf, AUDIO_BLOCK_SAMPLES,
                               block_peak_i,
                               corr_buf[block_peak_i]);
        if (psr < THRESHOLD_PSR) continue;

        // ---- 6. Debounce + fire ----
        int pos = global_pos - AUDIO_BLOCK_SAMPLES + block_peak_i;
        if ((pos - last_detect) >= MIN_GAP_SAMPLES) {
            last_detect = pos;
            onDetect(pos, block_peak, psr);
        }
    }
}
