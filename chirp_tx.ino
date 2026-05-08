#include <Audio.h>
#include <arm_math.h>

// ─── Audio Objects ─────────────────────────────────────────
AudioPlayQueue     queue;
AudioOutputI2S     i2s1;
AudioConnection    patchCord1(queue, 0, i2s1, 0);
AudioConnection    patchCord2(queue, 0, i2s1, 1);

// ─── Parameters ────────────────────────────────────────────
#define SAMPLE_RATE     44100
#define CHIRP_DURATION  0.1f
#define N_SAMPLES       (int)(SAMPLE_RATE * CHIRP_DURATION)

#define F_START 300.0f
#define F_END   1500.0f

#define ID 5
#define NUM_SLOTS 16

#define AMPLITUDE 0.25f

// ─── Buffer ────────────────────────────────────────────────
int16_t chirp_buffer[N_SAMPLES];

// ─── Generate CLEAN Cyclic Chirp ───────────────────────────
void generateChirp()
{
    float bandwidth = F_END - F_START;
    float slot_width = bandwidth / NUM_SLOTS;
    float f0 = F_START + ID * slot_width;
    float k = bandwidth / CHIRP_DURATION;
    float phase = 0.0f;

    for (int n = 0; n < N_SAMPLES; n++) {
        float t = (float)n / SAMPLE_RATE;
        float freq = f0 + k * t;
        if (freq > F_END) freq -= bandwidth;
        phase += 2.0f * PI * freq / SAMPLE_RATE;
        if (phase > 2 * PI) phase -= 2 * PI;
        float s = sinf(phase);
        float w = 0.5f * (1.0f - cosf(2 * PI * n / (N_SAMPLES - 1)));
        chirp_buffer[n] = (int16_t)(AMPLITUDE * s * w * 32767.0f);
    }
}

// ─── Play Function (Instrumented) ──────────────────────────
void playBuffer(int16_t *buf, int len)
{
    static uint32_t call_count = 0;
    call_count++;

    uint32_t t_start = micros();
    int idx = 0;
    int blocks_submitted = 0;
    uint32_t wait_total_us = 0;   // total time spent waiting for queue
    uint32_t wait_count    = 0;   // how many times we had to wait
    uint32_t worst_wait_us = 0;   // single longest wait

    Serial.printf("\n[chirp #%lu] START  idx=%d  len=%d  queue.avail=%d\n",
                  call_count, idx, len, queue.available());

    while (idx < len) {

        if (queue.available() > 0) {

            int16_t *block = (int16_t*)queue.getBuffer();

            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
                block[i] = (idx < len) ? buf[idx++] : 0;
            }

            queue.playBuffer();
            blocks_submitted++;

            // ── Log every 8 blocks so we can see progress without flooding ──
            if (blocks_submitted % 8 == 0) {
                Serial.printf("  [block %2d]  idx=%4d / %4d  t=%lu us  queue.avail=%d\n",
                              blocks_submitted, idx, len,
                              (uint32_t)(micros() - t_start),
                              queue.available());
            }

        } else {

            // ── Queue full — measure how long we spin ──
            uint32_t wait_start = micros();

            while (queue.available() == 0) { /* busy wait */ }

            uint32_t waited = micros() - wait_start;
            wait_total_us += waited;
            wait_count++;
            if (waited > worst_wait_us) worst_wait_us = waited;

            // Print every stall so we can see exactly where it happens
            Serial.printf("  [STALL #%lu]  idx=%4d  waited=%lu us  blocks_done=%d\n",
                          wait_count, idx, waited, blocks_submitted);
        }
    }

    uint32_t t_end = micros();

    Serial.printf("[chirp #%lu] END    blocks=%d  total_time=%lu us  "
                  "stalls=%lu  wait_total=%lu us  worst_stall=%lu us\n",
                  call_count, blocks_submitted,
                  (uint32_t)(t_end - t_start),
                  wait_count, wait_total_us, worst_wait_us);

    // ── Key diagnostic: how much time between this call returning
    //    and the next one starting (measured outside, see loop()) ──
}

// ─── Setup ─────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000);   // wait for USB serial (up to 3 s)

    AudioMemory(20);
    generateChirp();

    Serial.printf("N_SAMPLES=%d  AUDIO_BLOCK_SAMPLES=%d  blocks_needed=%.2f\n",
                  N_SAMPLES, AUDIO_BLOCK_SAMPLES,
                  (float)N_SAMPLES / AUDIO_BLOCK_SAMPLES);
}

// ─── Loop ──────────────────────────────────────────────────
void loop()
{
    static uint32_t t_last_end = 0;

    if (t_last_end != 0) {
        // Time between playBuffer() returning and being called again
        Serial.printf("[gap between calls: %lu us]\n", micros() - t_last_end);
    }

    playBuffer(chirp_buffer, N_SAMPLES);

    t_last_end = micros();
}