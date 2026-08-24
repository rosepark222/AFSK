#include <Audio.h>
#include <arm_math.h>

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

#define AMPLITUDE 0.8f // 0.25f

// ─── Buffer ────────────────────────────────────────────────
int16_t chirp_buffer[N_SAMPLES];

// ─── Generate CLEAN Cyclic Chirp ──────────────────────────
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

        // Wrap frequency
        if (freq > F_END)
            freq -= bandwidth;

        // Phase accumulator
        phase += 2.0f * PI * freq / SAMPLE_RATE;

        // Keep phase bounded
        if (phase > 2 * PI)
            phase -= 2 * PI;

        float s = sinf(phase);

        // Hann window
        float w = 0.5f * (1.0f - cosf(2 * PI * n / (N_SAMPLES - 1)));

        float out = AMPLITUDE * s * w;

        chirp_buffer[n] = (int16_t)(out * 32767.0f);
    }
}

// ─── Play Function ────────────────────────────────────────
void playBuffer(int16_t *buf, int len)
{
    int idx = 0;

    while (idx < len) {

        if (queue.available() > 0) {

            int16_t *block = (int16_t)queue.getBuffer();

            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {

                if (idx < len)
                    block[i] = buf[idx++];
                else
                    block[i] = 0;
            }

            queue.playBuffer();
        }
    }
}

// ─── Setup ─────────────────────────────────────────────────
void setup()
{
    AudioMemory(20);

    generateChirp();
}

// ─── Loop ──────────────────────────────────────────────────
void loop()
{
    // Play 100 ms chirp
    playBuffer(chirp_buffer, N_SAMPLES);

    // Wait 1 second before playing the next chirp
    delay(1000);
}