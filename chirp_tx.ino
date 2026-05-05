#include <Audio.h>
#include <arm_math.h>

// ─── Audio Objects ─────────────────────────────────────────
AudioPlayQueue     queue;
AudioOutputI2S     i2s1;
AudioConnection    patchCord1(queue, 0, i2s1, 0);
AudioConnection    patchCord2(queue, 0, i2s1, 1);

// ─── Parameters ────────────────────────────────────────────
#define SAMPLE_RATE 44100
#define CHIRP_DURATION 0.5f
#define N_SAMPLES (int)(SAMPLE_RATE * CHIRP_DURATION)

#define F_START 300.0f
#define F_END   1500.0f

#define ID 5
#define NUM_SLOTS 16

int16_t chirp_buffer[N_SAMPLES];

// ─── Generate Cyclic Chirp ─────────────────────────────────
void generateChirp()
{
    float bandwidth = F_END - F_START;
    float slot_width = bandwidth / NUM_SLOTS;
    float f0 = F_START + ID * slot_width;

    float k = bandwidth / CHIRP_DURATION;

    for (int n = 0; n < N_SAMPLES; n++) {
        float t = (float)n / SAMPLE_RATE;

        float freq = f0 + k * t;

        // Wrap around
        if (freq > F_END)
            freq -= bandwidth;

        float phase = 2.0f * PI * (f0 * t + 0.5f * k * t * t);

        float s = sinf(phase);

        chirp_buffer[n] = (int16_t)(s * 30000); // scale
    }
}

// ─── Play Function ─────────────────────────────────────────
void playBuffer(int16_t *buf, int len)
{
    int idx = 0;

    while (idx < len) {
        if (queue.available() > 0) {
            int16_t *block = (int16_t*)queue.getBuffer();

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

void setup() {
    AudioMemory(20);
    generateChirp();
}

void loop() {

    // Pilot chirp
    playBuffer(chirp_buffer, N_SAMPLES);

    // ID chirp (same for now, can vary later)
    playBuffer(chirp_buffer, N_SAMPLES);

    delay(1000); // gap
}