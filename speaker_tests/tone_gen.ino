#include <Audio.h>
#include <arm_math.h>

// ─── Audio Objects ─────────────────────────────────────────
AudioPlayQueue     queue;
AudioOutputI2S     i2s1;
AudioConnection    patchCord1(queue, 0, i2s1, 0);
AudioConnection    patchCord2(queue, 0, i2s1, 1);

// ─── Parameters ────────────────────────────────────────────
#define SAMPLE_RATE     44100
#define TONE_DURATION   0.1f
#define TONE_SAMPLES    (int)(SAMPLE_RATE * TONE_DURATION)

#define AMPLITUDE       0.5f

// ─── Tone buffer ────────────────────────────────────────────
int16_t tone_buffer[TONE_SAMPLES];

// ─── Generate a single sine tone ───────────────────────────
void generateTone(float frequency)
{
    for (int n = 0; n < TONE_SAMPLES; n++) {
        float t = (float)n / SAMPLE_RATE;
        float phase = 2.0f * PI * frequency * t;
        float sample = sinf(phase);

        tone_buffer[n] = (int16_t)(AMPLITUDE * sample * 32767.0f);
    }
}

// ─── Play Function ────────────────────────────────────────
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

void playTone(float frequency)
{
    generateTone(frequency);
    playBuffer(tone_buffer, TONE_SAMPLES);
}

// ─── Setup ─────────────────────────────────────────────────
void setup()
{
    AudioMemory(100);
}

// ─── Loop ──────────────────────────────────────────────────
void loop()
{
    static const float tones[] = {
        1000.0f, 2000.0f, 3000.0f, 4000.0f, 5000.0f, 6000.0f, 7000.0f, 8000.0f,
        9000.0f, 10000.0f, 11000.0f, 12000.0f, 13000.0f, 14000.0f, 15000.0f, 16000.0f,
        17000.0f, 18000.0f, 19000.0f, 20000.0f, 21000.0f, 22000.0f,
        21000.0f, 20000.0f, 19000.0f, 18000.0f, 17000.0f, 16000.0f, 15000.0f, 14000.0f,
        13000.0f, 12000.0f, 11000.0f, 10000.0f, 9000.0f, 8000.0f, 7000.0f, 6000.0f,
        5000.0f, 4000.0f, 3000.0f, 2000.0f, 1000.0f
    };

    for (size_t i = 0; i < sizeof(tones) / sizeof(tones[0]); i++) {
        playTone(tones[i]);
    }

    delay(500);
}
