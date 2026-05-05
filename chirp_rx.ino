#include <Audio.h>
#include <arm_math.h>

// ================= DEBUG SWITCHES =================
#define DEBUG_AUDIO   1
#define DEBUG_CORR    1
#define DEBUG_PEAKS   1
#define DEBUG_ID      1

// ================= AUDIO SETUP ====================
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;
AudioConnection   patchCord1(i2s_in, 0, queue1, 0);

// ================= PARAMETERS =====================
#define SAMPLE_RATE 44100
#define FFT_SIZE    1024
#define HOP_SIZE    256
#define CIRC_SIZE   4096

#define F_START 300.0f
#define F_END   1500.0f
#define CHIRP_DURATION 0.5f

#define MAX_PEAKS 4
#define THRESHOLD 0.5f   // start low!

// ================= BUFFERS ========================
float circ_buf[CIRC_SIZE];
int write_idx = 0;

float window_buf[FFT_SIZE];

// FFT buffers
float fft_rx[FFT_SIZE];
float fft_ref[FFT_SIZE];
float fft_corr[FFT_SIZE];

// unpacked complex
float rx_re[FFT_SIZE/2];
float rx_im[FFT_SIZE/2];
float ref_re[FFT_SIZE/2];
float ref_im[FFT_SIZE/2];
float corr_re[FFT_SIZE/2];
float corr_im[FFT_SIZE/2];

// dechirp
float down_re[FFT_SIZE];
float down_im[FFT_SIZE];

float fft_buf[FFT_SIZE];

arm_rfft_fast_instance_f32 rfft;

// ================= REFERENCE ======================
void generateReference()
{
    float k = (F_END - F_START) / CHIRP_DURATION;
    static float ref_time[FFT_SIZE];

    for (int n = 0; n < FFT_SIZE; n++) {
        float t = (float)n / SAMPLE_RATE;
        float phase = 2 * PI * (F_START * t + 0.5f * k * t * t);

        ref_time[n] = cosf(phase);

        down_re[n] =  cosf(phase);
        down_im[n] = -sinf(phase);
    }

    arm_rfft_fast_f32(&rfft, ref_time, fft_ref, 0);

    for (int k = 0; k < FFT_SIZE/2; k++) {
        ref_re[k] = fft_ref[2*k];
        ref_im[k] = fft_ref[2*k+1];
    }
}

// ================= AUDIO INPUT ====================
void pushAudio()
{
    static int counter = 0;
    float peak = 0;

    while (queue1.available()) {

        int16_t *data = queue1.readBuffer();

        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {

            float v = data[i] / 32768.0f;

            circ_buf[write_idx] = v;

            if (fabsf(v) > peak) peak = fabsf(v);

            write_idx = (write_idx + 1) % CIRC_SIZE;
        }

        queue1.freeBuffer();
    }

#if DEBUG_AUDIO
    if (++counter % 50 == 0) {
        Serial.print("Audio peak: ");
        Serial.println(peak, 6);
    }
#endif
}

// ================= WINDOW =========================
void getWindow(int start)
{
    float energy = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        int idx = (start + i) % CIRC_SIZE;
        window_buf[i] = circ_buf[idx];
        energy += window_buf[i] * window_buf[i];
    }

#if DEBUG_AUDIO
    Serial.print("Window energy: ");
    Serial.println(energy);
#endif
}

// ================= FFT CORRELATION ================
void fftCorrelation()
{
    arm_rfft_fast_f32(&rfft, window_buf, fft_rx, 0);

    for (int k = 0; k < FFT_SIZE/2; k++) {
        rx_re[k] = fft_rx[2*k];
        rx_im[k] = fft_rx[2*k+1];
    }

    for (int k = 0; k < FFT_SIZE/2; k++) {

        float a_re = rx_re[k];
        float a_im = rx_im[k];

        float b_re = ref_re[k];
        float b_im = -ref_im[k];

        corr_re[k] = a_re*b_re - a_im*b_im;
        corr_im[k] = a_re*b_im + a_im*b_re;
    }

    for (int k = 0; k < FFT_SIZE/2; k++) {
        fft_corr[2*k]   = corr_re[k];
        fft_corr[2*k+1] = corr_im[k];
    }

    arm_rfft_fast_f32(&rfft, fft_corr, fft_corr, 1);

    float max_val = 0;
    int max_idx = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        fft_corr[i] /= FFT_SIZE;

        float v = fabsf(fft_corr[i]);
        if (v > max_val) {
            max_val = v;
            max_idx = i;
        }
    }

#if DEBUG_CORR
    Serial.print("Corr max: ");
    Serial.print(max_val);
    Serial.print(" @ ");
    Serial.println(max_idx);
#endif
}

// ================= PEAK DETECTION =================
int findPeaks(int *peaks)
{
    int count = 0;

    for (int i = 1; i < FFT_SIZE-1; i++) {

        float v = fabsf(fft_corr[i]);

        if (v > THRESHOLD &&
            v > fabsf(fft_corr[i-1]) &&
            v > fabsf(fft_corr[i+1])) {

            peaks[count++] = i;

#if DEBUG_PEAKS
            Serial.print("Peak @ ");
            Serial.print(i);
            Serial.print(" val=");
            Serial.println(v);
#endif

            if (count >= MAX_PEAKS) break;
        }
    }

#if DEBUG_PEAKS
    if (count == 0) Serial.println("No peaks");
#endif

    return count;
}

// ================= DECHIRP ========================
void dechirp(int start)
{
#if DEBUG_ID
    Serial.print("Dechirp start: ");
    Serial.println(start);
#endif

    for (int i = 0; i < FFT_SIZE; i++) {

        int idx = (start + i) % CIRC_SIZE;

        float x = circ_buf[idx];

        float re = x * down_re[i];
        float im = x * down_im[i];

        fft_buf[i] = re; // simplified
    }
}

// ================= ID DETECTION ===================
int detectID()
{
    arm_rfft_fast_f32(&rfft, fft_buf, fft_buf, 0);

    float max_val = 0;
    int max_bin = 0;

    for (int i = 1; i < FFT_SIZE/2; i++) {

        float re = fft_buf[2*i];
        float im = fft_buf[2*i+1];

        float mag = re*re + im*im;

        if (mag > max_val) {
            max_val = mag;
            max_bin = i;
        }
    }

#if DEBUG_ID
    Serial.print("ID bin=");
    Serial.print(max_bin);
    Serial.print(" mag=");
    Serial.println(max_val);
#endif

    return max_bin;
}

// ================= SETUP ==========================
void setup()
{
    Serial.begin(115200);

    AudioMemory(80);
    queue1.begin();

    arm_rfft_fast_init_f32(&rfft, FFT_SIZE);

    generateReference();

    Serial.println("RX Started...");
}

// ================= LOOP ===========================
void loop()
{
    pushAudio();

    static int read_idx = 0;

    getWindow(read_idx);

    fftCorrelation();

    int peaks[MAX_PEAKS];
    int n = findPeaks(peaks);

    for (int i = 0; i < n; i++) {

        dechirp(peaks[i]);

        int id = detectID();

        Serial.print("FINAL ID=");
        Serial.println(id);
    }

    read_idx = (read_idx + HOP_SIZE) % CIRC_SIZE;
}