/*
Overall flow:
1. Search the incoming audio for a rough chirp start using a sliding cross-correlation
   against the known reference chirp (real_ref, loaded from SD).
2. When the correlation exceeds threshold, stop the audio queues and run
   processChirpDetection():
     a. save raw leftRingBuf / rightRingBuf (time-ordered) to SD   [SAVE_RAW_RINGS]
     b. fine cross-correlation, real only, debug                   [DEBUG_XCORR]
     c. analytic (I/Q) cross-correlation, debug                    [DEBUG_ANALYTIC_XCORR]
        (b and c never feed into the FFT)
     d. de-rotate + DC-remove left rx, mix with real_ref / imag_ref (in place)
     e. 32768-point FFT (8 x 4096, in place), magnitude, peak search
     f. save FFT results to SD, log peak to Serial + log file
     g. reset rings and restart the audio queues
3. The FFT replaces the expensive cross-correlation as the measurement.
   The FFT input uses ONLY the rough trigger alignment (searchHead) -- no lag from xcorr.

Analytic xcorr:
   I(lag) = sum_n real_ref[n] * rx[n - lag]
   Q(lag) = sum_n imag_ref[n] * rx[n - lag]     (imag_ref = Hilbert transform of real_ref)
   A(lag) = sqrt(I^2 + Q^2) / sqrt(rxEnergy * refEnergy)
   A is independent of the carrier phase of the received chirp. Lag sign convention is the
   same as the real xcorr: rx delayed by d samples peaks at lag = -d.

Buffer sizes (chirp is 32768 samples):
  real_ref[CHIRP_SAMPLES]       128 KB (static, RAM1)
  imag_ref[CHIRP_SAMPLES]       128 KB (static, RAM2)   ---- moved to non-malloc (preoccupy) to RAM2
  rowBuf + visitedBits + twiddles ~ 37 KB (static, RAM1)
  leftRingBuf                   128 KB (malloc) 
  rightRingBuf                  128 KB (malloc)


  Memory Usage on Teensy 4.1:
  FLASH: code:135816, data:54668, headers:9192   free for files:7926788
   RAM1: variables:235328, code:132792, padding:31048   free for local variables:125120
   RAM2: variables:191584  free for malloc/new:332704

note that variables and code left free < 128KB in RAM1. If imag_ref has been in RAM1,
it would not have been fit in RAM1.

*/

#include <Arduino.h>
#include <Audio.h>
#include <SD.h>
#include <MTP_Teensy.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <TimeLib.h>

// ============================================================
// File names (all SD file names live here)
// ============================================================

#define SD_PREFIX             "B/"

// Reference chirp (input, one float per line / whitespace separated)
#define FILE_REAL_REF         SD_PREFIX "real_300-1500hz_32k_chirp.txt"
#define FILE_IMAG_REF         SD_PREFIX "imag_300-1500hz_32k_chirp.txt"

// Log file
#define FILE_CHIRP_LOG        SD_PREFIX "c_chirp_log.txt"

// Per-detection dump files. Each is a printf format taking one %s: the event tag.
#define FILE_LEFT_DUMP_FMT    SD_PREFIX "c_left_%s.txt"
#define FILE_RIGHT_DUMP_FMT   SD_PREFIX "c_right_%s.txt"
#define FILE_XCORR_DUMP_FMT   SD_PREFIX "c_xcorr_%s.txt"
#define FILE_ANALYTIC_XCORR_DUMP_FMT   SD_PREFIX "c_analytic_xcorr_%s.txt"
#define FILE_FFT_REAL_FMT     SD_PREFIX "c_fftre_%s.txt"
#define FILE_FFT_IMAG_FMT     SD_PREFIX "c_fftim_%s.txt"
#define FILE_FFT_MAG_FMT      SD_PREFIX "c_fftmag_%s.txt"

// ============================================================
// Feature flags
// ============================================================

static constexpr bool SAVE_RAW_RINGS         = true;   // left/right dumps before the FFT destroys them
static constexpr bool DEBUG_XCORR            = true;   // real-only fine xcorr + its dump + its Serial/log lines
static constexpr bool DEBUG_ANALYTIC_XCORR   = true;   // analytic I/Q xcorr + its dump + its Serial/log lines
static constexpr bool SAVE_FFT_MAG           = true;   // magnitude file
static constexpr bool SAVE_FFT_REAL_IMAG     = false;  // FFT real/imag files

// 1: im_mixed = -rx*imag_ref (conjugate reference), 0: +rx*imag_ref
#define MIX_NEGATE_IMAG  1

// Peak search is restricted to |f| <= PEAK_SEARCH_MAX_HZ around DC (the low
// beat-frequency band), on both the positive and negative side. This avoids
// picking the high-frequency "sum" term of real-rx x complex-ref mixing.
static constexpr float PEAK_SEARCH_MAX_HZ = 300.0f;

// ============================================================
// Parameters
// ============================================================

static constexpr float FS = 44100.0f;

// Chirp is 8 * 4096 = 32768 samples (~0.743 s at 44.1 kHz)
static constexpr uint32_t CHIRP_SAMPLES = 8u * 4096u;
static constexpr float CHIRP_DUR_S = (float)CHIRP_SAMPLES / FS;
static constexpr float SILENCE_DUR_S = 2.0f;
static constexpr float SOS_PERIOD_S = CHIRP_DUR_S + SILENCE_DUR_S;

static constexpr float F0 = 300.0f;
static constexpr float F1 = 1500.0f;

// 4096-point complex FFT
static constexpr uint32_t FFT_N = 4096;
static constexpr uint32_t NUM_ROWS = 8;
static constexpr uint32_t N_TOTAL = FFT_N * NUM_ROWS;   // 32768
static_assert(N_TOTAL == CHIRP_SAMPLES, "FFT size must equal chirp length");

// Chirp period (chirp + silence) in samples
static constexpr uint32_t SOS_PERIOD_SAMPLES = (uint32_t)(FS * SOS_PERIOD_S);

// 32768 / 4096 = exactly 8 blocks
static constexpr uint32_t BLOCKS_PER_CHIRP = (CHIRP_SAMPLES + FFT_N - 1) / FFT_N;

// Detection threshold
static constexpr float DETECT_RATIO_THRESHOLD = 8.0f;
static constexpr float ROUGH_SEARCH_THRESHOLD = 0.1f; // 0.03 was used when DC offset inflated energy and deflated correlation
static constexpr float EPS = 1e-12f;

static constexpr int32_t FFT_SAVE_BINS = 20;
// ------------------------------------------------------------
// Sub-block ("stepped") correlation search.
//
// Instead of running one correlation per 128-sample audio block, the incoming
// block is fed into the ring buffer N samples at a time, and a correlation is
// computed after every N samples. This produces AUDIO_BLOCK_SAMPLES / N
// correlations per audio block, each one N samples apart in time.
//
// SEARCH_STEP_N must be one of: 16, 32, 64, 128, and must evenly divide
// AUDIO_BLOCK_SAMPLES (128 on Teensy Audio library).
// ------------------------------------------------------------
static constexpr uint32_t SEARCH_STEP_N = 64u;

static_assert(SEARCH_STEP_N == 16 || SEARCH_STEP_N == 32 ||
              SEARCH_STEP_N == 64 || SEARCH_STEP_N == 128,
              "SEARCH_STEP_N must be 16, 32, 64, or 128");
static_assert(AUDIO_BLOCK_SAMPLES % SEARCH_STEP_N == 0,
              "SEARCH_STEP_N must evenly divide AUDIO_BLOCK_SAMPLES");

// Number of correlations computed per incoming audio block.
static constexpr uint32_t STEPS_PER_BLOCK = AUDIO_BLOCK_SAMPLES / SEARCH_STEP_N;

static constexpr uint8_t MODE_BUTTON_PIN = 0;
static constexpr uint8_t STATUS_1_PIN = 1;

// ------------------------------------------------------------
// Fine cross-correlation (reference chirp vs. left channel), DEBUG ONLY.
// Used by both the real-only and the analytic xcorr.
// Lag range is +/- FINE_XCORR_MAX_LAG samples, giving
// 2*FINE_XCORR_MAX_LAG + 1 lag points.
// ------------------------------------------------------------
static constexpr int32_t FINE_XCORR_MAX_LAG = 100;
static constexpr uint32_t FINE_XCORR_NUM_LAGS = 2u * (uint32_t)FINE_XCORR_MAX_LAG + 1u;

// do not forget to *sizeof(type) -- otherwise strange things will happen
//
// Each channel gets its own dynamically-allocated CHIRP_SAMPLES-length
// ring buffer.
static constexpr size_t CHANNEL_BUF_SIZE = CHIRP_SAMPLES * sizeof(float);

bool isDSP = true;

float *leftRingBuf = nullptr;   // left-channel  rx ring
float *rightRingBuf = nullptr;  // right-channel rx ring


static bool debug_leftRingBuf_eq_real_ref = false; // debug: treat leftRingBuf as equal to real_ref

// ------------------------------------------------------------
// DC-offset removal for the fine cross-correlations (debug paths only).
// Set at the start of runFineXcorrDebug() and runAnalyticXcorr() (each sets
// it itself, so either can run alone). The FFT path does not use this; it
// uses the running ring sum below.
// ------------------------------------------------------------
static float g_leftMeanForXcorr = 0.0f;

// ------------------------------------------------------------
// Running (exact) sum of every sample currently sitting in leftRingBuf.
// Maintained incrementally, O(1) per sample, in loop(). g_leftRingBufSum /
// CHIRP_SAMPLES is always the exact current DC mean of the whole ring.
// Used by roughSearchForChirpStart() and by the FFT mix. Declared as double
// so the running sum doesn't drift over long uptimes.
// ------------------------------------------------------------
static double g_leftRingBufSum = 0.0;

// ------------------------------------------------------------
// Per-detection-event identifier, used to build unique dump filenames.
// ------------------------------------------------------------
static uint32_t g_detectionCounter = 0;

// ============================================================
// ==== DSP CORE BEGIN (from standalone FFT test, unchanged) ====
// ============================================================

static float rowBuf[2 * FFT_N];          // interleaved scratch row for arm_cfft_f32 (RAM1)
static uint8_t visitedBits[N_TOTAL / 8]; // cycle-following bitmap (RAM1)

// ---- Twiddle W^m = exp(-j*2*pi*m/32768), m < 32768, from two small tables:
//      m = 256*a + b  ->  W^m = twHi[a] * twLo[b]
static float twLo[2 * 256];
static float twHi[2 * 128];

void buildTwiddles()
{
    for (int b = 0; b < 256; b++) {
        double ang = -2.0 * M_PI * (double)b / (double)N_TOTAL;
        twLo[2 * b]     = (float)cos(ang);
        twLo[2 * b + 1] = (float)sin(ang);
    }
    for (int a = 0; a < 128; a++) {
        double ang = -2.0 * M_PI * (double)(a * 256) / (double)N_TOTAL;
        twHi[2 * a]     = (float)cos(ang);
        twHi[2 * a + 1] = (float)sin(ang);
    }
}

static inline void twiddle(uint32_t m, float &c, float &s)
{
    uint32_t a = m >> 8, b = m & 255;
    float ar = twHi[2 * a], ai = twHi[2 * a + 1];
    float br = twLo[2 * b], bi = twLo[2 * b + 1];
    c = ar * br - ai * bi;
    s = ar * bi + ai * br;
}

// ---- In-place 8-point DFT on re[8], im[8] (radix-2 DIT) ----
#define C8 0.70710678118f
static inline void dft8(float *re, float *im)
{
    float er[4], ei[4], odr[4], odi[4];

    for (int p = 0; p < 2; p++) {               // p=0: even inputs, p=1: odd inputs
        float a0r = re[0 + p], a0i = im[0 + p];
        float a1r = re[2 + p], a1i = im[2 + p];
        float a2r = re[4 + p], a2i = im[4 + p];
        float a3r = re[6 + p], a3i = im[6 + p];

        float t0r = a0r + a2r, t0i = a0i + a2i;
        float t1r = a0r - a2r, t1i = a0i - a2i;
        float t2r = a1r + a3r, t2i = a1i + a3i;
        float t3r = a1r - a3r, t3i = a1i - a3i;

        float *yr = p ? odr : er;
        float *yi = p ? odi : ei;
        yr[0] = t0r + t2r;  yi[0] = t0i + t2i;
        yr[2] = t0r - t2r;  yi[2] = t0i - t2i;
        yr[1] = t1r + t3i;  yi[1] = t1i - t3r;  // t1 - j*t3
        yr[3] = t1r - t3i;  yi[3] = t1i + t3r;  // t1 + j*t3
    }

    float w0r = odr[0],                  w0i = odi[0];
    float w1r = C8 * (odr[1] + odi[1]),  w1i = C8 * (odi[1] - odr[1]);
    float w2r = odi[2],                  w2i = -odr[2];
    float w3r = C8 * (odi[3] - odr[3]),  w3i = -C8 * (odr[3] + odi[3]);

    re[0] = er[0] + w0r;  im[0] = ei[0] + w0i;
    re[4] = er[0] - w0r;  im[4] = ei[0] - w0i;
    re[1] = er[1] + w1r;  im[1] = ei[1] + w1i;
    re[5] = er[1] - w1r;  im[5] = ei[1] - w1i;
    re[2] = er[2] + w2r;  im[2] = ei[2] + w2i;
    re[6] = er[2] - w2r;  im[6] = ei[2] - w2i;
    re[3] = er[3] + w3r;  im[3] = ei[3] + w3i;
    re[7] = er[3] - w3r;  im[7] = ei[3] - w3i;
}

// ---- 32768-point complex FFT on split arrays, completely in place ----
// re[], im[]: 32768 floats each. On return they hold Re(X[k]) and Im(X[k]), k = 0..32767.
void fft_8x4096(float *re, float *im)
{
    // Steps 1+2: eight 4096-pt FFTs. Row n2 = x[8*n1 + n2]; result F[n2][k1] is written
    // back to slot 8*k1 + n2, which is in the same set of slots the row was read from.
    for (uint32_t n2 = 0; n2 < NUM_ROWS; n2++) {
        for (uint32_t n1 = 0; n1 < FFT_N; n1++) {
            rowBuf[2 * n1]     = re[NUM_ROWS * n1 + n2];
            rowBuf[2 * n1 + 1] = im[NUM_ROWS * n1 + n2];
        }
        arm_cfft_f32(&arm_cfft_sR_f32_len4096, rowBuf, 0, 1);
        for (uint32_t k1 = 0; k1 < FFT_N; k1++) {
            re[NUM_ROWS * k1 + n2] = rowBuf[2 * k1];
            im[NUM_ROWS * k1 + n2] = rowBuf[2 * k1 + 1];
        }
    }

    // Steps 3+4: for each k1 the 8 values F[0..7][k1] are contiguous (slots 8*k1..8*k1+7).
    // Twiddle W^(n2*k1), then 8-pt DFT; X[k1 + 4096*k2] ends up in slot 8*k1 + k2.
    for (uint32_t k1 = 0; k1 < FFT_N; k1++) {
        float *pr = &re[NUM_ROWS * k1];
        float *pi = &im[NUM_ROWS * k1];
        float xr[8], xi[8];
        xr[0] = pr[0];
        xi[0] = pi[0];
        for (uint32_t n2 = 1; n2 < NUM_ROWS; n2++) {
            float c, s;
            twiddle(n2 * k1, c, s);
            xr[n2] = pr[n2] * c - pi[n2] * s;
            xi[n2] = pr[n2] * s + pi[n2] * c;
        }
        dft8(xr, xi);
        for (uint32_t k2 = 0; k2 < NUM_ROWS; k2++) {
            pr[k2] = xr[k2];
            pi[k2] = xi[k2];
        }
    }

    // Step 5: in-place permutation. Slot p = 8*k1 + k2 must move to index k1 + 4096*k2,
    // which equals (p * 4096) mod 32767 for p < 32767 (slot 0 and slot 32767 stay put).
    memset(visitedBits, 0, sizeof(visitedBits));
    for (uint32_t s = 1; s < N_TOTAL - 1; s++) {
        if (visitedBits[s >> 3] & (1u << (s & 7))) continue;
        float carryR = re[s];
        float carryI = im[s];
        uint32_t cur = s;
        do {
            uint32_t d = (cur * FFT_N) % (N_TOTAL - 1);
            float tr = re[d], ti = im[d];
            re[d] = carryR;  im[d] = carryI;
            carryR = tr;     carryI = ti;
            visitedBits[d >> 3] |= (uint8_t)(1u << (d & 7));
            cur = d;
        } while (cur != s);
    }
}
// ==== DSP CORE END ====

// ============================================================
// Audio Objects
// ============================================================

AudioInputI2S       i2s1;
AudioRecordQueue    queue1;   // left channel  (i2s1 output 0)
AudioRecordQueue    queue2;   // right channel (i2s1 output 1)

AudioConnection patchCord1(i2s1, 0, queue1, 0);
AudioConnection patchCord2(i2s1, 1, queue2, 0);


// ============================================================
// Large buffers
// ============================================================

// refEnergy is a normalization term (sum of real_ref^2) computed after loading
// the reference chirp, used by the rough and fine correlations.
static float refEnergy = 0.0f;

// Reference chirp buffers (loaded once from SD at boot), indexed linearly
// 0..CHIRP_SAMPLES-1. Not live rx rings.
// imag_ref is the Hilbert transform (90 deg shifted version) of real_ref.
static float real_ref[CHIRP_SAMPLES];            // RAM1 (DTCM): read constantly by the rough search
DMAMEM static float imag_ref[CHIRP_SAMPLES];     // RAM2: only used by analytic xcorr + FFT mix
static uint32_t searchHead = 0;   // write/read head of the rx rings (leftRingBuf/rightRingBuf)

static bool chirpStartFound = false;
static uint32_t chirpStartSample = 0;

enum BufferMode {
    BUFFER_SEARCH = 0,
    BUFFER_DECHIRP = 1
};

static BufferMode searchBufferMode = BUFFER_SEARCH;

File logFile;
char logBuf[200];


// ============================================================
// State
// ============================================================

enum MachineState {
    STATE_SEARCHING = 0,
    STATE_DECHIRPING = 1,
    STATE_SILENCE = 2
};

volatile uint32_t totalSamples = 0;

uint32_t chirpBlockCount = 0;
uint32_t searchIterations = 0;
uint32_t dechirpBlocksSeen = 0;
MachineState machineState = STATE_SEARCHING;

bool inChirpPrev = false;

struct ChirpState {
    float c;
    float s;
    float rotA;
    float rotB;
};

static void enterMtpMode()
{
    Serial.println("DSP stopped. USB MTP mode activated.");

    if (logFile) { logFile.close(); }

    queue1.end();
    queue2.end();
    isDSP = false;
}

static void enterDspMode()
{
    queue1.begin();
    queue2.begin();
    isDSP = true;
    Serial.println("Returning to DSP mode.");
}

static ChirpState dechirpState = {1.0f, 0.0f, 1.0f, 0.0f};

// ============================================================
// SD helpers
// ============================================================

// Parses whitespace / comma / newline separated floats. Returns count read, -1 on error.
// Zero-pads the remainder if the file has fewer than maxCount values.
int loadFloatFile(const char *name, float *dest, uint32_t maxCount)
{
    File f = SD.open(name, FILE_READ);
    if (!f) {
        Serial.printf("ERROR: cannot open %s\n", name);
        return -1;
    }

    static char rdbuf[512];
    char tok[40];
    uint32_t tl = 0, count = 0;

    while (f.available() && count < maxCount) {
        int n = f.read(rdbuf, sizeof(rdbuf));
        if (n <= 0) break;
        for (int i = 0; i < n && count < maxCount; i++) {
            char c = rdbuf[i];
            if (c == ' ' || c == '\n' || c == '\r' || c == '\t' || c == ',' || c == ';') {
                if (tl > 0) {
                    tok[tl] = 0;
                    dest[count++] = strtof(tok, NULL);
                    tl = 0;
                }
            } else if (tl < sizeof(tok) - 1) {
                tok[tl++] = c;
            }
        }
    }
    if (tl > 0 && count < maxCount) {
        tok[tl] = 0;
        dest[count++] = strtof(tok, NULL);
    }
    f.close();

    int got = (int)count;
    if (count < maxCount) {
        Serial.printf("WARNING: %s has only %d values, zero-padding to %lu\n", name, got, (unsigned long)maxCount);
        for (uint32_t i = count; i < maxCount; i++) dest[i] = 0.0f;
    }
    Serial.printf("  loaded %-28s %d values\n", name, got);
    return got;
}

// Writes `count` floats, one per line (8 decimals), reading buf[(head + n) % CHIRP_SAMPLES].
// head = 0 for a plain linear array; head = searchHead for a time-ordered ring dump.
// Output is batched through a 1 KB buffer instead of one println per sample (much faster).
// The caller guarantees the filename is unique (tagged), so no remove is needed.
static bool zzzsaveFloatsToFile(const char *name, const float *buf, uint32_t head, uint32_t count)
{
    File f = SD.open(name, FILE_WRITE);
    if (!f) {
        Serial.print("Failed to open ");
        Serial.println(name);
        return false;
    }

    static char out[1024];
    size_t len = 0;
    for (uint32_t n = 0; n < count; n++) {
        uint32_t idx = (head + n) % CHIRP_SAMPLES;
        len += snprintf(out + len, sizeof(out) - len, "%.8f\n", (double)buf[idx]);
        if (len > sizeof(out) - 40) {
            f.write((const uint8_t *)out, len);
            len = 0;
        }
    }
    if (len > 0) f.write((const uint8_t *)out, len);
    f.close();
    return true;
}

// Writes buf[first..last] inclusive, one value per line (8 decimals).
// Indices wrap modulo CHIRP_SAMPLES, so negative values address the end of the
// array (e.g. FFT bin -20 -> buf[32748]), and first > last-wrap works for rings.
// The caller guarantees the filename is unique (tagged), so no remove is needed.
static bool saveFloatsToFile(const char *name, const float *buf, int32_t first, int32_t last)
{
    File f = SD.open(name, FILE_WRITE);
    if (!f) {
        Serial.print("Failed to open ");
        Serial.println(name);
        return false;
    }

    const int32_t N = (int32_t)CHIRP_SAMPLES;
    static char out[1024];
    size_t len = 0;
    for (int32_t i = first; i <= last; i++) {
        int32_t idx = ((i % N) + N) % N;
        len += snprintf(out + len, sizeof(out) - len, "%.8f\n", (double)buf[idx]);
        if (len > sizeof(out) - 40) {
            f.write((const uint8_t *)out, len);
            len = 0;
        }
    }
    if (len > 0) f.write((const uint8_t *)out, len);
    f.close();
    return true;
}

// Loads real_ref and imag_ref, then recomputes refEnergy from real_ref.
void buildReferenceChirp()
{
    Serial.println("Reading reference chirps from SD... please wait...");

    refEnergy = 0.0f;
    int nr = loadFloatFile(FILE_REAL_REF, real_ref, CHIRP_SAMPLES);
    int ni = loadFloatFile(FILE_IMAG_REF, imag_ref, CHIRP_SAMPLES);

    if (nr < 0) Serial.println("ERROR: real_ref not loaded -- detection will not work.");
    if (ni < 0) Serial.println("ERROR: imag_ref not loaded -- FFT mix and analytic xcorr will be wrong.");

    if (nr != (int)CHIRP_SAMPLES) {
        Serial.println("WARNING: real_ref length != CHIRP_SAMPLES. Remaining samples stay zero.");
    }

    float imagEnergy = 0.0f;
    for (uint32_t i = 0; i < CHIRP_SAMPLES; i++) {
        refEnergy += real_ref[i] * real_ref[i];
        imagEnergy += imag_ref[i] * imag_ref[i];
    }

    Serial.printf("refEnergy = %.6f\n", refEnergy);
    // imag_ref is a Hilbert transform of real_ref, so its energy should be close to refEnergy.
    Serial.printf("imagEnergy = %.6f (ratio imag/real = %.4f, expect ~1)\n",
                  imagEnergy, imagEnergy / (refEnergy + EPS));
    for (int i = 0; i < 20; i++) {
        Serial.printf("First values [%d]: %.8f\n", i, real_ref[i]);
    }
}


// void setSearchBufferMode(BufferMode mode)
// {
//     return;
//     // intentionally empty (kept for compatibility)
// }


// Correlates the live left-channel rx ring (leftRingBuf), starting at searchHead,
// against the fixed reference chirp (real_ref). rightRingBuf is not used in the
// correlation.
float roughSearchForChirpStart()
{
    float acc = 0.0f;
    float rxEnergy = 0.0f;

    // Exact current DC mean of leftRingBuf, from the incrementally-maintained
    // running sum -- O(1), no extra pass over the buffer needed.
    float leftMean = (float)(g_leftRingBufSum / (double)CHIRP_SAMPLES);

    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
        float rx = leftRingBuf[idx] - leftMean;   // left-channel rx ring, DC-removed
        float ref = real_ref[n];                  // reference chirp (linear)

        acc += rx * ref;
        rxEnergy += rx * rx;
    }
    float denom = sqrtf(rxEnergy * refEnergy) + EPS;
    return fabsf(acc) / denom;
}

// Reads leftRingBuf as if it were a plain LINEAR array of length CHIRP_SAMPLES,
// time-ordered starting at searchHead. Any index outside [0, CHIRP_SAMPLES)
// returns 0.0f instead of wrapping (zero-padded, non-circular correlation).
// DC removal: in-range samples have g_leftMeanForXcorr subtracted.
// Used only by the debug fine xcorrs (real-only and analytic).
static inline float leftLinearZeroPad(int32_t offset_from_searchHead)
{
    if (offset_from_searchHead < 0 || offset_from_searchHead >= (int32_t)CHIRP_SAMPLES)
    {
        return 0.0f;
    }

    if (debug_leftRingBuf_eq_real_ref) {
        uint32_t idx = (uint32_t)offset_from_searchHead % CHIRP_SAMPLES;
        return real_ref[idx] - g_leftMeanForXcorr;
    } else {
        uint32_t idx = (searchHead + (uint32_t)offset_from_searchHead) % CHIRP_SAMPLES;
        return leftRingBuf[idx] - g_leftMeanForXcorr;
    }
}

// ============================================================
// Debug fine cross-correlation: reference chirp vs. leftRingBuf (REAL ONLY)
// ============================================================
// Read-only on the rings. Result goes to Serial / log / an SD dump only; it
// never influences the FFT.
static void runFineXcorrDebug(const char *eventTag, const char *crossCorrName)
{
    // DC mean of whichever signal leftLinearZeroPad() reads
    float leftMean = 0.0f;
    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        float v = debug_leftRingBuf_eq_real_ref ? real_ref[n] : leftRingBuf[n];
        leftMean += v;
    }
    leftMean /= (float)CHIRP_SAMPLES;
    g_leftMeanForXcorr = leftMean;

    float leftEnergyTotal = 0.0f;
    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        float raw = debug_leftRingBuf_eq_real_ref ? real_ref[n] : leftRingBuf[n];
        float v = raw - leftMean;
        leftEnergyTotal += v * v;
    }
    float xcorrDenom = sqrtf(leftEnergyTotal * refEnergy) + EPS;

    static float crossCorr[FINE_XCORR_NUM_LAGS];

    for (int32_t lag = -FINE_XCORR_MAX_LAG; lag <= FINE_XCORR_MAX_LAG; lag++)
    {
        float acc = 0.0f;
        for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
        {
            // Lag direction: reference chirp is abcd, leftRingBuf is ABCD.
            // lag = 0  -> corr(abcd, ABCD)
            // lag = -1 -> corr(abcd, BCD0) -- leftRingBuf is advanced
            // lag < 0 means leftRingBuf is advanced (arrived early).
            acc += real_ref[n] * leftLinearZeroPad((int32_t)n - lag);
        }
        crossCorr[lag + FINE_XCORR_MAX_LAG] = acc / xcorrDenom;

        if (lag == 0) {
            Serial.print(" leftMean : "); Serial.print(leftMean, 8);
            Serial.print(" leftEnergyTotal : "); Serial.print(leftEnergyTotal);
            Serial.print(" refEnergy : "); Serial.print(refEnergy);
            Serial.print(" lag: 0");
            Serial.println("");

            if (logFile) {
                logFile.print(eventTag);
                logFile.print(" leftMean : "); logFile.print(leftMean, 8);
                logFile.print(" leftEnergyTotal : "); logFile.print(leftEnergyTotal);
                logFile.print(" refEnergy : "); logFile.print(refEnergy);
                logFile.print(" lag: 0");
                logFile.println();
            }
        }
    }

    // dump crossCorr array
    File crossCorrFile = SD.open(crossCorrName, FILE_WRITE);
    if (crossCorrFile)
    {
        for (uint32_t k = 0; k < FINE_XCORR_NUM_LAGS; k++)
        {
            int32_t lag = (int32_t)k - FINE_XCORR_MAX_LAG;
            crossCorrFile.print(lag);
            crossCorrFile.print(", ");
            crossCorrFile.println(crossCorr[k], 8);
        }
        crossCorrFile.close();
    }
    else
    {
        Serial.print("Failed to open ");
        Serial.println(crossCorrName);
    }

    // peak (normalized) correlation and its lag
    uint32_t maxIdx = 0;
    float maxVal = crossCorr[0];
    for (uint32_t k = 1; k < FINE_XCORR_NUM_LAGS; k++)
    {
        if (crossCorr[k] > maxVal)
        {
            maxVal = crossCorr[k];
            maxIdx = k;
        }
    }
    int32_t bestLag = (int32_t)maxIdx - FINE_XCORR_MAX_LAG;

    Serial.print("[XCORR ref-vs-left] tag=");
    Serial.print(eventTag);
    Serial.print(" max=");
    Serial.print(maxVal, 6);
    Serial.print(" at lag=");
    Serial.println(bestLag);
}

// ============================================================
// Debug analytic (I/Q) cross-correlation: reference chirp vs. leftRingBuf
// ============================================================
// I(lag) = sum_n real_ref[n] * rx[n - lag]
// Q(lag) = sum_n imag_ref[n] * rx[n - lag]      (imag_ref = Hilbert(real_ref))
// A(lag) = sqrt(I^2 + Q^2) / sqrt(rxEnergy * refEnergy)
//
// A is independent of the carrier phase of the received chirp, unlike the
// real-only xcorr. Lag convention matches runFineXcorrDebug(): an rx delayed
// by d samples peaks at lag = -d.
//
// Read-only on the rings; never influences the FFT. Sets g_leftMeanForXcorr
// itself, so it works whether or not runFineXcorrDebug() ran first.
// Normalization uses refEnergy (real_ref only), matching the Python script,
// where both reference channels are scaled by the same factor. A >= |I|.
static void runAnalyticXcorr(const char *eventTag, const char *fileName)
{
    // DC mean of whichever signal leftLinearZeroPad() reads
    float leftMean = 0.0f;
    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        float v = debug_leftRingBuf_eq_real_ref ? real_ref[n] : leftRingBuf[n];
        leftMean += v;
    }
    leftMean /= (float)CHIRP_SAMPLES;
    g_leftMeanForXcorr = leftMean;

    float leftEnergyTotal = 0.0f;
    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        float raw = debug_leftRingBuf_eq_real_ref ? real_ref[n] : leftRingBuf[n];
        float v = raw - leftMean;
        leftEnergyTotal += v * v;
    }
    float denom = sqrtf(leftEnergyTotal * refEnergy) + EPS;

    static float corrI[FINE_XCORR_NUM_LAGS];
    static float corrQ[FINE_XCORR_NUM_LAGS];
    static float corrA[FINE_XCORR_NUM_LAGS];

    for (int32_t lag = -FINE_XCORR_MAX_LAG; lag <= FINE_XCORR_MAX_LAG; lag++)
    {
        float accI = 0.0f;
        float accQ = 0.0f;
        for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
        {
            float rx = leftLinearZeroPad((int32_t)n - lag);
            accI += real_ref[n] * rx;
            accQ += imag_ref[n] * rx;
        }
        uint32_t k = (uint32_t)(lag + FINE_XCORR_MAX_LAG);
        float I = accI / denom;
        float Q = accQ / denom;
        corrI[k] = I;
        corrQ[k] = Q;
        corrA[k] = sqrtf(I * I + Q * Q);
    }

    // dump: lag, I, Q, A
    File f = SD.open(fileName, FILE_WRITE);
    if (f)
    {
        for (uint32_t k = 0; k < FINE_XCORR_NUM_LAGS; k++)
        {
            int32_t lag = (int32_t)k - FINE_XCORR_MAX_LAG;
            f.print(lag);
            f.print(", ");
            f.print(corrI[k], 8);
            f.print(", ");
            f.print(corrQ[k], 8);
            f.print(", ");
            f.println(corrA[k], 8);
        }
        f.close();
    }
    else
    {
        Serial.print("Failed to open ");
        Serial.println(fileName);
    }

    // peak of the envelope A
    uint32_t maxIdx = 0;
    float maxVal = corrA[0];
    for (uint32_t k = 1; k < FINE_XCORR_NUM_LAGS; k++)
    {
        if (corrA[k] > maxVal)
        {
            maxVal = corrA[k];
            maxIdx = k;
        }
    }
    int32_t bestLag = (int32_t)maxIdx - FINE_XCORR_MAX_LAG;

    Serial.print("[XCORR analytic] tag=");
    Serial.print(eventTag);
    Serial.print(" max=");
    Serial.print(maxVal, 6);
    Serial.print(" at lag=");
    Serial.println(bestLag);

    if (logFile)
    {
        logFile.print(eventTag);
        logFile.print(" [XCORR analytic] max=");
        logFile.print(maxVal, 6);
        logFile.print(" at lag=");
        logFile.println(bestLag);
    }
}

// ============================================================
// FFT measurement path (does NOT use the fine xcorr in any way)
// ============================================================
// 1. rx = leftRingBuf de-rotated from searchHead, DC removed with the running
//    ring mean. Written into rightRingBuf (free after its raw dump).
// 2. left  = rx * real_ref
//    right = -rx * imag_ref   (MIX_NEGATE_IMAG)
// 3. fft_8x4096(left, right), save, magnitude, peak search.
// DESTROYS leftRingBuf and rightRingBuf.
static void runFftPath(const char *eventTag)
{
    uint32_t t0 = millis();

    // ---- 1. de-rotate + DC-remove into rightRingBuf ----
    float mean = (float)(g_leftRingBufSum / (double)CHIRP_SAMPLES);
    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
        rightRingBuf[n] = leftRingBuf[idx] - mean;
    }

    // ---- 2. mix in place ----
    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        float rx = rightRingBuf[n];
        leftRingBuf[n] = rx * real_ref[n];
#if MIX_NEGATE_IMAG
        rightRingBuf[n] = -rx * imag_ref[n];
#else
        rightRingBuf[n] =  rx * imag_ref[n];
#endif
    }
    uint32_t t1 = millis();

    // ---- 3. FFT ----
    uint32_t c0 = ARM_DWT_CYCCNT;
    fft_8x4096(leftRingBuf, rightRingBuf);
    uint32_t cyc = ARM_DWT_CYCCNT - c0;
    uint32_t t2 = millis();

    // ---- 4. optional real/imag saves (before magnitude overwrites left) ----
    if (SAVE_FFT_REAL_IMAG)
    {
        char nameRe[64], nameIm[64];
        snprintf(nameRe, sizeof(nameRe), FILE_FFT_REAL_FMT, eventTag);
        snprintf(nameIm, sizeof(nameIm), FILE_FFT_IMAG_FMT, eventTag);
        // saveFloatsToFile(nameRe, leftRingBuf,  0, N_TOTAL);
        // saveFloatsToFile(nameIm, rightRingBuf, 0, N_TOTAL);
        saveFloatsToFile(nameRe, leftRingBuf,  0, (int32_t)N_TOTAL - 1);
        saveFloatsToFile(nameIm, rightRingBuf, 0, (int32_t)N_TOTAL - 1);
    }

    // ---- 5. magnitude -> leftRingBuf ----
    for (uint32_t i = 0; i < N_TOTAL; i++)
    {
        float r = leftRingBuf[i];
        float m = rightRingBuf[i];
        leftRingBuf[i] = sqrtf(r * r + m * m);
    }

    // ---- 6. peak search in the low beat band (both signs) ----
    uint32_t K = (uint32_t)(PEAK_SEARCH_MAX_HZ * (float)N_TOTAL / FS);
    if (K > N_TOTAL / 2 - 1) K = N_TOTAL / 2 - 1;

    int32_t bestK = 0;
    float bestVal = leftRingBuf[0];
    for (int32_t k = -(int32_t)K; k <= (int32_t)K; k++)
    {
        uint32_t bin = (uint32_t)((k + (int32_t)N_TOTAL) % (int32_t)N_TOTAL);
        float v = leftRingBuf[bin];
        if (v > bestVal)
        {
            bestVal = v;
            bestK = k;
        }
    }
    float peakHz = (float)bestK * (FS / (float)N_TOTAL);

    // Global max (all bins), for diagnostics only
    uint32_t gIdx = 0;
    float gVal = leftRingBuf[0];
    for (uint32_t i = 1; i < N_TOTAL; i++)
    {
        if (leftRingBuf[i] > gVal) { gVal = leftRingBuf[i]; gIdx = i; }
    }

    // ---- 7. save magnitude ----
    if (SAVE_FFT_MAG)
    {
        char nameMag[64];
        snprintf(nameMag, sizeof(nameMag), FILE_FFT_MAG_FMT, eventTag);
        // saveFloatsToFile(nameMag, leftRingBuf, 0, N_TOTAL);


        saveFloatsToFile(nameMag, leftRingBuf, -FFT_SAVE_BINS, FFT_SAVE_BINS);
    }
    uint32_t t3 = millis();

    // ---- 8. report ----
    Serial.printf("[FFT] tag=%s peak bin=%ld val=%.6f freq=%.3f Hz (band +/-%.0f Hz) | global max bin=%lu val=%.6f\n",
                  eventTag, (long)bestK, bestVal, peakHz, PEAK_SEARCH_MAX_HZ,
                  (unsigned long)gIdx, gVal);
    Serial.printf("[FFT] timing: mix %lu ms, fft %lu ms (%lu cycles), peak+save %lu ms\n",
                  (unsigned long)(t1 - t0), (unsigned long)(t2 - t1),
                  (unsigned long)cyc, (unsigned long)(t3 - t2));

    if (logFile)
    {
        logFile.printf("%s [FFT] peak bin=%ld val=%.6f freq=%.3f Hz | global max bin=%lu val=%.6f\n",
                       eventTag, (long)bestK, bestVal, peakHz, (unsigned long)gIdx, gVal);
        logFile.flush();
    }
}

// ============================================================
// Post-detection processing
// ============================================================
//
// Order:
//   0. stop both audio queues
//   1. raw dumps of leftRingBuf / rightRingBuf        (SAVE_RAW_RINGS)
//   2. fine xcorr, real only, read-only on the rings  (DEBUG_XCORR)
//   3. analytic I/Q xcorr, read-only on the rings     (DEBUG_ANALYTIC_XCORR)
//   4. FFT path: mix + FFT + magnitude + peak + save  (destroys the rings)
//   5. reset rings, restart queues
//
// A failed debug/dump file never blocks the FFT measurement.
// ------------------------------------------------------------
void processChirpDetection(float roughCorr)
{
    // ---- 0. stop listening to both mic channels ----
    queue1.end();
    queue2.end();

    uint32_t tStart = millis();

    chirpStartSample = totalSamples - CHIRP_SAMPLES;
    chirpStartFound = true;
    machineState = STATE_DECHIRPING;

    //setSearchBufferMode(BUFFER_DECHIRP);

    g_detectionCounter++;
    char eventTag[40];
    snprintf(eventTag, sizeof(eventTag), "%010lu_%02d%02d%02d_%03lu",
             (unsigned long)micros(),
             hour(), minute(), second(),
             (unsigned long)g_detectionCounter);

    char leftDumpName[64];
    char rightDumpName[64];
    char crossCorrName[64];
    char analyticXcorrName[64];
    snprintf(leftDumpName, sizeof(leftDumpName), FILE_LEFT_DUMP_FMT, eventTag);
    snprintf(rightDumpName, sizeof(rightDumpName), FILE_RIGHT_DUMP_FMT, eventTag);
    snprintf(crossCorrName, sizeof(crossCorrName), FILE_XCORR_DUMP_FMT, eventTag);
    snprintf(analyticXcorrName, sizeof(analyticXcorrName), FILE_ANALYTIC_XCORR_DUMP_FMT, eventTag);

    Serial.print("[STATE=TRANSITION] SEARCHING -> DECHIRPING sample=");
    Serial.print(chirpStartSample);
    Serial.print(" searchIters=");
    Serial.print(searchIterations);
    Serial.print(" corr=");
    Serial.print(roughCorr, 4);
    Serial.print(" tag=");
    Serial.print(eventTag);
    Serial.println();

    digitalWrite(STATUS_1_PIN, HIGH);

    // ---- 1. raw dumps (time-ordered from searchHead) ----
    if (SAVE_RAW_RINGS)
    {
        // saveFloatsToFile(leftDumpName,  leftRingBuf,  searchHead, CHIRP_SAMPLES);
        // saveFloatsToFile(rightDumpName, rightRingBuf, searchHead, CHIRP_SAMPLES);
        saveFloatsToFile(leftDumpName,  leftRingBuf,  (int32_t)searchHead, (int32_t)searchHead + (int32_t)CHIRP_SAMPLES - 1);
        saveFloatsToFile(rightDumpName, rightRingBuf, (int32_t)searchHead, (int32_t)searchHead + (int32_t)CHIRP_SAMPLES - 1);
    }
    uint32_t tDumps = millis();

    // ---- 2. debug fine xcorr, real only (never feeds the FFT) ----
    if (DEBUG_XCORR)
    {
        runFineXcorrDebug(eventTag, crossCorrName);
    }
    uint32_t tXcorr = millis();

    // ---- 3. debug analytic I/Q xcorr (never feeds the FFT) ----
    if (DEBUG_ANALYTIC_XCORR)
    {
        runAnalyticXcorr(eventTag, analyticXcorrName);
    }
    uint32_t tAnalytic = millis();

    // ---- 4. FFT measurement (destroys the rings) ----
    runFftPath(eventTag);
    uint32_t tFft = millis();

    Serial.printf("[TIMING] tag=%s dumps %lu ms, xcorr %lu ms, analytic xcorr %lu ms, fft path %lu ms, total %lu ms\n",
                  eventTag,
                  (unsigned long)(tDumps - tStart),
                  (unsigned long)(tXcorr - tDumps),
                  (unsigned long)(tAnalytic - tXcorr),
                  (unsigned long)(tFft - tAnalytic),
                  (unsigned long)(tFft - tStart));

    // ---- 5. reset ring-buffer state so the next chirp can be captured ----
    chirpStartFound = false;
    searchHead = 0;
    g_leftMeanForXcorr = 0.0f;
    g_leftRingBufSum = 0.0;   // rings are about to be zeroed

    queue1.freeBuffer();
    queue2.freeBuffer();
    memset(leftRingBuf, 0, CHANNEL_BUF_SIZE);
    memset(rightRingBuf, 0, CHANNEL_BUF_SIZE);

    queue1.begin();
    queue2.begin();

} // processChirpDetection

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STATUS_1_PIN, OUTPUT);
    digitalWrite(STATUS_1_PIN, LOW);
    digitalWrite(STATUS_1_PIN, HIGH);
    delay(10);
    digitalWrite(STATUS_1_PIN, LOW);

    // cycle counter for FFT timing
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    buildTwiddles();

    if (SD.begin(BUILTIN_SDCARD))
    {
        MTP.begin();
        MTP.addFilesystem(SD, "Teensy SD");
    }
    else
    {
        Serial.println("SD init failed: MTP mode unavailable.");
    }

    while (!Serial &&
           millis() < 3000)
    {
    }


    AudioMemory(120);


    queue1.begin();
    queue2.begin();

    // Start each boot with a fresh log file
    if (SD.exists(FILE_CHIRP_LOG)) {
        SD.remove(FILE_CHIRP_LOG);
    }

    logFile = SD.open(FILE_CHIRP_LOG, FILE_WRITE);
    if (logFile) { Serial.print(FILE_CHIRP_LOG); Serial.println(" opened"); }
    else { Serial.print("Failed to open "); Serial.println(FILE_CHIRP_LOG); }


    // --- leftRingBuf ---
    leftRingBuf = static_cast<float *>(malloc(CHANNEL_BUF_SIZE));
    if (leftRingBuf != nullptr)
    {
        memset(leftRingBuf, 0, CHANNEL_BUF_SIZE);
        Serial.print("leftRingBuf allocated and cleared: ");
        Serial.println(CHANNEL_BUF_SIZE);
    }
    else
    {
        Serial.println("leftRingBuf allocation failed");
    }

    // --- rightRingBuf ---
    rightRingBuf = static_cast<float *>(malloc(CHANNEL_BUF_SIZE));
    if (rightRingBuf != nullptr)
    {
        memset(rightRingBuf, 0, CHANNEL_BUF_SIZE);
        Serial.print("rightRingBuf allocated and cleared: ");
        Serial.println(CHANNEL_BUF_SIZE);
    }
    else
    {
        Serial.println("rightRingBuf allocation failed");
    }

    Serial.printf(" mem_info: 0x2000_0000 to 0x2007_FFFF is RAM1\n"
                  " mem_info: 0x2020_0000 to 0x2027_FFFF is RAM2\n"
                  " mem_info: real_ref=%p imag_ref=%p rowBuf=%p leftRingBuf=%p rightRingBuf=%p\n",
                  real_ref, imag_ref, rowBuf, leftRingBuf, rightRingBuf);

    buildReferenceChirp();
}



void logFilePrint(const char* message) {
    if (logFile) {
        char timestamp[32];
        // Formats: YYYY-MM-DD HH:MM:SS.mmm (where mmm is millis % 1000)
        snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d.%03lu",
                year(), month(), day(), hour(), minute(), second(), millis() % 1000);

        logFile.print("[");
        logFile.print(timestamp);
        logFile.print("] ");
        logFile.println(message);
    }
}

// ============================================================
// Main loop
// ============================================================
void loop()
{
    static bool bufMissingWarned = false;


    if (!isDSP)
    {
        MTP.loop();

        if (digitalRead(MODE_BUTTON_PIN) == LOW)
        {
            enterDspMode();
            delay(500);
        }

        return;
    }

    // Safety guard: if either ring buffer failed to allocate, don't run the
    // DSP path -- this avoids dereferencing a null pointer deep inside the
    // audio servicing path (which is a classic silent-hang cause).
    if (leftRingBuf == nullptr || rightRingBuf == nullptr)
    {
        if (!bufMissingWarned)
        {
            Serial.println("leftRingBuf or rightRingBuf is null -- DSP path disabled. Check heap/allocation.");
            bufMissingWarned = true;
        }
        if (digitalRead(MODE_BUTTON_PIN) == LOW)
        {
            enterMtpMode();
            delay(500);
        }
        return;
    }

    while (queue1.available() > 0 && queue2.available() > 0)
    {
        int16_t *pL = queue1.readBuffer();
        int16_t *pR = nullptr;

        if (!pL) break;

        pR = queue2.readBuffer();
        if (!pR)
        {
            // Avoid leaking the left buffer we already claimed.
            queue1.freeBuffer();
            break;
        }

        // Correlations computed for this audio block. At most STEPS_PER_BLOCK
        // (AUDIO_BLOCK_SAMPLES / SEARCH_STEP_N) of these will be filled in;
        // fewer will be filled while totalSamples is still ramping up to
        // CHIRP_SAMPLES for the very first block(s).
        float corrResults[STEPS_PER_BLOCK];
        uint32_t corrCount = 0;

        for (uint32_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float sampleL = (float)pL[i] * (1.0f / 32768.0f);
            float sampleR = (float)pR[i] * (1.0f / 32768.0f);

            // Keep g_leftRingBufSum exactly equal to the sum of everything
            // currently in leftRingBuf: remove the sample about to be evicted
            // from this ring slot, then add the new one. O(1) per sample.
            g_leftRingBufSum -= (double)leftRingBuf[searchHead];
            leftRingBuf[searchHead] = sampleL;
            g_leftRingBufSum += (double)sampleL;

            rightRingBuf[searchHead] = sampleR;

            searchHead = (searchHead + 1) % CHIRP_SAMPLES;

            totalSamples++;

            // Every SEARCH_STEP_N samples fed into the ring, run one
            // correlation. This gives AUDIO_BLOCK_SAMPLES / SEARCH_STEP_N
            // correlations per incoming audio block, spaced SEARCH_STEP_N
            // samples apart.
            if (!chirpStartFound && totalSamples >= CHIRP_SAMPLES && ((i + 1) % SEARCH_STEP_N == 0))
            {
                machineState = STATE_SEARCHING;

                float roughCorr = roughSearchForChirpStart();
                searchIterations++;

                if (corrCount < STEPS_PER_BLOCK)
                {
                    corrResults[corrCount++] = roughCorr;
                }
            }
        }

        if (!chirpStartFound && corrCount > 0)
        {
            float maxCorr = corrResults[0];
            for (uint32_t k = 0; k < corrCount; k++)
            {
                if (corrResults[k] > maxCorr)
                {
                    maxCorr = corrResults[k];
                }
            }

            if (maxCorr > ROUGH_SEARCH_THRESHOLD)
            {
                processChirpDetection(maxCorr);
                break;
            }
        }

        queue1.freeBuffer();
        queue2.freeBuffer();
    } //while (queue1.available() > 0 && queue2.available() > 0)


    if (digitalRead(MODE_BUTTON_PIN) == LOW)
    {
        enterMtpMode();
        delay(500);
    }
}
