/*
Overall flow:
1. Search the incoming audio for a rough chirp start using a sliding cross-correlation
   against the known reference chirp (real_ref, loaded from SD).
2. When the correlation exceeds threshold, lock in a rough chirp start sample index.
3. Use that start anchor to dechirp every later FFT block with the correct phase and timing.
4. When the chirp burst ends and the 2s silence starts, re-arm the search for the next burst.
5. Continue FFT debug capture while in debug mode.

6. The compact expected pattern for the 32768-sample chirp (~0.743 s) / 2 s silence schedule.

Buffer sizes (chirp is 32768 samples):
  real_ref[CHIRP_SAMPLES]       32768 floats x 4 bytes = 128 KB (static)
  leftRingBuf                   32768 floats x 4 bytes = 128 KB (malloc)
  rightRingBuf                  32768 floats x 4 bytes = 128 KB (malloc)
  hann4096[FFT_N]               16 KB

Note: real_ref is the fixed reference chirp (linear, loaded once from SD).
      leftRingBuf / rightRingBuf are the live rx rings.
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

// Reference chirp (input, one float per line)
#define FILE_REAL_REF         SD_PREFIX "real_300-1500hz_32k_chirp.txt"

// Log file
#define FILE_CHIRP_LOG        SD_PREFIX "c_chirp_log.txt"

// Per-detection dump files. Each is a printf format taking one %s: the event tag.
#define FILE_LEFT_DUMP_FMT    SD_PREFIX "c_left_%s.txt"
#define FILE_RIGHT_DUMP_FMT   SD_PREFIX "c_right_%s.txt"
#define FILE_XCORR_DUMP_FMT   SD_PREFIX "c_xcorr_%s.txt"

// refEnergy is not the chirp waveform itself.
// It is a running normalization metric computed while loading the reference chirp,
// so the rough-correlation code can divide by a stable chirp energy term instead of
// using an arbitrary amplitude scale. This keeps the correlation denominator finite
// and avoids a meaningless zero/overflow case during search.

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

// Chirp period (chirp + silence) in samples
static constexpr uint32_t SOS_PERIOD_SAMPLES = (uint32_t)(FS * SOS_PERIOD_S);

// 32768 / 4096 = exactly 8 blocks
static constexpr uint32_t BLOCKS_PER_CHIRP = (CHIRP_SAMPLES + FFT_N - 1) / FFT_N;

// Detection threshold
static constexpr float DETECT_RATIO_THRESHOLD = 8.0f;
static constexpr float ROUGH_SEARCH_THRESHOLD = 0.1f; // 0.03 was used when DC offset inflated energy and deflated correlation
static constexpr float EPS = 1e-12f;

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

// Streamed FFT-magnitude debug capture.
// Each 4096-sample block is written immediately to the SD card to avoid large RAM usage.
static constexpr bool ENABLE_DEBUG_CAPTURE = true;
static constexpr uint32_t DEBUG_CAPTURE_BLOCKS = 32u;
static constexpr uint32_t DEBUG_CAPTURE_SAMPLES = DEBUG_CAPTURE_BLOCKS * FFT_N;
static constexpr uint32_t DEBUG_FLUSH_INTERVAL = 1u;

static constexpr uint8_t MODE_BUTTON_PIN = 0;
static constexpr uint8_t STATUS_1_PIN = 1;

// ------------------------------------------------------------
// Fine cross-correlation (reference chirp vs. left channel) done once a
// rough chirp start has been detected. Lag range is +/- FINE_XCORR_MAX_LAG
// samples, giving 2*FINE_XCORR_MAX_LAG + 1 lag points.
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
// DC-offset removal for the fine cross-correlation.
//
// A DC bias in leftRingBuf adds a constant term into both the energy sum and
// every lag's correlation sum, which inflates the energy (denominator) and
// deflates/blurs the true correlation peak. g_leftMeanForXcorr holds the
// mean of whichever signal leftLinearZeroPad() is currently reading (either
// leftRingBuf, or real_ref when debug_leftRingBuf_eq_real_ref is true), computed
// once per detection event in processChirpDetection(), and is subtracted
// from every sample leftLinearZeroPad() returns (real samples only -- the
// zero-padded region outside [0, CHIRP_SAMPLES) stays exactly 0, since
// there's no captured signal there to de-bias).
// ------------------------------------------------------------
static float g_leftMeanForXcorr = 0.0f;

// ------------------------------------------------------------
// Running (exact) sum of every sample currently sitting in leftRingBuf.
// Maintained incrementally, O(1) per sample, by subtracting the value about
// to be evicted from a ring slot and adding the new value in loop() -- see
// the write into leftRingBuf[searchHead] below. Since leftRingBuf is always
// exactly CHIRP_SAMPLES long, g_leftRingBufSum / CHIRP_SAMPLES is always the
// exact current DC mean of the whole ring, with no extra full-buffer pass
// needed. Used by roughSearchForChirpStart() to remove DC bias cheaply on
// every call. Declared as double so the running sum doesn't drift over long
// uptimes from repeated float add/subtract.
// ------------------------------------------------------------
static double g_leftRingBufSum = 0.0;

// ------------------------------------------------------------
// Per-detection-event identifier, used to build unique dump filenames and to
// tag the Serial "[XCORR ref-vs-left]" line so a printed maxVal/lag can be
// matched back to the exact left/right/cross-corr dump files that produced
// it. Combines a wall-clock timestamp (from TimeLib, if the RTC/time has
// been set) with a monotonically increasing counter, so filenames stay
// unique even if the clock hasn't been set (all zeros/epoch) or two
// detections land in the same second.
// ------------------------------------------------------------
static uint32_t g_detectionCounter = 0;

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

// Large reference arrays live in normal RAM.
// Keeping them in DMAMEM pushes the Teensy 4.1 DMA region over its limit once
// AudioMemory() and the audio queue are also allocated.
static float refEnergy = 0.0f;

// Reference chirp buffer.
// real_ref holds the fixed reference chirp waveform (loaded once from SD in
// buildReferenceChirp() from FILE_REAL_REF), indexed linearly
// 0..CHIRP_SAMPLES-1. It is not a live rx ring -- that role belongs to
// leftRingBuf / rightRingBuf above.
static float real_ref[CHIRP_SAMPLES];
static uint32_t searchHead = 0;   // write/read head of the rx rings (leftRingBuf/rightRingBuf)

// Phase 2: dedicated FFT/dechirp scratch buffers.
// These are not circular; they hold the exact workspace needed for a single FFT block
// and its coherent accumulation across the chirp burst.
// static float fftIn[2u * FFT_N];
// static float fftMag[FFT_N];
// static float sumSpecRe[FFT_N];
// static float sumSpecIm[FFT_N];

static bool chirpStartFound = false;
static uint32_t chirpStartSample = 0;

enum BufferMode {
    BUFFER_SEARCH = 0,
    BUFFER_DECHIRP = 1
};

static BufferMode searchBufferMode = BUFFER_SEARCH;

// Hann window
static float hann4096[FFT_N];

File debugFftFile;
File logFile;
char logBuf[200];

uint32_t debugWriteCount = 0;


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

volatile uint32_t debugFftCount = 0;
bool debugCaptureStarted = false;
bool debugCaptureDone = false;

struct ChirpState {
    float c;
    float s;
    float rotA;
    float rotB;
};

static void enterMtpMode()
{

    Serial.println("DSP stopped. USB MTP mode activated.");

    if (debugFftFile) { debugFftFile.close(); }
    if (logFile) { logFile.close(); }

    debugCaptureStarted = false;
    debugCaptureDone = false;
    debugWriteCount = 0;
    queue1.end();
    queue2.end();
    isDSP = false;

}

static void enterDspMode()
{
    queue1.begin();
    queue2.begin();
    isDSP = true;
    // if (ENABLE_DEBUG_CAPTURE)
    // {
    //     writeDebugCaptureFiles();
    // }
    Serial.println("Returning to DSP mode.");
}

static ChirpState dechirpState = {1.0f, 0.0f, 1.0f, 0.0f};


// Loads the reference chirp from FILE_REAL_REF (one float per line) into
// real_ref[] and computes refEnergy from it.
void buildReferenceChirp()
{
  uint32_t lineCount = 0;
  refEnergy = 0.0f;

  File myFile = SD.open(FILE_REAL_REF, FILE_READ);
  if (!myFile) {
    Serial.print("Error opening reference file: ");
    Serial.println(FILE_REAL_REF);
    return;
  }

  Serial.print("Reading reference chirp ");
  Serial.print(FILE_REAL_REF);
  Serial.println(" ... please wait...");

  // Keep reading until the file ends OR we hit exactly CHIRP_SAMPLES lines
  while (myFile.available() && lineCount < CHIRP_SAMPLES) {
    String line = myFile.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      real_ref[lineCount] = line.toFloat(); // Handles 0.0 perfectly
      refEnergy += real_ref[lineCount] * real_ref[lineCount];
      lineCount++;
    }
  }

  myFile.close();
  Serial.printf("Done! Loaded %lu lines into real_ref (expected %lu).\n",
                (unsigned long)lineCount, (unsigned long)CHIRP_SAMPLES);

  if (lineCount != CHIRP_SAMPLES) {
    Serial.println("WARNING: reference file length != CHIRP_SAMPLES. "
                   "Remaining samples in real_ref stay zero.");
  }

  Serial.printf("refEnergy = %.6f\n", refEnergy);
  for (int i = 0; i < 20; i++) {
    Serial.printf("First values [%d]: %.8f\n", i, real_ref[i]);
  }
}


void setSearchBufferMode(BufferMode mode)
{
    // searchBufferMode = mode;
    // memset(real_ref, 0, sizeof(real_ref));   // NOTE: would wipe the reference -- do not enable as-is
    // searchHead = 0;

    // if (mode == BUFFER_DECHIRP)
    // {
    //     memset(fftIn, 0, sizeof(fftIn));
    //     memset(fftMag, 0, sizeof(fftMag));
    //     memset(sumSpecRe, 0, sizeof(sumSpecRe));
    //     memset(sumSpecIm, 0, sizeof(sumSpecIm));
    // }
}


// Correlates the live left-channel rx ring (leftRingBuf), starting at searchHead,
// against the fixed reference chirp (real_ref). rightRingBuf is not used in the
// correlation yet.
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
// time-ordered starting at searchHead (same de-rotation used everywhere else
// in this file). Any index outside [0, CHIRP_SAMPLES) returns 0.0f instead
// of wrapping -- this is what makes the fine cross-correlation a proper
// zero-padded (non-circular) correlation instead of a circular one.
//
// DC removal: real samples (in-range) have g_leftMeanForXcorr subtracted
// before being returned. Out-of-range (zero-padded) samples stay exactly
// 0.0f -- there's no captured signal there, so there's nothing to de-bias.
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
// Post-detection processing: dumps + fine cross-correlation
// ============================================================
//
// Called once when the rough search declares a chirp start. Handles:
//   0. stopping both audio queues (no more mic listening)
//   1. dumping leftRingBuf  -> FILE_LEFT_DUMP_FMT
//   2. dumping rightRingBuf -> FILE_RIGHT_DUMP_FMT
//   3. fine cross-correlation: REFERENCE CHIRP (real_ref) vs. leftRingBuf,
//      lag -FINE_XCORR_MAX_LAG..+FINE_XCORR_MAX_LAG, energy-normalized,
//      with the DC offset of leftRingBuf removed first
//   5. dumping the crossCorr array -> FILE_XCORR_DUMP_FMT
//   6. printing the peak correlation value and its lag to Serial, tagged
//      with the same <tag> used for the dump filenames
//   7. closing all files
//
// <tag> is built once per call from the current time plus a monotonically
// increasing detection counter (g_detectionCounter), so every detection event
// gets its own set of left/right/xcorr dump files instead of overwriting the
// previous event's files, and the Serial line printed for that event's peak
// correlation can always be matched back to the exact files that produced it.
//
// Normalization note: because leftRingBuf is a full CHIRP_SAMPLES-length
// circular ring, the sum of squares over the whole ring is the same
// regardless of which sample we call "start" (a circular shift doesn't
// change total energy). So the local "left energy" term is just the total
// (mean-removed) leftRingBuf energy, computed once, and reused for every lag --
// no need to recompute a windowed energy per lag.
// ------------------------------------------------------------
void processChirpDetection(float roughCorr)
{
    // ---- 0. stop listening to both mic channels ----
    queue1.end();
    queue2.end();

    chirpStartSample = totalSamples - CHIRP_SAMPLES;
    chirpStartFound = true;
    machineState = STATE_DECHIRPING;

    setSearchBufferMode(BUFFER_DECHIRP);


    g_detectionCounter++;
    char eventTag[40];
    snprintf(eventTag, sizeof(eventTag), "%010lu_%02d%02d%02d_%03lu",
             (unsigned long)micros(),
             hour(), minute(), second(),
             (unsigned long)g_detectionCounter);

    char leftDumpName[64];
    char rightDumpName[64];
    char crossCorrName[64];
    snprintf(leftDumpName, sizeof(leftDumpName), FILE_LEFT_DUMP_FMT, eventTag);
    snprintf(rightDumpName, sizeof(rightDumpName), FILE_RIGHT_DUMP_FMT, eventTag);
    snprintf(crossCorrName, sizeof(crossCorrName), FILE_XCORR_DUMP_FMT, eventTag);

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

    // ---- 1. dump left channel (time-ordered) ----
    // No SD.exists()/SD.remove() needed -- eventTag makes this filename
    // unique per detection, so nothing is ever overwritten.
    File leftDumpFile = SD.open(leftDumpName, FILE_WRITE);
    if (leftDumpFile)
    {
        for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
        {
            uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
            leftDumpFile.println(leftRingBuf[idx], 8);
        }
        leftDumpFile.close();
    }
    else
    {
        Serial.print("Failed to open ");
        Serial.println(leftDumpName);
    }

    // ---- 2. dump right channel (time-ordered) ----
    File rightDumpFile = SD.open(rightDumpName, FILE_WRITE);
    if (rightDumpFile)
    {
        for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
        {
            uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
            rightDumpFile.println(rightRingBuf[idx], 8);
        }
        rightDumpFile.close();
    }
    else
    {
        Serial.print("Failed to open ");
        Serial.println(rightDumpName);
    }

    // ---- 3a. compute DC mean of whichever signal leftLinearZeroPad() reads ----
    // (leftRingBuf normally, or real_ref when debug_leftRingBuf_eq_real_ref is true)
    float leftMean = 0.0f;
    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        float v = debug_leftRingBuf_eq_real_ref ? real_ref[n] : leftRingBuf[n];
        leftMean += v;
    }
    leftMean /= (float)CHIRP_SAMPLES;
    g_leftMeanForXcorr = leftMean;  // used by leftLinearZeroPad() below

    // ---- 3b. fine cross-correlation: reference chirp vs. leftRingBuf ----
    // total left energy (DC removed), used as the (constant across lags)
    // normalization term
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

    // ---- 5. dump crossCorr array ----
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

    // ---- 6. find peak (normalized) correlation and its lag ----
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

    // Tag included here so this printed line can always be matched back to
    // the left/right/xcorr dump files on the SD card.
    Serial.print("[XCORR ref-vs-left] tag=");
    Serial.print(eventTag);
    Serial.print(" max=");
    Serial.print(maxVal, 6);
    Serial.print(" at lag=");
    Serial.println(bestLag);

    // ---- 7. reset ring-buffer state so the next chirp can be captured ----
    chirpStartFound = false;
    searchHead = 0;
    g_leftMeanForXcorr = 0.0f;
    // Clear out the running O(1) sum back to 0
    g_leftRingBufSum = 0.0;

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
        //logFile.close();
    }
}

// ============================================================
// Main loop
// ============================================================
void loop()
{
    static uint32_t fftWindowStart = 0;
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
