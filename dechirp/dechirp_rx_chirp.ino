/*
Overall flow:
1. Search the incoming audio for a rough chirp start using a sliding cross-correlation
   against the known reference chirp.
2. When the correlation exceeds threshold, lock in a rough chirp start sample index.
3. Use that start anchor to dechirp every later FFT block with the correct phase and timing.
4. When the chirp burst ends and the 2s silence starts, re-arm the search for the next burst.
5. Continue FFT debug capture while in debug mode.


6. the compact expected pattern for your 1 s chirp / 2 s silence schedule.

[STATE=SEARCHING] iter=N sample=<during silence> corr=... bestOffset=...
...
[STATE=TRANSITION] SEARCHING -> DECHIRPING sample=S0 searchIters=N corr=...
[STATE=DECHIRPING] blocks=1 peakBin=... peakFreq=... Hz peak=... mean=... ratio=...
[STATE=DECHIRPING] blocks=2 peakBin=... peakFreq=... Hz peak=... mean=... ratio=...
...
[STATE=DECHIRPING] blocks=10 peakBin=... peakFreq=... Hz peak=... mean=... ratio=...
[STATE=SILENCE] re-arming search after silence period at sample=S0 + 44100 + 88200
[STATE=SEARCHING] iter=1 sample=S0 + 132300 corr=... bestOffset=...


Memory Usage on Teensy 4.1:
  FLASH: code:66488, data:51720, headers:8764   free for files:7999492
   RAM1: variables:329856, code:63064, padding:2472   free for local variables:128896
   RAM2: variables:44128  free for malloc/new:480160

Memory Usage on Teensy 4.1:
  FLASH: code:121376, data:56884, headers:9128   free for files:7939076
   RAM1: variables:335616, code:117944, padding:13128   free for local variables:57600
   RAM2: variables:49216  free for malloc/new:475072 

 

Note:  searchBuf used in phase 1 (full 1sec) and 2 (FFT scratch even though only 4096 samples are used at a time, in fact no need for the full 1sec ring)
       fft buffers are for phase 2, then how to reuse them for future extension -- FSK tx and rx? 

searchBuf[CHIRP_SAMPLES]                      
44100 floats × 4 bytes ≈ 176 KB

fftIn[2 * FFT_N]
8192 floats × 4 bytes ≈ 32.8 KB
fftMag[FFT_N]/*
Overall flow:
1. Search the incoming audio for a rough chirp start using a sliding cross-correlation
   against the known reference chirp.
2. When the correlation exceeds threshold, lock in a rough chirp start sample index.
3. Use that start anchor to dechirp every later FFT block with the correct phase and timing.
4. When the chirp burst ends and the 2s silence starts, re-arm the search for the next burst.
5. Continue FFT debug capture while in debug mode.


6. the compact expected pattern for your 1 s chirp / 2 s silence schedule.

[STATE=SEARCHING] iter=N sample=<during silence> corr=... bestOffset=...
...
[STATE=TRANSITION] SEARCHING -> DECHIRPING sample=S0 searchIters=N corr=...
[STATE=DECHIRPING] blocks=1 peakBin=... peakFreq=... Hz peak=... mean=... ratio=...
[STATE=DECHIRPING] blocks=2 peakBin=... peakFreq=... Hz peak=... mean=... ratio=...
...
[STATE=DECHIRPING] blocks=10 peakBin=... peakFreq=... Hz peak=... mean=... ratio=...
[STATE=SILENCE] re-arming search after silence period at sample=S0 + 44100 + 88200
[STATE=SEARCHING] iter=1 sample=S0 + 132300 corr=... bestOffset=...


Memory Usage on Teensy 4.1:
  FLASH: code:66488, data:51720, headers:8764   free for files:7999492
   RAM1: variables:329856, code:63064, padding:2472   free for local variables:128896
   RAM2: variables:44128  free for malloc/new:480160

Memory Usage on Teensy 4.1:
  FLASH: code:121376, data:56884, headers:9128   free for files:7939076
   RAM1: variables:335616, code:117944, padding:13128   free for local variables:57600
   RAM2: variables:49216  free for malloc/new:475072 

Note:  searchBuf used in phase 1 (full 1sec) and 2 (FFT scratch even though only 4096 samples are used at a time, in fact no need for the full 1sec ring)
       fft buffers are for phase 2, then how to reuse them for future extension -- FSK tx and rx? 

searchBuf[CHIRP_SAMPLES]                      
44100 floats × 4 bytes ≈ 176 KB

fftIn[2 * FFT_N]
8192 floats × 4 bytes ≈ 32.8 KB
fftMag[FFT_N]
4096 floats × 4 bytes ≈ 16 KB
sumSpecRe[FFT_N]
16 KB
sumSpecIm[FFT_N]
16 KB
hann4096[FFT_N]
16 KB

176 + 32.8 + 16 + 16 + 16 + 16 ≈ 272.8 KB


log file:
[1970-01-01 00:00:11.952] SEARCHING -> DECHIRPING roughCorr  0.2890
[1970-01-01 00:00:12.027] process4096Block starts
[1970-01-01 00:00:12.028] process4096Block finished
[1970-01-01 00:00:12.028] process4096Block starts
[1970-01-01 00:00:12.029] process4096Block finished
[1970-01-01 00:00:12.029] process4096Block starts
[1970-01-01 00:00:12.030] process4096Block finished
[1970-01-01 00:00:12.030] process4096Block starts
[1970-01-01 00:00:12.030] process4096Block finished
[1970-01-01 00:00:12.030] process4096Block starts
[1970-01-01 00:00:12.031] process4096Block finished
[1970-01-01 00:00:12.031] process4096Block starts
[1970-01-01 00:00:12.032] process4096Block finished
[1970-01-01 00:00:12.032] process4096Block starts
[1970-01-01 00:00:12.033] process4096Block finished
[1970-01-01 00:00:12.033] process4096Block starts
[1970-01-01 00:00:12.033] process4096Block finished
[1970-01-01 00:00:12.034] process4096Block starts
[1970-01-01 00:00:12.034] process4096Block finished
[1970-01-01 00:00:12.034] process4096Block starts
[1970-01-01 00:00:12.035] process4096Block finished
[1970-01-01 00:00:12.035] process4096Block starts
[1970-01-01 00:00:12.036] [STATE=DECHIRPING] blocks=011 peakBin=3956 peakFreq=42592.676 Hz peak=4.408007 mean=0.034032 ratio=129.526  --> DETECT

*/

#include <Arduino.h>
#include <Audio.h>
#include <SD.h>
#include <MTP_Teensy.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <TimeLib.h>

// refEnergy is not the chirp waveform itself.
// It is a running normalization metric computed while generating the reference chirp,
// so the rough-correlation code can divide by a stable chirp energy term instead of
// using an arbitrary amplitude scale. This keeps the correlation denominator finite
// and avoids a meaningless zero/overflow case during search.

// ============================================================
// Parameters
// ============================================================

static constexpr float FS = 44100.0f;

static constexpr float CHIRP_DUR_S = 1.0f;
static constexpr float SILENCE_DUR_S = 2.0f;
static constexpr float SOS_PERIOD_S = CHIRP_DUR_S + SILENCE_DUR_S;

static constexpr float F0 = 300.0f;
static constexpr float F1 = 1500.0f;

// 4096-point complex FFT
static constexpr uint32_t FFT_N = 4096;

// One-second chirp
static constexpr uint32_t CHIRP_SAMPLES = (uint32_t)(FS * CHIRP_DUR_S);

// Three-second SOS period
static constexpr uint32_t SOS_PERIOD_SAMPLES = (uint32_t)(FS * SOS_PERIOD_S);

// 44100 / 4096 = 10 full blocks + partial block
static constexpr uint32_t BLOCKS_PER_CHIRP = (CHIRP_SAMPLES + FFT_N - 1) / FFT_N;

// Detection threshold
static constexpr float DETECT_RATIO_THRESHOLD = 8.0f;
static constexpr float ROUGH_SEARCH_THRESHOLD = 0.04f;
static constexpr float EPS = 1e-12f;

// Streamed FFT-magnitude debug capture.
// Each 4096-sample block is written immediately to the SD card to avoid large RAM usage.
static constexpr bool ENABLE_DEBUG_CAPTURE = true;
static constexpr uint32_t DEBUG_CAPTURE_BLOCKS = 32u;
static constexpr uint32_t DEBUG_CAPTURE_SAMPLES = DEBUG_CAPTURE_BLOCKS * FFT_N;
static constexpr uint32_t DEBUG_FLUSH_INTERVAL = 1u;

static constexpr uint8_t MODE_BUTTON_PIN = 0;
static constexpr uint8_t STATUS_1_PIN = 1;
static constexpr size_t MALLOC_TEST_SIZE = 1024u;
bool isDSP = true;
uint8_t *mallocTest = nullptr;
    


// ============================================================
// Audio Objects
// ============================================================

AudioInputI2S       i2s1;
AudioRecordQueue    queue1;

AudioConnection patchCord1(i2s1, 0, queue1, 0);


// ============================================================
// Large buffers
// ============================================================

// Large reference/search arrays live in normal RAM.
// Keeping them in DMAMEM pushes the Teensy 4.1 DMA region over its limit once
// AudioMemory() and the audio queue are also allocated.
static float refEnergy = 0.0f;

// Phase 1: search buffer for rough chirp-start detection.
// This remains a CHIRP_SAMPLES-sample circular ring while searching for the burst.
static float searchBuf[CHIRP_SAMPLES];
static uint32_t searchHead = 0;

// Phase 2: dedicated FFT/dechirp scratch buffers.
// These are not circular; they hold the exact workspace needed for a single FFT block
// and its coherent accumulation across the chirp burst.
static float fftIn[2u * FFT_N];
static float fftMag[FFT_N];
static float sumSpecRe[FFT_N];
static float sumSpecIm[FFT_N];

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
    if (debugFftFile) { debugFftFile.close(); }
    if (logFile) { logFile.close(); }

    debugCaptureStarted = false;
    debugCaptureDone = false;
    debugWriteCount = 0;
    queue1.end();
    isDSP = false;

    Serial.println("DSP stopped. USB MTP mode activated.");
}

static void enterDspMode()
{
    queue1.begin();
    isDSP = true;
    if (ENABLE_DEBUG_CAPTURE)
    {
        writeDebugCaptureFiles();
    }
    Serial.println("Returning to DSP mode.");
}

static ChirpState dechirpState = {1.0f, 0.0f, 1.0f, 0.0f};

static inline void advanceChirpState(ChirpState &state, float rotDeltaA, float rotDeltaB)
{
    const float nextC = state.c * state.rotA - state.s * state.rotB;
    const float nextS = state.s * state.rotA + state.c * state.rotB;

    state.c = nextC;
    state.s = nextS;

    const float nextRotA = state.rotA * rotDeltaA - state.rotB * rotDeltaB;
    const float nextRotB = state.rotB * rotDeltaA + state.rotA * rotDeltaB;

    state.rotA = nextRotA;
    state.rotB = nextRotB;
}

static void initChirpStateFromOffset(ChirpState &state, uint32_t sampleOffset)
{
    const float k = (F1 - F0) / CHIRP_DUR_S;
    const float phi0 = 2.0f * PI * F0 / FS;
    const float dphi = 2.0f * PI * k / (FS * FS);

    const float startRotA = arm_cos_f32(phi0);
    const float startRotB = arm_sin_f32(phi0);
    const float rotDeltaA = arm_cos_f32(dphi);
    const float rotDeltaB = arm_sin_f32(dphi);

    state.c = 1.0f;
    state.s = 0.0f;
    state.rotA = startRotA;
    state.rotB = startRotB;

    for (uint32_t n = 0; n < sampleOffset; n++)
    {
        advanceChirpState(state, rotDeltaA, rotDeltaB);
    }
}


// ============================================================
// Build reference chirp
// ============================================================

void buildReferenceChirp()
{
    const float k = (F1 - F0) / CHIRP_DUR_S;
    refEnergy = 0.0f;

    Serial.println("Building reference chirp...");

    // Use the complex-rotation recurrence from complex-rotation-chirp.md.
    // No extra chirp tables in RAM: keep the recurrence state local to the loop.
    float c = 1.0f;
    float s = 0.0f;

    const float phi0 = 2.0f * PI * F0 / FS;
    const float dphi = 2.0f * PI * k / (FS * FS);

    float rotA = arm_cos_f32(phi0);
    float rotB = arm_sin_f32(phi0);
    float rotDeltaA = arm_cos_f32(dphi);
    float rotDeltaB = arm_sin_f32(dphi);

    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        refEnergy += c * c;

        float nextC = c * rotA - s * rotB;
        float nextS = s * rotA + c * rotB;

        c = nextC;
        s = nextS;

        float nextRotA = rotA * rotDeltaA - rotB * rotDeltaB;
        float nextRotB = rotB * rotDeltaA + rotA * rotDeltaB;

        rotA = nextRotA;
        rotB = nextRotB;
    }

    Serial.println("Building Hann window...");

    for (uint32_t n = 0; n < FFT_N; n++)
    {
        hann4096[n] = 0.5f - 0.5f * arm_cos_f32((2.0f * PI * n) / (FFT_N - 1));
    }

    Serial.println("Reference generation complete.");
}

void getChirpSample(uint32_t sampleIdx, float &c, float &s)
{
    const float k = (F1 - F0) / CHIRP_DUR_S;
    const float phi0 = 2.0f * PI * F0 / FS;
    const float dphi = 2.0f * PI * k / (FS * FS);

    float rotA = arm_cos_f32(phi0);
    float rotB = arm_sin_f32(phi0);
    float rotDeltaA = arm_cos_f32(dphi);
    float rotDeltaB = arm_sin_f32(dphi);

    c = 1.0f;
    s = 0.0f;

    for (uint32_t n = 0; n < sampleIdx; n++)
    {
        float nextC = c * rotA - s * rotB;
        float nextS = s * rotA + c * rotB;

        c = nextC;
        s = nextS;

        float nextRotA = rotA * rotDeltaA - rotB * rotDeltaB;
        float nextRotB = rotB * rotDeltaA + rotA * rotDeltaB;

        rotA = nextRotA;
        rotB = nextRotB;
    }
}


// ============================================================
// Reset frequency-domain accumulator
// ============================================================

void resetAccumulator(float *sumSpecRe, float *sumSpecIm)
{
    memset(sumSpecRe, 0, FFT_N * sizeof(float));
    memset(sumSpecIm, 0, FFT_N * sizeof(float));

    chirpBlockCount = 0;
}

void resetAccumulator()
{
    resetAccumulator(sumSpecRe, sumSpecIm);
}

void setSearchBufferMode(BufferMode mode)
{
    searchBufferMode = mode;
    memset(searchBuf, 0, sizeof(searchBuf));
    searchHead = 0;

    if (mode == BUFFER_DECHIRP)
    {
        memset(fftIn, 0, sizeof(fftIn));
        memset(fftMag, 0, sizeof(fftMag));
        memset(sumSpecRe, 0, sizeof(sumSpecRe));
        memset(sumSpecIm, 0, sizeof(sumSpecIm));
    }
}

bool initDebugSD()
{
    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("SD init failed: no card or card not detected.");
        return false;
    }

    if (SD.exists("fft_mag.txt"))
    {
        Serial.println("Removing existing fft_mag.txt before new capture.");
        SD.remove("fft_mag.txt");
    }

    Serial.println("SD card initialized for debug capture.");
    return true;
}

float roughSearchForChirpStart()
{
    float acc = 0.0f;
    float rxEnergy = 0.0f;

    const float k = (F1 - F0) / CHIRP_DUR_S;
    const float phi0 = 2.0f * PI * F0 / FS;
    const float dphi = 2.0f * PI * k / (FS * FS);
    const float rotDeltaA = arm_cos_f32(dphi);
    const float rotDeltaB = arm_sin_f32(dphi);
    float refC = 1.0f;
    float refS = 0.0f;
    float rotA = arm_cos_f32(phi0);
    float rotB = arm_sin_f32(phi0);
        //Serial.print(" 2.11 ");

    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
        float rx = searchBuf[idx];

        acc += rx * refC;
        rxEnergy += rx * rx;

        float nextC = refC * rotA - refS * rotB;
        float nextS = refS * rotA + refC * rotB;
        refC = nextC;
        refS = nextS;

        float nextRotA = rotA * rotDeltaA - rotB * rotDeltaB;
        float nextRotB = rotB * rotDeltaA + rotA * rotDeltaB;
        rotA = nextRotA;
        rotB = nextRotB;
    }
    float denom = sqrtf(rxEnergy * refEnergy) + EPS;

    if(0) {
     Serial.print(" rxEnergy "); Serial.print( rxEnergy);
     Serial.print(" refEnergy "); Serial.print(  refEnergy);
     Serial.print(" denom "); Serial.print(  denom);
     Serial.print(" acc "); Serial.print(  acc);    
    }


    return fabsf(acc) / denom;
}

void writeDebugCaptureFiles()
{
    if (debugCaptureStarted || debugCaptureDone)
    {
        return;
    }

    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("SD write skipped: card unavailable.");
        return;
    }

    if (SD.exists("fft_mag.txt"))
    {
        Serial.println("Removing stale fft_mag.txt before open.");
        SD.remove("fft_mag.txt");
    }

    debugFftFile = SD.open("fft_mag.txt", FILE_WRITE);
    if (debugFftFile)
    {
        Serial.println("fft_mag.txt opened for streaming debug capture.");
        debugCaptureStarted = true;
    }
    else
    {
        Serial.println("Failed to open fft_mag.txt");
        return;
    }

    Serial.println("Debug capture streaming started.");
}

void streamDebugFftSpectrum(const float *mag)
{
    return; // do not do any for now

    if (!debugFftFile)
    {
        return;
    }

    for (uint32_t k = 0; k < FFT_N; k++)
    {
        debugFftFile.print(mag[k], 8);
        if (k + 1 < FFT_N)
        {
            debugFftFile.print(',');
        }
    }
    debugFftFile.println();
    debugWriteCount++;

    Serial.print("dump_done block=");
    Serial.print((totalSamples / FFT_N) - 1);
    Serial.print(" t=");
    Serial.print((float)totalSamples / FS, 3);
    Serial.print(" s count=");
    Serial.print(debugWriteCount);
    Serial.println();

    if (debugWriteCount >= DEBUG_CAPTURE_BLOCKS)
    {
        debugFftFile.close();
        Serial.print("fft_mag.txt written with ");
        Serial.print(debugWriteCount);
        Serial.println(" spectra");
        debugCaptureDone = true;
    }
}


// ============================================================
// Determine whether an absolute sample is inside the chirp
// ============================================================

inline bool isInChirp(uint32_t absSampleIdx)
{
    if (!chirpStartFound)
    {
        return false;
    }

    uint32_t rel = absSampleIdx - chirpStartSample;
    return (rel < CHIRP_SAMPLES);
}


// ============================================================
// Process one 4096-sample block
// ============================================================

inline uint32_t getSearchBufIndex(uint32_t absSampleIdx)
{
    // Phase 1 ring mapping: searchBuf[] is a CHIRP_SAMPLES-sample circular buffer.
    // Phase 2 does not change the underlying storage semantics: the same array is
    // reused as a fixed FFT scratch layout, so the buffer is not treated as a full-size
    // circular ring during dechirp processing.
    return (absSampleIdx + CHIRP_SAMPLES - totalSamples + searchHead + CHIRP_SAMPLES) % CHIRP_SAMPLES;
}

void process4096Block(uint32_t blockStartSample)
{
    // --------------------------------------------------------
    // Phase 1 uses searchBuf as a raw circular ring for chirp detection.
    // Phase 2 uses explicit FFT scratch arrays so the search ring remains valid.
    //
    // fftIn      : complex FFT input (interleaved real/imag)
    // fftMag     : magnitude spectrum after FFT
    // sumSpecRe  : coherent real accumulator for chirp energy
    // sumSpecIm  : coherent imag accumulator for chirp energy
    // --------------------------------------------------------

    const float k = (F1 - F0) / CHIRP_DUR_S;
    const float dphi = 2.0f * PI * k / (FS * FS);
    const float rotDeltaA = arm_cos_f32(dphi);
    const float rotDeltaB = arm_sin_f32(dphi);

    ChirpState chirpState = dechirpState;

    for (uint32_t n = 0; n < FFT_N; n++)
    {
        uint32_t absIdx = blockStartSample + n;
        uint32_t bufIdx = getSearchBufIndex(absIdx);
        float xn = searchBuf[bufIdx] * hann4096[n];

        float re = xn * chirpState.c;
        float im = -xn * chirpState.s;

        fftIn[2 * n + 0] = re;
        fftIn[2 * n + 1] = im;

        advanceChirpState(chirpState, rotDeltaA, rotDeltaB);
    }

    dechirpState = chirpState;

    if (ENABLE_DEBUG_CAPTURE)
    {
        Serial.print("mix_done block=");
        Serial.print(blockStartSample / FFT_N);
        Serial.print(" t=");
        Serial.print((float)blockStartSample / FS, 3);
        Serial.print(" s");
        Serial.println();
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftIn, 0, 1);

    for (uint32_t k = 0; k < FFT_N; k++)
    {
        float re = fftIn[2 * k + 0];
        float im = fftIn[2 * k + 1];
        fftMag[k] = sqrtf(re * re + im * im);
    }

    if (ENABLE_DEBUG_CAPTURE)
    {
        Serial.print("fft_done block=");
        Serial.print(blockStartSample / FFT_N);
        Serial.print(" t=");
        Serial.print((float)blockStartSample / FS, 3);
        Serial.print(" s");
        Serial.println();
    }

    // if (ENABLE_DEBUG_CAPTURE && debugCaptureStarted && !debugCaptureDone)
    // {
    //     streamDebugFftSpectrum(fftMag);
    // }

    bool blockInChirp = isInChirp(blockStartSample + FFT_N / 2);

    if (!inChirpPrev && blockInChirp)
    {
        resetAccumulator(sumSpecRe, sumSpecIm);
    }

    if (blockInChirp)
    {
        for (uint32_t k = 0; k < FFT_N; k++)
        {
            sumSpecRe[k] += fftIn[2 * k + 0];
            sumSpecIm[k] += fftIn[2 * k + 1];
        }

        chirpBlockCount++;
    }

    bool nowInChirp = isInChirp(blockStartSample + FFT_N - 1);

    if (inChirpPrev && !nowInChirp && chirpBlockCount > 0)
    {
        for (uint32_t k = 0; k < FFT_N; k++)
        {
            float re = sumSpecRe[k];
            float im = sumSpecIm[k];
            fftMag[k] = sqrtf(re * re + im * im);
        }

        float maxVal = 0.0f;
        uint32_t maxIdx = 0;

        arm_max_f32(fftMag, FFT_N, &maxVal, &maxIdx);

        float meanVal = 0.0f;
        arm_mean_f32(fftMag, FFT_N, &meanVal);

        float ratio = maxVal / (meanVal + EPS);
        float peakFreq = ((float)maxIdx * FS) / FFT_N;

        Serial.print("[STATE=DECHIRPING] ");
        Serial.print("blocks=");
        Serial.print(chirpBlockCount);
        Serial.print(" peakBin=");
        Serial.print(maxIdx);
        Serial.print(" peakFreq=");
        Serial.print(peakFreq, 3);
        Serial.print(" Hz");
        Serial.print(" peak=");
        Serial.print(maxVal, 6);
        Serial.print(" mean=");
        Serial.print(meanVal, 6);
        Serial.print(" ratio=");
        Serial.print(ratio, 3);



        if (ratio > DETECT_RATIO_THRESHOLD)
        {

            Serial.print("  --> DETECT");

            snprintf(logBuf, sizeof(logBuf),
                    "[STATE=DECHIRPING] blocks=%03lu peakBin=%03lu peakFreq=%.3f Hz peak=%.6f mean=%.6f ratio=%.3f  --> DETECT",
                    chirpBlockCount, maxIdx, peakFreq, maxVal, meanVal, ratio);
            logFilePrint(logBuf);
        }

        Serial.println();

        resetAccumulator(sumSpecRe, sumSpecIm);
    }

    inChirpPrev = nowInChirp;
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    mallocTest = static_cast<uint8_t *>(malloc(MALLOC_TEST_SIZE));
    if (mallocTest != nullptr)
    {
        memset(mallocTest, 0, MALLOC_TEST_SIZE);
        Serial.println("mallocTest allocated and cleared: 1024 bytes");
    }
    else
    {
        Serial.println("mallocTest allocation failed");
    }

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


    Serial.println();
    Serial.println("==============================");
    Serial.println("Teensy 4.1 Dechirp FFT");
    Serial.println("==============================");


    // --------------------------------------------------------
    // Audio memory
    // --------------------------------------------------------

    AudioMemory(120);


    // // --------------------------------------------------------
    // // SGTL5000
    // // --------------------------------------------------------

    // sgtl5000.enable();

    // sgtl5000.inputSelect(
    //     AUDIO_INPUT_LINEIN
    // );

    // sgtl5000.lineInLevel(5);

    // sgtl5000.volume(0.5);


    // --------------------------------------------------------
    // Start AudioRecordQueue
    // --------------------------------------------------------

    queue1.begin();

    if (ENABLE_DEBUG_CAPTURE)
    {
        initDebugSD();
        writeDebugCaptureFiles();
    }


    // --------------------------------------------------------
    // Build reference
    // --------------------------------------------------------

    buildReferenceChirp();


    // --------------------------------------------------------
    // Clear accumulator
    // --------------------------------------------------------

    resetAccumulator();


    // --------------------------------------------------------
    // Print FFT configuration
    // --------------------------------------------------------

    Serial.println();
    Serial.println("Configuration:");

    Serial.print("Sample rate: ");
    Serial.print(FS);
    Serial.println(" Hz");

    Serial.print("Chirp: ");
    Serial.print(F0);
    Serial.print(" -> ");
    Serial.print(F1);
    Serial.println(" Hz");

    Serial.print("Chirp duration: ");
    Serial.print(CHIRP_DUR_S);
    Serial.println(" sec");

    Serial.print("FFT size: ");
    Serial.println(FFT_N);

    Serial.print("FFT resolution: ");
    Serial.print(FS / FFT_N, 4);
    Serial.println(" Hz/bin");

    Serial.print("FFT time window: ");
    Serial.print((float)FFT_N / FS * 1000.0f, 3);
    Serial.println(" ms");

    Serial.print("Chirp samples: ");
    Serial.println(CHIRP_SAMPLES);

    Serial.print("FFT blocks/chirp: ");
    Serial.println(BLOCKS_PER_CHIRP);

    Serial.println();
    Serial.println("CMSIS-DSP CFFT = 4096");
    Serial.println("Using arm_cfft_sR_f32_len4096");
    Serial.println("Detector ready.");
    Serial.println("[STATE=SEARCHING] machine started in search mode");


    if (SD.exists("afsk_log.txt")) {
        SD.remove("afsk_log.txt");
    }

    logFile = SD.open("afsk_log.txt", FILE_WRITE);
    if (logFile) { Serial.println("afsk_log.txt opened"); }
    else { Serial.println("Failed to open afsk_log.txt"); }
}


// ============================================================
// Main loop
// ============================================================
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

void loop()
{
    static uint32_t fftWindowStart = 0;


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

    // --------------------------------------------------------
    // Process available 128-sample Audio blocks
    //
    // AUDIO_BLOCK_SAMPLES is already defined by Teensy Audio
    // library, so we do NOT redefine it.
    // --------------------------------------------------------
        //Serial.print("0 ");

    // too many logFilePrint("loop start");

    while (queue1.available() > 0)
    {
        int16_t *p = queue1.readBuffer();

        if (!p)
            break;

        //Serial.println("1 ");

        for (uint32_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float sample = (float)p[i] * (1.0f / 32768.0f);

            // Feed the rolling search buffer used for chirp start detection.
            searchBuf[searchHead] = sample;
            searchHead = (searchHead + 1) % CHIRP_SAMPLES;

            totalSamples++;

            if (machineState == STATE_DECHIRPING && (totalSamples - fftWindowStart) >= FFT_N)
            {
                snprintf(logBuf, sizeof(logBuf), "process4096Block starts"); 
                logFilePrint(logBuf);

                process4096Block(fftWindowStart);
                fftWindowStart += FFT_N;

                snprintf(logBuf, sizeof(logBuf), "process4096Block finished"); 
                logFilePrint(logBuf);
            }
        }
        //Serial.println("2 "); Serial.print(totalSamples); Serial.println();

        // Phase 1: searchBuf is a CHIRP_SAMPLES-sample circular ring used for rough
        // chirp-start detection. It is only indexed through searchHead / ring math here.
        //
        // Phase 2: once chirpStartFound is true, keep the raw-audio ring valid for sample
        // lookups, but do not treat searchBuf as FFT scratch. dechirpScratch[] owns the
        // FFT/data workspace for the dechirp path.
        if (!chirpStartFound && totalSamples >= CHIRP_SAMPLES)
        {
            //Serial.print("2.1");
            machineState = STATE_SEARCHING;
                // snprintf(logBuf, sizeof(logBuf), "roughSearchForChirpStart starts"); 
                // logFilePrint(logBuf);
            float roughCorr = roughSearchForChirpStart();
                // snprintf(logBuf, sizeof(logBuf), "roughSearchForChirpStart end roughCorr %7.4f", roughCorr); 
                // logFilePrint(logBuf);
            searchIterations++;
            //Serial.print("2.2-"); 
            //if(roughCorr > 0.001) 
                Serial.println(roughCorr, 6);

            // if ((searchIterations % 20u) == 0u || roughCorr > ROUGH_SEARCH_THRESHOLD)
            // {
            //     Serial.print("[STATE=SEARCHING] iter=");
            //     Serial.print(searchIterations);
            //     Serial.print(" sample=");
            //     Serial.print(totalSamples);
            //     Serial.print(" corr=");
            //     Serial.print(roughCorr, 4);
            //     Serial.println();
            // }
            //Serial.print("2.3 ");

            if (roughCorr > ROUGH_SEARCH_THRESHOLD)
            {

                snprintf(logBuf, sizeof(logBuf), "SEARCHING -> DECHIRPING roughCorr %7.4f", roughCorr); 
                logFilePrint(logBuf);

                // dumpSearchBuf();
                for (uint32_t n = 0; n < CHIRP_SAMPLES; n++) {
                    uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
                    float rx = searchBuf[idx];
                    logFile.println(rx, 8);
                }

                chirpStartSample = totalSamples - CHIRP_SAMPLES;
                chirpStartFound = true;
                machineState = STATE_DECHIRPING;
                setSearchBufferMode(BUFFER_DECHIRP);
                fftWindowStart = chirpStartSample;
                initChirpStateFromOffset(dechirpState, 0u);

                Serial.print("[STATE=TRANSITION] SEARCHING -> DECHIRPING sample=");
                Serial.print(chirpStartSample);
                Serial.print(" searchIters=");
                Serial.print(searchIterations);
                Serial.print(" corr=");
                Serial.print(roughCorr, 4);
                Serial.println();

                digitalWrite(STATUS_1_PIN, HIGH); 




            }
        }
        //Serial.print("2.3 ");

        // When the chirp ends, re-arm the search for the next burst.
        if (chirpStartFound && !debugCaptureDone && totalSamples > chirpStartSample + CHIRP_SAMPLES + (uint32_t)(SILENCE_DUR_S * FS))
        {
            chirpStartFound = false;
            machineState = STATE_SILENCE;
            setSearchBufferMode(BUFFER_SEARCH);
            fftWindowStart = totalSamples;
            Serial.print("[STATE=SILENCE] re-arming search after silence period at sample=");
            Serial.print(totalSamples);
            Serial.println();
            searchIterations = 0;

            snprintf(logBuf, sizeof(logBuf), "chirp end, STATE_SILENCE"); 
            logFilePrint(logBuf);
        }

        if (chirpStartFound && machineState == STATE_DECHIRPING)
        {
            dechirpBlocksSeen++;
        }

        queue1.freeBuffer();
    }

    //Serial.print("3 ");


    if (digitalRead(MODE_BUTTON_PIN) == LOW)
    {
        enterMtpMode();
        delay(500);
    }
}

