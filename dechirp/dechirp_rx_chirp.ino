/*
Overall flow:
1. Search the incoming audio for a rough chirp start using a sliding cross-correlation
   against the known reference chirp.
2. When the correlation exceeds threshold, lock in a rough chirp start sample index.
3. Use that start anchor to dechirp every later FFT block with the correct phase and timing.
4. When the chirp burst ends and the 2s silence starts, re-arm the search for the next burst.
5. Continue FFT debug capture while in debug mode.


6. the compact expected pattern for your 1 s chirp / 2 s silence schedule.
 
Memory Usage on Teensy 4.1:
  FLASH: code:66488, data:51720, headers:8764   free for files:7999492
   RAM1: variables:329856, code:63064, padding:2472   free for local variables:128896
   RAM2: variables:44128  free for malloc/new:480160

Memory Usage on Teensy 4.1:
  FLASH: code:121376, data:56884, headers:9128   free for files:7939076
   RAM1: variables:335616, code:117944, padding:13128   free for local variables:57600
   RAM2: variables:49216  free for malloc/new:475072 
 
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
static constexpr float ROUGH_SEARCH_THRESHOLD = 0.03f;
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
// Runtime switch: set to false to disable ALL right-channel (queue2) work --
// no queue2.begin()/.end(), no rightBuf allocation, no rightBuf writes, no
// queue2.readBuffer()/.freeBuffer(). With this false the sketch behaves as a
// left-channel-only capture/search, which is the safe fallback while
// debugging the memory/hang issue.
// ------------------------------------------------------------
static bool rightBufEnable = true;

// do not forget to *sizeof(type) -- otherwise strange things will happen
//
// Each channel now gets its own dynamically-allocated CHIRP_SAMPLES-length
// ring buffer instead of one combined mallocTest[] array. leftBuf is always
// allocated; rightBuf is only allocated (and only ever touched) when
// rightBufEnable is true.
static constexpr size_t CHANNEL_BUF_SIZE = CHIRP_SAMPLES * sizeof(float);

bool isDSP = true;

float *leftBuf = nullptr;   // left-channel  rx ring, always allocated
float *rightBuf = nullptr;  // right-channel rx ring, allocated only if rightBufEnable
    


// ============================================================
// Audio Objects
// ============================================================

AudioInputI2S       i2s1;
AudioRecordQueue    queue1;   // left channel  (i2s1 output 0)
AudioRecordQueue    queue2;   // right channel (i2s1 output 1) -- only started/read if rightBufEnable

AudioConnection patchCord1(i2s1, 0, queue1, 0);
AudioConnection patchCord2(i2s1, 1, queue2, 0);
// patchCord2 always exists (required at compile time), but is inert unless
// queue2.begin() is called, which only happens when rightBufEnable is true.


// ============================================================
// Large buffers
// ============================================================

// Large reference/search arrays live in normal RAM.
// Keeping them in DMAMEM pushes the Teensy 4.1 DMA region over its limit once
// AudioMemory() and the audio queue are also allocated.
static float refEnergy = 0.0f;

// Phase 1: reference chirp buffer.
// searchBuf holds the fixed reference chirp waveform (loaded once from SD in
// buildReferenceChirp()), indexed linearly 0..CHIRP_SAMPLES-1. It is not a
// live rx ring -- that role belongs to leftBuf / rightBuf above.
static float searchBuf[CHIRP_SAMPLES];
static uint32_t searchHead = 0;

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
    if (rightBufEnable)
    {
        queue2.end();
    }
    isDSP = false;

}

static void enterDspMode()
{
    queue1.begin();
    if (rightBufEnable)
    {
        queue2.begin();
    }
    isDSP = true;
    // if (ENABLE_DEBUG_CAPTURE)
    // {
    //     writeDebugCaptureFiles();
    // }
    Serial.println("Returning to DSP mode.");
}

static ChirpState dechirpState = {1.0f, 0.0f, 1.0f, 0.0f};
 
 
void buildReferenceChirp()
{
  // Reference chirp is loaded into searchBuf (linear, not circular) and
  // refEnergy is computed from it.
  u_int32_t lineCount = 0;
  refEnergy = 0.0f;

  File myFile = SD.open("ref_chirp.txt", FILE_READ);
  if (!myFile) {
    Serial.println("Error opening file!");
    return;
  }

  Serial.println("Reading data... please wait...");

  // Keep reading until the file ends OR we hit exactly 44100 lines
  while (myFile.available() && lineCount < CHIRP_SAMPLES) {
    String line = myFile.readStringUntil('\n');
    line.trim(); 

    if (line.length() > 0) {
      searchBuf[lineCount] = line.toFloat(); // Handles 0.0 perfectly
      refEnergy += searchBuf[lineCount] * searchBuf[lineCount];
      lineCount++;
    }
  }
  
  myFile.close();
  Serial.printf("Done! Loaded %d lines into the array.\n", lineCount);
  for(int i=0; i< 20; i++) {
    Serial.printf("Last value: %.8f\n", searchBuf[i]); 
  }
 
}

 
 

void setSearchBufferMode(BufferMode mode)
{
    // searchBufferMode = mode;
    // memset(searchBuf, 0, sizeof(searchBuf));
    // searchHead = 0;

    // if (mode == BUFFER_DECHIRP)
    // {
    //     memset(fftIn, 0, sizeof(fftIn));
    //     memset(fftMag, 0, sizeof(fftMag));
    //     memset(sumSpecRe, 0, sizeof(sumSpecRe));
    //     memset(sumSpecIm, 0, sizeof(sumSpecIm));
    // }
}

 
// Correlates the live left-channel rx ring (leftBuf), starting at searchHead,
// against the fixed reference chirp (searchBuf). rightBuf (when enabled) is
// not used in the correlation yet.
float roughSearchForChirpStart()
{
    float acc = 0.0f;
    float rxEnergy = 0.0f;

    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
        float rx = leftBuf[idx];      // left-channel rx ring
        float ref = searchBuf[n];     // reference chirp (linear)

        acc += rx * ref;
        rxEnergy += rx * rx;
    }
    float denom = sqrtf(rxEnergy * refEnergy) + EPS;
    return fabsf(acc) / denom;
}
 
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
    if (rightBufEnable)
    {
        queue2.begin();
    }
 
    if (SD.exists("afsk_log.txt")) {
        SD.remove("afsk_log.txt");
    }

    logFile = SD.open("afsk_log.txt", FILE_WRITE);
    if (logFile) { Serial.println("afsk_log.txt opened"); }
    else { Serial.println("Failed to open afsk_log.txt"); }


    // --- leftBuf: always allocated ---
    leftBuf = static_cast<float *>(malloc(CHANNEL_BUF_SIZE));
    if (leftBuf != nullptr)
    {
        memset(leftBuf, 0, CHANNEL_BUF_SIZE);
        Serial.print("leftBuf allocated and cleared: ");
        Serial.println(CHANNEL_BUF_SIZE);
    }
    else
    {
        Serial.println("leftBuf allocation failed");
    }

    // --- rightBuf: only allocated when rightBufEnable is true ---
    if (rightBufEnable)
    {
        rightBuf = static_cast<float *>(malloc(CHANNEL_BUF_SIZE));
        if (rightBuf != nullptr)
        {
            memset(rightBuf, 0, CHANNEL_BUF_SIZE);
            Serial.print("rightBuf allocated and cleared: ");
            Serial.println(CHANNEL_BUF_SIZE);
        }
        else
        {
            Serial.println("rightBuf allocation failed");
        }
    }
    else
    {
        Serial.println("rightBufEnable=false: skipping rightBuf allocation.");
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

    // Safety guard: if leftBuf failed to allocate, don't run the DSP path --
    // this avoids dereferencing a null pointer deep inside the audio ISR
    // servicing path (which is a classic silent-hang cause).
    if (leftBuf == nullptr)
    {
        if (!bufMissingWarned)
        {
            Serial.println("leftBuf is null -- DSP path disabled. Check heap/allocation.");
            bufMissingWarned = true;
        }
        if (digitalRead(MODE_BUTTON_PIN) == LOW)
        {
            enterMtpMode();
            delay(500);
        }
        return;
    }
 
    while (queue1.available() > 0 && (!rightBufEnable || queue2.available() > 0))
    {
        int16_t *pL = queue1.readBuffer();
        int16_t *pR = nullptr;

        if (!pL) break;

        if (rightBufEnable)
        {
            pR = queue2.readBuffer();
            if (!pR)
            {
                // Avoid leaking the left buffer we already claimed.
                queue1.freeBuffer();
                break;
            }
        }

        //Serial.println("1 ");

        // Correlations computed for this audio block. At most STEPS_PER_BLOCK
        // (AUDIO_BLOCK_SAMPLES / SEARCH_STEP_N) of these will be filled in;
        // fewer will be filled while totalSamples is still ramping up to
        // CHIRP_SAMPLES for the very first block(s).
        float corrResults[STEPS_PER_BLOCK];
        uint32_t corrCount = 0;

        for (uint32_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float sampleL = (float)pL[i] * (1.0f / 32768.0f);

            leftBuf[searchHead] = sampleL;

            if (rightBufEnable && rightBuf != nullptr)
            {
                float sampleR = (float)pR[i] * (1.0f / 32768.0f);
                rightBuf[searchHead] = sampleR;
            }

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
            // Print all of this block's correlations on a single line.
            float maxCorr = corrResults[0];
            // Serial.print("corr[N=");
            // Serial.print(SEARCH_STEP_N);
            // Serial.print(", cnt=");
            // Serial.print(corrCount);
            // Serial.print("]: ");
            for (uint32_t k = 0; k < corrCount; k++)
            {
                // Serial.print(corrResults[k], 6);
                // Serial.print(' ');
                if (corrResults[k] > maxCorr)
                {
                    maxCorr = corrResults[k];
                }
            }
            // Serial.print(" max=");
            // Serial.println(maxCorr, 6);

            if (maxCorr > ROUGH_SEARCH_THRESHOLD)
            {

              bool dumpLog = true; 
              if(dumpLog) {
                snprintf(logBuf, sizeof(logBuf), "SEARCHING -> DECHIRPING roughCorr %7.4f", maxCorr); 
//                logFilePrint(logBuf);

                // Dump the rx signal (left channel ring, leftBuf) in time order.
                for (uint32_t n = 0; n < CHIRP_SAMPLES; n++) {
                    uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
                    float rx = leftBuf[idx];
                    logFile.println(rx, 8);
                }
                
                if (rightBufEnable) {
                    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++) {
                        uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
                        float rx = rightBuf[idx];
                        logFile.println(rx, 8);
                    }
                }   


                logFile.close();   
                // logFile.println("==ref starts====");

                // for (uint32_t n = 0; n < CHIRP_SAMPLES; n++) {
                //     logFile.println(searchBuf[n], 8);
                // } 

                // logFile.println("==ref ends====");
              }

                chirpStartSample = totalSamples - CHIRP_SAMPLES;
                chirpStartFound = true;
                machineState = STATE_DECHIRPING;

                setSearchBufferMode(BUFFER_DECHIRP);
                // fftWindowStart = chirpStartSample;
                // initChirpStateFromOffset(dechirpState, 0u);

                Serial.print("[STATE=TRANSITION] SEARCHING -> DECHIRPING sample=");
                Serial.print(chirpStartSample);
                Serial.print(" searchIters=");
                Serial.print(searchIterations);
                Serial.print(" corr=");
                Serial.print(maxCorr, 4);
                Serial.println();

                digitalWrite(STATUS_1_PIN, HIGH); 
                delay(1000);

 
            }
        }
        //Serial.print("2.3 ");
 
        queue1.freeBuffer();
        if (rightBufEnable)
        {
            queue2.freeBuffer();
        }
    }

    //Serial.print("3 ");


    if (digitalRead(MODE_BUTTON_PIN) == LOW)
    {
        enterMtpMode();
        delay(500);
    }
}
