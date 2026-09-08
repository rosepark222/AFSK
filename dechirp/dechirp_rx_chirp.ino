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

// Streamed FFT-magnitude debug capture.
// Each 4096-sample block is written immediately to the SD card to avoid large RAM usage.
static constexpr bool ENABLE_DEBUG_CAPTURE = true;
static constexpr uint32_t DEBUG_CAPTURE_BLOCKS = 32u;
static constexpr uint32_t DEBUG_CAPTURE_SAMPLES = DEBUG_CAPTURE_BLOCKS * FFT_N;
static constexpr uint32_t DEBUG_FLUSH_INTERVAL = 1u;

static constexpr uint8_t MODE_BUTTON_PIN = 0;
static constexpr uint8_t STATUS_1_PIN = 1;

// do not forget to *sizeof(type) -- otherwise strange things will happen
static constexpr size_t MALLOC_TEST_SIZE = CHIRP_SAMPLES*sizeof(float); 
bool isDSP = true;
float *mallocTest = nullptr;
    


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
    isDSP = false;

}

static void enterDspMode()
{
    queue1.begin();
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
      mallocTest[lineCount] = line.toFloat(); // Handles 0.0 perfectly
      refEnergy += mallocTest[lineCount] * mallocTest[lineCount];
      lineCount++;
    }
  }
  
  myFile.close();
  Serial.printf("Done! Loaded %d lines into the array.\n", lineCount);
  for(int i=0; i< 20; i++) {
    Serial.printf("Last value: %.8f\n", mallocTest[i]); 
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

 

float roughSearchForChirpStart()
{
    float acc = 0.0f;
    float rxEnergy = 0.0f;

    for (uint32_t n = 0; n < CHIRP_SAMPLES; n++)
    {
        uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
        float rx = searchBuf[idx];
        acc += rx * mallocTest[n];

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
 
    if (SD.exists("afsk_log.txt")) {
        SD.remove("afsk_log.txt");
    }

    logFile = SD.open("afsk_log.txt", FILE_WRITE);
    if (logFile) { Serial.println("afsk_log.txt opened"); }
    else { Serial.println("Failed to open afsk_log.txt"); }


    mallocTest = static_cast<float *>(malloc(MALLOC_TEST_SIZE));
    if (mallocTest != nullptr)
    {
        memset(mallocTest, 0, MALLOC_TEST_SIZE);
        // for (uint32_t n = 0; n < CHIRP_SAMPLES; n++) {
        //     mallocTest[n] = 1.0f;
        // } 
        // mallocTest[0] = 1.0f;
        // mallocTest[1] = 2.0f;
        // mallocTest[2] = 3.0f;

        Serial.print("mallocTest allocated and cleared: ");
        Serial.println(MALLOC_TEST_SIZE);
    }
    else
    {
        Serial.println("mallocTest allocation failed");
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
 
    while (queue1.available() > 0)
    {
        int16_t *p = queue1.readBuffer();

        if (!p) break;

        //Serial.println("1 ");

        for (uint32_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            float sample = (float)p[i] * (1.0f / 32768.0f);

            // Feed the rolling search buffer used for chirp start detection.
            searchBuf[searchHead] = sample;
            searchHead = (searchHead + 1) % CHIRP_SAMPLES;

            totalSamples++;
 
        }
 
        if (!chirpStartFound && totalSamples >= CHIRP_SAMPLES)
        {
            //Serial.print("2.1");
            machineState = STATE_SEARCHING;
 
            float roughCorr = roughSearchForChirpStart();
 
            searchIterations++;
            //Serial.print("2.2-"); 
            if(roughCorr > 0.001) 
                Serial.println(roughCorr, 6);
 
            //Serial.print("2.3 ");

            if (roughCorr > ROUGH_SEARCH_THRESHOLD)
            {

              bool dumpLog = true; 
              if(dumpLog) {
                snprintf(logBuf, sizeof(logBuf), "SEARCHING -> DECHIRPING roughCorr %7.4f", roughCorr); 
//                logFilePrint(logBuf);

                // dumpSearchBuf();
                for (uint32_t n = 0; n < CHIRP_SAMPLES; n++) {
                    uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
                    float rx = searchBuf[idx];
                    logFile.println(rx, 8);
                }
                    
                // logFile.println("==ref starts====");

                // for (uint32_t n = 0; n < CHIRP_SAMPLES; n++) {
                //     //uint32_t idx = (searchHead + n) % CHIRP_SAMPLES;
                //     //float rx = searchBuf[idx];
                //     logFile.println(mallocTest[n], 8);
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
                Serial.print(roughCorr, 4);
                Serial.println();

                digitalWrite(STATUS_1_PIN, HIGH); 

 
            }
        }
        //Serial.print("2.3 ");
 
        queue1.freeBuffer();
    }

    //Serial.print("3 ");


    if (digitalRead(MODE_BUTTON_PIN) == LOW)
    {
        enterMtpMode();
        delay(500);
    }
}
