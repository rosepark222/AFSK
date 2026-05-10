#include <Audio.h>
#include <arm_math.h>

// ─── Method Selection ─────────────────────────────────────────────────────────
enum DemodMethod { DEMOD_GOERTZEL_IIR, DEMOD_FFT_SLIDING };
const DemodMethod DEMOD_SELECT = DEMOD_GOERTZEL_IIR;

// ─── AFSK Parameters ─────────────────────────────────────────────────────────
//#define FREQ_MARK   6000.0f
//#define FREQ_SPACE  8000.0f

#define FREQ_MARK   15000.0f
#define FREQ_SPACE  17000.0f

#define BAUD_RATE   100
#define SAMPLE_RATE 44100.0f
#define FFT_N       256
 
 
#define LPF_FC      2*BAUD_RATE // 600.0f   // try 2× baud rate first, then tune down
const float LPF_ALPHA = 1.0f - expf(-2.0f * PI * LPF_FC / SAMPLE_RATE);  // ≈ 0.0213

// ─── Audio Objects ────────────────────────────────────────────────────────────
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;
AudioConnection   patchCord1(i2s_in, 0, queue1, 0);
 
struct IQDetector {
  float phaseInc;   // 2π·f/fs
  float phase;      // running phase accumulator
  float lpf_I;      // LPF state for I
  float lpf_Q;      // LPF state for Q

  void init(float freq) {
    phaseInc = 2.0f * PI * freq / SAMPLE_RATE;
    phase    = 0.0f;
    lpf_I    = 0.0f;
    lpf_Q    = 0.0f;
  }

  // Returns instantaneous envelope energy E[n] = (LPF(x·cos))² + (LPF(x·sin))²
  float process(float x) {
    float c = cosf(phase);
    float s = sinf(phase);

    // Mix down to baseband
    float i_raw = x * c;
    float q_raw = x * s;

    // Single-pole LPF:  y[n] = y[n-1] + alpha*(x[n] - y[n-1])
    lpf_I += LPF_ALPHA * (i_raw - lpf_I);
    lpf_Q += LPF_ALPHA * (q_raw - lpf_Q);

    // Advance phase (stays numerically stable vs accumulating n*phaseInc)
    phase += phaseInc;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;

    return lpf_I * lpf_I + lpf_Q * lpf_Q;
  }
};

IQDetector markDet, spaceDet;
 
float fftRingBuf[FFT_N];
int   fftRingHead = 0;
int   fftHopCount = 0;
const int FFT_HOP = FFT_N / 4;  // overlap 75%

float hanWin[FFT_N];
float fftIn[FFT_N], fftOut[FFT_N];
arm_rfft_fast_instance_f32 fftInst;

void buildHann() {
  for (int i = 0; i < FFT_N; i++)
    hanWin[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_N - 1)));
}

int freqToBin(float f) { return (int)(f * FFT_N / SAMPLE_RATE + 0.5f); }


// ═════════════════════════════════════════════════════════════════════════════
//  Clock Recovery — simple early/late gate on the soft-decision stream
//  Detects transitions in the decision signal and realigns the sample clock.
// ═════════════════════════════════════════════════════════════════════════════

const int SAMPLES_PER_BIT = (int)(SAMPLE_RATE / BAUD_RATE);
const int mid = SAMPLES_PER_BIT / 2;
int  clockCounter  = 0;       // counts samples within current bit window
int  lastSoft     = -1;
int  lastDecision  = -1;      // previous per-sample decision
int  lastBit       = -1;      // last committed bit output


// Call every sample with the current soft decision (0 or 1).
// Returns committed bit when clock fires, -1 otherwise.
int clockRecovery(int softBit) {
  int committed = -1;
 
  if (softBit != -1 && lastSoft != -1 && softBit != lastSoft) {
    int mid = SAMPLES_PER_BIT / 2;
    if (clockCounter < mid)
      clockCounter += 2;   // bit transition (edge) is early (left) -> clock is too slow ->  speed up the clock
    else
      clockCounter -= 2;   // bit transition (edge) is late (right) -> clock is too fast ->  slow down the clock
    lastSoft = softBit;
  }
  if (softBit != -1) lastSoft = softBit;

  // Always advance by 1 sample — this is the key fix
  clockCounter++;
  if (clockCounter >= SAMPLES_PER_BIT) {
    clockCounter = 0;
    // Commit whatever the most recent soft decision was
    if (lastSoft != -1) committed = lastSoft;
  }
  return committed;
}
 
// ─── Bit stream printer ────────────────────────────────────────────────────
int  bitCount = 0;
char bitLine[9];
void emitBit(int bit) {
  Serial.print(bit);
  if (++bitCount % 100 == 0) Serial.println();
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.println("_START");
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  AudioMemory(16);
  queue1.begin();

  markDet.init(FREQ_MARK);
  spaceDet.init(FREQ_SPACE);

  arm_rfft_fast_init_f32(&fftInst, FFT_N);
  buildHann();
  memset(fftRingBuf, 0, sizeof(fftRingBuf));

  const char* m = (DEMOD_SELECT == DEMOD_GOERTZEL_IIR) ? "IQ-Envelope (per-sample)" : "Sliding FFT";
  Serial.printf("AFSK Demod | %s | LPF_alpha=%.4f | SPB=%d\n", m, LPF_ALPHA, SAMPLES_PER_BIT);
}

uint32_t prevTime; 
int prevBit;
 
void loop() {
  // Audio processing — fast, no Serial
  if (queue1.available()) {
    int16_t* block = queue1.readBuffer();
    for (int i = 0; i < 128; i++) {
      float x = block[i] / 32768.0f;

      int softBit = -1;

      if (DEMOD_SELECT == DEMOD_GOERTZEL_IIR) {
        float em = markDet.process(x);
        float es = spaceDet.process(x);
        softBit = (em > es) ? 1 : 0;
      } 
      // else {
      //   // ── Sliding FFT (hop-based) ───────────────────────────────────────────
      //   fftRingBuf[fftRingHead] = x;
      //   fftRingHead = (fftRingHead + 1) % FFT_N;
      //   fftHopCount++;
      //   if (fftHopCount >= FFT_HOP) {
      //     fftHopCount = 0;
      //     softBit = fftDecide();
      //   }
      //   // softBit stays -1 for the other HOP-1 samples
      // }

      int committed = clockRecovery(softBit);
      if (committed != -1) {
          //emitBit(committed);
          Serial.print(committed);
          if (++bitCount % 100 == 0) Serial.println();

          uint32_t printCycles = ARM_DWT_CYCCNT - prevTime;
          // Print this separately so it doesn't recurse :)
          if(committed == prevBit)
            //Serial.printf(" %d bit, bit interval %lu cycles (%.2f us)\n", committed, 
            //            printCycles, (float)printCycles / (F_CPU / 1000000.0f));
            Serial.printf("x");

          prevTime = ARM_DWT_CYCCNT;
          prevBit = committed;
      }
    }
    queue1.freeBuffer();
  }
}

 
