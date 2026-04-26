#include <Audio.h>
#include <arm_math.h>

// ─── Method Selection ─────────────────────────────────────────────────────────
enum DemodMethod { DEMOD_GOERTZEL_IIR, DEMOD_FFT_SLIDING };
const DemodMethod DEMOD_SELECT = DEMOD_GOERTZEL_IIR;

// ─── AFSK Parameters ─────────────────────────────────────────────────────────
#define FREQ_MARK   6000.0f
#define FREQ_SPACE  8000.0f
#define BAUD_RATE   300
#define SAMPLE_RATE 44100.0f
#define FFT_N       256

// ─── LPF Cutoff ───────────────────────────────────────────────────────────────
// Cutoff should be around 0.5 * baud_rate to pass the envelope, reject carrier.
// LPF alpha for single-pole IIR:  alpha = 1 - exp(-2π·fc/fs)
// fc = 150 Hz (half baud), fs = 44100
#define LPF_FC      150.0f
const float LPF_ALPHA = 1.0f - expf(-2.0f * PI * LPF_FC / SAMPLE_RATE);  // ≈ 0.0213

// ─── Audio Objects ────────────────────────────────────────────────────────────
AudioInputI2S     i2s_in;
AudioRecordQueue  queue1;
AudioConnection   patchCord1(i2s_in, 0, queue1, 0);

// ═════════════════════════════════════════════════════════════════════════════
//  METHOD A: Per-sample IQ envelope — Goertzel as a running IIR correlator
//  
//  Math:
//    I_mark[n]  = x[n]·cos(2π·f_mark·n/fs)   ──► LPF ──► I_m
//    Q_mark[n]  = x[n]·sin(2π·f_mark·n/fs)   ──► LPF ──► Q_m
//    E_mark[n]  = I_m² + Q_m²
//
//    Same for space. decision = E_mark > E_space
// ═════════════════════════════════════════════════════════════════════════════

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

// ═════════════════════════════════════════════════════════════════════════════
//  METHOD B: Sliding-window FFT (overlapping blocks, hop = 1 sample)
//  Full overlap is expensive — instead use hop = FFT_N/4 as a compromise.
//  Gives 4 decisions per FFT_N samples (much denser than block mode).
// ═════════════════════════════════════════════════════════════════════════════

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

int fftDecide() {
  // Copy ring buffer in order into fftIn[], apply window
  for (int i = 0; i < FFT_N; i++) {
    int idx = (fftRingHead + i) % FFT_N;
    fftIn[i] = fftRingBuf[idx] * hanWin[i];
  }
  arm_rfft_fast_f32(&fftInst, fftIn, fftOut, 0);

  // Magnitude at target bins (sum ±1 neighbor)
  auto mag = [&](int bin) -> float {
    float re = fftOut[2*bin], im = fftOut[2*bin+1];
    return re*re + im*im;  // squared magnitude, skip sqrt
  };
  int mb = freqToBin(FREQ_MARK), sb = freqToBin(FREQ_SPACE);
  float em = mag(mb-1)+mag(mb)+mag(mb+1);
  float es = mag(sb-1)+mag(sb)+mag(sb+1);
  return (em > es) ? 1 : 0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Clock Recovery — simple early/late gate on the soft-decision stream
//  Detects transitions in the decision signal and realigns the sample clock.
// ═════════════════════════════════════════════════════════════════════════════

const int SAMPLES_PER_BIT = (int)(SAMPLE_RATE / BAUD_RATE);  // 147
int  clockCounter  = 0;       // counts samples within current bit window
int  lastDecision  = -1;      // previous per-sample decision
int  lastBit       = -1;      // last committed bit output

// Call every sample with the current soft decision (0 or 1).
// Returns committed bit when clock fires, -1 otherwise.
int clockRecovery(int softBit) {
  int committed = -1;

  // Edge detected → nudge clock toward center of symbol
  // The logic is this. 
  // clockRecovery is called everytime softBit is calculated 
  // softBit are transitioning between 1 and 0 as : 1111 0000 11111 0000 
  // clockCounter is bit phase indicating the relative position within a bit
  // clockCounter:   0 -------- 73 -------- 147
  // The goal is to lock the bit phase to the softBit transition, so that 
  // 1, we know the mid phase of bit
  // 2, bit is recovered at the mid phase
  // 
   real signal:   | edge ---- midpoint ---- edge |
                ↑ aligned here
  within the 
  // draw a softBit If the bit edge (transition) is aligned with the counter center, 
  // the bit mid is aligned to the counter edge, where clockCounter == SAMPLES_PER_BIT 
  if (lastDecision != -1 && softBit != lastDecision) {
  
    // Pull clock counter toward SAMPLES_PER_BIT/2 (midpoint)
    int mid = SAMPLES_PER_BIT / 2;
    if (clockCounter < mid)
      clockCounter += 2;   // we're early → slow down slightly
    else
      clockCounter -= 2;   // we're late  → speed up slightly
  }
  lastDecision = softBit;

  // Sample at midpoint of bit window (most reliable point)
  clockCounter++;
  if (clockCounter >= SAMPLES_PER_BIT) {
    clockCounter = 0;
    committed = softBit;   // commit the decision at window center
  }

  return committed;
}

// ─── Bit stream printer ────────────────────────────────────────────────────
int  bitCount = 0;
char bitLine[9];

void emitBit(int bit) {
  Serial.print(bit);
  bitLine[bitCount++] = '0' + bit;
  if (bitCount == 8) {
    bitLine[8] = '\0';
    // Decode byte value too
    uint8_t byteVal = 0;
    for (int i = 0; i < 8; i++)
      if (bitLine[i] == '1') byteVal |= (1 << (7 - i));
    Serial.printf("  [%s] = 0x%02X\n", bitLine, byteVal);
    bitCount = 0;
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
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

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  if (!queue1.available()) return;

  int16_t* block = queue1.readBuffer();

  for (int i = 0; i < 128; i++) {
    float x = block[i] / 32768.0f;

    int softBit = -1;

    if (DEMOD_SELECT == DEMOD_GOERTZEL_IIR) {
      // ── Per-sample IQ envelope (your guideline) ───────────────────────────
      float em = markDet.process(x);
      float es = spaceDet.process(x);
      softBit = (em > es) ? 1 : 0;

    } else {
      // ── Sliding FFT (hop-based) ───────────────────────────────────────────
      fftRingBuf[fftRingHead] = x;
      fftRingHead = (fftRingHead + 1) % FFT_N;
      fftHopCount++;
      if (fftHopCount >= FFT_HOP) {
        fftHopCount = 0;
        softBit = fftDecide();
      }
    }

    // Clock recovery + bit commit
    if (softBit != -1) {
      int committed = clockRecovery(softBit);
      if (committed != -1) emitBit(committed);
    }
  }

  queue1.freeBuffer();
}
