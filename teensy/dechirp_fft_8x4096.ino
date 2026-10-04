/*
 * fft32k_sd_test_teensy41.ino
 *
 * ORIGINAL PROMPT (kept here as the spec for this sketch):
 * -----------------------------------------------------------------------------
 * I need to test the fft_32k in teensy 4.1, before do the live audio, let's do the below to test it out.
 * Do not worry about optimization, do exactly what I say
 * Use the pasted code for reference (e.g., MTP code).
 *
 * First,
 * implement a function fft_8x4096 (cooley-tukey method of 32k fft using 8 of 4096 fft and twiddle)
 *
 * Second,
 * do the below in the setup:
 *
 * 1,
 * allocate float arrays in RAM1
 * real_ref        32768  size
 * imag_ref        32768  size
 * they are holding reference chirp read from SD card.
 *
 * 2,
 * do malloc these float array in the RAM2
 * leftRingBuf     32768  size
 * rightRingBuf    32768  size
 * they are circular buffer holding incoming audio data but for now, we load from SD card
 *
 * 3, read file a_real_ref_32k.txt and initialize real_ref
 *    read file a_imag_ref_32k.txt and initialize imag_ref
 * 4, read  a_rx_test_left.txt and save to leftRingBuf
 *    read  a_rx_test_left.txt and save to rightRingBuf  (not needed now but do it)
 * 5, mix real_ref and leftRingBuf  and save to leftRingBuf
 *    mix imag_ref and leftRingBuf and save to rightRingBuf
 * 6, save leftRingBuf to a_mixed_32k_real.txt
 *    save rightRingBuf to a_mixed_32k_imag.txt
 * 7, do fft_8x4096 (leftRingBuf) and save to leftRingBuf (in place fft)
 *    do fft_8x4096 (rightRingBuf) and save to rightRingBuf (in place fft)
 * 8, save leftRingBuf to a_fft_32k_real.txt
 *    save rightRingBuf to a_fft_32k_imag.txt
 * 9, calcualte magnitude from real (leftRingBuf) and imagiary (rightRingBuf) and save to leftRingBuf
 * 10, print the index of the max in leftRingBuf and the max value
 * 11, save leftRingBuf to a_fft_32k_mag.txt
 * 12, enter MTP mode so that I can access the SD card through USB
 * -----------------------------------------------------------------------------
 *
 * HOW THE SPEC IS INTERPRETED
 *   - leftRingBuf  = real part, rightRingBuf = imaginary part of ONE complex signal.
 *   - fft_8x4096(re, im) is a complex FFT on split arrays, done in place. Step 7 is
 *     therefore ONE call, fft_8x4096(leftRingBuf, rightRingBuf): a complex FFT needs
 *     both parts at once, so it can't be done as two independent calls.
 *   - Step 5 is done in a single pass over i (rx must be read before leftRingBuf is
 *     overwritten, and both mixes need the ORIGINAL rx).
 *   - Imag mix uses a MINUS sign (im_mixed = -rx * imag_ref, i.e. mixing with the
 *     conjugate reference) as in the previous spec. Set MIX_NEGATE_IMAG to 0 to remove it.
 *   - Step 4 reads a_rx_test_left.txt for BOTH buffers exactly as written. rightRingBuf is
 *     fully overwritten in step 5, so this has no effect on the results.
 *
 * fft_8x4096 (decimation in time, completely in place, no extra 32k buffers):
 *   1. for each n2 in 0..7: gather x[8*n1+n2] -> 4096-pt arm_cfft_f32 -> scatter to the
 *      SAME slots 8*k1+n2 (row n2 reads and writes the same set of slots)
 *   2. for each k1: the 8 values in slots 8*k1..8*k1+7 get twiddle W32768^(n2*k1) and an
 *      8-point DFT; X[k1 + 4096*k2] is left in slot 8*k1+k2
 *   3. in-place permutation slot p -> p*4096 mod 32767 (cycle following) puts
 *      X[k1 + 4096*k2] at index k1 + 4096*k2
 *
 * ARDUINO SETTINGS: Tools -> USB Type -> "Serial + MTP Disk (Experimental)", 600 MHz.
 * SD FILES: put them in the SD root, or set SD_PREFIX to e.g. "A/" (folder must exist).


 Things to check on your side

Arduino settings: set Tools → USB Type to "Serial + MTP Disk (Experimental)".
File location: the files are read from the SD root. Set SD_PREFIX to "A/" if they are in your A folder.
Overwriting files: existing output files are removed before writing, because FILE_WRITE appends on Teensy.
Extras I added: the sketch prints the fft_8x4096 time in cycles and ms, and the peak frequency next to the max index. Bins above N/2 are shown as negative frequencies.


 
------------------------------------------  dechirp and fft mag dump   ------------------------

case 1:
when rx == ref, index 0 had the peak:
6143.81250000                 <-  index 0 , max
4095.97900391
1024.09887695
0.01875143
0.00413183
0.00149396
0.00069700

case 2:
when rx is 100 sample delayed of ref, observed fft max output
index 0 - 3 
0     86.90357208
1     11.31561852
2      2.93232012
3      1.04041684

index -4, -3 (max), -2, -1 are
-4      3221.81274414
-3      5991.50048828         <-    index -3, and max
-2      4908.64404297
-1      1639.22534180


-------bin / freq range / sample range ----------------------
bin       fre begin freq end          rx lag range
32765    -4.038    -2.692             73.5	     110.25
32766    -2.692    -1.346             36.75	    73.5
32767    -1.346    0                   0	       36.75
0	     0         1.346               -36.75      0
1	     1.346     2.692               -73.5     -36.75	 
2	     2.692     4.038              -110.25    -73.5	
3	     4.038     5.384              -147.0    -110.25	


-----------------------------------------------------------------
Q: if rx (a_rx_test_left.txt) is 100 sample delayed version of the reference, which bin in the FFT result would show the max?
Assume the chirp is 32k sample (0.743 sec duration) of 300hz to 1500hz up chirp.

A:
**The peak lands at bin 3 in magnitude, which is index 32765 with the current sketch settings.**

**Beat frequency**

- Chirp rate: k = B/T = 1200 Hz / 0.743 s ≈ 1615 Hz/s.
- Delay: τ = 100 / 44100 s = 2.268 ms.
- Beat frequency: f = k·τ ≈ **3.66 Hz**.
- Bin spacing: Δf = 44100 / 32768 = 1.346 Hz.
- Beat bin: 3.66 / 1.346 ≈ **2.72**.

Using the chirp's own numbers, the beat bin is just `B × delay / Fs` = 1200 × 100 / 44100 = 2.72. 
It doesn't depend on N, only on bandwidth times delay.

**Which index shows the max**

The sign depends on the mixing:

| Setting | Mix | Beat frequency | argmax index |
|---|---|---|---|
| `MIX_NEGATE_IMAG = 1` (current) | rx × conj(ref) | −3.66 Hz | **32765** (bin −3) |
| `MIX_NEGATE_IMAG = 0` | rx × ref | +3.66 Hz | **3** |

With conjugate mixing, the delayed copy has instantaneous frequency f(t) − kτ, so after mixing 
it sits at −kτ. A real rx also has a negative-frequency image that mixes into a wideband term 
at roughly 600 to 3000 Hz. That term is spread across many bins and doesn't form a competing peak.

This assumes `a_imag_ref_32k.txt` is +sin φ. If your file holds −sin φ, the signs swap.

**Other things to expect**

- **Fractional bin:** 2.72 is not on a bin, so energy spreads into neighbouring bins from the rectangular window. 
                      Bin 3 should win with bin 2 close behind.
- **Magnitude:** for unit-amplitude rx, the peak should be about 14,000 to 16,000. 
              That is roughly half of the 32,668 overlapping samples, reduced by about 12% for the 0.28-bin offset.
- **Delay resolution:** one bin is 1/(1200/44100) ≈ **36.7 samples of delay**. 

Bin 3 therefore only tells you the delay is somewhere around 82 to 110 samples.

- Parabolic interpolation on the three bins around the peak recovers the fractional bin, 2.72, and so the delay.
- Your fine cross-correlation gets sample-level lag.
- **Peak search range:** search the full 32768 bins. A search over `[0, N/2)` would miss 32765 in the current setting.

If the peak shows up elsewhere, check three things: whether the delay is circular or zero-padded, the sign of `imag_ref`, and the `MIX_NEGATE_IMAG` setting.





------------------------------------------  fft time complexity   ------------------------
Timing: I'd expect fft_8x4096 to take roughly 10 ms, but that is an estimate, not a measurement.


performance of FFT:
Running fft_8x4096...
  fft_8x4096 time: 8597208 cycles = 14.33 ms
------------------------------------------------------------------------------------------ 


------------------------------------------  memory  ------------------------------------------
Memory Usage on Teensy 4.1:
  FLASH: code:124988, data:53644, headers:8756   free for files:7939076
   RAM1: variables:358880, code:122424, padding:8648   free for local variables:34336
   RAM2: variables:28800  free for malloc/new:495488
----------------------------------------------------------------------------------------------
Q:
It seems the code allocated smaller memory than expected, 

    leftRingBuf  = static_cast<float *>(malloc(CHANNEL_BUF_SIZE));
    rightRingBuf = static_cast<float *>(malloc(CHANNEL_BUF_SIZE));
allocated 262144 bytes total, but RAM2 only used 28K. Does compile smart enough to allocate it to RAM1 , if it can fit the malloc in ? 

Memory Usage on Teensy 4.1:
  FLASH: code:124988, data:53644, headers:8756   free for files:7939076
   RAM1: variables:358880, code:122424, padding:8648   free for local variables:34336
   RAM2: variables:28800  free for malloc/new:495488

A:
No. The compiler and linker never place malloc memory. That report only counts statically allocated variables, and the heap is filled at runtime.
Reading your numbers

RAM2: variables:28800   free for malloc/new:495488
28,800 bytes is static data in RAM2 (DMAMEM variables and library buffers). It doesn't include your two malloc calls, because nothing has run yet.
495,488 bytes is the heap, and on Teensy 4.x the heap lives in RAM2.

At runtime your 2 × 131,072 = 262,144 bytes come out of that heap, leaving about 233 KB free. The compiler can't move a malloc into RAM1. If RAM2 were too small, malloc would return nullptr, which the sketch checks for.

Verify it yourself

Print the addresses after the malloc calls:

cpp
Serial.printf("real_ref=%p  leftRingBuf=%p  rightRingBuf=%p\n",
              real_ref, leftRingBuf, rightRingBuf);


 mem_info: At runtime your 2 x 131,072 = 262,144 bytes come out of that heap, leaving about 233 KB free
 mem_info: 0x2000_0000 to 0x2007_FFFF is RAM1
 mem_info: 0x2020_0000 to 0x2027_FFFF is RAM2
 mem_info: : real_ref=0x20037364  leftRingBuf=0x20207180  rightRingBuf=0x20227188

----------------------------------------------------------------------------------------------
Q. after malloc what will be the mem left 

A.
    RAM1:  34 KB free
    RAM2: 233 KB free
----------------------------------------------------------------------------------------------

 */

#include <Arduino.h>
#include <SD.h>
#include <MTP_Teensy.h>
#include <arm_math.h>
#include <arm_const_structs.h>

// ================= PARAMETERS ======================
#define SAMPLE_RATE      44100.0f
#define SD_PREFIX        "B/"            // e.g. "A/" if the files are in folder A
#define MIX_NEGATE_IMAG  1             // 1: im_mixed = -rx*imag_ref (conjugate reference)

#define FILE_REAL_REF    SD_PREFIX "real_300-1500hz_32k_chirp.txt"
#define FILE_IMAG_REF    SD_PREFIX "imag_300-1500hz_32k_chirp.txt"
#define FILE_RX_LEFT     SD_PREFIX "real_300-1500hz_32k_chirp_delay_100.txt"
#define FILE_RX_RIGHT    SD_PREFIX "real_300-1500hz_32k_chirp_delay_100.txt"   // as written in the spec (probably meant "right"); no effect, see header
#define FILE_MIXED_REAL  SD_PREFIX "a_mixed_32k_real.txt"
#define FILE_MIXED_IMAG  SD_PREFIX "a_mixed_32k_imag.txt"
#define FILE_FFT_REAL    SD_PREFIX "a_fft_32k_real.txt"
#define FILE_FFT_IMAG    SD_PREFIX "a_fft_32k_imag.txt"
#define FILE_FFT_MAG     SD_PREFIX "a_fft_32k_mag.txt"

// ==== DSP CORE BEGIN ====
static constexpr uint32_t FFT_N    = 4096;
static constexpr uint32_t NUM_ROWS = 8;
static constexpr uint32_t N_TOTAL  = FFT_N * NUM_ROWS;   // 32768

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

// ================= BUFFERS =========================
// Step 1: RAM1 (regular globals live in DTCM)
static float real_ref[N_TOTAL];
static float imag_ref[N_TOTAL];

// Step 2: RAM2 (malloc'd on Teensy 4.1)
float *leftRingBuf  = nullptr;
float *rightRingBuf = nullptr;

static constexpr size_t CHANNEL_BUF_SIZE = N_TOTAL * sizeof(float);   // do not forget sizeof(float)

// ================= SD HELPERS ======================
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

// One value per line. FILE_WRITE appends on Teensy, so remove any old file first.
bool saveFloatFile(const char *name, const float *src, uint32_t count)
{
    if (SD.exists(name)) SD.remove(name);
    File f = SD.open(name, FILE_WRITE);
    if (!f) {
        Serial.printf("ERROR: cannot create %s\n", name);
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        f.println(src[i], 8);
    }
    f.close();
    Serial.printf("  saved  %-28s %lu values\n", name, (unsigned long)count);
    return true;
}

// ================= TEST (steps 1-11) ================
bool runTest()
{
    // ---- 2. malloc leftRingBuf / rightRingBuf (RAM2) ----
    leftRingBuf  = static_cast<float *>(malloc(CHANNEL_BUF_SIZE));
    rightRingBuf = static_cast<float *>(malloc(CHANNEL_BUF_SIZE));
    if (leftRingBuf == nullptr || rightRingBuf == nullptr) {
        Serial.println("ERROR: leftRingBuf / rightRingBuf allocation failed");
        return false;
    }
    memset(leftRingBuf, 0, CHANNEL_BUF_SIZE);
    memset(rightRingBuf, 0, CHANNEL_BUF_SIZE);
    Serial.printf("leftRingBuf / rightRingBuf allocated: %lu bytes each\n", (unsigned long)CHANNEL_BUF_SIZE);
    
    Serial.printf(" mem_info: At runtime your 2 x 131,072 = 262,144 bytes come out of that heap, leaving about 233 KB free\n mem_info: 0x2000_0000 to 0x2007_FFFF is RAM1\n mem_info: 0x2020_0000 to 0x2027_FFFF is RAM2\n mem_info: : real_ref=%p  leftRingBuf=%p  rightRingBuf=%p\n", real_ref, leftRingBuf, rightRingBuf);

    // ---- 3. reference chirp -> real_ref, imag_ref (RAM1) ----
    Serial.println("Reading files from SD...");
    if (loadFloatFile(FILE_REAL_REF, real_ref, N_TOTAL) < 0) return false;
    if (loadFloatFile(FILE_IMAG_REF, imag_ref, N_TOTAL) < 0) return false;

    // ---- 4. rx test data -> leftRingBuf, rightRingBuf ----
    if (loadFloatFile(FILE_RX_LEFT,  leftRingBuf,  N_TOTAL) < 0) return false;
    if (loadFloatFile(FILE_RX_RIGHT, rightRingBuf, N_TOTAL) < 0) return false;

    // ---- 5. mix (single pass: both mixes need the ORIGINAL rx held in leftRingBuf) ----
    for (uint32_t i = 0; i < N_TOTAL; i++) {
        float rx = leftRingBuf[i];
        leftRingBuf[i]  = rx * real_ref[i];                 // real: rx * real_ref
#if MIX_NEGATE_IMAG
        rightRingBuf[i] = -rx * imag_ref[i];                // imag: -rx * imag_ref
#else
        rightRingBuf[i] =  rx * imag_ref[i];
#endif
    }

    // ---- 6. save mixed signals ----
    Serial.println("Saving mixed signals...");
    if (!saveFloatFile(FILE_MIXED_REAL, leftRingBuf,  N_TOTAL)) return false;
    if (!saveFloatFile(FILE_MIXED_IMAG, rightRingBuf, N_TOTAL)) return false;

    // ---- 7. FFT, in place: leftRingBuf = Re(X), rightRingBuf = Im(X) ----
    Serial.println("Running fft_8x4096...");
    uint32_t c0 = ARM_DWT_CYCCNT;
    fft_8x4096(leftRingBuf, rightRingBuf);
    uint32_t cyc = ARM_DWT_CYCCNT - c0;
    Serial.printf("  fft_8x4096 time: %lu cycles = %.2f ms\n", (unsigned long)cyc, cyc / (F_CPU / 1.0e3f));

    // ---- 8. save FFT result ----
    Serial.println("Saving FFT result...");
    if (!saveFloatFile(FILE_FFT_REAL, leftRingBuf,  N_TOTAL)) return false;
    if (!saveFloatFile(FILE_FFT_IMAG, rightRingBuf, N_TOTAL)) return false;

    // ---- 9. magnitude -> leftRingBuf ----
    for (uint32_t i = 0; i < N_TOTAL; i++) {
        float r = leftRingBuf[i];
        float m = rightRingBuf[i];
        leftRingBuf[i] = sqrtf(r * r + m * m);
    }

    // ---- 10. index and value of the max ----
    uint32_t maxIdx = 0;
    float maxVal = leftRingBuf[0];
    for (uint32_t i = 1; i < N_TOTAL; i++) {
        if (leftRingBuf[i] > maxVal) {
            maxVal = leftRingBuf[i];
            maxIdx = i;
        }
    }
    int32_t k = (maxIdx < N_TOTAL / 2) ? (int32_t)maxIdx : (int32_t)maxIdx - (int32_t)N_TOTAL;
    Serial.printf("MAX: index=%lu  value=%.6f  (%.3f Hz, bins above N/2 are negative)\n",
                  (unsigned long)maxIdx, maxVal, k * (SAMPLE_RATE / N_TOTAL));

    // ---- 11. save magnitude ----
    Serial.println("Saving magnitude...");
    if (!saveFloatFile(FILE_FFT_MAG, leftRingBuf, N_TOTAL)) return false;

    return true;
}

// ================= SETUP ===========================
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    buildTwiddles();

    if (SD.begin(BUILTIN_SDCARD)) {
        MTP.begin();
        MTP.addFilesystem(SD, "Teensy SD");

        if (runTest()) Serial.println("Test finished OK.");
        else           Serial.println("Test stopped because of an error (see above).");
    } else {
        Serial.println("SD init failed: MTP mode unavailable.");
    }

    // ---- 12. MTP mode: loop() serves the SD card over USB ----
    Serial.println("MTP mode: SD card is now accessible through USB.");
}

// ================= LOOP ============================
void loop()
{
    MTP.loop();
}
