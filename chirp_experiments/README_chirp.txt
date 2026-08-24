8/21
  two pending issues ; 128 corr resolution Alignment issue — correlation is sampled every 128 samples so you likely never hit the exact peak, giving 0.017 instead of ~1.0 even at close range.  
  Signal integrity issue — at distance, reverberation, multipath reflections, and background noise corrupt the chirp waveform so even a perfectly aligned correlation won't reach 1.0 because the received chirp no longer matches the clean reference exactly.
  


Good—this output is actually very informative. It tells us exactly where the problem is:
Window energy: 2.90        ✅ signal exists
Corr max: 0.00             ❌ correlation completely broken
No peaks

So:
👉 Audio path is working
👉 Chirp is being received
👉 BUT correlation is failing hard

🔥 Root Cause (very likely)
Your TX and RX chirps are NOT MATCHED.
Even a small mismatch → correlation ≈ 0
The usual culprits:
1. Chirp duration mismatch
Your RX assumes:
CHIRP_DURATION = 0.5s

But your TX might be:

slightly shorter
slightly longer
different sample count

👉 This alone will kill correlation.

2. FFT window too small (THIS IS HUGE)

You are using:

FFT_SIZE = 1024

But your chirp is:

0.5 sec × 44100 = 22050 samples

👉 You are correlating 1024 samples against a 22050-sample chirp
That’s like trying to detect a full song using 0.02 seconds.

👉 Result:

Corr max ≈ 0  (exactly what you see)

✅ Fix #1 (MOST IMPORTANT)
Match FFT window to chirp length

You must increase:

#define FFT_SIZE 2048   // minimum

Better:

#define FFT_SIZE 4096

Even better:

#define FFT_SIZE 8192

✅ Fix #2 (make RX chirp EXACTLY match TX)

Your RX currently uses:

float phase = 2 * PI * (F_START * t + 0.5f * k * t * t);

But your TX likely uses:

freq = f0 + k*t (with wrap)

These are NOT the same signal.

🔧 Replace RX chirp with THIS (matches TX)

Inside generateReference():
float bandwidth = F_END - F_START;
float k = bandwidth / CHIRP_DURATION;

for (int n = 0; n < FFT_SIZE; n++) {

    float t = (float)n / SAMPLE_RATE;

    float freq = F_START + k * t;

    if (freq > F_END)
        freq -= bandwidth;

    float phase = 2 * PI * (F_START * t + 0.5f * k * t * t);

    ref_time[n] = cosf(phase);

    down_re[n] =  cosf(phase);
    down_im[n] = -sinf(phase);
}

✅ Fix #3 (quick sanity test)

Temporarily disable correlation and test de-chirp directly.

In loop() replace everything with:

getWindow(read_idx);

dechirp(0);

int id = detectID();

Serial.println(id);

read_idx = (read_idx + HOP_SIZE) % CIRC_SIZE;

👉 If this works:

your chirp is correct
correlation is the only issue

✅ Fix #4 (lower threshold for now)
#define THRESHOLD 0.01f
🔥 What you SHOULD see after fix
Audio peak: 0.2
Window energy: 5.0
Corr max: 3.5 @ 200
Peak @ 200 val=3.5
ID bin=xx

