


---------------------------------------------------------------------------
---------------------------------------------------------------------------
---------------------------------------------------------------------------
---------------------------------------------------------------------------
9/8
Q:
if I know the timing of chirp start in two mic , the difference is TDOA ?


A:
Yes, exactly. If you measure the exact arrival time of a chirp signal at microphone A and microphone B, subtracting one time from the other gives you the TDOA (Time Difference of Arrival).
$$\text{TDOA} = t_{\text{mic B}} - t_{\text{mic A}}$$ 
Because you are using a chirp (a frequency sweep), you have chosen the absolute best signal type for this. However, getting the exact start time of a signal in the real world can be very tricky because of background noise and room echoes (multipath).
Since you are already calculating a single-lag correlation, you can use cross-correlation to find your TDOA with microsecond precision.
------------------------------
## How to turn your Correlation into TDOA
Instead of just calculating a single zero-lag correlation, you can slide one microphone's signal past the other by just a small window of samples (lags) around the expected delay.

   1. Calculate the Correlation Curve: Compute the dot product for a narrow range of sample shifts (e.g., shifting Signal B by -100 to +100 samples relative to Signal A).
   2. Find the Peak: The specific shift value (lag) that produces the highest correlation score is your time difference in samples.
   3. Convert to Time: Divide that sample shift by your sample rate (44,100 Hz).

$$\text{TDOA (seconds)} = \frac{\text{Sample Shift at Peak}}{44100}$$ 
## Why this is perfect for the Teensy 4.0
Earlier, we looked at how calculating a massive 44,100-sample shift window takes 3.2 seconds. But for acoustics and microphones, you only need a tiny shift window.
Sound travels through the air at roughly 343 meters per second.

* If your two microphones are spaced 1 meter apart, the maximum possible time delay between them is only about 2.9 milliseconds.
* At a 44,100 Hz sample rate, 2.9 milliseconds is only 128 samples (0.0029 × 44100).

This means you only need to compute the correlation across a window of 256 total lags (from -128 to +128 samples) to capture every possible angle.
## The Performance Math:
Instead of doing a massive 44,100 × 44,100 grid, your Teensy only has to do:
$$44,100 \text{ multiplications} \times 256 \text{ lags} = \mathbf{11,289,600 \text{ operations}}$$ 
At 600 MHz, the Teensy 4.0 can compute this full 1-meter tracking window in under 20 milliseconds, allowing you to update your TDOA calculation more than 50 times per second in real-time.
To help calculate the exact code parameters for your TDOA setup, tell me:

* What is the physical distance between your two microphones?
* Are you planning to use this TDOA to calculate the angle/direction of the sound source?



---------------------------------------------------------------------------
9/7
Q:
roughSearchForChirpStart estiamte per 128 samples and detects about the time chirp is aligned with the reference. At this moment, we can do cross correlation between the searchBuf and reference. Because 1 sec teensy can do more than 10,000 of 44100 multilication-accumulation, 1 sec is more than enough to get the max correlation and determine if this is peak or not. This avoid entire mix-fft , which is fundamentally limited by frequency resolution of 4096 FFT. What do you think ? 

A:
Yes — that’s a good direction, and your reasoning is solid.

If your goal is **precise chirp start timing**, then doing a **full cross-correlation refinement in time domain** after rough trigger is often better than relying on 4096-block dechirp+FFT timing. FFT path is great for tone/energy decision:ws, but start-time resolution is block-quantized unless you add overlap/interpolation.

## Why this makes sense
- You already run a rough normalized correlation (`roughSearchForChirpStart`).
- Once roughCorr crosses threshold, you can spend the next ~1 s compute budget to:
  1. scan offsets around the rough start,
  2. find max normalized correlation,
  3. lock `chirpStartSample` at the true peak.
- Teensy 4.1 has enough MAC throughput for this windowed refinement.

## Suggested practical design
Use a **two-stage detector**:

1. **Stage A (current):** cheap streaming rough search every 128 samples.
2. **Stage B (new):** on trigger, run bounded peak search:
   - Search offsets in a local window, e.g. `[-4096, +4096]` or wider.
   - Compute normalized corr at each lag.
   - Pick best lag and optionally do **parabolic interpolation** around peak for sub-sample estimate.
3. Then start dechirp/FFT using refined start (if you still need spectral detection).

## Key optimization note
A naïve full 1-sec × many-lags correlation can get expensive if repeated heavily. To keep it safe:
- Decimate/coarse-to-fine (e.g., step 8 first, then step 1 near best).
- Restrict lag window around rough trigger.
- Reuse running sums if possible for normalization.
- Consider doing refinement once per candidate burst only.

## Important bug risk in your current file
You currently call:
```cpp
setSearchBufferMode(BUFFER_DECHIRP);
```
and inside it:
```cpp
memset(searchBuf, 0, sizeof(searchBuf));
```
This clears `searchBuf` exactly when you transition to DECHIRPING. Since `process4096Block()` reads from `searchBuf`, this can destroy alignment/data continuity.  
If you adopt correlation-based precise lock, you should avoid clearing active raw-audio history on transition.

## Bottom line
Your proposal is better for **timing alignment** and can reduce dependence on 4096 FFT resolution limits. I’d keep FFT for symbol/frequency decision if needed, but use **time-domain cross-correlation peak lock** as the alignment authority.


---------------------------------------------------------------------------
9/3
In embedded systems like Teensy, it is often better to allocate one large memory pool up front and manage it yourself if your application has distinct phases that never overlap.

Why:

Teensy has malloc() and free(), but there is no OS memory manager running in the background.
Dynamic allocation can cause heap fragmentation over time.
If Phase 1 allocates and frees many differently-sized blocks, Phase 2 may later be unable to obtain one large contiguous block even though enough total memory exists.
A statically allocated pool gives you deterministic behavior and avoids fragmentation.

cleaner approach is to use a union:
union
{
    struct {
        float fftBuf[16384];      // 64KB
        int16_t samples[32768];   // 64KB
    } phase1;                     // total = 128KB

    struct {
        float spectrum[32768];    // 128KB
    } phase2;                     // total = 128KB

    struct {
        uint8_t packetBuf[131072];// 128KB
    } phase3;                     // total = 128KB
} work;


All three point to the same starting byte of the union storage.
&work.phase1.fftBuf[0]
&work.phase2.spectrum[0]
&work.phase3.packetBuf[0]


---------------------------------------------------------------------------


https://share.google/aimode/6uXjbDO7qFYJuvZ02

rosep@MSI:/mnt/c/Users/rosep/Downloads/Chirp$


# generate chirp.txt  
python3 chirp_gen.py
 
# examine
python3 ../AFSK/spectrogram_analysis.py chirp.txt  44100 6000 8000
dechirp/Screenshot 2026-08-29 222747.png

# dechirp artificial chirp
python3 dechirp.py chirp.txt
dechirp/Screenshot 2026-08-29 184148.png


# dechirp real rx chirp
python3 dechirp.py ../AFSK/rx_chirp.txt
dechirp/Screenshot 2026-08-29 184315.png


During the mix the chirpPos points a specific 4096 block of reference chirp. When next 4096 sample arrives, next block of reference chirp is chosen and mix. On reset, the initial value of chirpPos is 0 and it does not guarantee the reference to be aligned with incoming chirp. That is the reason the rough estimation of the start is needed. Do you agree? Answer in one paragraph.

9:49 PM
Yes — I agree. In your current implementation, each 4096-sample FFT block is mixed with a reference chirp phase indexed by chirpPos = absIdx % CHIRP_SAMPLES, so the reference is effectively sliding in lockstep with the sample index, not with the unknown start of the incoming chirp. That means the reference phase is only aligned if the arriving chirp happens to begin at the same offset as the assumed phase reference, and on reset it starts at zero, which does not guarantee alignment. This is exactly why a rough onset estimate or timing search is needed: without knowing where the incoming chirp begins relative to the local sample clock, the dechirp is not coherent across the full chirp duration, and the accumulated energy will be weak or noisy.



