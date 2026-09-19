# RX Processing for a 32,768-Sample Chirp

Reference implementation:

[`dechirp/dechirp_rx_chirp.ino`](https://github.com/rosepark222/AFSK/blob/b8229ef9b513bc4d2d99a7b3131f8f968ce3cb58/dechirp/dechirp_rx_chirp.ino#L329)

## 1. Chirp model

The receiver processes a linear chirp from $\omega_0 = 2\pi f_0$ to $\omega_1 = 2\pi f_1$ with:

- $f_0 = 300\ \text{Hz}$
- $f_1 = 1500\ \text{Hz}$
- $F_s = 44100\ \text{Hz}$
- $N = 32768$ samples

The chirp duration is:

$$
T = \frac{N}{F_s} = \frac{32768}{44100} \approx 0.743038\ \text{s}
$$

The chirp slope is:

$$
K = \frac{f_1 - f_0}{T}
  = \frac{1500 - 300}{0.743038}
  \approx 1614.844\ \text{Hz/s}
$$

The reference chirp phase is:

$$
\phi[n] = 2\pi\left(f_0 t[n] + \frac{1}{2}K t[n]^2\right)
$$

where

$$
t[n] = \frac{n}{F_s}
$$

The analytic reference chirp is:

$$
s_{ref}[n] = e^{j\phi[n]}
           = \cos(\phi[n]) + j\sin(\phi[n])
$$

The conjugate reference used during dechirp is:

$$
s_{ref}^*[n] = e^{-j\phi[n]}
            = \cos(\phi[n]) - j\sin(\phi[n])
$$

---

## 2. RX processing overview

The receiver uses three stages:

```text
Real RX samples
      │
      ├── Step 1: Real correlation
      │       └── Rough chirp-start estimate
      │
      ├── Step 2: Complex dechirp + FFT
      │       └── Beat-frequency bin
      │               └── Coarse delay estimate
      │
      └── Step 3: Analytic complex correlation
              └── Sample-level delay refinement
```

The purpose of each stage is:

1. Real correlation finds the rough chirp start.
2. Complex dechirp and FFT estimate the beat frequency.
3. Analytic correlation refines the delay to the sample level.

---

## 3. Step 1: Real correlation for rough chirp-start detection

### Purpose

The first stage locates the approximate start of the chirp. This is only a coarse estimate. We do not need exact sample alignment here.

The current code uses a real sliding correlation in:

```cpp
float roughSearchForChirpStart(uint32_t &bestOffset)
```

For each candidate offset $d$, compute:

$$
C_{real}[d] = \sum_{n=0}^{N-1} x[d+n]\,r[n]
$$

where:

- $x[n]$ is the received real audio
- $r[n] = \cos(\phi[n])$ is the real-valued reference chirp

Normalize the correlation to reduce amplitude dependence:

$$
\rho[d] =
\frac{|C_{real}[d]|}
{\sqrt{E_x[d]E_r} + \epsilon}
$$

where

$$
E_x[d] = \sum_{n=0}^{N-1} x[d+n]^2
$$

and

$$
E_r = \sum_{n=0}^{N-1} r[n]^2
$$

The candidate with the largest value is selected:

$$
d_{rough} = \arg\max_d \rho[d]
$$

When the correlation exceeds a threshold, we declare:

$$
\text{chirpStartSample} = d_{rough}
$$

### Output of Step 1

The output of this stage is:

```text
roughChirpStartSample
```

This gives the receiver a rough alignment. The later analytic correlation removes the remaining delay error.

---

## 4. Step 2: Complex dechirp and FFT

### Purpose

Once the chirp start is approximately known, the received signal is mixed with the conjugate reference. This converts the chirp into a lower-frequency beat term.

We compute:

$$
y[n] = x[n]\,e^{-j\phi[n]}
$$

or equivalently:

$$
I[n] = x[n]\cos(\phi[n])
$$

$$
Q[n] = -x[n]\sin(\phi[n])
$$

This is exactly the complex dechirp operation used by the receiver:

```cpp
float re = xn * c;
float im = -xn * s;
```

where:

- $c = \cos(\phi[n])$
- $s = \sin(\phi[n])$

Thus the signal becomes:

$$
y[n] = I[n] + jQ[n]
$$

The FFT is then computed on this complex signal:

$$
Y[k] = \text{FFT}\{y[n]\}
$$

### Why complex dechirp?

If the received chirp is delayed by $\tau$, then after dechirp the result is approximately:

$$
y[n] \approx A e^{j2\pi f_b n / F_s}
$$

where the beat frequency is:

$$
f_b = K\tau
$$

So the delay estimate is:

$$
\tau = \frac{f_b}{K}
$$

and the sample delay is:

$$
d = \tau F_s = \frac{f_b F_s}{K}
$$

This is the fundamental relationship that the receiver uses to estimate delay from the FFT peak.

### 32,768-point FFT resolution

For an FFT of length $N = 32768$,

$$
\Delta f = \frac{F_s}{N}
= \frac{44100}{32768}
\approx 1.345825\ \text{Hz}
$$

The corresponding delay spacing is:

$$
\Delta d = \Delta f \cdot \frac{F_s}{K}
\approx 1.345825 \cdot 27.3067
\approx 36.75\ \text{samples}
$$

So the FFT gives a coarse delay estimate with resolution of about:

```text
1.346 Hz per FFT bin
36.75 samples per FFT bin
```

This is the coarse estimate used before the fine search.

### Important note

The chirp length is 32768 samples, but this does not mean we should simply do eight independent 4096-point FFTs and treat the result as a single 32768-point FFT.

A true 32768-point FFT has spacing:

$$
\Delta f = \frac{44100}{32768}
$$

while a 4096-point FFT has:

$$
\Delta f = \frac{44100}{4096}
\approx 10.7666\ \text{Hz}
$$

which corresponds to roughly:

$$
\Delta d \approx 294\ \text{samples}
$$

To obtain the desired fine delay estimate, the receiver must either:

1. use a true 32768-point FFT, or
2. implement a correctly combined Cooley–Tukey 32768-point FFT from smaller sub-FFTs.

### FFT peak search

After the FFT, find the strongest bin:

$$
k_{peak} = \arg\max_k |Y[k]|
$$

Convert the bin to a signed frequency:

$$
k_{signed} =
\begin{cases}
k_{peak}, & k_{peak} \leq N/2 \\
k_{peak} - N, & k_{peak} > N/2
\end{cases}
$$

Then compute the beat frequency:

$$
f_b = \frac{k_{signed}F_s}{N}
$$

and the coarse delay estimate:

$$
d_{coarse} = \frac{f_b F_s}{K}
$$

### Output of Step 2

The output of this stage is:

```text
coarseBeatFrequency
coarseDelaySamples
```

This is only a coarse estimate because the FFT bin spacing is finite.

---

## 5. Step 3: Analytic complex correlation for fine delay

### Purpose

The FFT gives a coarse estimate of the delay. The final step refines that estimate to the sample level.

Use the analytic reference chirp:

$$
s_{ref}[n] = e^{j\phi[n]}
$$

For each candidate delay $d$ near the FFT estimate, compute:

$$
C[d] = \sum_{n=0}^{N_w-1} x[d+n]\; s_{ref}^*[n]
$$

where $N_w$ is the window length used for the local search.

The real and imaginary components are:

$$
C_{Re}[d] = \sum_{n=0}^{N_w-1} x[d+n]\cos(\phi[n])
$$

$$
C_{Im}[d] = -\sum_{n=0}^{N_w-1} x[d+n]\sin(\phi[n])
$$

Then compute the correlation magnitude:

$$
|C[d]| = \sqrt{C_{Re}[d]^2 + C_{Im}[d]^2}
$$

The best sample delay is the one with maximum magnitude:

$$
d_{fine} = \arg\max_d |C[d]|
$$

### Fine search window

The FFT estimate becomes the center of a local search:

```cpp
int32_t coarseDelay = ...;
int32_t searchRadius = 64;

for (int32_t d = coarseDelay - searchRadius;
             d <= coarseDelay + searchRadius;
             d++)
{
    // analytic correlation magnitude at this delay
}
```

Since one FFT bin corresponds to roughly 36.75 samples, a radius of 32 to 64 samples is a reasonable starting point for the fine search.

### Optional fractional refinement

After finding the best integer delay $d_0$, the neighboring magnitudes can be used for interpolation:

$$
M_{-1} = |C[d_0-1]|
$$

$$
M_0 = |C[d_0]|
$$

$$
M_{+1} = |C[d_0+1]|
$$

A parabolic fit gives:

$$
\delta =
\frac{1}{2}
\frac{M_{-1} - M_{+1}}
{M_{-1} - 2M_0 + M_{+1}}
$$

Then the refined delay is:

$$
d_{refined} = d_0 + \delta
$$

This yields a fractional-sample estimate when the signal-to-noise ratio is favorable.

---

## 6. Full RX sequence

The full receiver algorithm is:

```text
1. Receive real audio samples.

2. Store samples in a rolling search buffer.

3. Perform real correlation against the real chirp reference.

4. If the correlation exceeds the threshold:
       mark rough chirp start

5. Take the aligned chirp segment.

6. Complex dechirp:
       I[n] = x[n] cos(φ[n])
       Q[n] = -x[n] sin(φ[n])

7. Perform a true 32768-point complex FFT.

8. Find the strongest FFT bin.

9. Convert the signed bin to beat frequency.

10. Convert beat frequency to coarse delay.

11. Search around the coarse delay using analytic complex correlation.

12. Select the delay with the largest correlation magnitude.

13. Optionally interpolate the peak for fractional delay.

14. Re-arm the receiver after the chirp / silence interval.
```

---

## 7. Final summary

The complete RX flow is:

$$
\boxed{
\text{Real correlation}
\rightarrow
\text{rough chirp start}
}
$$

$$
\boxed{
\text{Complex dechirp}
\rightarrow
\text{32768-point FFT}
\rightarrow
\text{coarse beat frequency}
\rightarrow
\text{coarse delay}
}
$$

$$
\boxed{
\text{Analytic complex correlation}
\rightarrow
\text{sample-level delay estimate}
}
$$

The FFT gives the coarse estimate, and the analytic complex correlation provides the final fine alignment.

This is consistent with the overall RX strategy:

- real correlation is good for coarse detection,
- FFT is good for coarse frequency and delay estimation,
- analytic correlation is good for final sample-level refinement.

---

## 8. Key design points

### Real correlation is not the final answer

It finds where the chirp is likely to begin, but it is not the final delay measurement.

### Complex dechirp is required for full phase information

The dechirped signal is complex because:

$$
x[n]e^{-j\phi[n]}
$$

contains both $I$ and $Q$ terms. That is why the FFT is complex.

### The FFT is coarse, not final

For a 32768-point FFT:

$$
\Delta d \approx 36.75\ \text{samples}
$$

This is useful for narrowing the fine search area.

### Final delay is determined by analytic correlation

The final estimate should be chosen from the maximum of the local complex correlation, not from the FFT bin alone.






## Question 1
when is real signal correlation is useful then ?



## Answer
Wideband matched filtering — exactly what Step 1 of your own pipeline does. The rough chirp-start correlation in RX_PROCESSING.md correlates the entire 32768-sample chirp waveform against the received signal, not a single beat tone. A full linear chirp sweeps 300→1500 Hz — a large time-bandwidth product — so its real-valued autocorrelation function has one sharp, essentially unambiguous peak at the true alignment and low sidelobes everywhere else.


## Question 2
This is not entirely true, because cross correlation revealed a sinc like function having a peak and zero and negative peaks left and right and eventually dies out. The reason the rough chirp-start correlation may work with real correlation is that the chance of hitting zero correlation next to the peak is really low.

## Answer
You're right, and I overstated it — "one sharp peak with low sidelobes" implied a clean envelope shape, but the raw real-valued correlation output genuinely does oscillate through zero and go negative near the peak, for the reason you're describing. Let me fix the explanation rather than just concede the point.

Where the sinc-like ripple comes from

Using cos(A)cos(B) = ½[cos(A−B) + cos(A+B)], the real correlation C_real[d] = Σ x[d+n]cos(φ[n]) splits into a slow "difference" term and a fast "sum" term. Over the full 32768-sample chirp, the sum term oscillates at roughly 2×(300–1500 Hz) — hundreds of cycles across the ~0.74 s window — and integrates close to zero regardless of d, so it mostly washes out. What dominates is the difference term: for a given d, its phase rate is ≈ K·d·t, which is a beat frequency proportional to d. Summing cos(K·d·t) over the observation window produces exactly the sinc shape you're describing — a mainlobe at d=0, then a null, then alternating negative and positive sidelobes, decaying — the classic matched-filter response for a linear-FM pulse.

Why the peak is still findable — a resolution-cell argument, not luck

The sinc's mainlobe width is set by the chirp's time-bandwidth-limited resolution, ≈ 1/B in time. With B = 1200 Hz:


Δd≈B/Fs​​=1200/44100​≈36.75 samples

$$
\Delta d \approx \frac{F_s}{B} = \frac{44100}{1200} \approx 36.75\ \text{samples}
$$

That's the same 36.75-samples figure from Step 2's FFT-bin spacing — not a coincidence, it's the same underlying time-bandwidth-product resolution showing up in two different domains (correlation-domain mainlobe width vs. dechirped-FFT bin spacing).

So the first null isn't "right next to" the true peak in a fine-grained, sample-by-sample sense — it's roughly half a mainlobe width away, around 18+ samples out. Within that ±18-sample neighborhood you're inside the mainlobe, where C_real[d] is large and smoothly varying, nowhere near a zero-crossing. The genuinely fragile case would be if the sampling grid's nearest candidate to the true continuous-time peak happened to land near one of those nulls — but since the mainlobe spans ~37 samples and you're sampling at integer offsets, you'd need to be unluckily far (order of a mainlobe width) from the true alignment for that to matter, not merely "adjacent."

So the more accurate statement is:

Real correlation works for Step 1 not because the function is smooth (it isn't — it's a genuine sinc with sign-flipping sidelobes, as you said), but because (a) the mainlobe is ~37 samples wide, giving comfortable margin before the first null, and (b) RX_PROCESSING.md takes |C_real[d]|, so even the rectified negative sidelobes just become smaller positive bumps that stay well below the mainlobe's magnitude (a standard sinc's first sidelobe is roughly −13 dB down). The "chance of hitting a null" is low mainly because the null spacing is large relative to the search granularity — it's closer to a deterministic resolution-cell argument than a probabilistic one, though you could still frame it probabilistically if you imagine random sub-sample alignment jitter within that margin.


 
## Question 3

> On Teensy's performance, we cannot do the correlation at every sample — realistically every 64 samples. This means the search could easily miss the mainlobe of the real-correlation peak. Even if we miss the mainlobe, if we hit a sidelobe strong enough, could we still use that to tell the chirp is there?

## Answer

Yes — and it's worth putting numbers on it rather than just agreeing qualitatively.

### 1. Confirm the mainlobe-vs-stride mismatch

For an LFM matched filter, the compressed correlation peak has (approximately, from the stationary-phase/sinc approximation) a peak-to-first-null distance of about `1/B` in time, which in samples is:

$$
\Delta d \approx \frac{F_s}{B} = \frac{44100}{1200} \approx 36.75\ \text{samples}
$$

(Same figure as `RX_PROCESSING.md`'s FFT bin-spacing — the same time-bandwidth resolution limit showing up in two equivalent forms.)

With a **64-sample stride**, the worst-case gap between the true peak and the nearest tested sample is up to 32 samples — which is *inside* the sidelobe region, past the first null. So it's entirely possible, not just a theoretical corner case, that every tested point lands outside the mainlobe.

### 2. Is a strong sidelobe hit still useful? Yes — the processing-gain margin is large

The reason this still works is the chirp's time-bandwidth product, which sets the correlation processing gain:

$$
BT = 1200 \times 0.743 \approx 891 \quad\Rightarrow\quad 10\log_{10}(BT) \approx 29.5\ \text{dB}
$$

Sidelobe levels for this kind of matched-filter response follow roughly the sinc envelope: 1st sidelobe ≈ −13 dB, 2nd ≈ −18 dB, 3rd ≈ −21 dB relative to the mainlobe peak. Even hitting the 2nd or 3rd sidelobe, the net SNR improvement over the raw noise floor is still:

$$
29.5 - 18 \approx 11.5\ \text{dB}, \qquad 29.5 - 21 \approx 8.5\ \text{dB}
$$

That's generally plenty to clear a detection threshold cleanly, assuming input SNR isn't already marginal before correlation. Using a sidelobe hit as "a chirp is present, roughly around here" is a sound engineering call, not a hack.

### 3. Caution: don't confuse "detected" with "localized"

A sidelobe hit tells you a chirp is present and *roughly* where — but "roughly" now means ±1–3 mainlobe-widths (up to ~100 samples), not the ±37-sample precision a mainlobe hit would give. That matters because Step 1's actual job isn't final localization — it's picking *where to grab the 32768-sample segment* that Step 2 will dechirp and FFT. If that segment is offset by a lobe-width or two from the true chirp start, part of the actual chirp could fall outside the window, degrading Step 2's FFT peak (less energy, possibly spread across bins) rather than just shifting it.

### 4. Practical fix, consistent with the existing design

- Keep the coarse stride (64 samples) — the compute savings are real, and the detection-margin math above says it's fine for the "is it there" question.
- When a threshold-crossing hit occurs (main or sidelobe), grab a **slightly larger capture buffer** than exactly `N` samples — pad by roughly one stride plus a couple of mainlobe-widths on each side (e.g., an extra ~150–200 samples of margin) — so the true chirp is still fully contained even if the trigger point was a sidelobe some distance from the real start.
- Let Step 2's FFT do the actual localization from there, exactly as designed — that stage is built to resolve delay precisely and cheaply (one FFT vs. a dense per-sample search), so Step 1 only needs to be "close enough to capture the whole chirp," not "close enough to already know the delay."

### Bottom line

Agree with the core idea. The fix is less about improving Step 1's precision and more about making sure the buffer handed to Step 2 has enough slack to absorb the extra uncertainty a sidelobe-triggered detection introduces.





## Question 4
what is the meaning of delay resolution in radar application when a chirp is sent and received by an aircraft. If the delay resolution is high, it can detect small object?

## Answer 

What delay/range resolution actually means -- if two robots are chirping close each other, and delay resolution is low, we cannot detect if there are two or one robot. 

In an FMCW/pulse-compression radar (aircraft-mounted or ground-based), the chirp goes out, reflects off a target, and comes back delayed by τ = 2R/c (round trip, so factor of 2), where R is range and c is the speed of light. Delay resolution Δτ (or equivalently Δd in your derivation) tells you the smallest difference in delay the radar can distinguish between two returns — which translates directly to range resolution:
 
	​


(same 1/B relationship as your chirp math, just with c instead of your audio system's propagation being instantaneous over the short acoustic path).

Concretely: if two aircraft, or an aircraft and terrain, or two parts of the same target, are separated in range by less than ΔR, their two chirp echoes overlap in delay closely enough that the radar's matched filter can't tell them apart — they blur into one blob on the range profile. If they're separated by more than ΔR, the radar sees two distinct peaks. So delay resolution is fundamentally about distinguishing/separating two closely-spaced things in range, or equivalently, how precisely you can pin down where along the range axis a single target sits.

Detecting a small object is a different problem: sensitivity, not resolution

Whether the radar can detect a small object at all is governed by the radar equation — received power scales with the target's radar cross-section (RCS), transmit power, antenna gain, range (to the 4th power), and the noise floor. A small stealthy object (small RCS) might be completely undetectable even with extremely fine delay resolution, simply because too little energy comes back to clear the noise floor — that's an SNR/detection-threshold problem, not a resolution problem.

Conversely, you could have a large, easily-detected object (strong echo, easy to see above the noise) but coarse delay resolution — meaning you know something is out there and roughly how far, but you can't tell if it's one large object or two smaller ones close together, or exactly how far within that resolution cell.

So the honest answer to "if delay resolution is high, can it detect small objects?"

Not directly — high resolution helps in a related but distinct way: it improves processing gain and clutter separation. A finer range bin means the target's return is concentrated in a smaller resolution cell, competing against less clutter/noise energy within that cell (since noise power in a resolution cell scales with the cell's bandwidth/duration), which can improve effective SNR and thus indirectly help detect smaller/weaker targets that would otherwise be masked by clutter or noise spread across a wider cell. But this is a secondary, clutter-rejection effect — it's not the same mechanism as "resolution directly reveals small objects." The primary lever for detecting small objects is RCS, transmit power, integration time, and antenna gain (all radar-equation terms), not delay resolution per se.

One-line summary: delay resolution tells you how close together two targets can be before the radar can no longer tell them apart (or how precisely it can pin down one target's range); it doesn't by itself determine whether a faint/small target is detectable at all — that's a separate sensitivity question governed by the radar equation, though finer resolution can help indirectly by shrinking the noise/clutter competing with the target in each range cell.


## Quseiton

AI said: 
When a threshold-crossing hit occurs (main or sidelobe), grab a slightly larger capture buffer than exactly N samples — pad by roughly one stride plus a couple of mainlobe-widths on each side (e.g., an extra ~150–200 samples of margin) — so the true chirp is still fully contained even if the trigger point was a sidelobe some distance from the real start.

I ask : 
this does not solve the issue of false detection though. even if some noise or off-axis signal passed the first rough correaltion threshold, the second mix-fft would not show a desired shape of peak. Right?

## Answer
You're right, and it's an important distinction — buffer padding and false-alarm rejection solve two completely different problems, and I conflated them by putting the padding fix right after the false-detection concern.

Padding only fixes alignment, not validity. If Step 1 genuinely triggers on the real chirp (whether via mainlobe or sidelobe), padding just makes sure the whole chirp lands inside the buffer Step 2 works on. It says nothing about whether the trigger was a real chirp in the first place.

Why Step 2 does reject most false triggers, though — and here's the mechanism:

The dechirp-then-FFT operation is a coherent match to one specific signal shape: a linear sweep from f0 to f1 over exactly T seconds. That coherence is what concentrates energy into a single narrow FFT bin. Anything that isn't that shape doesn't get that concentration:

Random noise, after mixing with the swept reference and FFT'ing, stays noise-like — energy spread roughly evenly across bins, no dominant peak. You get a slightly elevated, incoherent floor, not a sharp spike.
An off-axis stationary tone (interference at some fixed frequency), after mixing with a swept reference, produces a signal whose instantaneous beat frequency itself sweeps over time (since the reference's phase is changing at rate K but the tone's isn't tracking it) — so its energy smears across many FFT bins rather than collapsing into one. No peak.
An unrelated chirp with a different rate/bandwidth, similarly, dechirps into a residual chirp (not a constant tone), which again spreads across bins instead of concentrating.

So a random correlation spike that happened to clear Step 1's rough threshold (whether from noise, an unrelated tone, or an off-axis signal) is very unlikely to also produce a strong, narrow, single-bin-dominant peak in Step 2 — because Step 2's processing gain (~29.5 dB from your BT product) only rewards signals that actually match the chirp's specific time-frequency law. That's precisely why cascading these two stages helps: each stage applies coherent gain against a slightly different aspect of the same waveform, so a false alarm has to survive two largely independent tests, not one.

The gap in the current design is that this isn't yet an explicit second gate. Right now Step 2 just does argmax and hands the peak to Step 3 — it doesn't check how peaky that peak actually is before trusting it. To actually use Step 2 as a false-alarm filter (which your reasoning suggests it should be), you'd want an explicit criterion like:

Peak-to-floor ratio: |Y[k_peak]| divided by the median or mean magnitude of the other bins — noise/off-axis signals give a low ratio, a true chirp gives a high one.
Peak-to-total-energy fraction: what fraction of total FFT energy sits in the peak bin (plus its immediate neighbors) — a genuine dechirp result concentrates a large fraction there; spread signals don't.

Require that ratio to clear a second threshold before proceeding to Step 3, and you get something close to a two-stage CFAR-style detector: Step 1's rough correlation as a cheap gate, Step 2's spectral concentration as a corroborating, largely independent gate, and Step 3's fine correlation magnitude as a third confirmation. A stray sidelobe hit on pure noise would very likely fail the Step 2 peakiness check and get discarded before ever reaching the fine search — which is the actual mechanism protecting you, not the buffer padding.

