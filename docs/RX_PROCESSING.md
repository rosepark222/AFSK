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

 
# Step 2 (Dechirp + FFT) vs. GCC-PHAT — Q&A

## Question 1

In this rx plan, I have a question on the step 2.

```
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

The step 2 looks for the delay estimate. Here it takes advantage of LFM of the chirp. Since chirp has different freq component each sub freq shows linear phase between rx signal and the reference. Typically GCC-PHAT is used for two mics and one mic is the reference. We can assume the reference signal is from a virtual mic and rx signal from another mic and then use GCC-PHAT to get the delay between the two. The difference is that GCC-PHAT does not take advantage of the signal being a chirp cause it works for any signal as long as they are wideband enough. Can you give your thought and compare the step 2 vs GCC-PHAT -- pros and cons.

## Answer

Good question, and it gets at something real: both methods ultimately rely on the same physical fact (a time delay is a linear phase ramp across frequency), but they extract and weight that phase information very differently.

### The common root

For any two signals related by a pure delay τ, their cross-spectrum has the form:

$$
X_1(f)X_2^*(f) \propto |S(f)|^2 e^{-j2\pi f\tau}
$$

The phase slope across frequency *is* the delay. Every delay-estimation method — dechirp+FFT, GCC-PHAT, ordinary cross-correlation — is extracting that slope in some form. Where they diverge is what they assume about $S(f)$ and how they weight it.

### What Step 2 actually exploits

Step 2 doesn't use the phase-slope argument directly. It uses a property specific to LFM: because instantaneous frequency is a **linear function of time**, a delayed copy of the chirp has an instantaneous frequency offset from the reference that is *constant* in time — $f_b = K\tau$. Mixing collapses the whole delay-estimation problem into estimating one number (a beat tone) rather than a phase-vs-frequency slope. This is exactly radar "stretch processing" / "deramp" — trading a general correlation problem for a single-frequency estimation problem, because you know the waveform's exact time-frequency law.

That's the crucial difference: **Step 2 needs to know the transmitted signal is a chirp with these exact parameters.** GCC-PHAT needs no such knowledge.

### What GCC-PHAT does instead

 
$$ 
R_{\text{PHAT}}(\tau) = \text{IFFT} \frac{X_1(f)X_2^*(f)}{|X_1(f)X_2^*(f)|} 
$$

$$ 
R_{\text{PHAT}}(\tau) 
$$

$$ 
  \text{IFFT} 
$$

$$ 
\frac {X_1(f)X_2^{*}(f)} {|X_1(f)X_2^{*}(f)|} 
$$

$$
\frac{X_1(f)X_2^*(f)}{|X_1(f)X_2^*(f)|}
$$

$$ 
\frac {1} {|X_1(f)X_2^*(f)|} 
$$

$$ 
\frac {X_1(f)X_2^*(f)} {1} 
$$

$$
\frac{X_1(f)X_2^*(f)}{\left|X_1(f)X_2^*(f)\right|}
$$

$$
\frac{X_1(f)X_2^*(f)}{\left\lvert X_1(f)X_2^*(f) \right\rvert}
$$

$$
\frac{X_{1}(f)X_{2}^{*}(f)}{\left|X_{1}(f)X_{2}^{*}(f)\right|}
$$

$$ 
\frac{X}{Y} 
$$

$$ 
 X_1(f) 
$$

$$ 
 X_2^*(f) 
$$

aa 

$$ 
 |X_1(f)X_2^*(f)| 
$$

bb 

$$ 
 X_2^*(f) 
$$

It divides out the magnitude of the cross-spectrum, keeping only phase, then inverse-transforms. This works for *any* sufficiently broadband signal because it never assumes anything about $S(f)$'s shape — only that there's usable phase coherence across the band. The whitening is specifically there to fix a problem GCC-PHAT was designed for: **reverberation**. Plain cross-correlation's peak shape is smeared by the source's own spectral coloring and by multipath (each echo path imposes its own magnitude/phase distortion); PHAT flattens magnitude so the peak sharpness is governed only by bandwidth, not by channel coloring.

### Pros / cons

**Step 2 (dechirp + FFT, chirp-matched)**
- ✅ Full coherent matched-filter gain (~29.5 dB here) — near-optimal in white noise, since it weights each frequency by its actual signal energy rather than artificially flattening it.
- ✅ Delay → single scalar (beat frequency) rather than a full correlation function — conceptually and computationally convenient for a staged, resource-constrained pipeline (cheap Step 1 gate → small dechirp+FFT only when triggered).
- ✅ Because you know the beat-frequency band in advance, you can lowpass + decimate the dechirped signal before the FFT — a genuine compute saving on Teensy that a generic method can't claim, since it doesn't know in advance where the "answer" will live spectrally.
- ❌ Waveform-specific — the whole trick evaporates if the transmit signal isn't LFM.
- ❌ No whitening, so channel coloring/mild multipath can bias the FFT peak or spread its energy, exactly like it would smear a plain (non-PHAT) cross-correlation.
- ❌ Still fundamentally limited to ~37-sample resolution by the BT product — needs Step 3 regardless.

**GCC-PHAT**
- ✅ Signal-agnostic — same mechanism works whether the source is a chirp, speech, or a noise burst. No need to know the transmitted waveform, only need two channels of it (or, per your framing, one real + one synthetic reference channel).
- ✅ Explicitly designed to sharpen the peak under reverberation/channel coloring, which a raw matched filter doesn't correct for.
- ✅ One FFT-based cross-correlation covers *all* candidate delays at once — no separate coarse-then-fine staging needed if you can afford an FFT sized to your full delay-uncertainty window.
- ❌ Whitening weights every bin equally regardless of actual SNR — bins where the received signal is mostly noise get boosted just as much as bins with real signal. This is a well-known weakness: PHAT is not SNR-optimal, and can perform worse than a plain matched filter at low SNR precisely because it throws away the magnitude information that the matched filter would have used to downweight noisy bins.
- ❌ Ignoring the known chirp structure forfeits "free" coherent gain that Step 2 gets for using the exact transmit law — you're solving a harder, more general problem than you need to.

### One practical takeaway

Since you *do* know the exact transmit waveform here, dechirp+FFT is the right tool — you get matched-filter-optimal gain in white noise essentially for free, plus the LFM-specific decimation trick. GCC-PHAT's real advantage — robustness to spectral coloring from reverberation/multipath — is worth borrowing selectively: if the acoustic path is reverberant, whitening $Y[k]$ (the dechirped spectrum) by its own magnitude before peak-picking in Step 2 would import some of PHAT's multipath robustness without giving up the LFM-specific computational shortcuts. That's a smaller, targeted change rather than replacing Step 2 wholesale with a generic GCC-PHAT.

---

## Question 2

Any research paper exploited advantage of step 2 and GCC-PHAT -- combining the two? Answer in one paragraph.

## Answer

A directly relevant paper is "Dynamic Adjustment of Weighted GCC-PHAT for Position Estimation in an Ultrasonic Local Positioning System" (Pérez-Solano, Ezpeleta, et al., *Sensors*, 2021, https://doi.org/10.3390/s21217051, also at PMC8587089). It uses matched filtering with encoded (chirp-like) emissions as the first detection stage, then applies GCC-PHAT-β with a dynamically adjustable weighting factor β to sharpen the correlation peak — this β acts as a tunable knob between plain cross-correlation (β = 0, matched-filter-optimal, SNR-weighted) and full PHAT whitening (β = 1, flat weighting, robust to multipath/reverberation), letting the system get chirp-matched processing gain at high SNR while blending in phase-transform-style robustness as multipath/reverberation degrades performance. The paper reports that dynamically adapting β cut positioning errors from 20 cm down to 2 cm in 80% of measurements — essentially the hybrid asked about: combining the coherent gain of a chirp-matched correlator (the Step 2 mechanism) with a partial PHAT-style spectral whitening rather than choosing one approach outright.

---

## Question 3

1200 Hz is wide enough to work in GCC-PHAT? Answer in < 3 sentences.

## Answer

Yes — 1200 Hz gives GCC-PHAT the same theoretical resolution as the chirp (Δd ≈ Fs/B ≈ 36.75 samples), since PHAT's resolution depends only on bandwidth, not waveform shape. It's actually a *large* fractional bandwidth (1200 Hz spread around a ~900 Hz center, i.e. ~133%), which is good for PHAT — the bigger risk is that so little energy sits above ~1500 Hz that other real-world noise/interference in that band could dominate the whitened spectrum. So bandwidth-wise it's fine; the practical concern is low absolute SNR per bin after whitening, not insufficient bandwidth.





# Step 2 (Dechirp + FFT) vs. Quadrature Demodulation in FSK — Q&A

## Question

Compare step 2 vs quadrature demodulation in FSK. The diff is in FSK, after the mix, DC value remains, but in step 2, a tone remains and simple LPF will not recover this — we need FFT to detect the tone.

## Answer

That's exactly the right distinction, and it comes down to *what the local oscillator (reference) is matched to* in each case.

### FSK quadrature demod: the LO frequency is fixed and matches the hypothesis

In the FSK receiver, you mix the received tone (at $f_1$ or $f_0$) against a local reference at a **fixed** frequency — the same $f_1$ or $f_0$ you're testing for. When the received tone actually matches that LO frequency, the mixing product is:

$$
I_{raw} = \frac{I}{2} + \frac{I}{2}\cos(2\omega_c t) - \frac{Q}{2}\sin(2\omega_c t)
$$

The wanted term $I/2$ is **DC** — a constant, not oscillating at all — because the received frequency and the LO frequency are identical, so their difference frequency is exactly 0 Hz. The unwanted term sits way up at $2\omega_c$. A simple LPF cleanly separates the two: everything above the cutoff is discarded, and what's left is a static number you can just read off (then combine as $\sqrt{I^2+Q^2}$ for the energy). LPF works here because "the answer" *is* a DC level — there is no frequency left to identify, only a magnitude.

### Step 2: the LO itself sweeps, so the residual is a tone, not DC

In dechirp, the reference isn't a fixed frequency — it's a chirp with the *same sweep rate* $K$ as the transmitted chirp. If the received chirp is delayed by $\tau$, then at any instant $t$, the received instantaneous frequency is $f_0 + K(t-\tau)$, while the reference's instantaneous frequency is $f_0 + Kt$. Their difference is:

$$
f_b = K\tau
$$

a **constant offset**, yes — but not zero unless $\tau = 0$. So the mixing product isn't DC; it's a complex exponential rotating at $f_b$ Hz:

$$
y[n] = I[n] + jQ[n] \approx Ae^{j2\pi f_b n/F_s}
$$

An LPF can still remove the unwanted $2\omega_c$ image the same way it does in FSK, but that's a separate job from the one you actually need done here. What survives after the LPF is *still oscillating* at $f_b$ — the LPF doesn't collapse it to a number, because $f_b$ is generally nonzero and unknown (it's the very thing you're trying to measure). Taking $\sqrt{I^2+Q^2}$ the way FSK does would just give you a roughly constant *amplitude* $A$, discarding exactly the piece of information ($f_b$) that encodes the delay. So you need a frequency-selective step — the FFT — to read out *which* frequency the residual tone sits at.

### The core structural difference

| | FSK quadrature demod | Step 2 dechirp |
|---|---|---|
| LO / reference | Fixed frequency, one per hypothesis ($f_1$, $f_0$) | Swept (chirp), same rate $K$ as TX |
| What the unknown is | Which of two known frequencies was sent | An unknown delay $\tau$, encoded as an unknown beat frequency |
| Residual after mixing (ignoring $2\omega_c$ image) | DC ($I/2$, $Q/2$) | AC tone at $f_b = K\tau$ |
| Why LPF suffices / doesn't | The wanted information *is* a static level — nothing left to resolve in frequency | The wanted information *is* a frequency — LPF only rejects the image, it can't identify $f_b$ |
| How the wanted quantity is extracted | Magnitude: $\sqrt{I^2+Q^2}$ (removes unknown phase $\phi$) | Spectrum: FFT + $\arg\max_k \lvert Y[k]\rvert$ (finds unknown frequency $f_b$) |
| What phase/frequency ambiguity is being defeated | Unknown/drifting phase $\phi$ at a known frequency | Unknown frequency $f_b$ itself, at (implicitly) unknown phase too |

### Why this makes sense conceptually

Both are instances of the same general move — correlate against a hypothesis, then look at what's left — but they push the unknown into different domains on purpose, matched to what's actually unknown at the receiver:

- In FSK, *frequency* is the known, discrete thing (only two candidates, $f_1$/$f_0$), and *phase* is the nuisance parameter. Energy detection ($I^2+Q^2$) is built specifically to erase phase while preserving "was this frequency present."
- In dechirp ranging, *delay* is the continuous unknown, and it has been deliberately re-encoded as a *frequency* by using a swept (not fixed) reference. Once delay becomes frequency, the natural tool to resolve it is the FFT — the same way FSK's natural tool to resolve amplitude-at-known-frequency was an LPF-then-magnitude.

So the FSK case ends in DC because the reference was chosen to exactly match the thing being tested (a fixed candidate frequency). Step 2's reference matches the transmit chirp's *shape* (its sweep rate) but deliberately does *not* match its *timing* — that timing mismatch is precisely what survives as a nonzero tone, and precisely what the FFT is there to measure.



