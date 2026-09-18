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