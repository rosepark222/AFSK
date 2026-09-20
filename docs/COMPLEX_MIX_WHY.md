
# Why We Mix With a Complex Exponential: From a Toy Example to Chirp Delay Detection

This note builds intuition for one specific step in the RX chirp pipeline: **why the dechirp mixer multiplies by `e^{-jφ[n]}` instead of `cos(φ[n])`**. It starts from a one-line algebra example, then connects that example to the actual mix-and-FFT stage used to estimate the time delay `τ`, and finally ties it to the broader idea of analytic-signal correlation.

Companion reading: [`RX_PROCESSING.md`](./RX_PROCESSING.md) (the 3-stage RX pipeline) and the analytic-signal correlation writeup referenced alongside it.

---

## 1. The toy example: a delay is just a phase shift

Model a delayed tone as a phase-shifted copy of a reference:

$$
x = \cos(\theta - \Delta)
$$

where `θ` is the reference phase and `Δ` is the phase offset caused by the delay. For a linear chirp, `Δ` grows over time at a rate proportional to the delay `τ` — that rate is exactly the **beat frequency** `f_b` from the RX pipeline. So:

$$
\text{sign of } \Delta\text{'s growth rate} = \text{sign of } \tau
$$

### Case 1 — real mixing: `x · cos(θ)`

$$
\cos(\theta-\Delta)\cos(\theta) = \tfrac{1}{2}\cos(\Delta) + \tfrac{1}{2}\cos(2\theta-\Delta)
$$

The `2θ` term is a fast "sum" component (near double frequency) — it gets filtered out (in the RX pipeline, it's the image that a low-pass or the FFT bin selection discards). What survives is the beat term:

$$
\tfrac{1}{2}\cos(\Delta)
$$

`cos` is an **even** function: `cos(Δ) = cos(-Δ)`. A `+Δ` and a `-Δ` produce the *exact same number*. The sign of the delay is destroyed by the multiply itself — no amount of downstream processing can recover it, because the information is simply gone.

### Case 2 — complex mixing: `x · e^{-jθ}`

$$
\cos(\theta-\Delta)e^{-j\theta} = \tfrac{1}{2}e^{-j\Delta} + \tfrac{1}{2}e^{-j(2\theta-\Delta)}
$$

Same product-to-sum mechanics as Case 1 (verify with Euler's formula), just kept complex. Drop the `2θ` sum term again. The beat term is now:

$$
\tfrac{1}{2}e^{-j\Delta}
$$

This time `e^{-jΔ} ≠ e^{+jΔ}` unless `Δ = 0`. Flipping the sign of `τ` (hence `Δ`) rotates this **phasor** the opposite way around the unit circle — a genuinely different complex number, not the same value in disguise.

> **One-line takeaway:** `cos(θ)` is even in the reference phase, so it can't tell `+Δ` from `−Δ` — the sign is thrown away at the mixer, before anything else runs. `e^{-jθ}` is *not* even, so it preserves the sign of `Δ` all the way through. The result of the complex mix, `½e^{-jΔ}`, is a **phasor**: a single complex number carrying both magnitude and (signed) phase, instead of one real number that has already collapsed two possibilities into one.

---

## 2. Mapping the toy example onto the real mix-and-FFT step

In the RX pipeline, the reference phase is the chirp phase:

$$
\phi[n] = 2\pi\left(f_0 t[n] + \tfrac{1}{2}K t[n]^2\right), \qquad t[n]=\frac{n}{F_s}
$$

and the complex dechirp is:

$$
y[n] = x[n]\,e^{-j\phi[n]} = I[n] + jQ[n], \qquad
I[n]=x[n]\cos(\phi[n]),\;\; Q[n]=-x[n]\sin(\phi[n])
$$

This is exactly Case 2 above, applied with `θ = φ[n]` at every sample. If the received chirp is delayed by `τ`, the local phase offset `Δ` at sample `n` grows linearly, producing a nearly constant beat frequency:

$$
y[n] \approx A\,e^{j2\pi f_b n/F_s}, \qquad f_b = K\tau
$$

The FFT of `y[n]` turns that phasor's rotation rate into a bin location — `f_b > 0` lands on the positive-frequency side, `f_b < 0` on the negative-frequency side (`k_{signed} = k_{peak} - N` for `k_{peak} > N/2`, per `RX_PROCESSING.md` §4). Because the mix preserved the sign of `Δ`, the FFT bin preserves the sign of `τ`.

Had the pipeline instead used real dechirp (`x[n]cos(φ[n])`), the result would be Case 1: a real signal with Hermitian-symmetric spectrum, `Y[-k] = Y[k]^*`. A single real tone at `f_b` always produces **two** equal-magnitude peaks, at `+f_b` and `-f_b` — regardless of which one is physically true. The spectrum cannot encode which is real, because both are always present. Complex mixing avoids this because `y[n]` is no longer real, so Hermitian symmetry no longer applies: `+f_b` and `-f_b` are genuinely different frequencies, giving one peak at the correct signed location.


## 3. concrete example ( tau = 100 sample )


From `RX_PROCESSING.md`: `K ≈ 1614.844 Hz/s`, `F_s = 44100 Hz`, `N = 32768`, `Δf = F_s/N ≈ 1.346 Hz/bin`.

Suppose the true delay is `τ =  0.00226 s` (chirp arrived 100 samples **later** than the rough alignment assumed):
$$
f_b = K\tau = 1614.844 \times (0.00226) \approx 3.66\ \text{Hz}
$$

- **Real dechirp:** `y[n] = x[n]\cos(\phi[n])` contains `cos(2π(3.66)n/F_s)`. Since `cos(-θ)=cos(θ)`, this is numerically identical to what a `-3.66 Hz` beat would produce. The FFT peak lands at the *same* bin either way. The receiver is left with two equally consistent hypotheses — `τ = -2.26 ms` or `τ = +2.26 ms` — with no way to choose between them. Guessing wrong means a 2*2.26 ms (200-sample) error, not because two things happened, but because the real-valued mix threw away the one bit of information (sign) that would have told it which single thing happened.
- **Complex dechirp:** `-3.66 Hz` and `+3.66 Hz` land at different bins (`k ≈ N-3` vs. `k ≈ +3`), so the correct, single, actual delay is recoverable directly from the FFT peak location.

<img width="1508" height="727" alt="image" src="https://github.com/rosepark222/AFSK/blob/main/docs/real_correl_problem.png" />

|  Aliased Bin | Physical Frequency ($f_b$ in Hz) | Time Delay ($\tau$ in ms) | Equivalent Delay (Samples) |
| :--- | :--- | :--- | :--- |
| 32758 | -13.458 Hz | -8.333 ms | -367.5 samples |
| 32759 | -12.112 Hz | -7.500 ms | -330.8 samples |
| 32760 | -10.767 Hz | -6.667 ms | -294.0 samples |
| 32761 | -9.421 Hz | -5.833 ms | -257.2 samples |
| 32762 | -8.075 Hz | -5.000 ms | -220.5 samples |
| 32763 | -6.729 Hz | -4.167 ms | -183.8 samples |
| 32764 | -5.383 Hz | -3.333 ms | -147.0 samples |
| 32765 | -4.037 Hz | -2.500 ms | -110.2 samples |
| 32766 | -2.692 Hz | -1.667 ms | -73.5 samples |
| 32767 | -1.346 Hz | -0.833 ms | -36.8 samples |
| 0 | 0.000 Hz | 0.000 ms | 0.0 samples |
| 1 | 1.346 Hz | 0.833 ms | 36.8 samples |
| 2 | 2.692 Hz | 1.667 ms | 73.5 samples |
| 2.721 (Your Peak)** | **3.662 Hz** | **2.268 ms** | **100.0 samples** |
| 3 | 4.037 Hz | 2.500 ms | 110.2 samples |
| 4 | 5.383 Hz | 3.333 ms | 147.0 samples |
| 5 | 6.729 Hz | 4.167 ms | 183.8 samples |
| 6 | 8.075 Hz | 5.000 ms | 220.5 samples |
| 7 | 9.421 Hz | 5.833 ms | 257.2 samples |
| 8 | 10.767 Hz | 6.667 ms | 294.0 samples |
| 9 | 12.112 Hz | 7.500 ms | 330.8 samples |
| 10 | 13.458 Hz | 8.333 ms | 367.5 samples |
---

### another example (using the pipeline's own numbers)

From `RX_PROCESSING.md`: `K ≈ 1614.844 Hz/s`, `F_s = 44100 Hz`, `N = 32768`, `Δf = F_s/N ≈ 1.346 Hz/bin`.

Suppose the true delay is `τ = -0.01 s` (chirp arrived 441 samples **earlier** than the rough alignment assumed):
$$
f_b = K\tau = 1614.844 \times (-0.01) \approx -16.15\ \text{Hz}
$$

- **Real dechirp:** `y[n] = x[n]\cos(\phi[n])` contains `cos(2π(-16.15)n/F_s)`. Since `cos(-θ)=cos(θ)`, this is numerically identical to what a `+16.15 Hz` beat would produce. The FFT peak lands at the *same* bin either way. The receiver is left with two equally consistent hypotheses — `τ = -10 ms` or `τ = +10 ms` — with no way to choose between them. Guessing wrong means a 20 ms (882-sample) error, not because two things happened, but because the real-valued mix threw away the one bit of information (sign) that would have told it which single thing happened.
- **Complex dechirp:** `-16.15 Hz` and `+16.15 Hz` land at different bins (`k ≈ N-12` vs. `k ≈ +12`), so the correct, single, actual delay is recoverable directly from the FFT peak location.

---

## 4. The general picture: mixing is the first step of analytic-signal correlation

The complex mix `x[n]·e^{-jφ[n]}` is not a special trick unique to dechirping — it's the same operation that turns a real signal into an **analytic signal representation** for correlation purposes, as described in the companion analytic-signal correlation note:

- A chirp can be viewed as a sum of many instantaneous single-tone components, each behaving locally like the toy example in §1.
- For a given frequency `f` and delay `τ`, that component acquires a phase shift `Δφ(f) = 2πfτ`.
- Real correlation collapses each component's contribution into a single real number, and — just like Case 1 above — a real number can't distinguish two phase relationships that happen to have the same cosine.
- Complex mixing keeps each component as a phasor (magnitude *and* phase), so components can be added **as vectors**. At the correct delay, all component-phasors align and add constructively; at the wrong delay, different frequencies have rotated by different amounts (`Δφ(f)` is frequency-dependent) and their phasors partially cancel.

This is exactly why `RX_PROCESSING.md` Step 2 (complex dechirp → FFT) and Step 3 (analytic complex correlation) both use `cos(φ[n])`/`sin(φ[n])` as an `(I, Q)` pair rather than `cos(φ[n])` alone: the pair is what constructs the phasor at each point, and the phasor is what makes the sign of `τ` (and, more generally, the full phase relationship between `rx` and the reference) observable at all.

> **Benefit of the phasor, in one sentence:** a real correlation value tells you *how much* two signals agree, but a phasor tells you *how much and in which rotational sense* — and for a signal built from many frequency components each rotating at a different delay-dependent rate, that rotational information is exactly what lets the components reinforce at the true delay and cancel everywhere else, instead of folding two different physical situations into one indistinguishable number.





# Why Mixing With a Real Reference Erases Phase Information (Orthogonality View)

Companion reading: [`PHASOR_MIX_INTUITION.md`](./PHASOR_MIX_INTUITION.md) and [`RX_PROCESSING.md`](./RX_PROCESSING.md).

## Question

> In general, a modulated signal is received, and correlating it against a real reference signal loses the phase information. Thus, an analytic signal should be used — the dechirp must mix `rx` with a **complex** reference chirp before the FFT. Even though the FFT itself uses complex arithmetic, mixing with a **real** reference chirp has already removed the phase information beforehand. Imagine `rx` has cos and sin components (as in an FFT decomposition); mixing with `cos` alone just erases the sin component due to orthogonality. Do you agree?

## Answer

Yes — and the orthogonality framing is actually a more precise way to state this than an "even function" argument alone.

### The precise version of the claim

At any instant, write the received sinusoid relative to the reference phase `θ`, with phase offset `Δ` (caused by the delay `τ`):

$$
x = A\cos(\theta - \Delta) = A\cos(\Delta)\cos(\theta) + A\sin(\Delta)\sin(\theta)
$$

This is just the angle-sum identity, read as: *relative to the reference basis, `x` has an in-phase component `∝ cos(Δ)` on the `cos(θ)` axis, and a quadrature component `∝ sin(Δ)` on the `sin(θ)` axis.*

Now correlate (mix + integrate) with `cos(θ)` alone:

$$
\langle x,\cos\theta\rangle \;\propto\; A\cos(\Delta)\cdot\underbrace{\langle\cos\theta,\cos\theta\rangle}_{\neq 0} \;+\; A\sin(\Delta)\cdot\underbrace{\langle\sin\theta,\cos\theta\rangle}_{=\,0\ \text{(orthogonal)}}
$$

The second term vanishes **identically**, because `sin` and `cos` at the same frequency are orthogonal. So the `sin(Δ)` term — the quadrature component, which carries the *sign* of `Δ` — isn't degraded or attenuated. It is **exactly zeroed by projection**, before an FFT ever sees the data. What survives is only `cos(Δ)`, and since `cos` is even, that single surviving number cannot distinguish `+Δ` from `−Δ`.

This matches the earlier "even function" conclusion, but now shows the mechanism: the information is discarded by orthogonal projection at the mix step, not lost in some vague downstream sense.

### Why "the FFT uses complex numbers" doesn't rescue you

This is the key part of the question, and the intuition is correct. The FFT's own basis functions are complex (`e^{-j2\pi kn/N} = \cos - j\sin`), so an FFT *can* represent both in-phase and quadrature content — but only if that content is still present in its input.

If `y[n]` is already real (because the mix used `cos(θ)` only), the quadrature component was zeroed at the mix stage, and no amount of complex arithmetic in the FFT afterward can regenerate it. Concretely, a real `y[n]`'s FFT is Hermitian symmetric:

$$
Y[-k] = Y[k]^{*}
$$

That symmetry is the frequency-domain fingerprint of "the quadrature information was never there to begin with." The FFT is complex-*capable*, but garbage in (real, projection-collapsed) still gives a mirrored, sign-ambiguous spectrum out.

### The correct requirement

Mix with **both** `cos(θ)` and `sin(θ)` — i.e., with `e^{-jθ}` — so the projection is onto a complete 2D (I/Q) basis instead of a single 1D axis. Nothing gets orthogonally discarded: both `cos(Δ)` and `sin(Δ)` survive as the real and imaginary parts of the resulting phasor `e^{-jΔ}`. Handing that complex `y[n]` to the FFT is then meaningful — the FFT has something to distinguish `+f_b` from `-f_b` with.

### One small correction to the framing

It's not that `rx` "has cos and sin components, as in an FFT" independent of mixing — `rx` by itself is just one real number per sample. The cos/sin decomposition only becomes meaningful *relative to the reference phase* at the mixing step; it is the **reference's** cos and sin that form the orthogonal basis being projected onto, and mixing with a cos-only reference is what throws away the axis that was never projected onto.

The core mechanism, though — orthogonality causing an outright erasure, not just a fuzzy loss of information — is exactly right, and it's a cleaner way to express the same conclusion as the earlier "even function" argument.

