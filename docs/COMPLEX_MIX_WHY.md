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

---

## 3. Concrete ambiguity example (using the pipeline's own numbers)

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
