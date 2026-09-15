# I/Q Correlation and the Hilbert Transform

## Why plain real-vs-real correlation isn't enough for oscillating signals

## 1. The textbook picture: correlating a rectangular pulse

Most people are first taught correlation and convolution using a **rectangular
pulse** (or a triangle, or some other simple baseband shape):

```
rect(t) = 1  for  0 ≤ t ≤ T
rect(t) = 0  otherwise
```

If you cross-correlate `rect(t)` with a delayed copy of itself, you get a
clean **triangular peak**:

$$
R(\tau) = \int_{-\infty}^{\infty} \text{rect}(t)\, \text{rect}(t+\tau)\, dt
$$

This result is smooth and single-peaked because `rect(t)` is a **baseband
signal** — it has no oscillation, no carrier, and its value never changes
sign. As you slide the two rectangles past each other, the overlap area
changes *monotonically and smoothly*. There's nothing inside the pulse to
create ripples: the correlation is entirely explained by geometric overlap.

This is the intuition most people carry forward — and it silently assumes the
signal being correlated has **no internal oscillation**.

## 2. What changes when the signal has a carrier (e.g. a chirp)

A radar/sonar/acoustic chirp is not a rectangular pulse. It's a real,
**oscillating** signal — typically something like:

$$
s(t) = A(t)\, \cos\big(\phi(t)\big), \qquad \phi(t) = 2\pi\left(f_0 t + \tfrac{1}{2}k t^2\right)
$$

where `A(t)` is a slowly varying envelope (e.g. a rectangular window or a
taper), and `φ(t)` is a fast time-varying phase that sweeps frequency from
`f0` to `f0 + kT` over the pulse duration `T`. Even though `A(t)` might be a
simple rectangle, the transmitted waveform itself is a **cosine riding on
that rectangle** — it oscillates rapidly within the pulse.

This is the key difference from the classroom rect-vs-rect example: the
transmitted signal is **not just an envelope, it's an envelope multiplied by
a fast oscillating carrier**.

### Why real-vs-real correlation now ripples

When you correlate a real received signal `r(t)` with a real reference chirp
`s(t)`, the output is:

$$
C(\tau) = \int r(t)\, s(t+\tau)\, dt
$$

Because both signals are oscillating at (roughly) the same carrier/sweep
rate, this integral behaves like the correlation of two cosines: as `τ`
changes, the phase alignment between the two cosines changes continuously,
so `C(τ)` **oscillates positive and negative at the carrier rate**, riding on
top of a slower envelope that reflects the *true* time alignment.

This is exactly what you saw in your autocorrelation plot: a peak
surrounded by decaying ripples that cross zero. Those ripples are **not**
noise, and they are **not** evidence of multiple targets or an ambiguous
chirp — they are the beat pattern between the two carriers as the reference
slides past the received signal in lag. A real-valued correlation of an
oscillating signal literally cannot avoid this: any bandpass (carrier-based)
signal correlated as pure real-vs-real data will show carrier ripple,
regardless of how "clean" the chirp is.

**In short: the rectangular-pulse intuition (smooth triangle, single peak)
only holds for baseband, non-oscillating signals. As soon as your signal has
a carrier or is intrinsically oscillatory (a chirp, a sinusoidal burst, a
modulated pulse), the raw real correlation becomes phase-sensitive, and the
sign/magnitude at any single lag no longer tells you how well-aligned the
signals are.**

## 3. The fix: strip out carrier phase using I/Q (quadrature) processing

The standard solution — used throughout radar, sonar, and communications —
is to compute the correlation using an **analytic (complex) reference**
instead of a purely real one. This gives you two correlation channels,
**in-phase (I)** and **quadrature (Q)**, that are 90° apart in carrier
phase. Combining them recovers the true envelope, independent of carrier
phase.

### 3.1 The analytic signal and the Hilbert transform

For a real signal `x(t)`, the **Hilbert transform** is defined as:

$$
\hat{x}(t) = \mathcal{H}\{x(t)\} = \frac{1}{\pi} \, \text{p.v.} \int_{-\infty}^{\infty} \frac{x(\tau)}{t - \tau}\, d\tau
$$

In the frequency domain, the Hilbert transform is simply a **90° phase
shift applied to every frequency component** of `x(t)`:

$$
\hat{X}(f) = -j\,\text{sgn}(f)\, X(f)
$$

i.e. positive frequencies are shifted by −90°, negative frequencies by
+90°. Applying this to a pure cosine gives you a pure sine:

$$
\mathcal{H}\{\cos(2\pi f_0 t)\} = \sin(2\pi f_0 t)
$$

The **analytic signal** of `x(t)` is then formed as:

$$
x_a(t) = x(t) + j\,\hat{x}(t)
$$

This is a complex signal whose real part is the original signal and whose
imaginary part is its quadrature (90°-shifted) companion. For a chirp
`s(t) = A(t)\cos(\phi(t))`, the analytic signal is (to good approximation,
for signals with well-separated positive/negative spectral content):

$$
s_a(t) \approx A(t)\, e^{j\phi(t)} = A(t)\cos(\phi(t)) + j\, A(t)\sin(\phi(t))
$$

Note that the envelope `A(t)` — the thing you actually care about — now sits
cleanly outside the oscillating phase term.

### 3.2 I/Q correlation

Given a real reference chirp `s(t)`, build its analytic version via the
Hilbert transform:

- **In-phase reference:** $s_I(t) = s(t)$ (the reference itself)
- **Quadrature reference:** $s_Q(t) = \hat{s}(t) = \mathcal{H}\{s(t)\}$ (the Hilbert-shifted reference)

Correlate the **real received signal** `r(t)` against both:

$$
I(\tau) = \int r(t)\, s_I(t+\tau)\, dt, \qquad Q(\tau) = \int r(t)\, s_Q(t+\tau)\, dt
$$

These two are equivalent to the real and imaginary parts of a single
complex correlation between `r(t)` and the analytic reference `s_a(t)`:

$$
I(\tau) + jQ(\tau) = \int r(t)\, s_a^{*}(t+\tau)\, dt
$$

### 3.3 The envelope

The **magnitude** of this complex correlation is the carrier-independent
envelope:

$$
A(\tau) = \sqrt{I(\tau)^2 + Q(\tau)^2}
$$

Because `I` and `Q` are 90° apart in carrier phase, whatever null one of
them hits at a given lag (due to unlucky phase alignment), the other is near
its peak — the two channels' ripples cancel out in quadrature sum, leaving a
smooth envelope that reflects only the true degree of time alignment between
`r(t)` and `s(t)`, with the fast carrier oscillation removed.

You can also compute the **phase** of the correlation if it's useful:

$$
\theta(\tau) = \arctan_2\big(Q(\tau),\, I(\tau)\big)
$$

which tells you the carrier phase offset at the point of best alignment
(useful for coherent detection, Doppler estimation, etc.), separate from the
magnitude, which tells you how well-aligned the pulses are.

## 4. When do you actually need I/Q correlation?

| Signal type | Real-vs-real correlation sufficient? | Why |
|---|---|---|
| Rectangular / triangular / baseband pulse (no carrier) | ✅ Yes | No internal oscillation — overlap changes monotonically, single clean peak |
| Envelope-only detection of a slow, non-oscillatory waveform | ✅ Yes | Same reasoning — nothing for carrier phase to interact with |
| LFM chirp, tone burst, or any bandpass/carrier-modulated pulse (radar, sonar, acoustic ranging, ultrasound) | ❌ No — use I/Q | The signal itself oscillates at a carrier/sweep rate; real correlation is carrier-phase-sensitive and produces misleading zero-crossings and sign flips |
| Communications matched filtering (QAM, PSK, etc.) | ❌ No — use I/Q (this is standard practice) | Same reason; receivers are built around I/Q sampling for exactly this purpose |
| You already have complex baseband (I/Q) sampled data from an SDR/receiver | N/A — you're already in the analytic domain | No Hilbert transform needed; your ADC/DDC already gives you I and Q directly |

**Rule of thumb:** if the signal you're correlating has a carrier or sweeps
through several cycles of oscillation within the pulse (as almost any
chirp, tone burst, or modulated waveform does), you need I/Q (or equivalent
analytic-signal) processing to get a physically meaningful, phase-independent
correlation magnitude. If the signal is a slowly-varying, non-oscillatory
envelope with no internal carrier (like the rectangular pulse from
introductory DSP courses), plain real-vs-real correlation is already the
right, sufficient answer — there is no carrier phase to average out.

## 5. Practical notes

- If your receiver already produces complex baseband (I/Q) samples (common
  in SDR-based systems), you don't need to compute a Hilbert transform at
  all — you already have the analytic signal `s_a(t)` directly from the
  hardware/DDC chain. The Hilbert transform is only needed when you're
  starting from **real-only** sampled data (e.g. a single real ADC channel,
  or a `.wav` file of an acoustic signal).
- The Hilbert transform should be applied to the **known reference**
  (transmitted chirp), not the noisy received signal, since the reference is
  clean and its true quadrature companion is well-defined; applying it to
  noisy received data doesn't give you anything meaningful about the
  transmitted waveform's timing.
- Finite-length Hilbert transforms (e.g. via FFT-based implementations such
  as `scipy.signal.hilbert`) can have edge artifacts if the reference
  signal doesn't start/end near zero. Applying a light taper/window to the
  reference before the Hilbert transform, or generating the quadrature
  version analytically from the chirp's known phase law
  (`sin(φ(t))` instead of `cos(φ(t))`), avoids this.
- Once you have the envelope `A(τ)`, all the standard matched-filter
  diagnostics apply normally: peak-to-sidelobe ratio, mainlobe width,
  comparison to the theoretical -13.5 dB sidelobe level for an unweighted
  linear FM chirp, etc.

## 6. Summary

- A rectangular pulse correlated with itself gives a clean single peak
  because it has no internal oscillation.
- A chirp (or any carrier-based waveform) is a cosine riding inside an
  envelope; correlating it as a raw real signal makes the result
  phase-sensitive, producing ripples and sign flips that reflect carrier
  beat frequency, not target ambiguity or noise.
- The fix is to correlate against **both** the reference and its
  Hilbert-transformed (90°-shifted) quadrature companion, producing I(τ) and
  Q(τ), then take the magnitude `A(τ) = √(I² + Q²)` to get a carrier-phase
  independent envelope — the quantity that should actually be used to judge
  peak location, mainlobe width, and sidelobe levels.
