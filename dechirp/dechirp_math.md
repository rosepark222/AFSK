# Dechirp receiver math background

This note explains the math behind the `mix + FFT` dechirp receiver used in this project, especially why a real-valued received signal can be processed using a complex chirp and why the dechirp converts a chirp into a narrowband beat signal.

## 1. Signal model

Let the received RF/baseband signal be a real-valued time signal:

$$
 x(t) = A \cos\left(2\pi\left(f_0 t + \frac{k}{2} t^2\right) + \phi\right)
$$

where:

- $A$ is amplitude
- $f_0$ is the starting frequency
- $k = \frac{f_1 - f_0}{T}$ is the chirp slope
- $T$ is the chirp duration
- $\phi$ is an arbitrary phase offset

This is a real chirp. It is not complex-valued in the RF sense; it is a real sinusoid whose instantaneous frequency sweeps from $f_0$ to $f_1$.

---

## 2. Pure real $x(t)$ and the mixing step with different phase

Now consider a real received chirp with a phase offset relative to the local reference:

$$
 x(t) = A\cos\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right) + \phi_x\right)
$$

The receiver generates a reference chirp with the same chirp rate but a different phase:

$$
 r(t) = e^{-j\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right) + \phi_r\right)}
$$

This is realistic: the received chirp and the local reference usually do not start at the exact same phase. The mixer output is

$$
 y(t) = x(t)\,r(t)
$$

Write the real signal as the real part of a complex exponential:

$$
 x(t) = \Re\left\{A e^{j\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right) + \phi_x\right)}\right\}
$$

Then the product becomes

$$
 y(t) = \Re\left\{A e^{j\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right) + \phi_x\right)}\right\}
       e^{-j\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right) + \phi_r\right)}
$$

Using the identity

$$
\cos\alpha = \frac{e^{j\alpha}+e^{-j\alpha}}{2}
$$

we get

$$
 y(t) = \frac{A}{2}e^{j(\phi_x-\phi_r)} + \frac{A}{2}e^{-j\left(2\pi\left(2f_0 t + k t^2\right) + \phi_x + \phi_r\right)}
$$

The two terms are:

- a low-frequency term: $\frac{A}{2}e^{j(\phi_x-\phi_r)}$
- a high-frequency term: $\frac{A}{2}e^{-j\left(2\pi\left(2f_0 t + k t^2\right) + \phi_x + \phi_r\right)}$

The first term is the useful dechirped component. Its magnitude depends on the phase difference $\phi_x - \phi_r$:

$$
|y_{base}(t)| = \frac{A}{2} \left|e^{j(\phi_x-\phi_r)}\right| = \frac{A}{2}
$$

In a perfectly matched case, $\phi_x = \phi_r$, the baseband term is maximized and the chirp collapses cleanly. If the phases differ, the complex term still exists, but the mixing result is effectively rotated in the complex plane. When we then apply the FFT or a low-pass filter, this phase rotation is treated as a complex phase factor rather than as a change in the chirp’s frequency content.

This is the realistic situation in hardware: the incoming chirp and the local reference almost never have identical phase. The dechirp step is still valid, because the chirp phase is cancelled by the conjugate reference, leaving a residual complex term whose phase depends on the mismatch, while the amplitude of the compressed signal remains tied to the matched energy.

This is why, in implementation, we usually do not expect the signal to be perfectly aligned in phase, and why the FFT magnitude or power is a more useful detection quantity than the raw complex phase.

---

## 3. Complex representation of a real sinusoid

A real sinusoid can be written as the real part of a complex exponential:

$$
 A \cos(2\pi f t + \phi) = \Re\{A e^{j(2\pi f t + \phi)}\}
$$

So the real signal can be represented as:

$$
 x(t) = \Re\{ s(t) \}
$$

where the analytic/complex form is:

$$
 s(t) = A e^{j(2\pi f t + \phi)}
$$

For a chirp, the complex form is:

$$
 s(t) = A e^{j\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right) + \phi\right)}
$$

This is the quantity used in the dechirp math even though the ADC stores the real-valued waveform.

The real signal is recovered by taking the real part afterward, but the matched-filter math is normally carried out in the complex domain because it simplifies the phase terms.

---

## 3. Complex conjugate chirp reference

The receiver generates a reference chirp with the same slope and duration:

$$
 r(t) = e^{-j\left(2\pi\left(f_0 t + \frac{k}{2} t^2\right)\right)}
$$

This is the complex conjugate of the nominal chirp phase. The multiplication is:

$$
 y(t) = x(t) \cdot r(t)
$$

Using the complex form of the chirp:

$$
 x(t) \approx A e^{j\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right) + \phi\right)}
$$

then:

$$
 y(t) = A e^{j\phi} e^{j\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right)\right)} \cdot e^{-j\left(2\pi\left(f_0 t + \frac{k}{2}t^2\right)\right)}
= A e^{j\phi}
$$

This is the key idea: for a perfectly matched reference, the quadratic phase terms cancel exactly, leaving a constant complex term (plus any residual mismatch). In other words, the chirp is “compressed” into a DC-like tone.

This is why the dechirp step is often called time compression or matched filtering.

---

## 4. Why the real signal still works with a complex chirp

The received signal is real-valued, but when we create the reference chirp we use a complex exponential with phase:

$$
 r[n] = \cos(\theta[n]) - j\sin(\theta[n])
$$

This is equivalent to mixing the real signal with the in-phase and quadrature copies:

$$
 x[n] \cdot r[n] = x[n] \cdot \cos(\theta[n]) - j x[n] \cdot \sin(\theta[n])
$$

So the action of the complex multiply is really the same as mixing the real signal with both cosine and sine components:

- real part: $x[n]\cos(\theta[n])$
- imag part: $-x[n]\sin(\theta[n])$

In practice, the ADC stores only the real signal, but we emulate the complex mixing by treating the chirp as a complex reference and keeping both I and Q components of the mixed signal in software. The end result is the same as doing a quadrature demodulation.

This is especially important because the chirp is swept phase, so using a complex chirp makes the cancellation exact in the complex plane.

---

## 5. Beat frequency after dechirp

If the received chirp is not perfectly matched in timing or frequency, then the phase cancellation is not exact and a residual beat term remains:

$$
 y(t) = A e^{j\phi} e^{j2\pi\Delta f t}
$$

where $\Delta f$ is the residual frequency difference caused by mismatch between the received chirp and the reference chirp.

This means the dechirped signal is no longer a wideband chirp; it collapses to a lower-frequency tone whose frequency depends on the mismatch. In the ideal matched case, $\Delta f = 0$, and the signal becomes almost DC.

This is why after mixing, the chirp energy is concentrated into a narrow frequency region instead of being spread over the original sweep bandwidth.

---

## 6. FFT after dechirp

Once the signal is dechirped, we take a Fourier transform:

$$
 Y[k] = \sum_{n=0}^{N-1} y[n] e^{-j2\pi kn/N}
$$

If the chirp is properly aligned with the reference chirp, then the energy in $y[n]$ is concentrated into a narrow part of the spectrum, so the FFT will show a prominent peak at the beat frequency or near DC. If the chirp is not aligned, the energy is spread and the FFT peak is weak.

This makes the FFT useful as a detection stage after dechirp because the chirp’s swept energy has been compressed into a much smaller effective bandwidth.

---

## 7. Why not just FFT the raw chirp?

If we FFT the raw chirp directly, the energy is spread over the full chirp bandwidth, because the instantaneous frequency is changing over time. That makes detection less efficient.

After dechirp:

- the time-varying phase is removed
- the signal becomes a narrowband beat
- the FFT peak becomes much more obvious

This is the entire purpose of dechirp processing.

---

## 8. What the receiver is actually doing in code

The code does the following in a block of samples:

1. for each sample index $n$ in the block:
   - compute the reference chirp phase $\theta[n]$
   - form a complex multiply:
     $$
     re = x[n] \cdot \cos(\theta[n])
     $$
     $$
     im = -x[n] \cdot \sin(\theta[n])
     $$
2. store the result in the complex FFT buffer
3. run a complex FFT on that block
4. compute magnitude spectrum:
   $$
   |Y[k]| = \sqrt{\Re\{Y[k]\}^2 + \Im\{Y[k]\}^2}
   $$
5. inspect the peak or peak-to-average ratio to detect the chirp

Since the signal is real-valued, the complex dechirp is simply a quadrature mixing step, where the reference chirp is represented by both cosine and sine components.

---

## 9. Summary

The core idea is:

- a chirp is a frequency-swept signal
- multiplying by the conjugate of the expected chirp removes the sweep
- the result is a narrowband beat signal
- FFT then reveals that compressed energy much more clearly than the raw chirp spectrum

The real-valued received signal still works with the complex chirp because a real sinusoid is the real part of a complex exponential, and the complex multiply is simply a convenient way to implement quadrature mixing in the I/Q domain.

---

## 10. Practical interpretation

This works because dechirp is essentially a matched filter. The receiver is not “guessing” the signal content; it is correlating the received waveform with the expected chirp waveform. The correlation is strongest when the chirp timing and phase are aligned, and the FFT then reveals that coherent energy as a peak.

This is exactly why the receiver can detect a chirp even when the raw signal looks like a broad, noisy sweep.
