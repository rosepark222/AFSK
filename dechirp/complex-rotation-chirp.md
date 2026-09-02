# Complex Rotation for Efficient Chirp Generation

## Idea

Instead of calling `sin()` and `cos()` for every sample, generate the next chirp sample by rotating the current complex value by a small phase increment.

If the current reference sample is represented as:

- `c = cos(φ)`
- `s = sin(φ)`

and the next phase advances by `Δφ`, then:

- `cos(φ + Δφ) = cos(φ)cos(Δφ) - sin(φ)sin(Δφ)`
- `sin(φ + Δφ) = sin(φ)cos(Δφ) + cos(φ)sin(Δφ)`

So once you know:

- current `cos(φ)`, `sin(φ)`
- `cos(Δφ)`, `sin(Δφ)`

you can compute the next sample using only multiplies and adds.

---

## Why this helps

Calling trig functions like `sin()` and `cos()` repeatedly is expensive.

Using complex rotation:

- avoids per-sample trig calls
- replaces them with simple arithmetic
- is faster and more predictable on embedded CPUs like Teensy 4.1

This is especially useful in a 44,100-sample correlation loop.

---

## Constant-frequency example

For a fixed-frequency tone:

- sample rate `Fs = 44100`
- tone frequency `f = 1000 Hz`

Then the phase step is:

- `Δφ = 2πf / Fs`
- `Δφ ≈ 2π * 1000 / 44100 ≈ 0.14248 rad`

Precompute:

- `a = cos(Δφ)`
- `b = sin(Δφ)`

Start with:

- `c0 = cos(0) = 1`
- `s0 = sin(0) = 0`

Then generate the next samples:

- `c1 = c0*a - s0*b`
- `s1 = s0*a + c0*b`

Since `s0 = 0`, this becomes:

- `c1 = a`
- `s1 = b`

Next step:

- `c2 = c1*a - s1*b`
- `s2 = s1*a + c1*b`

And so on.

---

## Chirp example

A chirp changes frequency over time, so the phase increment is not constant.

For a linear chirp:

- `f(t) = F0 + k*t`
- `k = (F1 - F0) / T`

The phase step at sample `n` is approximately:

- `Δφ[n] = 2π * f[n] / Fs`

where:

- `f[n] = F0 + k * (n / Fs)`

So each sample uses a slightly different rotation.

### Efficient approach

Instead of recomputing trig:

1. Track the current phase step `Δφ`
2. Track how `Δφ` changes each sample
3. Update the complex reference using a small rotation

This keeps the runtime cost low.

---

## Concrete chirp example

Suppose:

- `Fs = 44100`
- `F0 = 300 Hz`
- `F1 = 1500 Hz`
- `T = 1.0 s`

Then:

- `k = (1500 - 300) / 1 = 1200 Hz/s`

At sample `n = 0`:

- `f[0] = 300 Hz`
- `Δφ[0] = 2π * 300 / 44100 ≈ 0.04273 rad`

At sample `n = 1`:

- `f[1] = 300 + 1200/44100 ≈ 300.0272 Hz`
- `Δφ[1] ≈ 0.04273... rad`

The phase step changes only slightly each sample, so the reference can be updated incrementally rather than recomputed from scratch.

---

## Pseudocode

```c
float c = 1.0f;
float s = 0.0f;

float phi_step = 2.0f * PI * f0 / Fs;
float dphi_step = 2.0f * PI * k / (Fs * Fs);

for (int n = 0; n < N; n++) {
    // Use current reference sample
    float ref_cos = c;
    float ref_sin = s;

    // Rotate by the current step
    float a = cosf(phi_step);
    float b = sinf(phi_step);

    float next_c = c * a - s * b;
    float next_s = s * a + c * b;

    c = next_c;
    s = next_s;

    // Update the step for the chirp
    phi_step += dphi_step;
}
```

This version still computes trig for `phi_step`, but only for the changing step, not for the full chirp phase each sample. You can optimize further by updating the rotation itself recursively.

---

## More efficient variant

A better embedded approach is to avoid trig in the loop entirely by using a recurrence for the rotation factors too.

You keep:

- the current complex sample `(c, s)`
- the current rotation factors `(a, b)`

Then update both gradually.

This reduces the loop to:

- a few multiplies
- a few additions

---

## Notes on numerical drift

Recursive methods can slowly drift due to floating-point error.

To control that:

- occasionally renormalize `(c, s)` so that `c² + s² ≈ 1`
- for long runs, re-anchor from an exact phase value every so often

For a 1-second chirp, drift is usually manageable.

---

## Summary

Complex rotation lets you generate chirp samples efficiently:

- no per-sample `sin()` / `cos()` calls
- fast enough for embedded real-time processing
- ideal for large correlation loops

For your Teensy code, this is a good way to save RAM while keeping compute cost reasonable.