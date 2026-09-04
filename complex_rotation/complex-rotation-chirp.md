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

This version still computes trig for `phi_step`, but only for the changing step, not for the full chirp phase each sample. You can optimize further by updating the rotation itself iteratively using a recurrence.

---

## More efficient variant

A better embedded approach is to avoid trig in the loop entirely by using a recurrence for the rotation factors too.

You keep:

- the current complex sample `(c, s)`
- the current rotation factors `(a, b)`
- a fixed tiny update rotation `(ca, sa)` for how the step changes

Then update both iteratively.

The loop becomes:

```c
float c = 1.0f, s = 0.0f;          // current chirp sample
float a = cosf(phi_step0);         // current per-sample rotation
float b = sinf(phi_step0);

float ca = cosf(dphi_step);        // tiny update rotation
float sa = sinf(dphi_step);

for (int n = 0; n < N; n++) {
    // use current reference
    float ref_cos = c;
    float ref_sin = s;

    // advance the chirp sample
    float next_c = c * a - s * b;
    float next_s = s * a + c * b;

    // advance the rotation factor
    float next_a = a * ca - b * sa;
    float next_b = b * ca + a * sa;

    c = next_c;
    s = next_s;
    a = next_a;
    b = next_b;
}
```

This means:

- no trig inside the loop
- no phase recomputation
- just arithmetic

---

## Why this works

A complex number on the unit circle can be multiplied by another unit complex number to rotate it.

So:

- chirp evolution = repeated multiplication by a varying unit rotation
- step evolution = repeated multiplication by a tiny fixed unit rotation

This is numerically and computationally cheaper than calling trig repeatedly.

---

## Example interpretation

Suppose your chirp starts at 300 Hz and ramps upward.

At the beginning:

- `Δφ` is the phase step for 300 Hz
- `a,b` are the rotation for that step

A moment later:

- `Δφ` is slightly larger
- instead of recalculating `cos(Δφ)` and `sin(Δφ)`, you nudge `(a,b)` forward with the small update rotation `(ca, sa)`

So the loop “walks” both:

- the output point on the circle
- the step size that drives that point

---

## What you gain

This method is useful because it:

- removes all per-sample trig calls
- keeps the inner loop predictable
- is fast on embedded CPUs
- reduces dependence on expensive math library calls

On something like a Teensy 4.1, that can matter a lot in a tight correlation or signal-generation loop.

---

## Tradeoff: drift

The downside is floating-point error accumulates.

Over time:

- `(c, s)` may stop being perfectly unit length
- `(a, b)` may drift too

Common fixes:

- renormalize occasionally:
  - `mag = sqrt(c*c + s*s)`
  - `c /= mag; s /= mag`
- or rebuild from an exact phase every so often
- or use fixed intervals, like every few thousand samples

For short runs, this is usually fine.

---

## A cleaner pseudocode version

```c
float c = 1.0f, s = 0.0f;          // current chirp sample
float a = cosf(phi_step0);         // current per-sample rotation
float b = sinf(phi_step0);

float ca = cosf(dphi_step);        // tiny update rotation
float sa = sinf(dphi_step);

for (int n = 0; n < N; n++) {
    // use current reference
    float ref_cos = c;
    float ref_sin = s;

    // advance chirp sample
    float next_c = c * a - s * b;
    float next_s = s * a + c * b;

    // advance the rotation factor
    float next_a = a * ca - b * sa;
    float next_b = b * ca + a * sa;

    c = next_c;
    s = next_s;
    a = next_a;
    b = next_b;
}
```

---

## Even simpler way to think about it

You are doing two iterative rotations:

- **outer rotation**: the signal sample moves around the circle
- **inner rotation**: the step size itself changes gradually

That’s why the document calls it a “more efficient variant”: it eliminates trig from the hot path entirely.

If you want, I can also rewrite that section of `complex-rotation-chirp.md` into a clearer embedded-style explanation with a better code example.
