## Relationship between correlation step and pulse bnandwith

For an LFM chirp, the compressed pulse width is approximately:

$$
\tau_c \approx \frac{1}{B}
$$

where:

- $B$ = chirp bandwidth
- $\tau_c$ = compressed pulse width

As chirp bandwidth increases, pulse compression becomes stronger and the matched-filter autocorrelation peak becomes narrower.

Because the correlation peak becomes narrower, the correlator must be evaluated more frequently to avoid skipping over the peak.


example:
<img width="1254" height="395" alt="Screenshot 2026-05-06 140409" src="https://github.com/rosepark222/AFSK/blob/3da923685299670e11aec0b1b14db7f29de4981f/dechirp/cross_correlation_result.9_7.png" />

To find the zero crossings on your graph, look for the envelope pinch-points at **lag -5** and **lag 63** directly on the horizontal 0.0 line.

### 1. Left Envelope Null (Around Lag -5)
* **Main Peak:** Start at the highest peak at lag 29 (red dashed line).
* **Deep Valley:** Move left to the deep valley at lag 6 (value is near $-1.0$).
* **Next Peak:** Keep moving left until the next positive peak at lag -16.
* **Pinch Point:** Look at the horizontal 0.0 line between lag -16 and lag 6. The outer boundary squeezes down to a single point right at **lag -5**.

### 2. Right Envelope Null (Around Lag 63)
* **Main Peak:** Start at the highest peak at lag 29.
* **First Valley:** Move right to the valley at lag 52 (value is around $-0.65$).
* **Next Peak:** Keep moving right to the smaller positive peak at lag 75 (value is around $+0.32$).
* **Pinch Point:** Look at the horizontal 0.0 line between lag 52 and lag 75. The outer boundary squeezes down to a point right at **lag 63**.

### Pulse Width Summary

* **Time per Sample:**

$$
\text{Time per sample} = \frac{1}{44100} \approx 22.68 \ \mu\text{s/sample}
$$

* **Total Samples (Null-to-Null): 68** 

* **Null-to-Null Width ($2/B$):** 

$$ 
68 \text{ samples} \times 22.68 \ \mu\text{s/sample} \approx 1.54 \text{ ms} 
$$

* **Empirical Pulse Width ($1/B$):**
  
$$ 
\text{Pulse Width} = \frac{1.54 \text{ ms}}{2} = \mathbf{0.77 \text{ ms}} 
$$

The empirical calculation matches the theoretical value of 

$$
\frac{1}{B} = \frac{1}{1200 \text{ Hz}} \approx 0.833 \text{ ms}
$$


why measure $2/B$ and then $1/B$ ? 

Engineers measure null-to-null and divide by 2 in real-world signal processing for three practical reasons:Averages Out Noise and Distortion: In actual field data, noise, multipath reflections, or hardware distortions can warp the signal. One side of the pulse might get slightly squished or stretched. Measuring the full width and dividing by 2 averages out this asymmetry.Reduces Sampling Jitter: Because digital signals are sampled at discrete intervals, the true analog peak might actually fall right between two samples. Relying heavily on a single peak sample as your reference point introduces more measurement error than measuring a wider span across the entire main lobe.Checks Signal Quality: Verifying that the peak is perfectly centered between both nulls is a quick way to ensure your system doesn't suffer from phase errors or Doppler shifts.


---

## Correlation Step Requirement

To reliably detect the peak:

$$
T_{step} \ll \tau_c
$$

Since

$$
\tau_c \approx \frac{1}{B}
$$

the maximum allowable correlation step is approximately:

$$
T_{step} \propto \frac{1}{B}
$$

This means:

- Larger bandwidth $\rightarrow$ narrower correlation peak
- Narrower correlation peak $\rightarrow$ smaller allowable correlation step
- Smaller correlation step $\rightarrow$ more correlation computations

---

## Conceptual Relationship

```text
Required Correlation Step
(samples)

 ^
 |
 |\
 | \
 |  \
 |   \
 |    \
 |      \
 |        \
 |          \
 +-----------------------> Chirp Bandwidth
```

The relationship is approximately inverse:

$$
step_{max} \approx \frac{F_s}{4B}
$$

where:

- $F_s$ = sample rate
- $B$ = chirp bandwidth

The factor of 4 is a practical design margin so that multiple correlation evaluations occur within the compressed pulse width.

---

## Example ($F_s = 44.1\ \text{kHz}$)

| Bandwidth | Maximum Step |
| ---------- | ---------- |
| 1 kHz | 11.03 samples |
| 2 kHz | 5.51 samples |
| 4 kHz | 2.76 samples |
| 8 kHz | 1.38 samples |
| 16 kHz | < 1 (0.69) sample |

---

## Impact of a Fixed 128-Sample Hop

A 128-sample hop at 48 kHz corresponds to:

$$
\frac{128}{44100} = 2.9\ \text{ms}
$$

This means the correlator only checks for a peak every 2.9 ms.

 
