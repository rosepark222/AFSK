Question: compare fsk_rx.ino c code demodulation vs. python version leaky_integrator:
1. What the original IQ envelope detector really does (important)
Your C code implements this chain:
FSK tone
  ↓
mix to baseband (cos/sin)
  ↓
low‑pass filter (integrate over bit)
  ↓
magnitude² (energy)

This has several very important properties:
✅ Frequency selectivity

Rejects other tones
Rejects broadband noise
Sensitive only near the target frequency

✅ Phase independence
Works regardless of carrier phase
Correct for non‑coherent FSK

✅ Proper energy integration
Energy ∝ amplitude² × time
Smooth, monotonic envelope growth

✅ Correct transient behavior
Transition energy bleeds realistically
No artificial “instant” knowledge

This is essentially a matched filter for each tone.

2. What the Python detector does (and does NOT do)
The Python model:

Assumes perfect frequency separation
Assumes perfect detection of “mark present”
Assumes no noise
Assumes no phase
Assumes no spectral leakage

It is intentionally cheating.
That is why it was useful for:

clock recovery reasoning
causality analysis
early/late behavior
sampling placement

But it is not a physical detector.

A graph from python leaky_integrator
<img width="1508" height="727" alt="image" src="https://github.com/user-attachments/assets/4867e602-4918-4e18-9ed6-30ffdc36c2b6" />

How to explain this to anyone (including a high‑schooler)

| line      | note |
| :--- | :--- |
| Black dashed line      | the real bits: 101010… |
| Blue & magenta curves  | smooth “confidence meters” for MARK and SPACE |
| Green curve            | subtraction: positive = MARK, negative = SPACE |
| Black dots             | exact moments we decide the bit (middle of each bit) |
| Gray horizontal line (zero) | decision threshold |

<img width="1143" height="882" alt="image" src="https://github.com/user-attachments/assets/13fa9053-ca50-453e-80b4-2afeb347f795" />
purple triangle is clockCount ranged from 0 to 441 (sample per bit). The goal of this clock is to align its mid point to the bit transition, so that the wrap point is when to sample the bit.


The figure shows phase jumping forward in early stage of clock recovery. Why jumping forward?   
1, clock phase < mid point. In PLL textbook, this means the bit edge is "early" relative to the clock)  
2, clock is too slow, causing sample point late (see that black dots are pushed toward the end of the bit duration)  
3, clock phase should jump forward to pull the sampling point to the left  
4, in the below example, 20 to 30 jump pulls the sampling point to the left by 10 samples  
  $0, 1, 2, ... 20, 30, 31, ... 441 → wrap → sample$    

---

<img width="1148" height="889" alt="image" src="https://github.com/user-attachments/assets/c2dcce9e-76c7-48df-be05-d1fadfc2cdab" />
When the bit edge is recovered, we can see clock is pulled toward left in one bit and toward right for the next bit, etc.

---

first 13 min of https://youtu.be/w1SB9Ry8_Jg?si=cyaR_r0YtE0NOga2 
shows clock pulse passing through a RC circuit, which is very similar to rx preamble for clock recovery


fsk_rx.ino has the line:
// Single-pole LPF:  $$y[n] = y[n-1] + \alpha*(x[n] - y[n-1])$$

I observed that, 
if x[n] is 0, y keeps decreasing because it subtract portion of previous y as y[n] = y[n-1] - LPF_ALPHA * y[n-1], thus called leaky; 
if x[n] is 1, even though y still decays, it adds LPF_ALPHA * 1, which overcomes the decay.
 Even if this is simple filter, it taps into a one semester worth of college level filter design theory. Can you give a gentle introduction to the filter design theory, why this design is chosen and how to determine the parameter LPF_ALPHA? while the above intuition helps, it must have followed some theory -- the pole location of IIR filter design.

1. What this filter really is (conceptually)
Your filter:
$$y[n] = y[n−1] + α (x[n] − y[n−1])$$

can be rewritten as:
$$y[n] = (1 − α) y[n−1] + α x[n]$$

This is the simplest possible IIR low‑pass filter:

One state (y[n−1])
One pole
No zeros (other than implicit ones)

Conceptually, it is:

A running average that exponentially forgets the past

This idea shows up everywhere:

RC circuits
Thermal systems
Population decay
PLL loop filters
Envelope detectors

So while the math is DSP, the idea is universal.

2. Continuous‑time origin: the RC low‑pass (key intuition)
The theoretical origin is the first‑order analog low‑pass filter:
$$\tau \frac{dy(t)}{dt} + y(t) = x(t)$$
Where:

τ = time constant
Bigger τ → slower response → more smoothing

For a step input:

Output rises exponentially
Never overshoots
Settles smoothly

This is exactly the behavior you observed.

3. Discretizing the analog filter (where α comes from)
When sampled at period T = 1/fs, the RC filter becomes:
 

$$y[n] = e^{-T/\tau} y[n-1] + (1 - e^{-T/\tau}) x[n]$$


Compare with your code:
$$y[n] = (1 − α) y[n−1] + α x[n]$$

That gives:
$$ \boxed{\alpha = 1 - e^{-T/\tau}} ​$$
This is not arbitrary.
It’s the exact discrete‑time equivalent of an analog RC filter.

4. Pole location (the “theory” part)


In DSP terms, this filter has:

$$\text{Pole at } z = 1 - \alpha$$

What the pole means physically

In DSP terms, this filter has:
Pole at z=1−α\text{Pole at } z = 1 − \alphaPole at z=1−α

| Pole location | Behavior |
| :--- | :--- |
| $z = 1$ | Pure integrator (never forgets) |
| $0 < z < 1$ | Leaky integrator |
| $z \approx 1$ | Long memory, slow response |
| $z \approx 0$ | Short memory, fast response |

 
Your observation:

“If x[n]=0, y decays by subtracting α·y”

That is the pole at work.
The pole controls how fast past energy decays.

5. Why this design is chosen (especially for FSK)
This one‑pole IIR is chosen because it gives the best possible tradeoff between:
✅ Simplicity
✅ Stability
✅ Low CPU cost
✅ Smooth envelope
✅ Predictable phase behavior
For your FSK demod:

You want energy accumulation
You want leakage to forget old bits
You do not want ringing
You do not want overshoot
You do not want delay ripple

This filter is provably optimal for:

envelope tracking with minimal complexity

That’s why almost every non‑coherent FSK receiver uses exactly this structure.

6. How to choose LPF_ALPHA (the practical part)
This is the most important takeaway.
You usually choose a cutoff frequency f_c (Hz), then compute:

$$α=1−e−2πfc​/fs​​$$

Which matches your code:
C++LPF_ALPHA = 1.0f - expf(-2.0f * PI * LPF_FC / SAMPLE_RATE);Show more lines
What f_c should be (rule of thumb)
For envelope tracking of symbols:
f_c ≈ 1× baud  to  3× baud

Why?

Too low → sluggish, ISI, delay
Too high → noisy, jittery envelope

For your case (100 baud):
LPF_FC = 200–300 Hz   ← excellent choice

That gives:

Fast enough rise within 10 ms bit
Enough smoothing to reject noise
Good separation at mid‑bit

7. Why the filter does NOT “integrate forever”
You already answered this intuitively, but here’s the formal reason:
The steady‑state output is:

$$\lim y[n] = x_{\text{avg}}$$

Because:

Input term pushes toward x
Pole continually leaks energy away
Equilibrium occurs when both balance

That’s the core reason it settles instead of diverging.

8. One‑phrase explanation (good for teaching)

“The pole decides how fast the filter forgets the past — α just sets that forgetting rate.”

9. Mapping back to your intuition (validation)
Your statement:

“If x[n] is 1, the added α·1 overcomes the decay”

✔ Correct
✔ Physically meaningful
✔ Exactly what the pole math describes
You have already internalized the core DSP idea — the math just formalizes it.

10. Why this alone is worth a semester
Because this one filter connects:

Differential equations
Laplace transforms
Z‑transforms
Stability
Frequency response
Time constants
Physical systems

But you don’t need all that to use it correctly — just the few rules above.

Final takeaway

This filter is not arbitrary
It is the discrete form of an RC low‑pass
α directly encodes the time constant
Pole location = memory length
It is ideal for envelope detection
Your intuition is already correct

If you want next, I can:

Show the exact step response math
Plot pole vs time constant intuition
Compare this to FIR averaging
Show when this filter fails and why

Just say where you want to dig next.
