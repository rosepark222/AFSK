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
  $\text{Pole at } z = 1 - \alpha$

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
C++ LPF_ALPHA = 1.0f - expf(-2.0f * PI * LPF_FC / SAMPLE_RATE); 
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
