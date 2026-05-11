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

