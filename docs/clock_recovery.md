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
purple triangle is clockCount ranged from 0 to 441 (sample per bit) assuming 100 baud (10ms per bit).  

The goal of clock recovery is to align its mid point (220 ish) to the bit transition, so that the wrap point (441->0) is timing to determine the bit.

For 44.1Khz sampling rate, 10ms bit duration produces 441 sample. The quadrature demodulation makes decision per sample called softBit.

softBit makes a transition from 0 to 1 or 1 to 0 at the bit boundary --  this is somewhat naive approach.  

and this bit boundary is where the mid point of clockCount (220) should be, so that clockCount wrap (441) is where the hard decision for the bit is made. 

If the bit boundary is not clockCount=220, clockCount should be adjusted.

In the above figure, clockCount advances 5 times. because  
1, clockCount < 220. In PLL textbook, this means the bit edge is "early" relative to the clock)  
2, sampling point (clockCount=441) is too late  
3, clockCount should move forward (added some value) to pull the sampling point to the left  
 

---

<img width="1148" height="889" alt="image" src="https://github.com/user-attachments/assets/c2dcce9e-76c7-48df-be05-d1fadfc2cdab" />


The above figure shows When the bit boundary is where clockCount = 220 (clock is recovered).  
Here, clockCount is pushed and pulled and keeps the wrap point to the center of the bit. 



---

