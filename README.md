# AFSK
<img width="1254" height="395" alt="Screenshot 2026-05-06 140409" src="https://github.com/user-attachments/assets/c246b733-13c9-428a-9c97-e8837fd05069" />

rx chirp vs ref chirp


<img width="1508" height="727" alt="image" src="https://github.com/user-attachments/assets/4867e602-4918-4e18-9ed6-30ffdc36c2b6" />

How to explain this to anyone (including a high‑schooler)

Black dashed line      the real bits: 101010…
Blue & magenta curves  smooth “confidence meters” for MARK and SPACE
Green curve            subtraction:
                       positive = MARK, negative = SPACE
Black dots             exact moments we decide the bit (middle of each bit)
Gray horizontal line (zero)  decision threshold

first 13 min of https://youtu.be/w1SB9Ry8_Jg?si=cyaR_r0YtE0NOga2 
shows clock pulse passing through a RC circuit, which is very similar to rx preamble for clock recovery



Question: For fsk decoding should I use low pass filter after mixing down or use goertzel filter for mark and space tones ?

Both methods work, but the choice depends on your hardware and how much noise you expect. For most modern digital projects, using two Goertzel filters (or Bandpass Filters) is generally easier and more efficient for detecting discrete "Mark" and "Space" tones.
1. Goertzel Filter Approach (Non-Coherent Detection)
 This is widely used for FSK because it acts as two separate, very narrow bandpass filters tuned exactly to your Mark and Space frequencies.
 How it works: You run two Goertzel filters in parallel on every block of N samples. One is tuned to $\(f_{mark}\)$ and the other to $\(f_{space}\)$.
 Decision: You simply compare the output magnitude (energy) of the two filters. If $\(E_{mark} > E_{space}\)$, you have a binary 1; otherwise, it's a 0.
 Best for: Low-power microcontrollers (like Arduino/ESP32) and systems where you don't want to deal with complex mixing or phase synchronization.
2. Mixer + Low-Pass Filter (Non-Coherent/Quadrature Detection)
 This is the "traditional" radio approach. It converts the frequency changes into a varying voltage (baseband signal).
 How it works: You mix (multiply) the incoming signal with a local oscillator. This shifts the signal down so the "Mark" and "Space" are centered around zero or a very low IF. A Low-Pass Filter (LPF) is then used to remove the high-frequency "sum" products and noise, leaving only the clean data bits.
 Best for: High-performance hardware (like SDRs or specialized FSK modem chips) and environments with high noise, as it can be more robust if you implement it as a "Quadrature Demodulator".


Question: Why Mixer + Low-Pass Filter is Non-Coherent?

If your mixer uses a local oscillator that is not phase-locked to the incoming carrier, it is a non-coherent receiver.The Problem: Without phase-locking, the output of a single mixer might disappear if the oscillators are 90° out of phase. The Solution: Engineers use Quadrature (I/Q) detection. They mix the signal with two local oscillators (Sine and Cosine) and then low-pass filter both.Result: By calculating the magnitude ($\(\sqrt{I^{2}+Q^{2}}\)$), you recover the signal regardless of the phase. This is a "non-coherent" process because you don't care about the phase; you only care about the energy.
