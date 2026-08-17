 
------------------------------
## Quadrature Modulation as the fundamental building block of non-coherent Frequency Shift Keying (FSK) receivers  
## 1. Signal Representations
A signal with an arbitrary phase $\phi$ and carrier frequency $\omega_c$ can be represented in two equivalent ways.
## Polar Coordinates (Magnitude & Phase)
This is how we often visualize a wave:
 
$$x(t)= A\cos(\omega_c t + \phi)$$

## Cartesian Coordinates (I & Q Components)
Using the trigonometric identity  

$$\cos(\alpha + \beta) = \cos\alpha \cos\beta - \sin\alpha \sin\beta$$  

, we can rewrite the signal as:  

$$x(t) = A \cos(\phi) \cos(\omega_c t) - A \sin(\phi) \sin(\omega_c t)$$

or

$$x(t) = I \cos(\omega_c t) - Q \sin(\omega_c t)$$  

Where In-phase and quadrature components are  

$$I = A \cos(\phi)$$   

$$Q = A \sin(\phi)$$

  

------------------------------
## 2. Quadrature Modulation (Transmitter)
To send a complex signal through a single physical channel (like a speaker), we "pack" the $I$ and $Q$ data onto orthogonal carriers:

   1. Multiply $I$ by $\cos(\omega_c t)$.
   2. Multiply $Q$ by $\sin(\omega_c t)$.
   3. Subtract them to create the Real transmitted wave:
   $$x_{TX}(t) = I \cos(\omega_c t) - Q \sin(\omega_c t)$$

------------------------------
## 3. Why mix up and down?

<img width="800" height="547" alt="image" src="https://github.com/user-attachments/assets/8ecfead5-9fab-40ce-bde5-6672881f7d8a" />

https://circuitcellar.com/cc-blog/fundamentals-of-iq-signals/


Mixing Up (Up-conversion): You take your low-frequency information (the baseband signal) and multiply it by a high-frequency carrier, $\cos(\omega t)$, to shift it into the radio or audio frequency range for transmission.  

Mixing Down (Down-conversion): You take that high-frequency received signal and multiply it by a local oscillator at the same frequency. This shifts the information back down to 0 Hz (baseband) so you can process it.  

Why do we have to mix it up and place it to high frequency, then mix it down ? Why not just send baseband signal?  

Think of the baseband signal (like your voice or music) as a person and the high-frequency carrier as an airplane. A person can't fly across the ocean on their own, but they can get inside an airplane to make the trip. Here is why we need that "flight":  
1. The Antenna Problem (The "Giant Ear" issue)  
In physics, the size of an antenna needs to be related to the length of the wave it’s sending. 
Baseband: Low-frequency sound waves (like 100 Hz) have massive wavelengths—miles long. To send your voice directly through the air as a radio wave, you would need an antenna 150 miles tall.  
High Frequency: By "mixing up" to a high frequency (like 100 MHz), the waves become tiny. Now, your phone can have a tiny antenna just a few inches long that fits in your pocket.  
2. The "Crowded Room" Problem (Interference)  
Imagine a room where 50 people are all shouting at the same time. You wouldn’t be able to understand anyone. This is what would happen if every radio station and cell phone sent their "baseband" signal directly.  
The Solution: By "mixing up," we give every person their own "floor" in a skyscraper. Station A is at 90 MHz. Station B is at 101 MHz. They can all talk at once without bumping into each other.  
3. Why mix it back down?  
Our ears (and our computers) can’t "hear" at 100 million cycles per second—that's way too fast. The Airplane Analogy: Once the airplane lands at the destination, the person has to get out of the plane to walk around. Mixing down is just "taking the person off the plane." It removes the high-speed carrier so we are left with the original, slow voice or music that we can actually understand.  

Summary  
Mix Up: To make the signal "fit" onto small antennas and to avoid everyone talking over each other.  
Mix Down: To turn that high-speed "transport" wave back into something our speakers and ears can actually use.  




------------------------------
## 4. Quadrature Demodulation (Receiver)
To recover the original data at the receiver, we split the incoming signal into two paths and multiply them by local oscillators. Before math, let's put down some lingos:  



## The In-phase Path ($I$)
Multiply the received signal by $\cos(\omega_c t)$:

$$I_{raw} = x_{RX}(t) \cdot \cos(\omega_c t)$$

$$I_{raw} = [I \cos(\omega_c t) - Q \sin(\omega_c t)] \cdot \cos(\omega_c t)$$

$$I_{raw} = I \cos^2(\omega_c t) - Q \sin(\omega_c t)\cos(\omega_c t)$$

Using identities  

$$\cos\theta \cos\theta = \frac{1}{2} [ 1 + \cos(2\theta) ]$$ and $$\sin\theta\cos\theta = \frac{\sin(2\theta)}{2}$$

then  

$$I_{raw} =  \frac{I}{2}  +  \frac{I}{2}\cos(2\omega_c t) - \frac{Q}{2}\sin(2\omega_c t)$$  

which is  

$$I_{raw} =  \frac{I}{2}  +  \text{High Frequency Junk (at } 2\omega_c \text{)} $$  


## The Quadrature Path ($Q$)
Multiply the received signal by $-\sin(\omega_c t)$:  

$$Q_{raw} = [I \cos(\omega_c t) - Q \sin(\omega_c t)] \cdot (-\sin(\omega_c t))$$  

$$Q_{raw} =  \frac{Q}{2}  +  \text{High Frequency Junk (at } 2\omega_c \text{)} $$  

------------------------------
## 5. The Role of the Low Pass Filter (LPF)
In the physical world (using real numbers), multiplication always creates a "sum" frequency ($2\omega_c$). We must apply an LPF to isolate the baseband data:

$$I_{final} = \text{LPF}\{I_{raw}\} = \frac{I}{2}$$  

$$Q_{final} = \text{LPF}\{Q_{raw}\} = \frac{Q}{2}$$  

------------------------------
## 6. Non-Coherent Detection (Energy)
In a non-coherent receiver, the absolute phase $\phi$ is unknown or shifting. To detect the signal regardless of the phase, we calculate the Magnitude or Energy:
Recovered Amplitude:  

$$A \propto \sqrt{I_{final}^2 + Q_{final}^2}$$  

Signal Energy:  

$$E \propto [I_{final}^2 + Q_{final}^2]$$  


remember that 
* $I = A \cos(\phi)$ (In-phase component)  
* $Q = A \sin(\phi)$ (Quadrature component)  
and because $\cos^2\phi + \sin^2\phi = 1$, this calculation removes the phase dependency, allowing you to "see" the signal even if the phase is unknown.
------------------------------
## 7. How is it related to FSK?
The tramitter sends two signals depending on the data  
for bit=1 (mark)  

$$x_{mark}(t)= A\cos(2\pi f_{1} t   + \phi_{1}) $$  

for bit=0 (space)  

$$x_{space}(t)= A\cos(2\pi f_{0} t   + \phi_{0}) $$  



With a 100 baud rate, the duration of each bit is 10 ms. Therefore, transmitting the data "101" corresponds to sending  

$$x_{mark}(t), x_{space}(t), x_{mark}(t)$$   

sequentially, with each signal lasting 10 ms and no time gap between them.

At a given time, the received signal is either $x_{mark}(t)$ or $x_{space}(t)$, not both. However, we do not know which one is transmitted, we check both and decide which bit was transmitted.  

Receiver measures the signal energy at $f_{1}$ and $f_{0}$ using two separate quadrature demodulations   
 
$$A_{mark} = \sqrt{I_{mark}^2 + Q_{mark}^2}$$
$$A_{space} = \sqrt{I_{space}^2 + Q_{space}^2}$$

FSK decision:  
if $A_{mark} > A_{space}$, the receiver decides that the transmitted bit=1      
if $A_{mark} < A_{space}$, the receiver decides that the transmitted bit=0    

Remember, each bit lasts for its duration (e.g., 10ms for 100 baud, 1ms for 1000 baud)  
Thus it is important to know when each bit starts and ends, because receiver does not want to make bit decision at the bit transition. It is best to make a decision at the center of bit duration.  

For this, we need to learn about clock recovery method [clock_recovery](clock_recovery.md)



## References:

$$\cos(\alpha + \beta) = \cos\alpha \cos\beta - \sin\alpha \sin\beta$$      
useful to express signal with arbitrary phase to I and Q components   

$$\cos\alpha \cos\beta = \frac{1}{2}[\cos(\alpha + \beta) + \cos(\alpha - \beta)]$$    
useful for modulation/demodulation   



