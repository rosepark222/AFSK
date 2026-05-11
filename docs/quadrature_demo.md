 
------------------------------
## Quadrature Modulation as the fundamental building block of non-coherent Frequency Shift Keying (FSK) receivers  
## 1. Signal Representations
A signal with an arbitrary phase $\phi$ and carrier frequency $\omega_c$ can be represented in two equivalent ways.
## Polar Coordinates (Magnitude & Phase)
This is how we often visualize a wave:
 
$$x(t)= A\cos(\omega_c t + \phi)$$

## Cartesian Coordinates (I & Q Components)
Using the trigonometric identity $$\cos(\alpha + \beta) = \cos\alpha \cos\beta - \sin\alpha \sin\beta$$, we can rewrite the signal as:
$$x(t) = A \cos(\phi) \cos(\omega_c t) - A \sin(\phi) \sin(\omega_c t)$$
or
$$x(t) = I \cos(\omega_c t) - Q \sin(\omega_c t)$$
Where:

* $I = A \cos(\phi)$ (In-phase component)
* $Q = A \sin(\phi)$ (Quadrature component)

------------------------------
## 2. Quadrature Modulation (Transmitter)
To send a complex signal through a single physical channel (like a speaker), we "pack" the $I$ and $Q$ data onto orthogonal carriers:

   1. Multiply $I$ by $\cos(\omega_c t)$.
   2. Multiply $Q$ by $\sin(\omega_c t)$.
   3. Subtract them to create the Real transmitted wave:
   $$x_{TX}(t) = I \cos(\omega_c t) - Q \sin(\omega_c t)$$

------------------------------
## 3. Quadrature Demodulation (Receiver)
To recover the original data at the receiver, we split the incoming signal into two paths and multiply them by local oscillators.
## The In-phase Path ($I$)
Multiply the received signal by $\cos(\omega_c t)$:
$$I_{raw} = [I \cos(\omega_c t) - Q \sin(\omega_c t)] \cdot \cos(\omega_c t)$$
$$I_{raw} = I \cos^2(\omega_c t) - Q \sin(\omega_c t)\cos(\omega_c t)$$
Using identities $$\cos^2\theta = \frac{1+\cos(2\theta)}{2}$$ and $$\sin\theta\cos\theta = \frac{\sin(2\theta)}{2}$$:
$$I_{raw} = \underbrace{\frac{I}{2}}_{\text{Baseband (DC)}} + \underbrace{\frac{I}{2}\cos(2\omega_c t) - \frac{Q}{2}\sin(2\omega_c t)}_{\text{High Frequency Junk}}$$
## The Quadrature Path ($Q$)
Multiply the received signal by $-\sin(\omega_c t)$:
$$Q_{raw} = [I \cos(\omega_c t) - Q \sin(\omega_c t)] \cdot (-\sin(\omega_c t))$$
$$Q_{raw} = \underbrace{\frac{Q}{2}}_{\text{Baseband (DC)}} + \underbrace{\text{High Frequency Junk (at } 2\omega_c \text{)}}_{\dots}$$
------------------------------
## 4. The Role of the Low Pass Filter (LPF)
In the physical world (using real numbers), multiplication always creates a "sum" frequency ($2\omega_c$). We must apply an LPF to isolate the baseband data:
$$I_{final} = \text{LPF}\{I_{raw}\} = \frac{I}{2}$$
$$Q_{final} = \text{LPF}\{Q_{raw}\} = \frac{Q}{2}$$
------------------------------
## 5. Non-Coherent Detection (Energy)
In a non-coherent receiver, the absolute phase $\phi$ is unknown or shifting. To detect the signal regardless of the phase, we calculate the Magnitude or Energy:
Recovered Amplitude:
$$A \propto \sqrt{I_{final}^2 + Q_{final}^2}$$
Signal Energy:
$$E \propto I_{final}^2 + Q_{final}^2$$

remember that 
* $I = A \cos(\phi)$ (In-phase component)  
* $Q = A \sin(\phi)$ (Quadrature component)  
and because $\cos^2\phi + \sin^2\phi = 1$, this calculation removes the phase dependency, allowing you to "see" the signal even if the phase is spinning.
------------------------------


