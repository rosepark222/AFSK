

## Quadrature Modulation as the Fundamental Building Block of Non-Coherent FSK Receivers

## 1. Signal Representations

A signal with an arbitrary phase $\phi$ and carrier frequency $\omega_c$ can be represented in two equivalent ways.

### Polar Coordinates (Magnitude and Phase)

This is how we often visualize a wave:

$$
x(t) = A\cos(\omega_c t + \phi)
$$

### Cartesian Coordinates (I and Q Components)

Using the trigonometric identity

$$
\cos(\alpha + \beta) = \cos\alpha\cos\beta - \sin\alpha\sin\beta
$$

we can rewrite the signal as

$$
x(t) = A\cos(\phi)\cos(\omega_c t) - A\sin(\phi)\sin(\omega_c t)
$$

or

$$
x(t) = I\cos(\omega_c t) - Q\sin(\omega_c t)
$$

where the in-phase and quadrature components are

$$
I = A\cos(\phi), \qquad Q = A\sin(\phi)
$$

---

## 2. Quadrature Modulation (Transmitter)

To send a complex signal through a single physical channel, such as a speaker, we pack the $I$ and $Q$ data onto orthogonal carriers:

1. Multiply $I$ by $\cos(\omega_c t)$.
2. Multiply $Q$ by $\sin(\omega_c t)$.
3. Subtract the two terms to create the real transmitted wave:

$$
x_{TX}(t) = I\cos(\omega_c t) - Q\sin(\omega_c t)
$$

---

## 3. Why Mix Up and Down?

<img width="800" height="547" alt="image" src="https://github.com/user-attachments/assets/8ecfead5-9fab-40ce-bde5-6672881f7d8a" />

https://circuitcellar.com/cc-blog/fundamentals-of-iq-signals/

Mixing up (up-conversion): take a low-frequency baseband signal and multiply it by a high-frequency carrier, $\cos(\omega t)$, to shift it into the radio or audio range for transmission.

Mixing down (down-conversion): take a high-frequency received signal and multiply it by a local oscillator at the same frequency. This shifts the information back down to 0 Hz so it can be processed.

Why do we mix it up to a high frequency and then mix it down again? Why not just send the baseband signal directly?

Think of the baseband signal as a person and the high-frequency carrier as an airplane. A person cannot travel across the ocean on their own, but they can ride in an airplane to make the trip. This is why we need the "flight":

1. The antenna problem: baseband signals have wavelengths that are far too long to be practical.
2. The crowded-room problem: multiple signals would interfere with one another if they all shared the same low-frequency band.
3. The receive problem: our electronics cannot process a 100 MHz waveform directly in the same way they can process a low-frequency baseband signal.

Summary:

- Mix up: to move the signal to a practical frequency band and avoid interference.
- Mix down: to remove the high-frequency carrier and recover the original information.

---

## 4. Quadrature Demodulation (Receiver)

To recover the original data at the receiver, we split the incoming signal into two paths and multiply them by local oscillators.

### The In-Phase Path ($I$)

Multiply the received signal by $\cos(\omega_c t)$:

$$
I_{raw} = x_{RX}(t)\cos(\omega_c t)
$$

$$
I_{raw} = [I\cos(\omega_c t) - Q\sin(\omega_c t)]\cos(\omega_c t)
$$

$$
I_{raw} = I\cos^2(\omega_c t) - Q\sin(\omega_c t)\cos(\omega_c t)
$$

Using the identities

$$
\cos\theta\cos\theta = \frac{1}{2}[1 + \cos(2\theta)]
$$

and

$$
\sin\theta\cos\theta = \frac{\sin(2\theta)}{2}
$$

we get

$$
I_{raw} = \frac{I}{2} + \frac{I}{2}\cos(2\omega_c t) - \frac{Q}{2}\sin(2\omega_c t)
$$

This is

$$
I_{raw} = \frac{I}{2} + \text{high-frequency terms at } 2\omega_c
$$

### The Quadrature Path ($Q$)

Multiply the received signal by $-\sin(\omega_c t)$:

$$
Q_{raw} = [I\cos(\omega_c t) - Q\sin(\omega_c t)](-\sin(\omega_c t))
$$

This produces

$$
Q_{raw} = \frac{Q}{2} + \text{high-frequency terms at } 2\omega_c
$$

---

## 5. The Role of the Low-Pass Filter (LPF)

In the real world, multiplication creates a high-frequency term at $2\omega_c$. We apply a low-pass filter to isolate the baseband data:

$$
I_{final} = \text{LPF}\{I_{raw}\} = \frac{I}{2}
$$

$$
Q_{final} = \text{LPF}\{Q_{raw}\} = \frac{Q}{2}
$$

---

## 6. Non-Coherent Detection (Energy)

In a non-coherent receiver, the absolute phase $\phi$ is unknown or drifting. To detect the signal regardless of phase, we calculate the magnitude or energy:

Recovered amplitude:

$$
A \propto \sqrt{I_{final}^2 + Q_{final}^2}
$$

Signal energy:

$$
E \propto I_{final}^2 + Q_{final}^2
$$

Remember that

- $I = A\cos(\phi)$
- $Q = A\sin(\phi)$

and since

$$
\cos^2\phi + \sin^2\phi = 1
$$

this calculation removes the phase dependency and lets us detect the signal even when the phase is unknown.

---

## 7. How Is This Related to FSK?

The transmitter sends two possible signals depending on the bit value:

For bit = 1 (mark):

$$
x_{mark}(t) = A\cos(2\pi f_1 t + \phi_1)
$$

For bit = 0 (space):

$$
x_{space}(t) = A\cos(2\pi f_0 t + \phi_0)
$$

With a 100 baud rate, the duration of each bit is 10 ms. Therefore, transmitting the data "101" corresponds to sending

$$
x_{mark}(t),\ x_{space}(t),\ x_{mark}(t)
$$

sequentially, with each signal lasting 10 ms and no gap between them.

At any instant, the received signal is either $x_{mark}(t)$ or $x_{space}(t)$, not both. We do not know which one has been transmitted, so we check both and decide which bit was sent.

The receiver measures signal energy at $f_1$ and $f_0$ using two separate quadrature demodulations:

$$
A_{mark} = \sqrt{I_{mark}^2 + Q_{mark}^2}
$$

$$
A_{space} = \sqrt{I_{space}^2 + Q_{space}^2}
$$

FSK decision:

- if $A_{mark} > A_{space}$, the receiver decides the bit is 1
- if $A_{mark} < A_{space}$, the receiver decides the bit is 0

Remember that each bit lasts for a fixed duration. For example, 100 baud gives 10 ms per bit and 1000 baud gives 1 ms per bit. The receiver should make the decision near the center of the bit interval, not at the transition.

For this reason, clock recovery is important; see [clock_recovery](clock_recovery.md).

---

## References

$$
\cos(\alpha + \beta) = \cos\alpha\cos\beta - \sin\alpha\sin\beta
$$

This is useful for expressing a signal with arbitrary phase in terms of $I$ and $Q$ components.

$$
\cos\alpha\cos\beta = \frac{1}{2}[\cos(\alpha + \beta) + \cos(\alpha - \beta)]
$$

This identity is useful for modulation and demodulation.



