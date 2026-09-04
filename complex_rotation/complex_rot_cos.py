import numpy as np
import matplotlib.pyplot as plt

# ------------------------------------------------------------
# Parameters
# ------------------------------------------------------------

Fs = 44100 # Sampling rate (Hz)
f = 300 # Tone frequency (Hz)
duration = 1.0 # seconds

N = int(Fs * duration)

# ------------------------------------------------------------
# Method A:
# Complex rotation
#
# z[n+1] = z[n] * exp(j*w)
# ------------------------------------------------------------

w = 2 * np.pi * f / Fs

rotator = np.exp(1j * w)

z = np.zeros(N, dtype=np.complex128)

z[0] = 1 + 0j

for n in range(N - 1):
    z[n + 1] = z[n] * rotator

cos_complex_rotation = np.real(z)

# ------------------------------------------------------------
# Method B:
# Direct cosine evaluation
# ------------------------------------------------------------

n = np.arange(N)

cos_direct = np.cos(w * n)

# ------------------------------------------------------------
# Difference
# ------------------------------------------------------------

difference = cos_complex_rotation - cos_direct

print()
print("Maximum absolute error:")
print(np.max(np.abs(difference)))

print()
print("RMS error:")
print(np.sqrt(np.mean(difference**2)))

# ------------------------------------------------------------
# Plot the full requested duration
# ------------------------------------------------------------

samples_to_show = N

plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(cos_complex_rotation[:samples_to_show])
plt.title("A) Complex Rotation")
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(cos_direct[:samples_to_show])
plt.title("B) Direct cos()")
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(difference[:samples_to_show])
plt.title("C) Difference")
plt.grid(True)

plt.tight_layout()
plt.show()

