import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# Parameters
# ============================================================
fs = 441000            # try 1000 or 441000
baud = 300
SAMPLES_PER_BIT = int(fs / baud)
duration = 0.06

t = np.arange(0, duration, 1/fs)

# ============================================================
# TX bits (mid-bit transitions)
# ============================================================
bits = np.zeros_like(t)
for i in range(len(t)):
    bit_index = (i + SAMPLES_PER_BIT//2) // SAMPLES_PER_BIT
    bits[i] = 1 if (bit_index % 2 == 0) else 0

mark_in  = bits
space_in = 1 - bits

# ============================================================
# LPF (CORRECTLY SCALED)
# ============================================================
LPF_FC = 200.0  # Hz
alpha = 1.0 - np.exp(-2*np.pi*LPF_FC / fs)
print("alpha", alpha)

mark_lpf  = np.zeros_like(t)
space_lpf = np.zeros_like(t)

for i in range(1, len(t)):
    mark_lpf[i]  = mark_lpf[i-1]  + alpha*(mark_in[i]  - mark_lpf[i-1])
    space_lpf[i] = space_lpf[i-1] + alpha*(space_in[i] - space_lpf[i-1])

diff = mark_lpf - space_lpf

# ============================================================
# Clock recovery (fs-independent)
# ============================================================
EL_STEP = max(1, int(0.05 * SAMPLES_PER_BIT))  # 5% of bit
#EL_STEP = 2
print("Early Late STEP", EL_STEP)

clockCounter = 500
lastSoft = -1
clock_phase = np.zeros_like(t)
sample_points = []

for i in range(len(t)):
    softBit = 1 if diff[i] > 0 else 0

    if lastSoft != -1 and softBit != lastSoft:
        mid = SAMPLES_PER_BIT // 2
        if clockCounter < mid:
            clockCounter += EL_STEP  # bit transition (edge) is early (left) -> clock is too slow ->  speed up the clock
        else:
            clockCounter -= EL_STEP  # bit transition (edge) is late (right) -> clock is too fast ->  slow down the clock

    lastSoft = softBit

    clockCounter += 1
    if clockCounter >= SAMPLES_PER_BIT:
        clockCounter = 0
        sample_points.append(i)

    clock_phase[i] = clockCounter / SAMPLES_PER_BIT

# ============================================================
# Single plot
# ============================================================
plt.figure(figsize=(11,4))

plt.step(t*1000, mark_in*0.6, where='post', linestyle=':', label='mark_in')
plt.step(t*1000, space_in*0.6, where='post', linestyle=':', label='space_in')

plt.plot(t*1000, mark_lpf, label='MARK LPF', color='blue')
plt.plot(t*1000, space_lpf, label='SPACE LPF', color='red')
plt.plot(t*1000, diff, label='MARK - SPACE', color='green')
plt.plot(t*1000, clock_phase*0.5, label='clockCounter', color='purple')

plt.scatter(np.array(sample_points)/fs*1000,
            diff[sample_points], color='black')

plt.axhline(0, color='gray', linestyle='--')
plt.legend()
plt.grid(True)
plt.title(f'Robust FSK RX (fs = {fs} Hz)')
plt.xlabel('Time (ms)')
plt.show()
