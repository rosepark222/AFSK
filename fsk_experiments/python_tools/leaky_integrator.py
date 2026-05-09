import numpy as np
import matplotlib.pyplot as plt

# Sampling parameters
fs = 1000            # samples/sec
duration = 0.3       # seconds
t = np.arange(0, duration, 1/fs)

bit_ms = 10
bit_samples = int(bit_ms * fs / 1000)

# Original bit pattern: 101010...
bits = np.zeros_like(t)
for i in range(len(t)):
    bits[i] = 1 if ((i // bit_samples) % 2 == 0) else 0

# MARK / SPACE presence
mark_in = bits
space_in = 1 - bits

# Leaky integrator (LPF)
alpha = 0.03
mark_lpf = np.zeros_like(t)
space_lpf = np.zeros_like(t)

for i in range(1, len(t)):
    mark_lpf[i]  = mark_lpf[i-1]  + alpha * (mark_in[i]  - mark_lpf[i-1])
    space_lpf[i] = space_lpf[i-1] + alpha * (space_in[i] - space_lpf[i-1])

# Decision metric
diff = mark_lpf - space_lpf

# Sampling instants (mid‑bit)
num_bits = int(duration * fs / bit_samples)
sample_indices = [
    i * bit_samples + bit_samples // 2
    for i in range(num_bits)
    if i * bit_samples + bit_samples // 2 < len(t)
]

# Plot
fig, ax1 = plt.subplots(figsize=(11,5))

ax1.plot(t*1000, mark_lpf, label='MARK LPF', linewidth=2)
ax1.plot(t*1000, space_lpf, label='SPACE LPF', linewidth=2)
ax1.plot(t*1000, diff, label='MARK − SPACE (decision)', color='green', linewidth=2)

# Sample points
ax1.scatter(
    np.array(sample_indices)/fs*1000,
    diff[sample_indices],
    color='black',
    zorder=5,
    label='Sample points (mid‑bit)'
)

ax1.axhline(0, color='gray', linestyle='--')
ax1.set_xlabel('Time (ms)')
ax1.set_ylabel('Envelope / Difference')
ax1.grid(True)

# Original bits overlay
ax2 = ax1.twinx()
ax2.step(
    t*1000, bits,
    where='post',
    linestyle='--',
    color='black',
    label='Original bits (101010)'
)
ax2.set_ylabel('Bit value')
ax2.set_ylim(-0.2, 1.2)

# Legend
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

plt.title('FSK Detection: MARK/SPACE LPFs, Difference, and Sampling')
plt.show()