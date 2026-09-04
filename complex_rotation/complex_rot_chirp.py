import numpy as np
import matplotlib.pyplot as plt

# Parameters
fs = 44100.0
chirp_duration = 1.0
duration_begin = 0.8
duration_end = 0.9
f0 = 300.0
f1 = 1500.0

if not 0.0 <= duration_begin < duration_end <= chirp_duration:
    raise ValueError("Display interval must be inside the chirp duration")

begin_sample = int(fs * duration_begin)
end_sample = int(fs * duration_end)
sample_count = end_sample - begin_sample
full_sample_count = end_sample
full_time = np.arange(full_sample_count) / fs
time = full_time[begin_sample:end_sample]
chirp_rate = (f1 - f0) / chirp_duration

# Exact complex reference, used only as a comparison signal.
# np.exp(1j * phase) is equivalent to cos(phase) + 1j * sin(phase).
phase = 2.0 * np.pi * (f0 * full_time + 0.5 * chirp_rate * full_time**2)
original_complex = np.exp(1j * phase)
original_real = np.real(original_complex)

# More efficient variant:
# z[n + 1] = z[n] * rotation[n]
# rotation[n + 1] = rotation[n] * rotation_increment
phase_step = 2.0 * np.pi * f0 / fs
phase_step_increment = 2.0 * np.pi * chirp_rate / (fs * fs)

complex_chirp_full = np.zeros(full_sample_count, dtype=np.complex128)
complex_chirp_full[0] = 1.0 + 0.0j

rotation = np.exp(1j * phase_step)
rotation_increment = np.exp(1j * phase_step_increment)

for sample_index in range(full_sample_count - 1):
    complex_chirp_full[sample_index + 1] = complex_chirp_full[sample_index] * rotation
    rotation *= rotation_increment

complex_chirp = complex_chirp_full[begin_sample:end_sample]
original_complex = original_complex[begin_sample:end_sample]
original_real = original_real[begin_sample:end_sample]

complex_error = np.abs(complex_chirp - original_complex)
real_error = np.abs(np.real(complex_chirp) - original_real)
complex_magnitude = np.abs(complex_chirp)

print("Maximum complex-reference error:")
print(np.max(np.abs(complex_error)))
print()
print("RMS real-waveform error:")
print(np.sqrt(np.mean(real_error**2)))
print()
print("Maximum magnitude deviation from 1:")
print(np.max(np.abs(complex_magnitude - 1.0)))

# The real part is the original real-valued chirp. The magnitude is its
# complex envelope and should remain close to one, so they are plotted apart.
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time, original_real, label="Original real chirp")
plt.plot(time, np.real(complex_chirp), "--", label="Complex chirp real part")
plt.ylabel("Amplitude")
plt.title("Real waveform comparison")
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(time, complex_magnitude, color="tab:orange")
plt.axhline(1.0, color="black", linestyle="--", linewidth=1)
plt.ylabel("Magnitude")
plt.title("Complex chirp magnitude")
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(time, real_error, color="tab:red")
error_limit = np.max(real_error)
error_padding = error_limit * 0.1 if error_limit > 0.0 else 1.0e-15
plt.ylim(0.0, error_limit + error_padding)
plt.ticklabel_format(axis="y", style="scientific", scilimits=(0, 0))
plt.xlabel("Time (seconds)")
plt.ylabel("Error")
plt.title("Real-part error")
plt.grid(True)

plt.tight_layout()
plt.show()