import sys
import numpy as np
import matplotlib.pyplot as plt

# ─── Parameters ────────────────────────────────────────────
SAMPLE_RATE   = 44100
WINDOW_SEC    = 0.1
WINDOW_SIZE   = int(SAMPLE_RATE * WINDOW_SEC)  # 4410 samples per window

# ─── Load signal ───────────────────────────────────────────
if len(sys.argv) < 2:
    print("Usage: python analyze.py <filename of one number per line format>")
    sys.exit(1)

filename = sys.argv[1]

with open(filename, "r") as f:
    samples = np.array([float(line.strip()) for line in f if line.strip()])

print(f"Loaded {len(samples)} samples = {len(samples)/SAMPLE_RATE:.2f} seconds")

# ─── Analyze frequency per window ──────────────────────────
n_windows  = len(samples) // WINDOW_SIZE
times      = []
peak_freqs = []

for i in range(n_windows):
    start  = i * WINDOW_SIZE
    end    = start + WINDOW_SIZE
    window = samples[start:end]

    hann     = np.hanning(len(window))
    windowed = window * hann

    fft_mag  = np.abs(np.fft.rfft(windowed))
    freqs    = np.fft.rfftfreq(len(windowed), d=1.0/SAMPLE_RATE)

    peak_idx  = np.argmax(fft_mag)
    peak_freq = freqs[peak_idx]
    peak_time = (start + WINDOW_SIZE / 2) / SAMPLE_RATE

    times.append(peak_time)
    peak_freqs.append(peak_freq)

    print(f"t={peak_time:.2f}s  peak_freq={peak_freq:.1f} Hz")

# ─── Plot ──────────────────────────────────────────────────
# Trim samples to an integer number of windows so the time axis aligns with frequency windows
trim_length = n_windows * WINDOW_SIZE
samples_trim = samples[:trim_length]
t_samples = np.arange(len(samples_trim)) / SAMPLE_RATE

fig, (ax_wave, ax_freq) = plt.subplots(2, 1, sharex=True, figsize=(12, 6),
                                       gridspec_kw={"height_ratios": [1, 1]})

# Time-domain waveform (top)
ax_wave.plot(t_samples, samples_trim, color="C0", linewidth=0.7)
ax_wave.set_ylabel("Amplitude")
ax_wave.set_title(f"Time-domain Waveform — {filename}")
ax_wave.grid(True)

# Optionally mark window boundaries (uncomment if you want visual window markers)
# for i in range(n_windows + 1):
#     ax_wave.axvline(i * WINDOW_SEC, color="gray", linestyle=":", linewidth=0.5)
#     ax_freq.axvline(i * WINDOW_SEC, color="gray", linestyle=":", linewidth=0.5)

# Frequency peaks per window (bottom)
ax_freq.plot(times, peak_freqs, marker='o', linewidth=1.5, color="C1")
ax_freq.set_xlabel("Time (s)")
ax_freq.set_ylabel("Peak Frequency (Hz)")
ax_freq.set_title(f"Peak Frequency per {WINDOW_SEC:.2f}s Window")
ax_freq.grid(True)

plt.tight_layout()
outname = "freq_analysis.png"
plt.savefig(outname, dpi=150)
plt.show()
print(f"Saved {outname}")
