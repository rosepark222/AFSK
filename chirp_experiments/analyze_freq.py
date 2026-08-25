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
plt.figure(figsize=(12, 4))
plt.plot(times, peak_freqs, marker='o', linewidth=1.5)
plt.xlabel("Time (s)")
plt.ylabel("Peak Frequency (Hz)")
plt.title(f"Peak Frequency per 0.1s Window — {filename}")
plt.grid(True)
plt.tight_layout()
plt.savefig("freq_analysis.png", dpi=150)
plt.show()
print("Saved freq_analysis.png")
