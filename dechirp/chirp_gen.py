# chirp_gen.py
import numpy as np

# System Parameters
fs = 44100          # 44.1 kHz Sampling Rate
total_dur = 3.0     # 3 seconds total window
chirp_dur = 1.0     # 1 second chirp duration
f0 = 300.0          # Start frequency
f1 = 1500.0         # End frequency

# Time vector for full 3 seconds
total_samples = int(fs * total_dur)
t_total = np.arange(total_samples) / fs

# Create background factory noise (Wideband)
noise = np.random.normal(0, 0.5, total_samples)

# Create permanent factory motor harmonics (Constant tones)
harmonic1 = 0.4 * np.cos(2 * np.pi * 400.0 * t_total)   # 400 Hz line
harmonic2 = 0.4 * np.cos(2 * np.pi * 1000.0 * t_total)  # 1000 Hz line

# Generate the hidden 1-second SOS Chirp
chirp_start_time = 1.2  # Hidden at exactly 1.2 seconds
chirp_start_idx = int(chirp_start_time * fs)
chirp_end_idx = chirp_start_idx + int(fs * chirp_dur)

t_chirp = np.arange(int(fs * chirp_dur)) / fs
k = (f1 - f0) / chirp_dur  # Chirp rate
# Real-valued chirp signal as received by a single microphone
chirp_signal = np.cos(2 * np.pi * (f0 * t_chirp + 0.5 * k * t_chirp**2))

# Construct full environment file
rx_signal = noise + harmonic1 + harmonic2
rx_signal[chirp_start_idx:chirp_end_idx] += chirp_signal

# Dump to chirp.txt (one sample per line)
print("Writing 3 seconds of simulation data to chirp.txt...")
np.savetxt("chirp.txt", rx_signal, fmt="%.6f")
print("Done! File generated successfully.")

