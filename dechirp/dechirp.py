# dechirp.py
import numpy as np
import matplotlib.pyplot as plt
import sys

# System Parameters
fs = 44100
chirp_dur = 1.0
f0 = 300.0
f1 = 1500.0
k = (f1 - f0) / chirp_dur  # Chirp rate (1200 Hz/s)
fft_size = 1024
# ─── Load signal ───────────────────────────────────────────
if len(sys.argv) < 2:
    print("Usage: python analyze.py <filename>")
    sys.exit(1)

filename = sys.argv[1]

print("Loading chirp.txt...")
rx_signal = np.loadtxt(filename) #"chirp.txt")
total_samples = len(rx_signal)

# 1. Generate Local Complex Conjugate Reference Chirp (1 second long)
t_ref = np.arange(int(fs * chirp_dur)) / fs
phase_ref = 2 * np.pi * (f0 * t_ref + 0.5 * k * t_ref**2)
# Negative sine forces the complex conjugate s*(t)
s_conj = np.cos(phase_ref) - 1j * np.sin(phase_ref)

# 2. Sliding Window Setup (Step every 0.1 seconds)
step_size = int(0.1 * fs)  # 4410 samples
window_samples = int(chirp_dur * fs)  # 44100 samples

time_steps = []
npar_history = []
saved_fft_spectrum = None
best_step_time = 0
max_npar = 0

# Loop through the 3-second file
for start_idx in range(0, total_samples - window_samples + 1, step_size):
    current_time = start_idx / fs
    time_steps.append(current_time)
    
    # Extract 1-second chunk of raw received data
    r_t = rx_signal[start_idx:start_idx + window_samples]
    
    # MIX STEP: Element-by-element complex multiplication
    y_dechirp = r_t * s_conj
    
    # Slice out a 1024-sample block from the dechirped vector to run FFT
    # (Using the middle of the window to avoid edge transitions)
    # fft_block = y_dechirp[window_samples//2 : window_samples//2 + fft_size]
    fft_block = y_dechirp[0 : fft_size]
    
    # Apply a Hanning window to prevent spectral leakage
    fft_block = fft_block * np.hanning(fft_size)
    
    # Run Complex FFT
    fft_out = np.fft.fft(fft_block)
    fft_mag = np.abs(fft_out)
    
    # Compute Normalized Peak-to-Average Ratio (NPAR) to eliminate factory noise bias
    max_val = np.max(fft_mag)
    mean_val = np.mean(fft_mag)
    npar = max_val / mean_val
    npar_history.append(npar)
    
    # Capture the best aligned profile for visualization
    if npar > max_npar:
        max_npar = npar
        best_step_time = current_time
        saved_fft_spectrum = np.fft.fftshift(fft_mag) # Center 0 Hz

# 3. Plotting the Visualizations
frequencies = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1/fs))

plt.figure(figsize=(12, 5))

# Plot A: The Tracking curve (How the Teensy spots the chirp start boundary)
plt.subplot(1, 2, 1)
plt.plot(time_steps, npar_history, 'o-', color='tab:blue', linewidth=2)
#plt.axvline(x=1.2, color='tab:red', linestyle='--', label='True Chirp Onset (1.2s)')
plt.title("Sliding Window Timing Alignment Tracker")
plt.xlabel("Window Start Timestamp (Seconds)")
plt.ylabel("FFT Peak Strength (NPAR)")
plt.grid(True)
plt.legend()

# Plot B: The Complex FFT output at optimal alignment
plt.subplot(1, 2, 2)
plt.plot(frequencies, saved_fft_spectrum, color='tab:orange')
plt.title(f"1024-Point Dechirped FFT Spectrum\n(Best Alignment at t = {best_step_time:.1f}s)")
plt.xlabel("Beat Frequency Bin (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)

plt.tight_layout()
print(f"Chirp detected! Best match found at processing window: {best_step_time:.1f} seconds.")
plt.show()

