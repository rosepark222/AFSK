#!/usr/bin/env python3
"""
inspect_fsk.py

Usage:
    python inspect_fsk.py samples.txt [mark_freq] [space_freq] [sample_rate]

- samples.txt: one number per line (same format as analyze_freq.py)
- mark_freq, space_freq: optional FSK tone frequencies in Hz (default: 1200, 2200)
- sample_rate: optional sample rate (default: 44100)
"""
import sys
import numpy as np
import matplotlib.pyplot as plt

try:
    from scipy.signal import spectrogram, hilbert
except Exception as e:
    print("This script requires scipy. Install with: pip install scipy")
    raise

def load_samples(path):
    with open(path, "r") as f:
        return np.array([float(line.strip()) for line in f if line.strip()])

def goertzel_block_energy(x, fs, target_f):
    # Returns magnitude-squared energy for block x at target_f using Goertzel
    N = len(x)
    k = int(0.5 + (N * target_f) / fs)
    w = (2.0 * np.pi * k) / N
    coeff = 2.0 * np.cos(w)
    s0 = s1 = s2 = 0.0
    for n in x:
        s0 = coeff * s1 - s2 + n
        s2, s1 = s1, s0
    real = s1 - s2 * np.cos(w)
    imag = s2 * np.sin(w)
    return real*real + imag*imag

def main():
    if len(sys.argv) < 2:
        print("Usage: python inspect_fsk.py samples.txt [mark_freq] [space_freq] [sample_rate]")
        sys.exit(1)

    filename = sys.argv[1]
    mark_freq = float(sys.argv[2]) if len(sys.argv) > 2 else 1200.0
    space_freq = float(sys.argv[3]) if len(sys.argv) > 3 else 2200.0
    SAMPLE_RATE = int(sys.argv[4]) if len(sys.argv) > 4 else 44100

    samples = load_samples(filename)
    duration = len(samples) / SAMPLE_RATE
    print(f"Loaded {len(samples)} samples = {duration:.3f} s")

    # Parameters tuned to 100 baud (0.01 s per symbol) by default:
    symbol_sec = 0.01
    symbol_len = int(round(symbol_sec * SAMPLE_RATE))  # e.g., 441 at 44.1 kHz

    # Spectrogram params
    nperseg = symbol_len
    noverlap = symbol_len // 2
    nfft = 1024

    # Spectrogram (magnitude)
    f, t_spec, Sxx = spectrogram(samples, fs=SAMPLE_RATE, window='hann',
                                nperseg=nperseg, noverlap=noverlap, nfft=nfft,
                                mode='magnitude', scaling='spectrum')
    Sxx_db = 20 * np.log10(Sxx + 1e-12)

    # Per-symbol Goertzel energy (non-overlapping symbols)
    n_windows = len(samples) // symbol_len
    times = (np.arange(n_windows) * symbol_len + symbol_len/2) / SAMPLE_RATE
    energy_mark = np.zeros(n_windows)
    energy_space = np.zeros(n_windows)
    for i in range(n_windows):
        start = i * symbol_len
        block = samples[start:start + symbol_len] * np.hanning(symbol_len)
        energy_mark[i] = goertzel_block_energy(block, SAMPLE_RATE, mark_freq)
        energy_space[i] = goertzel_block_energy(block, SAMPLE_RATE, space_freq)

    # Instantaneous frequency via Hilbert
    analytic = hilbert(samples)
    phase = np.unwrap(np.angle(analytic))
    inst_freq = np.diff(phase) / (2.0 * np.pi) * SAMPLE_RATE  # len = len(samples)-1
    t_inst = (np.arange(len(inst_freq)) + 0.5) / SAMPLE_RATE

    # Plotting
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(6, 1)

    ax_wave = fig.add_subplot(gs[0:2, 0])
    ax_spec = fig.add_subplot(gs[2:5, 0], sharex=ax_wave)
    ax_energy = fig.add_subplot(gs[5, 0], sharex=ax_wave)

    # Waveform
    t_samples = np.arange(len(samples)) / SAMPLE_RATE
    ax_wave.plot(t_samples, samples, color='C0', linewidth=0.6)
    ax_wave.set_ylabel('Amplitude')
    ax_wave.set_title(f"Waveform + Spectrogram + Goertzel energy — {filename}")
    ax_wave.grid(True)

    # Spectrogram
    Sxx_norm = Sxx / (np.max(Sxx) + 1e-20)
    Sxx_db = 20 * np.log10(Sxx_norm + 1e-12)   # now values are <= 0 dB
    Sxx_db = np.clip(Sxx_db, -80, 0)           # show only top 80 dB of dynamic range
    pcm = ax_spec.pcolormesh(t_spec, f, Sxx_db, shading='gouraud', cmap='viridis', vmin=-80, vmax=0)
    # pcm = ax_spec.pcolormesh(t_spec, f, Sxx_db, shading='gouraud', cmap='viridis')

    ax_spec.set_ylabel('Frequency (Hz)')
    ax_spec.set_ylim(0, min(5000, SAMPLE_RATE/2))
    ax_spec.grid(False)
    cbar = fig.colorbar(pcm, ax=ax_spec, pad=0.01, label='Magnitude (dB)')

    # Mark expected tone frequencies on spectrogram
    ax_spec.axhline(mark_freq, color='w', linestyle='--', linewidth=1, alpha=0.7)
    ax_spec.axhline(space_freq, color='w', linestyle='--', linewidth=1, alpha=0.7)
    ax_spec.text(0.01 * duration, mark_freq + 50, f"mark {mark_freq:.0f}Hz", color='w')
    ax_spec.text(0.01 * duration, space_freq + 50, f"space {space_freq:.0f}Hz", color='w')

    # Energy traces (Goertzel)
    ax_energy.plot(times, energy_mark, label=f"mark {mark_freq:.0f}Hz", color='C1', marker='o')
    ax_energy.plot(times, energy_space, label=f"space {space_freq:.0f}Hz", color='C2', marker='o')
    ax_energy.set_xlabel('Time (s)')
    ax_energy.set_ylabel('Energy')
    ax_energy.legend(loc='upper right')
    ax_energy.grid(True)

    # Optionally add instantaneous frequency as a twin axis below spectrogram (comment/uncomment)
    ax_inst = ax_spec.twinx()
    ax_inst.plot(t_inst, inst_freq, color='k', linewidth=0.4, alpha=0.5, label='inst freq')
    ax_inst.set_ylabel('Inst Freq (Hz)')
    ax_inst.set_ylim(0, min(5000, SAMPLE_RATE/2))
    # Keep its ticks subtle
    ax_inst.tick_params(axis='y', labelsize=8)

    plt.tight_layout()
    outname = "inspect_fsk.png"
    plt.savefig(outname, dpi=150)
    plt.show()
    print(f"Saved {outname}")

if __name__ == "__main__":
    main()

