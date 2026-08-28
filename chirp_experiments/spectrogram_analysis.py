#!/usr/bin/env python3
"""
spectrogram_analysis.py

Plots waveform + spectrogram + per-block Goertzel energy for a received
chirp (or two-tone) signal.

Usage:
    python spectrogram_analysis.py samples.txt [sample_rate] [fmin] [fmax]

- samples.txt: one number per line
- sample_rate: optional, default 44100
- fmin/fmax: optional tone/band-edge frequencies in Hz used for the
             dashed reference lines on the spectrogram AND as the two
             tones tracked by the Goertzel energy panel at the bottom
             (default: 300, 1500)
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import spectrogram, hilbert


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
    return real * real + imag * imag


def main():
    if len(sys.argv) < 2:
        print("Usage: python spectrogram_analysis.py samples.txt [sample_rate] [fmin] [fmax]")
        sys.exit(1)

    filename = sys.argv[1]
    SAMPLE_RATE = int(sys.argv[2]) if len(sys.argv) > 2 else 44100
    fmin = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0
    fmax = float(sys.argv[4]) if len(sys.argv) > 4 else 1500.0

    # Tones tracked by the Goertzel energy panel (reuse fmin/fmax as mark/space)
    mark_freq = fmin
    space_freq = fmax

    samples = load_samples(filename)
    duration = len(samples) / SAMPLE_RATE
    print(f"Loaded {len(samples)} samples = {duration:.3f} s")

    # Spectrogram params - fine time/freq resolution for a 1s chirp
    nperseg = 1024
    noverlap = int(nperseg * 0.9)
    nfft = 2048

    f, t_spec, Sxx = spectrogram(samples, fs=SAMPLE_RATE, window='hann',
                                  nperseg=nperseg, noverlap=noverlap, nfft=nfft,
                                  mode='magnitude', scaling='spectrum')

    Sxx_norm = Sxx / (np.max(Sxx) + 1e-20)
    Sxx_db = 20 * np.log10(Sxx_norm + 1e-12)
    Sxx_db = np.clip(Sxx_db, -80, 0)

    # Instantaneous frequency via Hilbert transform (computed, not plotted by default)
    analytic = hilbert(samples)
    phase = np.unwrap(np.angle(analytic))
    inst_freq = np.diff(phase) / (2.0 * np.pi) * SAMPLE_RATE
    t_inst = (np.arange(len(inst_freq)) + 0.5) / SAMPLE_RATE

    # Per-block Goertzel energy at mark_freq/space_freq
    block_sec = 0.01  # 10 ms blocks
    block_len = int(round(block_sec * SAMPLE_RATE))
    n_windows = len(samples) // block_len
    energy_times = (np.arange(n_windows) * block_len + block_len / 2) / SAMPLE_RATE
    energy_mark = np.zeros(n_windows)
    energy_space = np.zeros(n_windows)
    window = np.hanning(block_len)
    for i in range(n_windows):
        start = i * block_len
        block = samples[start:start + block_len] * window
        energy_mark[i] = goertzel_block_energy(block, SAMPLE_RATE, mark_freq)
        energy_space[i] = goertzel_block_energy(block, SAMPLE_RATE, space_freq)

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(7, 1)

    ax_wave = fig.add_subplot(gs[0:2, 0])
    ax_spec = fig.add_subplot(gs[2:5, 0], sharex=ax_wave)
    ax_energy = fig.add_subplot(gs[5:7, 0], sharex=ax_wave)

    # Waveform
    t_samples = np.arange(len(samples)) / SAMPLE_RATE
    ax_wave.plot(t_samples, samples, color='C0', linewidth=0.5)
    ax_wave.set_ylabel('Amplitude')
    ax_wave.set_title(f"Received Chirp — Waveform + Spectrogram + Energy — {filename}")
    ax_wave.grid(True)

    # Spectrogram
    pcm = ax_spec.pcolormesh(t_spec, f, Sxx_db, shading='gouraud', cmap='viridis',
                              vmin=-80, vmax=0)
    ax_spec.set_ylabel('Frequency (Hz)')
    # ax_spec.set_ylim(0, min(3000, SAMPLE_RATE / 2))
    ax_spec.set_ylim(0, 15000)
    ax_spec.grid(False)
    fig.colorbar(pcm, ax=ax_spec, pad=0.01, label='Magnitude (dB)')

    # Mark expected chirp bounds / tone frequencies
    ax_spec.axhline(fmin, color='w', linestyle='--', linewidth=1, alpha=0.7)
    ax_spec.axhline(fmax, color='w', linestyle='--', linewidth=1, alpha=0.7)
    ax_spec.text(0.01 * duration, fmin + 30, f"{fmin:.0f} Hz", color='w')
    ax_spec.text(0.01 * duration, fmax + 30, f"{fmax:.0f} Hz", color='w')

    # Energy traces (Goertzel) at the bottom
    ax_energy.plot(energy_times, energy_mark, label=f"mark {mark_freq:.0f}Hz", color='C1', marker='o', markersize=3)
    ax_energy.plot(energy_times, energy_space, label=f"space {space_freq:.0f}Hz", color='C2', marker='o', markersize=3)
    ax_energy.set_xlabel('Time (s)')
    ax_energy.set_ylabel('Energy')
    ax_energy.legend(loc='upper right')
    ax_energy.grid(True)

    plt.tight_layout()
    outname = "spectrogram_output.png"
    plt.savefig(outname, dpi=150)
    plt.show()
    print(f"Saved {outname}")


if __name__ == "__main__":
    main()
