"""
Cross-correlation analysis with I/Q envelope detection.

Usage:
    python cross_correlation_iq.py fileA.txt fileB.txt [--fs 44100]

fileA = known reference / template chirp (the "clean" transmitted chirp)
fileB = received signal

The quadrature (Q) reference is derived from the REFERENCE chirp (fileA)
via the Hilbert transform -- it is NOT generated from the received signal.
This is what makes the envelope A(k) = sqrt(I(k)^2 + Q(k)^2) a valid
measure of the compressed-pulse magnitude, independent of carrier phase.
"""

import argparse
import numpy as np
from scipy.signal import hilbert
import matplotlib.pyplot as plt


def load_signal(filepath):
    data = np.loadtxt(filepath, delimiter=None)
    if data.ndim > 1:
        data = data[:, 0]
    return data.astype(float)


def cross_correlate_iq(reference, received, normalize=True):
    """
    Compute I(k), Q(k), and envelope A(k) = sqrt(I^2 + Q^2)
    for a received real signal against a known reference chirp.

    Parameters
    ----------
    reference : array_like
        The known/clean reference (template) chirp.
    received : array_like
        The received real signal to correlate against the reference.

    Returns
    -------
    lags : ndarray
        Lag indices (in samples).
    I : ndarray
        Correlation of `received` with the in-phase reference.
    Q : ndarray
        Correlation of `received` with the quadrature (90 deg shifted)
        version of the reference, obtained via the Hilbert transform
        of `reference`.
    A_env : ndarray
        Envelope sqrt(I^2 + Q^2), independent of carrier phase.
    """
    reference = np.asarray(reference, dtype=float)
    received = np.asarray(received, dtype=float)

    # Build the analytic version of the REFERENCE only.
    # real part -> reference itself (in-phase reference)
    # imag part -> Hilbert transform of reference (quadrature reference)
    ref_analytic = hilbert(reference)
    ref_inphase = np.real(ref_analytic)      # == reference
    ref_quad = np.imag(ref_analytic)         # 90-degree shifted version

    if normalize:
        received = (received - np.mean(received)) / (np.std(received) * len(received))
        # Normalize both reference channels by the SAME scale factor
        # (std of the in-phase reference) so I and Q are on equal footing.
        ref_scale = np.std(ref_inphase)
        ref_inphase = (ref_inphase - np.mean(ref_inphase)) / ref_scale
        ref_quad = ref_quad / ref_scale  # no separate mean-subtraction; keep phase relation intact

    I = np.correlate(received, ref_inphase, mode="full")
    Q = np.correlate(received, ref_quad, mode="full")
    A_env = np.sqrt(I**2 + Q**2)

    lags = np.arange(-len(reference) + 1, len(received))
    return lags, I, Q, A_env


def main():
    parser = argparse.ArgumentParser(description="I/Q cross-correlation envelope of two signals from files.")
    parser.add_argument("fileA", help="Path to file containing reference signal A")
    parser.add_argument("fileB", help="Path to file containing received signal B")
    parser.add_argument("--fs", type=float, default=44100, help="Sampling frequency (Hz).")
    parser.add_argument("--xlim", type=float, default=4000, help="X-axis half-range in samples for the correlation plot.")
    parser.add_argument("--no-normalize", action="store_true", help="Disable normalization.")
    args = parser.parse_args()

    A = load_signal(args.fileA)
    B = load_signal(args.fileB)

    tA = np.arange(len(A)) / args.fs
    tB = np.arange(len(B)) / args.fs

    lags, I, Q, env = cross_correlate_iq(A, B, normalize=not args.no_normalize)
    lag_times = lags / args.fs

    best_idx = np.argmax(env)
    best_lag = lags[best_idx]
    print(f"Peak envelope = {env[best_idx]:.4f} at lag = {best_lag} samples "
          f"({best_lag / args.fs:.4f} s)")

    fig, axes = plt.subplots(4, 1, figsize=(10, 11))

    axes[0].plot(tA, A, color="tab:blue")
    axes[0].set_title(f"{args.fileA} (reference)")
    axes[0].set_xlabel("Time (s)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(tB, B, color="tab:orange")
    axes[1].set_title(f"{args.fileB} (received)")
    axes[1].set_xlabel("Time (s)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(lags, I, color="tab:green", label="I(k) (in-phase)")
    axes[2].plot(lags, Q, color="tab:purple", label="Q(k) (quadrature)", alpha=0.7)
    axes[2].axvline(best_lag, color="red", linestyle="--")
    axes[2].set_xlim(-args.xlim, args.xlim)
    axes[2].set_title("Raw I/Q correlation (carrier ripple still present)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(lags, env, color="black", label="A(k) = sqrt(I^2+Q^2)")
    axes[3].axvline(best_lag, color="red", linestyle="--",
                     label=f"Peak lag = {best_lag} samples ({best_lag/args.fs:.4f} s)")
    axes[3].set_xlim(-args.xlim, args.xlim)
    axes[3].set_ylim(0, 1)
    axes[3].set_title("Correlation envelope (carrier-independent)")
    axes[3].set_xlabel("Lag (samples)")
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
