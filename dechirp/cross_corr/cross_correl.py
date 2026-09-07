"""
Cross-correlation analysis of two signals loaded from files.

Usage:
    python cross_correlation.py fileA.txt fileB.txt [--fs 1.0]

Each input file should contain one numeric sample per line
(e.g. plain text / CSV single-column). Adjust `load_signal()`
below if your files have a different format (e.g. multi-column CSV).
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt


def load_signal(filepath):
    """
    Load a 1-D signal from a text/CSV file.
    Assumes one numeric value per line (whitespace/comma separated,
    first column used if multiple columns are present).
    """
    data = np.loadtxt(filepath, delimiter=None)
    if data.ndim > 1:
        data = data[:, 0]
    return data.astype(float)


def cross_correlate(a, b, normalize=True):
    """
    Compute the cross-correlation between signals a and b.

    Returns:
        lags : array of lag indices (in samples), same length as corr
        corr : cross-correlation values
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    if normalize:
        # Remove mean and normalize by std * length for a
        # correlation-coefficient-like result in [-1, 1]
        a = (a - np.mean(a)) / (np.std(a) * len(a))
        b = (b - np.mean(b)) / np.std(b)

    corr = np.correlate(a, b, mode="full")
    lags = np.arange(-len(b) + 1, len(a))

    return lags, corr


def main():
    parser = argparse.ArgumentParser(description="Cross-correlate two signals from files.")
    parser.add_argument("fileA", help="Path to file containing signal A")
    parser.add_argument("fileB", help="Path to file containing signal B")
    parser.add_argument("--fs", type=float, default=1.0,
                         help="Sampling frequency (Hz) used to build time axes. Default 1.0 (samples).")
    parser.add_argument("--no-normalize", action="store_true",
                         help="Disable normalization of the cross-correlation.")
    args = parser.parse_args()

    # --- 1. Load signals ---
    A = load_signal(args.fileA)
    B = load_signal(args.fileB)

    tA = np.arange(len(A)) / args.fs
    tB = np.arange(len(B)) / args.fs

    # --- 2. Cross-correlation ---
    lags, C = cross_correlate(A, B, normalize=not args.no_normalize)
    lag_times = lags / args.fs

    best_lag_idx = np.argmax(C)
    best_lag = lags[best_lag_idx]
    print(f"Peak correlation = {C[best_lag_idx]:.4f} at lag = {best_lag} samples "
          f"({best_lag / args.fs:.4f} s)")

    # --- 3. Plot ---
    fig, axes = plt.subplots(3, 1, figsize=(10, 9))

    axes[0].plot(tA, A, color="tab:blue")
    axes[0].set_title(args.fileA)
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Amplitude")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(tB, B, color="tab:orange")
    axes[1].set_title(args.fileB)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Amplitude")
    axes[1].grid(True, alpha=0.3)

    # axes[2].bar(lags, C, color="tab:green", width=0.4)
    axes[2].plot(lags, C, color="tab:green")

    axes[2].axvline(best_lag, color="red", linestyle="--",
                     label=f"Peak lag (+ means A lags B) = {best_lag} samples")
    axes[2].set_xlim(best_lag - 200, best_lag + 200)
    axes[2].set_title("Cross-correlation (A ⋆ B)")
    axes[2].set_xlabel(f"Peak correlation = {C[best_lag_idx]:.4f} at lag = {best_lag} samples "
                        f"({best_lag / args.fs:.4f} s)")
    axes[2].set_ylabel("Correlation")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = "cross_correlation_result.png"
    plt.savefig(out_path, dpi=150)
    print(f"Plot saved to {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
