#!/usr/bin/env python3
"""Read fft_mag.txt and plot the captured FFT magnitude rows in one line chart."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_fft_mag(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            values = [float(v) for v in line.split(",") if v.strip()]
            if values:
                rows.append(np.asarray(values, dtype=np.float32))

    if not rows:
        raise ValueError(f"No FFT data found in {path}")

    return np.vstack(rows)


def plot_fft_mag(path: Path, save_path: str | None = None) -> None:
    data = load_fft_mag(path)

    # Use a single pane, single line per FFT block.
    # The default plotting behavior follows the same Matplotlib usage pattern as the
    # working spectrogram script, without forcing an offscreen backend.
    fig, ax = plt.subplots(figsize=(12, 6))

    if data.shape[0] == 1:
        ax.plot(np.arange(data.shape[1]), data[0], color="C0", linewidth=1.5)
        ax.set_title(f"{path.name} — FFT magnitude (single block)")
    else:
        # Plot the first FFT row by default; user can edit this if they want a different block.
        ax.plot(np.arange(data.shape[1]), data[0], color="C0", linewidth=1.5)
        ax.set_title(f"{path.name} — FFT magnitude (block 0)")

    ax.set_xlabel("FFT bin")
    ax.set_ylabel("Magnitude")
    ax.grid(True, alpha=0.3)

    if save_path:
        out = Path(save_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=200)
        print(f"Saved plot to {out}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot fft_mag.txt in a single-pane line chart")
    parser.add_argument("file", nargs="?", default="fft_mag.txt", help="Path to fft_mag.txt")
    parser.add_argument("--save", type=str, default=None, help="Optional output PNG file")
    args = parser.parse_args()

    path = Path(args.file).expanduser()
    plot_fft_mag(path, args.save)


if __name__ == "__main__":
    main()
