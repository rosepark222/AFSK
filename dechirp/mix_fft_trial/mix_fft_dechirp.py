import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# PARAMETERS
# ============================================================

FILENAME = "rx_50cm.txt"

FS = 44100.0

CHIRP_DURATION = 1.0
CHIRP_SAMPLES = int(FS * CHIRP_DURATION)

F0 = 300.0
F1 = 1500.0

K = (F1 - F0) / CHIRP_DURATION

FFT_N = 4096

# Small correlation search window around mix-FFT estimate
CORR_HALF_WINDOW = 500

# Optional finer correlation spacing
CORR_STEP = 1

EPS = 1e-12


# ============================================================
# LOAD RECEIVED CHIRP
# ============================================================

def load_received_chirp(filename):

    print()
    print("Loading:", filename)

    received = np.loadtxt(filename, dtype=np.float64)

    print("Samples in file:", len(received))

    if len(received) < CHIRP_SAMPLES:
        raise ValueError(
            f"File contains only {len(received)} samples, "
            f"but {CHIRP_SAMPLES} are required."
        )

    received = received[:CHIRP_SAMPLES]

    print("Using:", len(received), "samples")

    return received


# ============================================================
# GENERATE REFERENCE CHIRP
# ============================================================

def generate_reference():

    n = np.arange(CHIRP_SAMPLES)

    t = n / FS

    phase = 2.0 * np.pi * (
        F0 * t +
        0.5 * K * t * t
    )

    reference = np.cos(phase)

    return reference, phase


# ============================================================
# ORIGINAL FFT
#
# This is just for visualization.
# ============================================================

def calculate_original_fft(received):

    x = np.zeros(FFT_N)

    n = min(len(received), FFT_N)

    x[:n] = received[:n]

    spectrum = np.fft.fft(x)

    magnitude = np.abs(spectrum)

    frequency = np.fft.fftfreq(
        FFT_N,
        d=1.0 / FS
    )

    return frequency, magnitude


# ============================================================
# MIX + 4096 FFT + COMPLEX ACCUMULATION
#
# This models the Teensy process4096Block().
# ============================================================

def dechirp_fft_accumulate(received, reference_phase):

    sum_spec = np.zeros(FFT_N, dtype=np.complex128)

    block_results = []

    number_of_blocks = int(
        np.ceil(CHIRP_SAMPLES / FFT_N)
    )

    print()
    print("Mix + FFT")
    print("FFT size:", FFT_N)
    print("Number of blocks:", number_of_blocks)

    for block in range(number_of_blocks):

        start = block * FFT_N
        end = min(start + FFT_N, CHIRP_SAMPLES)

        length = end - start

        # ----------------------------------------------------
        # Receive block
        # ----------------------------------------------------

        rx = np.zeros(FFT_N)

        rx[:length] = received[start:end]

        # ----------------------------------------------------
        # Reference phase
        # ----------------------------------------------------

        phase = np.zeros(FFT_N)

        phase[:length] = reference_phase[start:end]

        # ----------------------------------------------------
        # Complex conjugate mixing
        #
        # rx * exp(-j phase)
        # ----------------------------------------------------

        mixer = np.exp(-1j * phase)

        mixed = rx * mixer

        # ----------------------------------------------------
        # FFT
        # ----------------------------------------------------

        spectrum = np.fft.fft(mixed)

        # ----------------------------------------------------
        # Complex accumulation
        # ----------------------------------------------------

        sum_spec += spectrum

        block_results.append({
            "start": start,
            "end": end,
            "length": length,
            "mixed": mixed.copy(),
            "spectrum": spectrum.copy(),
        })

        print(
            f"  block {block + 1:2d}: "
            f"samples {start:5d} .. {end - 1:5d} "
            f"length={length:4d}"
        )

    magnitude = np.abs(sum_spec)

    frequency = np.fft.fftfreq(
        FFT_N,
        d=1.0 / FS
    )

    return (
        sum_spec,
        magnitude,
        frequency,
        block_results
    )


# ============================================================
# FIND FFT PEAK
# ============================================================

def find_fft_peak(frequency, magnitude):

    peak_bin = np.argmax(magnitude)

    peak_frequency = frequency[peak_bin]

    return peak_bin, peak_frequency


# ============================================================
# PARABOLIC SUB-BIN FREQUENCY ESTIMATION
# ============================================================

def estimate_subbin_frequency(
    frequency,
    magnitude,
    peak_bin
):

    # Need neighbors
    if peak_bin <= 0 or peak_bin >= len(magnitude) - 1:
        return frequency[peak_bin], 0.0

    y1 = magnitude[peak_bin - 1]
    y2 = magnitude[peak_bin]
    y3 = magnitude[peak_bin + 1]

    denominator = (
        y1 -
        2.0 * y2 +
        y3
    )

    if abs(denominator) < EPS:
        delta = 0.0
    else:
        delta = 0.5 * (
            (y1 - y3) /
            denominator
        )

    bin_spacing = FS / FFT_N

    estimated_frequency = (
        frequency[peak_bin] +
        delta * bin_spacing
    )

    return estimated_frequency, delta


# ============================================================
# PHASE/FREQUENCY ESTIMATION
#
# This is the important new method.
#
# Instead of using FFT-bin resolution, we estimate the
# frequency of the complex dechirped signal from its phase
# progression.
# ============================================================

def estimate_frequency_from_phase(
    sum_spec,
    received,
    reference_phase
):

    # --------------------------------------------------------
    # We need the actual time-domain dechirped signal.
    #
    # For this we mix the complete received chirp with
    # the complex conjugate reference.
    #
    # This is NOT a correlation.
    # It is simply complex mixing.
    # --------------------------------------------------------

    mixer = np.exp(-1j * reference_phase)

    dechirped = received * mixer

    # --------------------------------------------------------
    # Remove DC offset.
    #
    # This helps when the recording has microphone/DC bias.
    # --------------------------------------------------------

    dechirped = (
        dechirped -
        np.mean(dechirped)
    )

    # --------------------------------------------------------
    # Ignore very weak samples near the boundaries.
    #
    # The dechirped signal is strongest where the received
    # chirp actually overlaps the reference.
    # --------------------------------------------------------

    magnitude = np.abs(dechirped)

    threshold = (
        0.10 *
        np.max(magnitude)
    )

    valid = magnitude > threshold

    if np.count_nonzero(valid) < 100:

        print(
            "WARNING: not enough strong dechirped samples "
            "for phase estimation"
        )

        return None, None, dechirped

    # --------------------------------------------------------
    # Unwrap complex phase
    # --------------------------------------------------------

    phase = np.unwrap(
        np.angle(dechirped)
    )

    # --------------------------------------------------------
    # Phase slope:
    #
    # phase[n] = 2*pi*f*n/FS + constant
    #
    # slope = 2*pi*f/FS
    #
    # therefore:
    #
    # f = slope * FS / (2*pi)
    # --------------------------------------------------------

    indices = np.arange(
        CHIRP_SAMPLES
    )

    indices_valid = indices[valid]
    phase_valid = phase[valid]

    # --------------------------------------------------------
    # Linear least-squares fit
    # --------------------------------------------------------

    slope, intercept = np.polyfit(
        indices_valid,
        phase_valid,
        1
    )

    frequency = (
        slope *
        FS /
        (2.0 * np.pi)
    )

    return (
        frequency,
        slope,
        dechirped
    )


# ============================================================
# IMPROVED PHASE ESTIMATION USING LAGGED PHASE DIFFERENCE
#
# This avoids fitting the absolute phase over the entire
# record and instead measures the phase rotation between
# adjacent samples.
# ============================================================

def estimate_frequency_from_phase_difference(
    received,
    reference_phase
):

    mixer = np.exp(-1j * reference_phase)

    dechirped = (
        received *
        mixer
    )

    # --------------------------------------------------------
    # Adjacent-sample complex phase rotation:
    #
    # z[n+1] * conj(z[n])
    #
    # angle() gives phase change per sample.
    # --------------------------------------------------------

    z1 = dechirped[:-1]
    z2 = dechirped[1:]

    phase_step = np.angle(
        z2 * np.conj(z1)
    )

    amplitude = (
        np.abs(z1) *
        np.abs(z2)
    )

    # Only trust strong samples
    threshold = (
        0.10 *
        np.max(amplitude)
    )

    valid = amplitude > threshold

    if np.count_nonzero(valid) < 100:

        return None, dechirped

    # --------------------------------------------------------
    # Weighted average phase rotation.
    #
    # Strong samples receive more weight.
    # --------------------------------------------------------

    weights = amplitude[valid]

    mean_phase_step = np.average(
        phase_step[valid],
        weights=weights
    )

    frequency = (
        mean_phase_step *
        FS /
        (2.0 * np.pi)
    )

    return frequency, dechirped


# ============================================================
# RESIDUAL FREQUENCY → TIME LAG
# ============================================================

def frequency_to_lag(
    residual_frequency
):

    # For a chirp:
    #
    # f_residual ≈ -K * delta_t
    #
    # delta_t = -f_residual / K

    delta_t = (
        -residual_frequency /
        K
    )

    delta_samples = (
        delta_t *
        FS
    )

    return delta_t, delta_samples


# ============================================================
# CORRELATION
#
# Calculate normalized correlation at one candidate lag.
#
# lag = 0:
#   received[0] corresponds to reference[0]
#
# positive lag:
#   received starts later than reference
# ============================================================

def normalized_correlation_at_lag(
    received,
    reference,
    lag
):

    if lag >= 0:

        rx_start = lag
        ref_start = 0

    else:

        rx_start = 0
        ref_start = -lag

    length = min(
        len(received) - rx_start,
        len(reference) - ref_start
    )

    if length <= 0:
        return 0.0

    rx = received[
        rx_start:
        rx_start + length
    ]

    ref = reference[
        ref_start:
        ref_start + length
    ]

    rx_energy = np.sum(
        rx * rx
    )

    ref_energy = np.sum(
        ref * ref
    )

    if rx_energy < EPS or ref_energy < EPS:
        return 0.0

    correlation = np.sum(
        rx * ref
    )

    normalized = (
        correlation /
        np.sqrt(
            rx_energy *
            ref_energy
        )
    )

    return normalized


# ============================================================
# SMALL CORRELATION SEARCH
#
# This is the eventual expensive operation, but only over
# a small region around the mix-FFT estimate.
# ============================================================

def small_correlation_search(
    received,
    reference,
    estimated_lag,
    half_window,
    step=1
):

    center = int(
        round(estimated_lag)
    )

    start_lag = center - half_window
    end_lag = center + half_window

    lags = np.arange(
        start_lag,
        end_lag + 1,
        step,
        dtype=int
    )

    correlations = np.zeros(
        len(lags)
    )

    print()
    print("Small correlation search")
    print(
        f"  estimated lag : {estimated_lag:.2f} samples"
    )
    print(
        f"  search center  : {center} samples"
    )
    print(
        f"  window         : "
        f"[{start_lag}, {end_lag}]"
    )
    print(
        f"  points         : {len(lags)}"
    )

    for i, lag in enumerate(lags):

        correlations[i] = (
            normalized_correlation_at_lag(
                received,
                reference,
                lag
            )
        )

    best_index = np.argmax(
        correlations
    )

    best_lag = lags[best_index]

    best_correlation = (
        correlations[best_index]
    )

    return (
        best_lag,
        best_correlation,
        lags,
        correlations
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print()
    print("==============================================")
    print(" Mix + FFT Chirp Localization")
    print("==============================================")

    print()
    print("FS                =", FS)
    print("Chirp samples     =", CHIRP_SAMPLES)
    print("F0                =", F0)
    print("F1                =", F1)
    print("Chirp slope       =", K, "Hz/s")
    print("FFT size          =", FFT_N)
    print(
        "FFT resolution    =",
        FS / FFT_N,
        "Hz"
    )

    # --------------------------------------------------------
    # Load received chirp
    # --------------------------------------------------------

    received = load_received_chirp(
        FILENAME
    )

    # --------------------------------------------------------
    # Generate reference
    # --------------------------------------------------------

    reference, reference_phase = (
        generate_reference()
    )

    # --------------------------------------------------------
    # Original FFT
    # --------------------------------------------------------

    original_frequency, original_magnitude = (
        calculate_original_fft(
            received
        )
    )

    # --------------------------------------------------------
    # Mix + FFT accumulation
    # --------------------------------------------------------

    (
        sum_spec,
        final_magnitude,
        final_frequency,
        block_results
    ) = dechirp_fft_accumulate(
        received,
        reference_phase
    )

    # --------------------------------------------------------
    # FFT peak
    # --------------------------------------------------------

    peak_bin, peak_frequency = (
        find_fft_peak(
            final_frequency,
            final_magnitude
        )
    )

    (
        subbin_frequency,
        subbin_offset
    ) = estimate_subbin_frequency(
        final_frequency,
        final_magnitude,
        peak_bin
    )

    # --------------------------------------------------------
    # Phase slope frequency
    # --------------------------------------------------------

    (
        phase_frequency,
        phase_slope,
        dechirped
    ) = estimate_frequency_from_phase(
        sum_spec,
        received,
        reference_phase
    )

    # --------------------------------------------------------
    # Phase-difference frequency
    # --------------------------------------------------------

    (
        phase_diff_frequency,
        dechirped2
    ) = estimate_frequency_from_phase_difference(
        received,
        reference_phase
    )

    # --------------------------------------------------------
    # Print frequency results
    # --------------------------------------------------------

    print()
    print("==============================================")
    print(" MIX + FFT RESULTS")
    print("==============================================")

    print(
        f"FFT peak bin       : {peak_bin}"
    )

    print(
        f"FFT peak frequency : "
        f"{peak_frequency:.6f} Hz"
    )

    print(
        f"Sub-bin frequency  : "
        f"{subbin_frequency:.6f} Hz"
    )

    print(
        f"Sub-bin offset     : "
        f"{subbin_offset:.6f} bins"
    )

    if phase_frequency is not None:

        print(
            f"Phase-fit freq     : "
            f"{phase_frequency:.6f} Hz"
        )

        print(
            f"Phase slope        : "
            f"{phase_slope:.9e} rad/sample"
        )

    if phase_diff_frequency is not None:

        print(
            f"Phase-diff freq    : "
            f"{phase_diff_frequency:.6f} Hz"
        )

    # --------------------------------------------------------
    # Convert frequency to lag
    # --------------------------------------------------------

    if phase_diff_frequency is not None:

        selected_frequency = (
            phase_diff_frequency
        )

        selected_method = (
            "phase-difference"
        )

    elif phase_frequency is not None:

        selected_frequency = (
            phase_frequency
        )

        selected_method = (
            "phase-fit"
        )

    else:

        selected_frequency = (
            subbin_frequency
        )

        selected_method = (
            "FFT sub-bin"
        )

    delta_t, estimated_lag = (
        frequency_to_lag(
            selected_frequency
        )
    )

    print()
    print("==============================================")
    print(" INITIAL LAG ESTIMATE")
    print("==============================================")

    print(
        f"Method             : {selected_method}"
    )

    print(
        f"Residual frequency : "
        f"{selected_frequency:.6f} Hz"
    )

    print(
        f"Estimated time lag : "
        f"{delta_t * 1000.0:.6f} ms"
    )

    print(
        f"Estimated lag      : "
        f"{estimated_lag:.3f} samples"
    )

    # --------------------------------------------------------
    # Small correlation search
    # --------------------------------------------------------

    (
        best_lag,
        best_correlation,
        corr_lags,
        correlations
    ) = small_correlation_search(
        received,
        reference,
        estimated_lag,
        CORR_HALF_WINDOW,
        CORR_STEP
    )

    print()
    print("==============================================")
    print(" FINAL CORRELATION")
    print("==============================================")

    print(
        f"Best lag           : "
        f"{best_lag} samples"
    )

    print(
        f"Best correlation   : "
        f"{best_correlation:.9f}"
    )

    # --------------------------------------------------------
    # Difference between estimate and true correlation peak
    # --------------------------------------------------------

    error_samples = (
        best_lag -
        estimated_lag
    )

    print(
        f"Estimate error     : "
        f"{error_samples:.3f} samples"
    )

    print()
    print("==============================================")
    print(" DONE")
    print("==============================================")


    # ========================================================
    # PLOTS
    # ========================================================

    fig = plt.figure(
        figsize=(14, 14)
    )

    # --------------------------------------------------------
    # Plot 1: received waveform
    # --------------------------------------------------------

    ax1 = plt.subplot(5, 1, 1)

    t = np.arange(
        CHIRP_SAMPLES
    ) / FS

    ax1.plot(
        t,
        received
    )

    ax1.set_title(
        "Received 1-second chirp"
    )

    ax1.set_xlabel(
        "Time (seconds)"
    )

    ax1.set_ylabel(
        "Amplitude"
    )

    ax1.grid(True)

    # --------------------------------------------------------
    # Plot 2: original FFT
    # --------------------------------------------------------

    ax2 = plt.subplot(5, 1, 2)

    shifted_freq = np.fft.fftshift(
        original_frequency
    )

    shifted_mag = np.fft.fftshift(
        original_magnitude
    )

    ax2.plot(
        shifted_freq,
        shifted_mag
    )

    ax2.set_xlim(
        -3000,
        3000
    )

    ax2.set_title(
        "Original 4096-point FFT"
    )

    ax2.set_xlabel(
        "Frequency (Hz)"
    )

    ax2.set_ylabel(
        "Magnitude"
    )

    ax2.grid(True)

    # --------------------------------------------------------
    # Plot 3: final accumulated dechirped FFT
    # --------------------------------------------------------

    ax3 = plt.subplot(5, 1, 3)

    shifted_freq = np.fft.fftshift(
        final_frequency
    )

    shifted_mag = np.fft.fftshift(
        final_magnitude
    )

    ax3.plot(
        shifted_freq,
        shifted_mag
    )

    ax3.set_xlim(
        -100,
        100
    )

    ax3.set_title(
        "Final coherent accumulated dechirped FFT"
    )

    ax3.set_xlabel(
        "Residual frequency (Hz)"
    )

    ax3.set_ylabel(
        "Magnitude"
    )

    ax3.grid(True)

    # --------------------------------------------------------
    # Plot 4: phase-derived dechirped signal
    # --------------------------------------------------------

    ax4 = plt.subplot(5, 1, 4)

    dechirped_magnitude = np.abs(
        dechirped
    )

    ax4.plot(
        t,
        dechirped_magnitude
    )

    ax4.set_title(
        "Dechirped signal magnitude"
    )

    ax4.set_xlabel(
        "Time (seconds)"
    )

    ax4.set_ylabel(
        "Magnitude"
    )

    ax4.grid(True)

    # --------------------------------------------------------
    # Plot 5: small correlation search
    # --------------------------------------------------------

    ax5 = plt.subplot(5, 1, 5)

    ax5.plot(
        corr_lags,
        correlations
    )

    ax5.axvline(
        estimated_lag,
        linestyle="--",
        label="Mix-FFT estimated lag"
    )

    ax5.axvline(
        best_lag,
        linestyle="--",
        label="Correlation peak"
    )

    ax5.set_title(
        "Small correlation search around mix-FFT estimate"
    )

    ax5.set_xlabel(
        "Lag (samples)"
    )

    ax5.set_ylabel(
        "Normalized correlation"
    )

    ax5.grid(True)

    ax5.legend()

    plt.tight_layout()

    plt.show()


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":
    main()
