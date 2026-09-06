# Development History

This file is a human-readable history of the AFSK repository, focused on major milestones, new file additions, and markdown documentation growth.

## Chronological history

- **2026-09-06 — Development history summary added**
  - `dev_history.md` was created to capture the repository’s evolution in one place.
  - Commit message: `Add development history summary`

- **Approx. early project stage — AFSK/FSK receiver exploration begins**
  - The repository’s original focus appears to be AFSK/FSK demodulation, tone detection, and symbol recovery.
  - The early work centers on how to detect mark/space tones and turn received waveforms into bits.

- **Approx. next phase — README becomes the main explanatory entry point**
  - `README.md` develops into the project’s primary overview.
  - It discusses rx chirp vs ref chirp, decoding options, and the tradeoff between Goertzel filtering and mixer + low-pass filter approaches.
  - This suggests the repo was actively comparing receiver architectures and explaining why certain designs are non-coherent.

- **Approx. documentation phase — quadrature and I/Q explanations are added**
  - `docs/quadrature_demo.md` introduces the I/Q representation of signals.
  - It explains upconversion, downconversion, and quadrature demodulation as the foundation for non-coherent FSK receiver behavior.
  - This marks a shift from simple detection ideas toward a more complete mathematical framework.

- **Approx. filtering phase — low-pass / leaky-integrator reasoning is added**
  - `docs/LPF.md` explains the one-pole IIR low-pass filter used in the receiver.
  - The notes connect the code-level update rule to RC circuits, pole location, and exponential forgetting.
  - This gives the project a stronger DSP foundation for understanding envelope smoothing and tone energy tracking.

- **Approx. timing-recovery phase — clock recovery notes are added**
  - `docs/clock_recovery.md` adds a discussion of symbol timing, soft decisions, and sampling alignment.
  - The project expands beyond tone detection into when to make bit decisions.
  - This is an important step toward a full receiver chain, because a good detector still fails without correct clock recovery.

- **Approx. chirp experimentation phase — chirp comparison artifacts appear**
  - `chirp_experiments/README.md` records a received-chirp vs reference-chirp comparison.
  - This suggests the project began exploring chirp-based signal processing alongside the FSK/AFSK work.
  - The repo’s scope becomes broader and more experimental.

- **Approx. dechirp phase — dechirp math is documented**
  - `dechirp/dechirp_math.md` explains the mix + FFT dechirp receiver approach.
  - The documentation focuses on how chirp multiplication can convert a sweep into a narrowband beat signal.
  - This shows the repository evolving from receiver notes into more advanced chirp-processing analysis.

- **Approx. implementation optimization phase — efficient chirp generation is documented**
  - `complex_rotation/complex-rotation-chirp.md` describes generating chirps efficiently using complex rotation.
  - This reduces repeated trig calls and makes the implementation more practical for embedded or performance-sensitive use.
  - It indicates the project was not only exploring theory, but also refining implementation details.

## File history by documentation addition

- **2026-09-06 — `dev_history.md`**
  - Added as a summary of the repository’s development path.

- **Approx. early documentation phase — `README.md`**
  - Established the project’s core narrative for FSK/AFSK receiver ideas.

- **Approx. early experiments phase — `chirp_experiments/README.md`**
  - Added a minimal artifact describing chirp comparison experiments.

- **Approx. documentation expansion phase — `docs/LPF.md`**
  - Added an explanation of the one-pole IIR low-pass filter / leaky integrator used in the receiver.

- **Approx. documentation expansion phase — `docs/quadrature_demo.md`**
  - Added a conceptual walkthrough of I/Q representation, up/down conversion, and quadrature demodulation.

- **Approx. documentation expansion phase — `docs/clock_recovery.md`**
  - Added timing recovery reasoning and the relationship between soft decisions and bit boundaries.

- **Approx. dechirp phase — `dechirp/dechirp_math.md`**
  - Added mathematical background for dechirp processing.

- **Approx. implementation notes phase — `complex_rotation/complex-rotation-chirp.md`**
  - Added notes on efficient chirp synthesis using complex rotation.

## Overall progression

- The repo starts with classic FSK/AFSK receiver thinking.
- It then moves into clearer explanations of non-coherent detection and I/Q processing.
- After that, timing recovery becomes a major topic.
- Finally, the project expands into chirp experiments, dechirp mathematics, and implementation optimization.
- The documentation suggests the project is exploratory, with code and markdown evolving together to capture signal-processing reasoning.
