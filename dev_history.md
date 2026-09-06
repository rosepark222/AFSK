# Development History

This file is a quick, human-readable history of the AFSK repository, focused on major milestones, new file additions, and markdown documentation additions.

## Major milestones
- **Initial AFSK/FSK receiver exploration**: The repository centers on AFSK/FSK demodulation, clock recovery, and tone detection ideas.
- **Quadrature / non-coherent receiver direction**: Documentation evolved to explain why mixer + low-pass filter and I/Q-based detection are non-coherent in carrier phase terms.
- **Chirp experimentation and dechirp math**: The repo later expanded into chirp-based experiments, reference-vs-received chirp comparisons, and dechirp receiver math.
- **Clock recovery analysis**: Added notes explaining symbol timing, leaky integrators, and PLL-style sampling alignment.

## New file additions
- `dev_history.md` — this summary file.
- `README.md` — main project overview and explanation of FSK/AFSK receiver ideas.
- `chirp_experiments/README.md` — minimal chirp comparison note.
- `docs/LPF.md` — low-pass filter / leaky integrator explanation.
- `docs/clock_recovery.md` — clock recovery and symbol timing notes.
- `docs/quadrature_demo.md` — quadrature modulation and non-coherent receiver explanation.
- `dechirp/dechirp_math.md` — math background for the mix + FFT dechirp receiver.
- `complex_rotation/complex-rotation-chirp.md` — efficient chirp generation via complex rotation.

## Markdown file additions and documentation growth
### Early markdown additions
- `README.md` established the project’s core explanatory narrative.
- `chirp_experiments/README.md` added a first chirp comparison artifact.

### Documentation expansion
- `docs/LPF.md` added a gentle explanation of the one-pole IIR low-pass filter used in the receiver.
- `docs/quadrature_demo.md` added a conceptual walkthrough of I/Q representation, up/down conversion, and quadrature demodulation.
- `docs/clock_recovery.md` added timing recovery reasoning and the relationship between soft decisions and bit boundaries.
- `dechirp/dechirp_math.md` added the mathematical background for dechirp processing.
- `complex_rotation/complex-rotation-chirp.md` added implementation notes on efficient chirp synthesis.

## Notes
- The repository appears to be actively exploratory and documentation-heavy, with markdown files being used to capture design reasoning alongside implementation work.
- The most visible theme across the docs is moving from raw FSK/AFSK detection toward clearer explanations of non-coherent detection, timing recovery, and chirp-based processing.
