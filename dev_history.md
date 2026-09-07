- **2026-09-06 -- corr peak detected, 
[1970-01-01 00:00:11.952] SEARCHING -> DECHIRPING roughCorr  0.2890
[1970-01-01 00:00:12.027] process4096Block starts
[1970-01-01 00:00:12.028] process4096Block finished
[1970-01-01 00:00:12.028] process4096Block starts
[1970-01-01 00:00:12.029] process4096Block finished
[1970-01-01 00:00:12.029] process4096Block starts
[1970-01-01 00:00:12.030] process4096Block finished
[1970-01-01 00:00:12.030] process4096Block starts
[1970-01-01 00:00:12.030] process4096Block finished
[1970-01-01 00:00:12.030] process4096Block starts
[1970-01-01 00:00:12.031] process4096Block finished
[1970-01-01 00:00:12.031] process4096Block starts
[1970-01-01 00:00:12.032] process4096Block finished
[1970-01-01 00:00:12.032] process4096Block starts
[1970-01-01 00:00:12.033] process4096Block finished
[1970-01-01 00:00:12.033] process4096Block starts
[1970-01-01 00:00:12.033] process4096Block finished
[1970-01-01 00:00:12.034] process4096Block starts
[1970-01-01 00:00:12.034] process4096Block finished
[1970-01-01 00:00:12.034] process4096Block starts
[1970-01-01 00:00:12.035] process4096Block finished
[1970-01-01 00:00:12.035] process4096Block starts
[1970-01-01 00:00:12.036] [STATE=DECHIRPING] blocks=011 peakBin=3956 peakFreq=42592.676 Hz peak=4.408007 mean=0.034032 ratio=129.526  --> DETECT

when i added log, I got the above . it seems the mix-fft (dechirp) starts 75ms later the correlation detected. the mix-fft should perform for the 1sec searchBuf of peak correlation, or wait for 2 sec and wait for next chirp ( 1sec chirp + 2sec silence ) pattern. The current code does not seem doing so. Explain isInChirp, noinChirp usage, which is very confusing to me. 


# Development History

This file is a human-readable history of the AFSK repository, focused on major milestones, new file additions, and markdown documentation growth.

## Chronological history




- **2026-04-25 — Repository initial commit**
  - The AFSK repository begins with its first commit.
  - This marks the start of the project’s versioned history.

- **2026-05-08T21:36:58Z to 2026-05-10T05:11:49Z — README becomes the main explanatory entry point**
  - `README.md` develops into the project’s primary overview.
  - It begins with a visual reference, then expands to discuss clock recovery and AFSK receiver ideas.
  - The README then adds explanations of FSK decoding methods, including Goertzel filtering and mixer + low-pass filter approaches.
  - This suggests the repo was actively comparing receiver architectures and explaining why certain designs are non-coherent.

- **2026-05-08T22:48:14Z to 2026-08-17T04:53:46Z — Clock recovery notes are added and refined**
  - `README.md` first adds clock recovery details and images.
  - Later, `docs/clock_recovery.md` expands the explanation of symbol timing, soft decisions, and sampling alignment.
  - The project moves beyond tone detection into when to make bit decisions.
  - This is an important step toward a full receiver chain, because a good detector still fails without correct clock recovery.

- **2026-05-09T02:59:23Z — `leak_integrator.py` is added**
  - The repository adds a leaky integrator implementation.
  - This fits the broader receiver theme of smoothing and tracking energy over time.

- **2026-05-10T03:45:10Z to 2026-05-10T05:11:49Z — FSK decoding methods and non-coherent detection are expanded**
  - `README.md` adds detailed explanations of FSK decoding methods.
  - The documentation explains both Goertzel and mixer + low-pass filter approaches.
  - It then expands to clarify non-coherent detection and PLL tracking concepts.
  - This shows the project actively comparing receiver architectures and phase-insensitive detection ideas.

- **2026-05-11T03:49:09Z to 2026-05-11T04:23:40Z — Documentation is reorganized and cleaned up**
  - The docs are rearranged and typo fixes are applied.
  - This suggests the repository’s explanatory structure was being refined for readability.

- **2026-05-19T17:49:01Z — Demo documentation is expanded**
  - The demo documentation is beefed up with more detail.
  - This indicates the project’s explanatory material is growing around the core signal-processing ideas.

- **2026-07-13T00:54:07Z — Demo section numbers and references are updated**
  - The demo document receives section-number and reference updates.
  - This is another sign of documentation maturation and cleanup.

- **2026-08-17T01:35:27Z to 2026-08-17T04:53:46Z — Quadrature and clock recovery docs are refined**
  - `docs/quadrature_demo.md` is refined with clearer equations and descriptions.
  - `docs/clock_recovery.md` is also clarified, especially around clock count and sampling points.
  - Together, these changes strengthen the mathematical and timing-recovery foundation of the receiver.

- **2026-09-01T07:12:24Z to 2026-09-01T21:15:42Z — Dechirp work becomes more detailed**
  - `dechirp/dechirp_math.md` is updated with corrected mathematical notation.
  - A related commit adds correlation search before dechirp and accumulates complex FFT data.
  - The repo expands from receiver notes into more advanced chirp-processing analysis.

- **2026-09-04T05:28:22Z to 2026-09-04T05:29:25Z — Complex rotation chirp work is fixed and merged**
  - `complex_rotation/complex-rotation-chirp.md` receives fixes around complex rotation.
  - A merge follows shortly after.
  - This indicates implementation details were being refined alongside the documentation.

- **2026-09-06T23:09:09Z — Development history summary added**
  - `dev_history.md` was created to capture the repository’s evolution in one place.
  - Commit message: `Add development history summary`

- **2026-05-08 to 2026-05-09 — Early AFSK/FSK receiver exploration begins**
  - The repository’s original focus appears to be AFSK/FSK demodulation, tone detection, and symbol recovery.
  - The early work centers on how to detect mark/space tones and turn received waveforms into bits.

- **2026-05-08T21:36:58Z to 2026-08-17T01:35:27Z — Quadrature and I/Q explanations are added and refined**
  - `docs/quadrature_demo.md` introduces the I/Q representation of signals.
  - It explains upconversion, downconversion, and quadrature demodulation as the foundation for non-coherent FSK receiver behavior.
  - This marks a shift from simple detection ideas toward a more complete mathematical framework.

- **2026-05-08T22:48:14Z to 2026-08-17T04:53:46Z — Low-pass / leaky-integrator reasoning is added**
  - `docs/LPF.md` explains the one-pole IIR low-pass filter used in the receiver.
  - The notes connect the code-level update rule to RC circuits, pole location, and exponential forgetting.
  - This gives the project a stronger DSP foundation for understanding envelope smoothing and tone energy tracking.

- **2026-05-19T17:49:01Z to 2026-05-19T17:49:01Z — Chirp comparison artifacts appear**
  - `chirp_experiments/README.md` records a received-chirp vs reference-chirp comparison.
  - This suggests the project began exploring chirp-based signal processing alongside the FSK/AFSK work.
  - The repo’s scope becomes broader and more experimental.

- **2026-09-01T07:12:24Z to 2026-09-01T21:15:42Z — Dechirp math is documented**
  - `dechirp/dechirp_math.md` explains the mix + FFT dechirp receiver approach.
  - The documentation focuses on how chirp multiplication can convert a sweep into a narrowband beat signal.
  - This shows the repository evolving from receiver notes into more advanced chirp-processing analysis.

- **2026-09-04T05:28:22Z to 2026-09-04T05:29:25Z — Efficient chirp generation is documented**
  - `complex_rotation/complex-rotation-chirp.md` describes generating chirps efficiently using complex rotation.
  - This reduces repeated trig calls and makes the implementation more practical for embedded or performance-sensitive use.
  - It indicates the project was not only exploring theory, but also refining implementation details.

## File history by documentation addition

- **2026-09-06T23:09:09Z — `dev_history.md`**
  - Added as a summary of the repository’s development path.

- **2026-05-08T21:36:58Z to 2026-05-10T05:11:49Z — `README.md`**
  - Established the project’s core narrative for FSK/AFSK receiver ideas.

- **2026-05-19T17:49:01Z — `chirp_experiments/README.md`**
  - Added a minimal artifact describing chirp comparison experiments.

- **2026-05-08T22:48:14Z to 2026-08-17T04:53:46Z — `docs/LPF.md`**
  - Added an explanation of the one-pole IIR low-pass filter / leaky integrator used in the receiver.

- **2026-08-17T01:35:27Z to 2026-08-17T04:53:46Z — `docs/quadrature_demo.md`**
  - Added a conceptual walkthrough of I/Q representation, up/down conversion, and quadrature demodulation.

- **2026-05-08T22:48:14Z to 2026-08-17T04:53:46Z — `docs/clock_recovery.md`**
  - Added timing recovery reasoning and the relationship between soft decisions and bit boundaries.

- **2026-09-01T07:12:24Z to 2026-09-01T21:15:42Z — `dechirp/dechirp_math.md`**
  - Added mathematical background for dechirp processing.

- **2026-09-04T05:28:22Z to 2026-09-04T05:29:25Z — `complex_rotation/complex-rotation-chirp.md`**
  - Added notes on efficient chirp synthesis using complex rotation.

## Overall progression

- The repo starts with classic FSK/AFSK receiver thinking.
- It then moves into clearer explanations of non-coherent detection, I/Q processing, and clock recovery.
- After that, chirp experimentation and dechirp analysis become major topics.
- Finally, the project refines implementation details such as efficient chirp generation.
- The documentation suggests the project is exploratory, with code and markdown evolving together to capture signal-processing reasoning.
