# DCO Autotune Architecture & Documentation

## 1. Gap Measurement (`find_gap`)
Measures the duty-cycle error on `DCO_calibration_pin` by timing rising/falling edges.
- **Cycle Counting**: Uses `rp2040.getCycleCount()` for 8ns resolution to prevent quantisation errors inherent to `micros()`.
- **Timeouts**: Dead oscillators trigger a timeout (`kGapTimeoutUs`). Longest segments are capped at `kGapTimeoutMaxUs` to prevent hanging.
- **Segment Gates**: Segments between 1% and 99% of the ideal period are accepted to reject glitches. Mode 3 probes reject one-sided or off-period readings caused by lopsided waveforms at low frequencies.

## 2. Calibration Precision Profiles
Trade-offs between speed and measurement quality:
- **NORMAL**: Builds a table from scratch quickly.
- **FINE**: Re-measures existing tables as accurately as hardware allows. Uses larger measurement windows.
- **FAST**: Cut-down version of NORMAL for rapid testing.

## 3. Pulse Width (PW) Search
Finds the ideal PW by targeting specific duty fractions (50% Center, 2% Low, 98% High).
- **Phase 1 (Coarse Scan)**: Sweeps the range looking for a sign-change bracket around the target duty.
- **Phase 2 (Bisection/Fine Scan)**: Bisects the bracket, or performs a local fine scan if no bracket was found.
- **Phase 3 (Lock-in)**: Demands 3 consecutive measurements within tolerance. Refines locally (PW-2..PW+2).

## 4. Amplitude Calibration (FREQ_TRACE)
Traces the `freq(amp comp)` curve outward from a 440 Hz manual anchor.
- **Anchor**: Re-anchors the manual 440Hz operating point to ensure accurate starting curvature.
- **Bootstrap**: Probes 4 points around the anchor to build the initial log-log curve.
- **Ladder**: Derives rung spacing from measured points, stepping outward.
- **Endpoints**: Endpoints (Amp=0, Amp=Full) are extrapolated and carefully measured, as these are the points where pulse collapse is most likely. 
- **Amp-0 Fit**: Linear least-squares fit on the bottom-most points ensures a stable baseline intercept for runtime interpolation.