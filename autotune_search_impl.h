#ifndef __AUTOTUNE_SEARCH_IMPL_H__
#define __AUTOTUNE_SEARCH_IMPL_H__

#include "autotune.h"
#include <math.h>

// =============================================================================
// Search State & Internal Telemetry
// =============================================================================

static float g_lastFreqBisectGapUs  = kGapTimeoutSentinel;
static int   g_lastFreqBisectProbes = 0;
static int   g_lastSettleChecks     = 0;
static bool  g_lastDutyUnsettled    = false;
static float g_lastAmp0ScanSeedHz   = 0.0f;

// Forward internal declarations
static float freq_trace_guess(const float *xs, const float *ys, int count, float x);
static float amp0_fit_freq(const float *amps, const float *freqs, int count);
static float freq_trace_power_seed(const float *freqs, const float *amps, int count, float ampTarget);
static String freq_trace_quality(float gapUs, float freqHz, int probes, int settleChecks);

// =============================================================================
// Curve Fitting & Math Interpolation Utilities
// =============================================================================

float linearInterpolation(float x0, float y0, float x1, float y1, float x) {
  if (x0 == x1) return y0;
  float m = (y1 - y0) / (x1 - x0);
  return y0 + m * (x - x0);
}

float quadraticInterpolation(float x0, float y0, float x1, float y1, float x2, float y2, float x) {
  float denom = (x0 - x1) * (x0 - x2) * (x1 - x2);
  if (fabsf(denom) < 1e-9f) return y0;
  float a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom;
  float b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / denom;
  float c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / denom;
  return a * x * x + b * x + c;
}

uint16_t logarithmicInterpolation(float x0, float y0, float x1, float y1, float x) {
  if (x0 <= 0.0f || x1 <= 0.0f || x <= 0.0f || x0 == x1) return 0;
  float a = (y1 - y0) / (logf(x1) - logf(x0));
  float b = y0 - a * logf(x0);
  return (uint16_t)roundf(a * logf(x) + b);
}

double expInterpolationSolveY(double x, double x0, double x1, double y0, double y1) {
  if (x0 <= 0.0 || x1 <= 0.0 || y0 <= 0.0 || y1 <= 0.0 || x0 == x1) return NAN;
  double log_y0 = log(y0);
  double log_y1 = log(y1);
  return exp(log_y0 + (log_y1 - log_y0) * (x - x0) / (x1 - x0));
}

static constexpr float kGuessMinSpread = 0.10f;

// Least-squares quadratic through 4+ points
static float lsq_quadratic(const float *xs, const float *ys, const int *idx, int n, float targetX) {
  double s0 = (double)n;
  double s1 = 0.0, s2 = 0.0, s3 = 0.0, s4 = 0.0;
  double t0 = 0.0, t1 = 0.0, t2 = 0.0;

  for (int k = 0; k < n; ++k) {
    const double u  = (double)xs[idx[k]] - (double)targetX;
    const double u2 = u * u;
    const double y  = (double)ys[idx[k]];
    s1 += u;   s2 += u2;   s3 += u2 * u;   s4 += u2 * u2;
    t0 += y;   t1 += u * y; t2 += u2 * y;
  }

  const double det = s0 * (s2 * s4 - s3 * s3)
                   - s1 * (s1 * s4 - s3 * s2)
                   + s2 * (s1 * s3 - s2 * s2);

  const double scale = s2 / (double)n;
  if (!(det > 1e-9 * scale * scale * scale)) return NAN;

  return (float)((t0 * (s2 * s4 - s3 * s3)
                - s1 * (t1 * s4 - s3 * t2)
                + s2 * (t1 * s3 - s2 * t2)) / det);
}

static float extrapolate_amp_for_freq(int nPoints, float x1, float y1, float x2, float y2, float x3, float y3, float targetX) {
  if (nPoints >= 3) return quadraticInterpolation(x3, y3, x2, y2, x1, y1, targetX);
  if (nPoints == 2) return (float)logarithmicInterpolation(x2, y2, x1, y1, targetX);
  return (x1 > 0.0f) ? y1 * (targetX / x1) : y1;
}

static float freq_trace_guess(const float *xs, const float *ys, int count, float x) {
  if (count <= 0) return 0.0f;

  int idx[4];
  int n = 0;

  auto far_enough = [&](int cand) {
    for (int k = 0; k < n; ++k) {
      const float a = xs[cand], b = xs[idx[k]];
      const float ref = fmaxf(fabsf(a), fabsf(b));
      if (ref <= 0.0f || fabsf(a - b) < kGuessMinSpread * ref) return false;
    }
    return true;
  };

  auto pick = [&](int side) {
    int best = -1;
    for (int i = 0; i < count; ++i) {
      bool used = false;
      for (int k = 0; k < n; ++k) { if (idx[k] == i) used = true; }
      if (used) continue;
      if (side < 0 && xs[i] > x) continue;
      if (side > 0 && xs[i] < x) continue;
      if (!far_enough(i)) continue;
      if (best < 0 || fabsf(xs[i] - x) < fabsf(xs[best] - x)) best = i;
    }
    return best;
  };

  int cand = pick(-1); if (cand >= 0) idx[n++] = cand;
  cand = pick(+1);     if (cand >= 0) idx[n++] = cand;
  while (n < 4) {
    cand = pick(0);
    if (cand < 0) break;
    idx[n++] = cand;
  }
  if (n == 0) return 0.0f;

  // Order nearest-first
  for (int i = 1; i < n; ++i) {
    for (int j = i; j > 0 && fabsf(xs[idx[j]] - x) < fabsf(xs[idx[j - 1]] - x); --j) {
      int t = idx[j]; idx[j] = idx[j - 1]; idx[j - 1] = t;
    }
  }

  if (n >= 4) {
    float fit = lsq_quadratic(xs, ys, idx, n, x);
    if (!isnan(fit)) return fit;
    n = 3;
  }

  return extrapolate_amp_for_freq(n, xs[idx[0]], ys[idx[0]],
                                  (n > 1) ? xs[idx[1]] : 0.0f, (n > 1) ? ys[idx[1]] : 0.0f,
                                  (n > 2) ? xs[idx[2]] : 0.0f, (n > 2) ? ys[idx[2]] : 0.0f, x);
}

static float amp0_fit_freq(const float *amps, const float *freqs, int count) {
  int idx[kAmp0FitPoints];
  int n = 0;

  while (n < kAmp0FitPoints) {
    int best = -1;
    for (int i = 0; i < count; ++i) {
      if (!(amps[i] > 0.0f && freqs[i] > 0.0f)) continue;
      bool usable = true;
      for (int k = 0; k < n; ++k) {
        if (idx[k] == i || fabsf(amps[i] - amps[idx[k]]) < kGuessMinSpread * fmaxf(amps[i], amps[idx[k]])) {
          usable = false; break;
        }
      }
      if (usable && (best < 0 || amps[i] < amps[best])) best = i;
    }
    if (best < 0) break;
    idx[n++] = best;
  }
  if (n < 2) return 0.0f;

  double sumA = 0.0, sumF = 0.0, sumAA = 0.0, sumAF = 0.0;
  float lowestFreq = freqs[idx[0]];
  for (int k = 0; k < n; ++k) {
    double a = (double)amps[idx[k]];
    double f = (double)freqs[idx[k]];
    sumA += a; sumF += f; sumAA += a * a; sumAF += a * f;
    if (freqs[idx[k]] < lowestFreq) lowestFreq = freqs[idx[k]];
  }

  double det = (double)n * sumAA - sumA * sumA;
  if (det <= 0.0) return 0.0f;

  double slope     = ((double)n * sumAF - sumA * sumF) / det;
  double intercept = (sumF - slope * sumA) / (double)n;

  if (slope <= 0.0 || !(intercept > 0.0) || intercept >= (double)lowestFreq) return 0.0f;

  Serial.println((String)"[AMP0_FIT] DCO=" + currentDCO + " f0=" + fmt_freq((float)intercept) +
                 " Hz slope=" + (float)(1.0 / slope) + " cnt/Hz points=" + n);
  return (float)intercept;
}

static float freq_trace_local_slope(const float *freqs, const float *amps, int count, float freqRef) {
  if (freqRef <= 0.0f) return 1.0f;
  int i1 = -1, i2 = -1;
  for (int i = 0; i < count; ++i) {
    if (freqs[i] <= 0.0f || amps[i] <= 0.0f) continue;
    float d = fabsf(logf(freqs[i] / freqRef));
    if (i1 < 0 || d < fabsf(logf(freqs[i1] / freqRef))) { i2 = i1; i1 = i; }
    else if (i2 < 0 || d < fabsf(logf(freqs[i2] / freqRef))) { i2 = i; }
  }
  if (i1 < 0 || i2 < 0 || amps[i1] == amps[i2] || freqs[i1] == freqs[i2]) return 1.0f;

  float s = logf(freqs[i2] / freqs[i1]) / logf(amps[i2] / amps[i1]);
  if (!(s > 0.5f)) return 0.5f;
  if (s > 2.0f)    return 2.0f;
  return s;
}

static float freq_trace_power_seed(const float *freqs, const float *amps, int count, float ampTarget) {
  if (ampTarget <= 0.0f) return 0.0f;
  int top = -1;
  for (int i = 0; i < count; ++i) {
    if (freqs[i] <= 0.0f || amps[i] <= 0.0f) continue;
    if (top < 0 || amps[i] > amps[top]) top = i;
  }
  if (top < 0) return 0.0f;
  float s = freq_trace_local_slope(freqs, amps, count, freqs[top]);
  return freqs[top] * powf(ampTarget / amps[top], s);
}

// =============================================================================
// Low-Level Search and Probing Primitives
// =============================================================================

double compute_gap_tolerance_for_freq(double freqHz, double dutyErrorFraction) {
  if (freqHz <= 0.0) return 1e6;
  return 2.0 * dutyErrorFraction * (1e6 / freqHz);
}

static inline float calibration_interval_ratio() {
  return powf(2.0f, (float)calibration_note_interval / 12.0f);
}

static float freq_move_cents(float fromHz, float toHz) {
  if (fromHz <= 0.0f || toHz <= 0.0f) return 1e9f;
  return fabsf(1200.0f * log2f(toHz / fromHz));
}

static void wait_periods(float freqHz, float periods, uint32_t minUs) {
  uint32_t us = minUs;
  if (freqHz > 0.0f) {
    uint32_t p = (uint32_t)(periods * 1000000.0f / freqHz + 0.999f);
    if (p > us) us = p;
  }
  if (us >= 1000u) delay(us / 1000u);
  uint32_t rem = us % 1000u;
  if (rem) delayMicroseconds(rem);
}

static void drive_freq(float freqHz, uint16_t amp) {
  calibrationFreqHz     = freqHz;
  ampCompCalibrationVal = amp;

  // EXPLICITLY PROGRAM THE RANGE PWM SLICE HARDWARE
  write_range_pwm(currentDCO, amp);
  autotune_drive_core(currentDCO, freqHz, amp);
  g_lastDrivenFreqHz    = freqHz;
}

static float search_step_cap_cents(float freqHz) {
  if (freqHz >= kSearchStepHighHz)    return kSearchStepCentsHigh;
  if (freqHz >= kSearchStepLowHz)     return kSearchStepCentsMid;
  if (freqHz >= kSearchStepVeryLowHz) return kSearchStepCentsLow;
  return kSearchStepCentsVeryLow;
}

static double snap_min_freq_step(double from, double to) {
  if (fabs(to - from) >= (double)kMinFreqStepHz) return to;
  return (to >= from) ? (from + (double)kMinFreqStepHz) : (from - (double)kMinFreqStepHz);
}

static double next_probe_in_bracket(double fLo, double fHi, double gLo, double gHi, bool edgeFromTimeout, double noiseGapUs) {
  double lLo = log(fLo);
  double lHi = log(fHi);

  bool interpolate = (autotuneSearchMode != SEARCH_BISECT) && !edgeFromTimeout && gLo < 0.0 && gHi > 0.0;
  if (interpolate && autotuneSearchMode == SEARCH_GATED) {
    interpolate = (-gLo > noiseGapUs) && (gHi > noiseGapUs);
  }

  double next = 0.0;
  if (interpolate) {
    next = exp(lLo + (lHi - lLo) * (gLo / (gLo - gHi)));
    double guardLo = exp(lLo + (lHi - lLo) * 0.05);
    double guardHi = exp(lHi - (lHi - lLo) * 0.05);
    if (!(next > guardLo && next < guardHi)) next = 0.0;
  }
  return (next > 0.0) ? next : sqrt(fLo * fHi);
}

float measure_duty_at_freq(float freqHz, uint16_t amp, bool hiRes) {
  const CalPrecisionProfile &prec = cal_precision();
  float movedCents = freq_move_cents(g_lastDrivenFreqHz, freqHz);
  bool  bigMove    = (movedCents >= kSettleBigMoveCents);

  gapGateFreqHz = freqHz;
  drive_freq(freqHz, amp);
  wait_periods(freqHz, prec.settlePeriods, prec.settleMinMs * 1000u);

  uint8_t mode = hiRes ? 3 : 0;
  ++g_lastFreqBisectProbes;
  ++calRunProbes;

  GapMeasurement gm = measure_gap(mode);
  if (gm.timedOut && bigMove) {
    ++g_lastFreqBisectProbes;
    ++g_lastSettleChecks;
    ++calRunProbes;
    gm = measure_gap(mode);
  }
  if (gm.timedOut) {
    if (autotuneDebug >= 2) {
      Serial.println((String)"  [FREQ_PROBE] DCO=" + currentDCO + " Freq=" + fmt_freq(freqHz) + "Hz" +
      " TargetAMP=" + amp + " (HW RangeCC=" + range_level_readback(currentDCO) +
      " PW_CC=" + pw_level_readback(cal_pw_channel(currentDCO)) + ") -> TIMEOUT");
    }
    gapGateFreqHz = 0.0f;
    return kGapTimeoutSentinel;
  }

  int checks = (movedCents >= kSettleSkipCents) ? (bigMove ? (int)prec.settleMaxChecks : 1) : 0;
  if (amp == 0 && checks < 3) checks = 3;

  double stableTol = compute_gap_tolerance_for_freq(freqHz, prec.bisectDutyTol);
  if (stableTol < prec.bisectGapFloorUs) stableTol = prec.bisectGapFloorUs;
  stableTol *= (double)prec.settleStableMult;

  float value   = gm.value;
  bool  settled = (checks == 0);
  g_lastDutyUnsettled = false;

  for (int c = 0; c < checks; ++c) {
    if (calibrationCancelRequested) break;
    ++g_lastFreqBisectProbes;
    ++g_lastSettleChecks;
    ++calRunProbes;
    GapMeasurement again = measure_gap(mode);
    if (again.timedOut) continue;

    if (fabsf(again.value - value) <= (float)stableTol) {
      value   = 0.5f * (value + again.value);
      settled = true;
      break;
    }
    if (amp == 0) {
      if (fabsf(again.value) < fabsf(value)) value = again.value;
    } else {
      value = again.value;
    }
  }

  if (!settled && amp == 0) g_lastDutyUnsettled = true;
  gapGateFreqHz = 0.0f;

  float result = -(value - duty_trim_gap_us(currentDCO, freqHz));
  if (autotuneDebug >= 2) {
    Serial.println((String)"  [FREQ_PROBE] DCO=" + currentDCO + " Freq=" + fmt_freq(freqHz) + "Hz" +
    " TargetAMP=" + amp + " (HW RangeCC=" + range_level_readback(currentDCO) +
    " PW_CC=" + pw_level_readback(cal_pw_channel(currentDCO)) + ")" +
    " -> GapUs=" + result + " DutyErr=" + String(duty_err_pct_from_gap(result, freqHz), 3) + "%");
  }
  return result;
}

float find_freq_for_duty50(uint16_t amp, float freqGuess, float windowRatio, bool refine, const FreqSearchBounds *bounds) {
  if (windowRatio < 1.05f) windowRatio = 1.05f;
  if (freqGuess <= 0.0f)   return 0.0f;

  double boundLo = (bounds != nullptr) ? (double)bounds->loHz : 0.0;
  double boundHi = (bounds != nullptr) ? (double)bounds->hiHz : 0.0;
  bool   bounded = (boundLo > 0.0 && boundHi > boundLo);

  if (bounded) {
    if (freqGuess < (float)boundLo) freqGuess = (float)boundLo;
    if (freqGuess > (float)boundHi) freqGuess = (float)boundHi;
  }

  ampCompCalibrationVal  = amp;
  g_lastFreqBisectGapUs  = kGapTimeoutSentinel;
  g_lastFreqBisectProbes = 0;
  g_lastSettleChecks     = 0;

  const CalPrecisionProfile &prec = cal_precision();
  int   maxProbes   = refine ? prec.bisectIters : 24;
  float windowCents = 1200.0f * log2f(windowRatio);
  int   windows     = refine ? prec.bisectWindows : 2;
  float travelBudget = windowCents * (float)(windows + 1);

  float bestFreq = 0.0f, bestAbsGap = 1e9f, bestSignedGap = 0.0f;
  double fLo = 0.0, fHi = 0.0, gLo = 0.0, gHi = 0.0;
  bool hiFromTimeout = false, loFromTimeout = false;
  int lastSide = 0, timeouts = 0;

  double f = (double)freqGuess;
  float stepCents = fminf(search_step_cap_cents(freqGuess), 0.25f * windowCents);
  float travelled = 0.0f;
  double lowGoodFreq = 0.0, highGoodFreq = 0.0;
  double tol = prec.bisectGapFloorUs;

  for (int probe = 0; probe < maxProbes; ++probe) {
    if (calibrationCancelRequested) return bestFreq;

    float diff     = measure_duty_at_freq((float)f, amp, refine);
    bool  timedOut = (diff == kGapTimeoutSentinel);
    bool  signOnly = timedOut || (amp == 0 && g_lastDutyUnsettled);

    if (!timedOut) {
      if (f > highGoodFreq) highGoodFreq = f;
      if (lowGoodFreq == 0.0 || f < lowGoodFreq) lowGoodFreq = f;
      if (fabsf(diff) < bestAbsGap) {
        bestAbsGap    = fabsf(diff);
        bestSignedGap = diff;
        bestFreq      = (float)f;
      }
      tol = compute_gap_tolerance_for_freq(f, prec.bisectDutyTol);
      if (tol < prec.bisectGapFloorUs) tol = prec.bisectGapFloorUs;
      if (fabsf(diff) <= tol) break;
    }

    int side = 0;
    if (!timedOut) {
      side = (diff > 0.0f) ? +1 : -1;
      timeouts = 0;
    } else {
      ++timeouts;
      if (lowGoodFreq > 0.0 && f < lowGoodFreq) side = -1;
      else side = +1;
    }

    if (side > 0) { fHi = f; gHi = timedOut ? 0.0 : (double)diff; hiFromTimeout = signOnly; }
    else          { fLo = f; gLo = timedOut ? 0.0 : (double)diff; loFromTimeout = signOnly; }

    if (side == lastSide) {
      if (side > 0) gLo *= 0.5;
      else          gHi *= 0.5;
    }
    lastSide = side;

    if (!bounded && timeouts >= kMaxSearchTimeouts) break;

    if (fLo > 0.0 && fHi > 0.0) {
      double widthHz    = fHi - fLo;
      double widthCents = 1200.0 * log2(fHi / fLo);
      if (widthHz < (double)kMinFreqStepHz || (widthCents < 3.0 && bestAbsGap <= tol)) break;

      double prevF = f;
      f = next_probe_in_bracket(fLo, fHi, gLo, gHi, hiFromTimeout || loFromTimeout, tol * (double)prec.settleStableMult);
      f = snap_min_freq_step(prevF, f);
      if (f <= fLo) f = fLo + (double)kMinFreqStepHz;
      if (f >= fHi) f = fHi - (double)kMinFreqStepHz;
      if (f <= fLo || f >= fHi) break;
      continue;
    }

    if (travelled >= travelBudget) break;

    if (!timedOut) {
      float propCents = fabsf(duty_err_pct_from_gap(diff, (float)f)) * (100.0f / kSearchSlopeMinPctPer100Cents);
      stepCents = fminf(stepCents, fmaxf(propCents, kSearchStepFloorCents));
    }

    double prev = f;
    f = f * pow(2.0, (side > 0 ? -1.0 : 1.0) * (double)stepCents / 1200.0);
    f = snap_min_freq_step(prev, f);

    if (bounded) {
      if (f < boundLo) f = boundLo;
      if (f > boundHi) f = boundHi;
      if (f == prev) break;
    }
    travelled += stepCents;

    bool nothingMeasuredYet = (highGoodFreq == 0.0);
    stepCents = fminf(stepCents * 1.6f, (bounded && nothingMeasuredYet) ? kHuntStepMaxCents : search_step_cap_cents((float)f));
    if (timedOut && !nothingMeasuredYet) stepCents *= 0.5f;
  }

  if (bestAbsGap >= 1e9f) return 0.0f;
  if (!refine) {
    g_lastFreqBisectGapUs = bestSignedGap;
    return bestFreq;
  }

  // Refinement Confirm
  int confirmReads  = (prec.confirmReads  < 1) ? 1 : prec.confirmReads;
  int confirmRounds = (prec.confirmRounds < 1) ? 1 : prec.confirmRounds;
  float bestConfirmedFreq = 0.0f, bestConfirmedGap = 1e9f, bestConfirmedSign = 0.0f;
  f = (double)bestFreq;

  for (int round = 0; round < confirmRounds; ++round) {
    if (calibrationCancelRequested) break;
    float sum = 0.0f;
    int   n   = 0;
    for (int read = 0; read < confirmReads; ++read) {
      float d = measure_duty_at_freq((float)f, amp, true);
      if (d == kGapTimeoutSentinel) { n = 0; break; }
      sum += d;
      ++n;
    }
    if (n == 0) break;

    float avg = sum / (float)n;
    if (fabsf(avg) < bestConfirmedGap) {
      bestConfirmedGap  = fabsf(avg);
      bestConfirmedSign = avg;
      bestConfirmedFreq = (float)f;
    }

    double acceptFrac = (amp == 0) ? ((double)kEndpointAcceptDutyPct / 100.0) : prec.bisectDutyTol;
    double roundTol = compute_gap_tolerance_for_freq(f, acceptFrac);
    if (roundTol < prec.bisectGapFloorUs) roundTol = prec.bisectGapFloorUs;
    if (fabsf(avg) <= roundTol) break;

    if (avg > 0.0f) { fHi = f; gHi = (double)avg; hiFromTimeout = false; }
    else            { fLo = f; gLo = (double)avg; }
    if (!(fLo > 0.0 && fHi > 0.0)) break;

    double next = next_probe_in_bracket(fLo, fHi, gLo, gHi, hiFromTimeout, roundTol * (double)prec.settleStableMult);
    if (!(next > 0.0)) break;
    f = snap_min_freq_step(f, next);
  }

  if (bestConfirmedGap >= 1e9f) {
    g_lastFreqBisectGapUs = bestSignedGap;
    return bestFreq;
  }

  float searchDuty  = fabsf(duty_err_pct_from_gap(bestAbsGap, bestFreq));
  float confirmDuty = fabsf(duty_err_pct_from_gap(bestConfirmedGap, bestConfirmedFreq));
  float acceptPct   = (amp == 0) ? kEndpointAcceptDutyPct : (float)(prec.bisectDutyTol * 100.0);

  if (searchDuty <= acceptPct && confirmDuty > acceptPct) {
    g_lastFreqBisectGapUs = bestSignedGap;
    return bestFreq;
  }

  g_lastFreqBisectGapUs = bestConfirmedSign;
  return bestConfirmedFreq;
}

// =============================================================================
// Endpoint & Scan Helpers
// =============================================================================

static FreqSearchBounds amp0_search_band(float firstPairHz) {
  float hi = firstPairHz * 0.99f;
  float lo = firstPairHz / kAmp0BandRatio;
  if (lo < kAmp0MinFreqHz) lo = kAmp0MinFreqHz;
  if (!(lo < hi)) lo = hi / 1.05f;
  return { lo, hi };
}

static float scan_duty_at_freq(float freqHz, uint16_t amp) {
  ampCompCalibrationVal = amp;
  gapGateFreqHz = freqHz;

  drive_freq(freqHz, amp);

  uint32_t settleMs = kAmp0ScanSettleMs;
  if (freqHz > 0.0f) {
    uint32_t onePeriodMs = (uint32_t)(1000.0f / freqHz) + 1;
    if (onePeriodMs > settleMs) settleMs = onePeriodMs;
  }
  delay(settleMs);
  ++calRunProbes;

  GapMeasurement gm = measure_gap(3);
  gapGateFreqHz = 0.0f;
  if (gm.timedOut) return kGapTimeoutSentinel;
  return -(gm.value - duty_trim_gap_us(currentDCO, freqHz));
}

static FreqSearchBounds amp0_prescan(FreqSearchBounds band, float fallbackHz, float *seedOut) {
  *seedOut = fallbackHz;
  g_lastAmp0ScanSeedHz = 0.0f;
  if (!(band.loHz > 0.0f && band.hiHz > band.loHz) || kAmp0ScanPoints < 2) return band;

  double ratio = pow((double)band.loHz / (double)band.hiHz, 1.0 / (double)(kAmp0ScanPoints - 1));
  float bestFreq = 0.0f, bestGap = 0.0f;
  bool found = false;
  float aboveFreq = 0.0f, aboveGap = 0.0f, belowFreq = 0.0f, belowGap = 0.0f;
  double f = (double)band.hiHz;

  for (int i = 0; i < kAmp0ScanPoints; ++i, f *= ratio) {
    if (calibrationCancelRequested) break;
    float gap = scan_duty_at_freq((float)f, 0);
    if (gap == kGapTimeoutSentinel) continue;

    if (!found || fabsf(gap) < fabsf(bestGap)) {
      bestFreq = (float)f;
      bestGap  = gap;
      found    = true;
    }
    if (gap > 0.0f) {
      aboveFreq = (float)f; aboveGap = gap;
      continue;
    }
    belowFreq = (float)f; belowGap = gap;
    break;
  }

  if (belowFreq > 0.0f && aboveFreq > belowFreq && aboveGap > belowGap) {
    double t = (double)(-belowGap) / (double)(aboveGap - belowGap);
    *seedOut = (float)((double)belowFreq * pow((double)aboveFreq / (double)belowFreq, t));
    g_lastAmp0ScanSeedHz = *seedOut;
    return { belowFreq, aboveFreq };
  }

  if (found) *seedOut = bestFreq;
  return band;
}

float measure_lowest_freq_at_amp0(float freqSeedHz, const FreqSearchBounds *bounds) {
  if (freqSeedHz <= 0.0f) return 0.0f;
  if (bounds == nullptr) {
    return find_freq_for_duty50(0, freqSeedHz, kBottomEndpointWindowRatio, true);
  }
  float seedHz = freqSeedHz;
  FreqSearchBounds bracket = amp0_prescan(*bounds, freqSeedHz, &seedHz);
  return find_freq_for_duty50(0, seedHz, kBottomEndpointWindowRatio, true, &bracket);
}

void apply_measured_lowest_freq(DCOCalibrationContext& ctx) {
  float prevHz = (float)ctx.calibrationData[0] / 100.0f;
  if (ctx.calibrationData[2] == 0) return;

  float firstPairHz = (float)ctx.calibrationData[2] / 100.0f;
  FreqSearchBounds bounds = amp0_search_band(firstPairHz);

  float seedHz = prevHz;
  if (!(seedHz >= bounds.loHz && seedHz <= bounds.hiHz)) {
    seedHz = (seedHz < bounds.loHz) ? bounds.loHz : bounds.hiHz;
  }

  float foundHz = measure_lowest_freq_at_amp0(seedHz, &bounds);
  if (foundHz <= 0.0f) return;

  float foundErr = duty_err_pct_from_gap(g_lastFreqBisectGapUs, foundHz);
  if (!(foundHz >= bounds.loHz && foundHz <= bounds.hiHz) || fabsf(foundErr) > kEndpointAcceptDutyPct) {
    return;
  }

  ctx.calibrationData[0] = (uint32_t)(foundHz * 100.0f);
  ctx.calibrationData[1] = 0;
  cal_report_set_pair_from_gap(0, g_lastFreqBisectGapUs, foundHz, CAL_SRC_ENDPOINT_AMP0);
}

float find_highest_freq(DCOCalibrationContext& ctx, int pairsFilled) {
  CalPrecisionOverride fineForEndpoint(
    (calibrationPrecision == CAL_PRECISION_FAST) ? calibrationPrecision : CAL_PRECISION_FINE);

  float knownFreq[kCalReportPairs];
  float knownAmp[kCalReportPairs];
  int knownCount = 0;
  if (pairsFilled > kCalReportPairs) pairsFilled = kCalReportPairs;

  for (int p = 1; p < pairsFilled; ++p) {
    float f = (float)ctx.calibrationData[2 * p] / 100.0f;
    float a = (float)ctx.calibrationData[2 * p + 1];
    if (f <= 0.0f || f >= 100000.0f || a <= 0.0f) continue;
    knownFreq[knownCount] = f;
    knownAmp[knownCount]  = a;
    ++knownCount;
  }

  float fSeed = freq_trace_power_seed(knownFreq, knownAmp, knownCount, (float)DIV_COUNTER);
  float windowRatio = kTopEndpointWindowRatio;
  if (fSeed <= 0.0f) {
    fSeed = note_to_freq(DCO_calibration_current_note);
    windowRatio = calibration_interval_ratio();
  }

  float bestFreq = find_freq_for_duty50(DIV_COUNTER, fSeed, windowRatio, true);
  float lastFreq = (knownCount > 0) ? knownFreq[knownCount - 1] : 0.0f;

  if (bestFreq <= lastFreq && knownCount > 0 && !calibrationCancelRequested) {
    float fRetry = lastFreq * calibration_interval_ratio();
    bestFreq = find_freq_for_duty50(DIV_COUNTER, fRetry, calibration_interval_ratio(), true);
  }
  if (bestFreq <= 0.0f) {
    bestFreq = (lastFreq > 0.0f) ? lastFreq * calibration_interval_ratio() : note_to_freq(DCO_calibration_current_note);
  }

  return bestFreq * 100.0f;
}

float find_lowest_freq() {
  ampCompCalibrationVal = 0;
  if (chanLevelVoiceDataSize < 8) return 0.0f;

  float fitAmps[kAmp0FitPoints + 3];
  float fitFreqs[kAmp0FitPoints + 3];
  int fitCount = 0;

  for (int j = 2; j + 1 < chanLevelVoiceDataSize && fitCount < (int)(sizeof(fitAmps) / sizeof(fitAmps[0])); j += 2) {
    float fHz = (float)calibrationData[j] / 100.0f;
    float amp = (float)calibrationData[j + 1];
    if (!(fHz > 0.0f) || !(amp > 0.0f)) continue;
    fitAmps[fitCount]  = amp;
    fitFreqs[fitCount] = fHz;
    ++fitCount;
  }

  float fitHz = amp0_fit_freq(fitAmps, fitFreqs, fitCount);
  if (fitHz > 0.0f) return fitHz * 100.0f;

  float f0 = (float)calibrationData[2], p0 = (float)calibrationData[3];
  float f1 = (float)calibrationData[4], p1 = (float)calibrationData[5];
  float f2 = (float)calibrationData[6], p2 = (float)calibrationData[7];

  if (p0 == p1 || p1 == p2 || p0 == p2) {
    return linearInterpolation(p0, f0, p1, f1, 0.0f);
  }

  float est = quadraticInterpolation(p0, f0, p1, f1, p2, f2, 0.0f);
  return (est < 0.0f) ? 0.0f : est;
}

// =============================================================================
// Calibration Routine A: Classic Step Search
// =============================================================================

static float measure_gap_for_amp(uint16_t ampPwm) {
  const float freqHz = note_to_freq(DCO_calibration_current_note);
  ampCompCalibrationVal = ampPwm;

  // EXPLICITLY PROGRAM THE RANGE PWM SLICE HARDWARE
  write_range_pwm(currentDCO, ampPwm);
  autotune_drive_core(currentDCO, note_to_freq(DCO_calibration_current_note), ampPwm);
  settle_for_freq((double)freqHz);
  ++calRunProbes;

  GapMeasurement gm = measure_gap(0);

  if (gm.timedOut) {
    if (autotuneDebug >= 2) {
      Serial.println((String)"  [AMP_PROBE] DCO=" + currentDCO + " TargetAMP=" + ampPwm +
      " (HW RangeCC=" + range_level_readback(currentDCO) + ") -> TIMEOUT");
    }
    return kGapTimeoutSentinel;
  }

  float result = -(gm.value - duty_trim_gap_us(currentDCO, freqHz));
  if (autotuneDebug >= 2) {
    double dutyErrPct = duty_err_pct_from_gap(result, freqHz);
    Serial.println((String)"  [AMP_PROBE] DCO=" + currentDCO + " TargetAMP=" + ampPwm +
    " (HW RangeCC=" + range_level_readback(currentDCO) + ")" +
    " -> GapUs=" + result + " DutyErr=" + String(dutyErrPct, 3) + "%");
  }
  return result;
}

// Helper: compute initial amplitude (range PWM) guess with strict anti-runaway clamping
static uint16_t compute_initial_amp_for_note(
  const DCOCalibrationContext& ctx,
  int j
) {
  float guess = 0.0f;
  float prevAmp = (j >= 6) ? (float)ctx.calibrationData[j - 1] : (float)ctx.calibrationData[3];
  if (prevAmp < 1.0f) prevAmp = (float)ctx.initManualAmpByOsc[ctx.dcoIndex];

  if (j == 4) {
    guess = (float)(ctx.initManualAmpByOsc[ctx.dcoIndex] + ctx.manualOffsetByOsc[ctx.dcoIndex]) * 1.35f;
  } else if (j == 6) {
    guess = (float)logarithmicInterpolation(
      ctx.calibrationData[2],
      ctx.calibrationData[3],
      ctx.calibrationData[4],
      ctx.calibrationData[5],
      note_to_freq(ctx.currentNote) * 100.0f
    );
  } else {
    guess = quadraticInterpolation(
      (float)ctx.calibrationData[j - 6], (float)ctx.calibrationData[j - 5],
                                   (float)ctx.calibrationData[j - 4], (float)ctx.calibrationData[j - 3],
                                   (float)ctx.calibrationData[j - 2], (float)ctx.calibrationData[j - 1],
                                   note_to_freq(ctx.currentNote) * 100.0f
    );
  }

  // Anti-Runaway Clamp:
  // For a 5-semitone interval (~1.33x freq), amp comp should realistically grow by 1.1x to 1.6x.
  // If the polynomial dips negative, is NaN, or explodes higher than 1.8x, fall back to safe scaling.
  if (isnan(guess) || guess < prevAmp || guess > (prevAmp * 1.8f)) {
    guess = prevAmp * 1.33f;
  }

  if (guess < 1.0f) guess = 1.0f;
  if (guess > (float)DIV_COUNTER) guess = (float)DIV_COUNTER;

  return (uint16_t)lroundf(guess);
}

void calibrate_DCO(DCOCalibrationContext& ctx, double dutyErrorFraction) {
  const int rangeSamples = 2;
  const int numPresetVoltages = chanLevelVoiceDataSize;
  const int kMaxSearchIterations = 150;
  const unsigned long kMaxNoteSearchMs = 25000;
  const int kMaxConsecutiveTimeouts = 20;

  Serial.println((String)"\n=======================================================");
  Serial.println((String)"[DCO_CAL] Starting Classic Amp Calibration for DCO=" + ctx.dcoIndex);
  Serial.println("=======================================================");

  for (int j = 4; j < numPresetVoltages; j += 2) {
    if (calibrationCancelRequested) return;

    ctx.currentNote = DCO_calibration_start_note + (calibration_note_interval * (j - 4) / 2);
    VOICE_NOTES[0] = ctx.currentNote;

    uint16_t initialAmpGuess = compute_initial_amp_for_note(ctx, j);
    uint16_t currentAmpCompCalibrationVal = initialAmpGuess;

    const double freqHz = note_to_freq(ctx.currentNote);
    double tolerance = compute_gap_tolerance_for_freq(freqHz, dutyErrorFraction);
    const double periodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;

    Serial.println((String)"\n--- [DCO=" + ctx.dcoIndex + " Note=" + ctx.currentNote +
    " (" + fmt_freq((float)freqHz) + " Hz)] Initial Guess AMP=" + currentAmpCompCalibrationVal + " ---");

    // Only abort if a real, verified note reached the top rail
    if (currentAmpCompCalibrationVal >= (uint16_t)(DIV_COUNTER * 0.98f)) {
      Serial.println((String)"[DCO_CAL] Reached physical PWM ceiling at Note=" + ctx.currentNote);
      float highestFreqFound = find_highest_freq(ctx, j / 2);
      float lowestFreqCalc   = find_lowest_freq();

      ctx.calibrationData[j]     = (uint32_t)highestFreqFound;
      ctx.calibrationData[j + 1] = DIV_COUNTER;
      cal_report_set_pair_from_gap(j / 2, g_lastFreqBisectGapUs, highestFreqFound / 100.0f, CAL_SRC_ENDPOINT_FULL);

      ctx.calibrationData[0] = (uint32_t)lowestFreqCalc;
      ctx.calibrationData[1] = 0;

      for (int i = j + 2; i < numPresetVoltages; i += 2) {
        ctx.calibrationData[i]     = 20000000;
        ctx.calibrationData[i + 1] = DIV_COUNTER;
        cal_report_set_pair(i / 2, kCalDutyErrUnknown, CAL_SRC_SENTINEL);
      }
      break;
    }

    // Dynamic wide search window
    uint16_t minAmpComp = (currentAmpCompCalibrationVal > 200) ? (currentAmpCompCalibrationVal - 200) : 1;
    uint16_t maxAmpComp = min((uint32_t)DIV_COUNTER, (uint32_t)(currentAmpCompCalibrationVal + 300));

    autotune_drive_core(currentDCO, freqHz, currentAmpCompCalibrationVal);
    delay(20);

    uint16_t bestAmpComp = currentAmpCompCalibrationVal;
    float closestToZero = 50000.0f, previousAvgValue = 0.0f;
    int flipCounter = 0, consecutiveTimeouts = 0;
    unsigned long noteSearchStartMs = millis();

    for (int iteration = 0;; ++iteration) {
      if (calibrationCancelRequested) break;
      if (iteration >= kMaxSearchIterations || (millis() - noteSearchStartMs) > kMaxNoteSearchMs) {
        Serial.println((String)"  [TIMEOUT_GUARD] Max iterations reached. Keeping best AMP=" + bestAmpComp);
        break;
      }

      float avgValue = measure_gap_for_amp(currentAmpCompCalibrationVal);

      if (avgValue == kGapTimeoutSentinel) {
        Serial.println((String)"  Iter " + iteration + ": AMP=" + currentAmpCompCalibrationVal + " -> TIMEOUT (No signal)");
        if (++consecutiveTimeouts >= kMaxConsecutiveTimeouts) {
          Serial.println((String)"  [ABORT_NOTE] " + consecutiveTimeouts + " timeouts in a row. Keeping AMP=" + bestAmpComp);
          break;
        }
        // Step faster when lost in timeout territory
        int step = (consecutiveTimeouts > 4) ? 8 : 3;
        if (currentAmpCompCalibrationVal + step <= maxAmpComp) {
          currentAmpCompCalibrationVal += step;
        } else {
          maxAmpComp = min((uint32_t)DIV_COUNTER, (uint32_t)(maxAmpComp + 200));
          currentAmpCompCalibrationVal += step;
        }
        continue;
      }
      consecutiveTimeouts = 0;

      double dutyErrPct = duty_err_pct_from_gap(avgValue, freqHz);
      Serial.println((String)"  Iter " + iteration + ": AMP=" + currentAmpCompCalibrationVal +
      " -> gapUs=" + avgValue + " dutyErr=" + String(dutyErrPct, 3) + "%");

      if (fabsf(avgValue) < fabsf(closestToZero)) {
        closestToZero = avgValue;
        bestAmpComp = currentAmpCompCalibrationVal;
      }

      // Check for zero-crossing bracket
      if ((previousAvgValue > 0.0f && avgValue < 0.0f) || (previousAvgValue < 0.0f && avgValue > 0.0f)) {
        Serial.println("  --> Sign change crossed! Probing local neighbours...");
        for (int i = 0; i < rangeSamples; i++) {
          uint16_t lowV  = (currentAmpCompCalibrationVal > i + 1) ? (currentAmpCompCalibrationVal - (i + 1)) : 1;
          uint16_t highV = min((uint32_t)DIV_COUNTER, (uint32_t)(currentAmpCompCalibrationVal + (i + 1)));
          float lowM  = measure_gap_for_amp(lowV);
          float highM = measure_gap_for_amp(highV);
          if (fabsf(lowM) < fabsf(closestToZero))  { closestToZero = lowM;  bestAmpComp = lowV; }
          if (fabsf(highM) < fabsf(closestToZero)) { closestToZero = highM; bestAmpComp = highV; }
        }
        if (fabsf(closestToZero) <= tolerance) {
          Serial.println((String)"  [LOCKED] Optimal AMP=" + bestAmpComp + " (gapUs=" + closestToZero + ")");
          break;
        }
        tolerance *= 1.2;
        if (++flipCounter >= 3 && fabsf(closestToZero) <= tolerance * 2) break;
        tolerance *= 1.5;
      }

      // Adaptive step size based on error magnitude
      int magnitude = (fabsf(avgValue) < tolerance * 15) ? 1 : ((fabsf(avgValue) < tolerance * 40) ? 2 : 4);
      int32_t nextAmp = (int32_t)currentAmpCompCalibrationVal + ((avgValue > 0) ? magnitude : -magnitude);

      // Auto-widen window if pushing against boundary without sign change
      if (nextAmp > (int32_t)maxAmpComp && maxAmpComp < DIV_COUNTER) {
        maxAmpComp = min((uint32_t)DIV_COUNTER, (uint32_t)(maxAmpComp + 250));
      }
      if (nextAmp < (int32_t)minAmpComp && minAmpComp > 1) {
        minAmpComp = (minAmpComp > 150) ? (minAmpComp - 150) : 1;
      }

      nextAmp = constrain(nextAmp, (int32_t)minAmpComp, (int32_t)maxAmpComp);

      if ((uint16_t)nextAmp == currentAmpCompCalibrationVal) {
        Serial.println((String)"  [HARD_BOUND] Reached limit AMP=" + currentAmpCompCalibrationVal);
        break;
      }

      currentAmpCompCalibrationVal = (uint16_t)nextAmp;
      previousAvgValue = avgValue;
    }

    ctx.calibrationData[j]     = note_to_freq(ctx.currentNote) * 100;
    ctx.calibrationData[j + 1] = bestAmpComp;
    cal_report_set_pair_from_gap(j / 2, (fabsf(closestToZero) < 40000.0f) ? closestToZero : kGapTimeoutSentinel,
                                 note_to_freq(ctx.currentNote), CAL_SRC_RUNG);

    Serial.println((String)"[NOTE_RESULT] Note=" + ctx.currentNote + " Stored AMP=" + bestAmpComp +
    " Achieved Error=" + String(duty_err_pct_from_gap(closestToZero, freqHz), 3) + "%\n");
  }
}

// =============================================================================
// Calibration Routine B: FREQ_TRACE Method
// =============================================================================

static bool cal_table_is_monotonic(const uint32_t *data, int numPairs, uint8_t dcoIndex, const char *tag) {
  bool ok = true;
  uint32_t prevFreq = data[0], prevAmp = data[1];
  for (int p = 1; p < numPairs; ++p) {
    uint32_t f = data[2 * p], a = data[2 * p + 1];
    if (f < prevFreq || a < prevAmp) {
      Serial.println((String)"[" + tag + "] DCO=" + dcoIndex + " non-monotonic at pair " + p +
                     " (freq " + prevFreq + "->" + f + ", amp " + prevAmp + "->" + a + ")");
      ok = false;
    }
    prevFreq = f; prevAmp = a;
  }
  return ok;
}

static String freq_trace_quality(float gapUs, float freqHz, int probes, int settleChecks) {
  return (String)" gapUs=" + gapUs + " dutyErr=" + duty_err_pct_from_gap(gapUs, freqHz) +
         "% probes=" + probes + " settle=" + settleChecks;
}

struct FreqTraceProbeInfo {
  float gapUs;
  int   probes;
  int   settleChecks;
};

bool calibrate_DCO_freq_trace(DCOCalibrationContext& ctx) {
  constexpr int numPairs  = (int)(chanLevelVoiceDataSize / 2);
  constexpr int firstRung = 1;
  constexpr int lastRung  = numPairs - 2;
  constexpr int nRungs    = lastRung - firstRung + 1;
  const float   r         = calibration_interval_ratio();

  constexpr int kMaxKnown = numPairs + 8;
  float knownFreq[kMaxKnown];
  float knownAmp[kMaxKnown];
  int   knownCount = 0;

  auto add_known = [&](float f, float a) {
    if (knownCount < kMaxKnown) {
      knownFreq[knownCount] = f; knownAmp[knownCount] = a; ++knownCount;
    }
  };

  float    freqByPair[numPairs];
  uint16_t ampByPair[numPairs];

  // --- 1. Anchor Diagnostic & Probe ---
  uint16_t anchorAmp = ampComp440[ctx.dcoIndex];
  uint8_t  pwCh      = cal_pw_channel(ctx.dcoIndex);

  Serial.println((String)"[FREQ_TRACE_INIT] DCO=" + ctx.dcoIndex +
  " Recalled anchorAmp=" + anchorAmp +
  " | PW Channel=" + pwCh + " (GP" + PW_PINS[pwCh] +
  ") PW_CENTER=" + PW_CENTER[pwCh] +
  " (Hardware Readback CC=" + pw_level_readback(pwCh) + ")");

  if (anchorAmp == 0) {
    Serial.println((String)"[FREQ_TRACE_GUARD] DCO=" + ctx.dcoIndex +
    " 440 Hz anchor is 0! Run manual step 2 first.");
    return false;
  }

  float anchorFreq = find_freq_for_duty50(
    anchorAmp, note_to_freq(manual_cal_reference_note),
                                          kAnchorAcquireWindowRatio, true);

  if (calibrationCancelRequested || anchorFreq <= 0.0f) {
    Serial.println((String)"[FREQ_TRACE_GUARD] DCO=" + ctx.dcoIndex +
    " No signal at manual anchor amp=" + anchorAmp +
    " PW_raw=" + pw_level_readback(pwCh) +
    " (center=" + PW_CENTER[pwCh] + "); aborting.");
    return false;
  }

  add_known(anchorFreq, (float)anchorAmp);
  float anchorGapUs = g_lastFreqBisectGapUs;
  Serial.println((String)"[FREQ_TRACE] DCO=" + ctx.dcoIndex +
  " Anchor acquired: AMP=" + anchorAmp + " Freq=" + fmt_freq(anchorFreq) + " Hz" +
  freq_trace_quality(anchorGapUs, anchorFreq, g_lastFreqBisectProbes, g_lastSettleChecks));

  // 2. Manual Trim Point Measurement
  {
    int32_t manualAmp = (int32_t)ctx.initManualAmpByOsc[ctx.dcoIndex] + (int32_t)ctx.manualOffsetByOsc[ctx.dcoIndex];
    if (manualAmp < 1) manualAmp = 1;
    if (manualAmp > (int32_t)DIV_COUNTER) manualAmp = (int32_t)DIV_COUNTER;
    float nominalHz = note_to_freq(manual_DCO_calibration_start_note);
    float found = find_freq_for_duty50((uint16_t)manualAmp, nominalHz, kManualNoteWindowRatio, true);
    if (calibrationCancelRequested) return false;
    if (found > 0.0f) add_known(found, (float)manualAmp);
  }

  // 3. Anchor Refinement
  {
    float target440 = note_to_freq(manual_cal_reference_note);
    uint16_t storedAmp = anchorAmp;
    float cents = 1200.0f * log2f(anchorFreq / target440);
    int tries = cal_precision().anchorTries;

    for (int attempt = 0; attempt < tries && fabsf(anchorFreq - target440) > 0.1f; ++attempt) {
      if (calibrationCancelRequested) return false;
      float ampGuess = freq_trace_guess(knownFreq, knownAmp, knownCount, target440);
      int32_t ampNext = (int32_t)lroundf(ampGuess);
      if (ampNext < 1) ampNext = 1;
      if (ampNext > (int32_t)DIV_COUNTER) ampNext = (int32_t)DIV_COUNTER;
      if (ampNext == (int32_t)anchorAmp) ampNext += (anchorFreq < target440) ? 1 : -1;
      if (ampNext < 1 || ampNext > (int32_t)DIV_COUNTER) break;

      float found = find_freq_for_duty50((uint16_t)ampNext, target440, kAnchorWindowRatio, true);
      if (found <= 0.0f) break;
      add_known(found, (float)ampNext);

      float newCents = 1200.0f * log2f(found / target440);
      if (fabsf(newCents) < fabsf(cents)) {
        anchorAmp = (uint16_t)ampNext; anchorFreq = found; anchorGapUs = g_lastFreqBisectGapUs; cents = newCents;
      }
    }

    if (anchorAmp != storedAmp) {
      ampComp440[ctx.dcoIndex] = anchorAmp;
      update_FS_AmpComp440(ctx.dcoIndex, anchorAmp);
    }
  }

  // 4. Bootstrap Probes
  {
    int nBootstrap = (calibrationPrecision == CAL_PRECISION_FAST) ? 2 : 4;
    for (int b = 0; b < nBootstrap; ++b) {
      if (calibrationCancelRequested) return false;
      int32_t amp = lroundf((float)anchorAmp * exp2f((float)kBootstrapSemitones[b] / 12.0f));
      if (amp < 1) amp = 1;
      if (amp > (int32_t)DIV_COUNTER) amp = (int32_t)DIV_COUNTER;
      if (amp == (int32_t)anchorAmp) continue;

      float fSeed = freq_trace_guess(knownAmp, knownFreq, knownCount, (float)amp);
      if (fSeed <= 0.0f) fSeed = anchorFreq;
      float fFound = find_freq_for_duty50((uint16_t)amp, fSeed, r, true);
      if (fFound > 0.0f) add_known(fFound, (float)amp);
    }
  }

  // 5. Derive Ladder Geometry
  int ladderInterval = calibration_note_interval;
  int anchorPair     = firstRung + (nRungs - 1) / 2;
  {
    float fFloorHz = note_to_freq(manual_DCO_calibration_start_note);
    float fLowEst  = freq_trace_guess(knownAmp, knownFreq, knownCount, 1.0f);
    if (!(fLowEst > fFloorHz)) fLowEst = fFloorHz;

    float ampHigh  = (float)DIV_COUNTER * 0.98f;
    float fHighEst = freq_trace_guess(knownAmp, knownFreq, knownCount, ampHigh);

    if (fLowEst > 0.0f && fHighEst > fLowEst) {
      float spanSemi = 12.0f * log2f(fHighEst / fLowEst);
      int want = (int)ceilf(spanSemi / (float)(nRungs + 1));
      if (want < 3)  want = 3;
      if (want > 12) want = 12;
      ladderInterval = want;

      float frac = logf(anchorFreq / fLowEst) / logf(fHighEst / fLowEst);
      if (frac < 0.0f) frac = 0.0f;
      if (frac > 1.0f) frac = 1.0f;
      anchorPair = firstRung + (int)lroundf(frac * (float)(nRungs - 1));
      if (anchorPair < firstRung) anchorPair = firstRung;
      if (anchorPair > lastRung)  anchorPair = lastRung;
    }
  }

  const float ladderRatio = powf(2.0f, (float)ladderInterval / 12.0f);
  calReportLadderInterval = ladderInterval;
  calReportAnchorPair     = anchorPair;

  freqByPair[anchorPair] = anchorFreq;
  ampByPair[anchorPair]  = anchorAmp;
  cal_report_set_pair_from_gap(anchorPair, anchorGapUs, anchorFreq, CAL_SRC_ANCHOR);

  auto retry_rung = [&](int p, float fTarget, int32_t& ampFixed, float& found,
                        FreqTraceProbeInfo& info, int32_t ampMin, int32_t ampMax) {
    for (int retry = 0; retry < cal_precision().rungRetries; ++retry) {
      float cents = 1200.0f * log2f(found / fTarget);
      if (fabsf(cents) <= 25.0f || calibrationCancelRequested) return;
      float slope = freq_trace_local_slope(knownFreq, knownAmp, knownCount, fTarget);
      int32_t ampNext = (int32_t)lroundf((float)ampFixed * powf(fTarget / found, 1.0f / slope));
      if (ampNext == ampFixed) ampNext += (found < fTarget) ? 1 : -1;
      if (ampNext < ampMin || ampNext > ampMax) return;

      float again = find_freq_for_duty50((uint16_t)ampNext, fTarget, ladderRatio, true);
      if (again <= 0.0f) return;
      add_known(again, (float)ampNext);

      float againCents = 1200.0f * log2f(again / fTarget);
      if (fabsf(againCents) < fabsf(cents)) {
        ampFixed = ampNext; found = again;
        info.gapUs = g_lastFreqBisectGapUs; info.probes = g_lastFreqBisectProbes; info.settleChecks = g_lastSettleChecks;
      }
    }
  };

  // 6. Trace Upward
  int highestTraced = anchorPair;
  for (int p = anchorPair + 1; p <= lastRung; ++p) {
    if (calibrationCancelRequested) return false;
    float fTarget = anchorFreq * powf(ladderRatio, (float)(p - anchorPair));
    float ampGuess = freq_trace_guess(knownFreq, knownAmp, knownCount, fTarget);
    if (ampGuess >= (float)DIV_COUNTER * 0.98f) break;

    int32_t ampFixed = (int32_t)(ampGuess + 0.5f);
    if (ampFixed <= (int32_t)ampByPair[p - 1]) ampFixed = (int32_t)ampByPair[p - 1] + 1;
    if (ampFixed > (int32_t)DIV_COUNTER) break;

    float found = find_freq_for_duty50((uint16_t)ampFixed, fTarget, ladderRatio, true);
    if (found <= 0.0f) break;

    add_known(found, (float)ampFixed);
    FreqTraceProbeInfo info = { g_lastFreqBisectGapUs, g_lastFreqBisectProbes, g_lastSettleChecks };
    retry_rung(p, fTarget, ampFixed, found, info, (int32_t)ampByPair[p - 1] + 1, (int32_t)DIV_COUNTER);

    freqByPair[p] = found; ampByPair[p] = (uint16_t)ampFixed;
    cal_report_set_pair_from_gap(p, info.gapUs, found, CAL_SRC_RUNG);
    highestTraced = p;
  }

  // 7. Trace Downward
  int lowestTraced = anchorPair;
  for (int p = anchorPair - 1; p >= firstRung; --p) {
    if (calibrationCancelRequested) return false;
    float fTarget = anchorFreq * powf(ladderRatio, (float)(p - anchorPair));
    float ampGuess = freq_trace_guess(knownFreq, knownAmp, knownCount, fTarget);
    int32_t ampFixed = (int32_t)(ampGuess + 0.5f);
    if (ampFixed >= (int32_t)ampByPair[p + 1]) ampFixed = (int32_t)ampByPair[p + 1] - 1;
    if (ampFixed < 1) break;

    float fGuess = freq_trace_guess(knownAmp, knownFreq, knownCount, (float)ampFixed);
    if (fGuess <= 0.0f) fGuess = freqByPair[p + 1] * ((float)ampFixed / (float)ampByPair[p + 1]);

    float found = find_freq_for_duty50((uint16_t)ampFixed, fGuess, ladderRatio, true);
    if (found <= 0.0f) break;

    add_known(found, (float)ampFixed);
    FreqTraceProbeInfo info = { g_lastFreqBisectGapUs, g_lastFreqBisectProbes, g_lastSettleChecks };
    retry_rung(p, fTarget, ampFixed, found, info, 1, (int32_t)ampByPair[p + 1] - 1);

    freqByPair[p] = found; ampByPair[p] = (uint16_t)ampFixed;
    cal_report_set_pair_from_gap(p, info.gapUs, found, CAL_SRC_RUNG);
    lowestTraced = p;
  }

  if (calibrationCancelRequested) return false;

  // 8. Full-Amp Endpoint
  int topPair = (highestTraced < lastRung) ? (highestTraced + 1) : (numPairs - 1);
  {
    CalPrecisionOverride fineForEndpoint(
      (calibrationPrecision == CAL_PRECISION_FAST) ? calibrationPrecision : CAL_PRECISION_FINE);

    float fPower = freq_trace_power_seed(knownFreq, knownAmp, knownCount, (float)DIV_COUNTER);
    float fQuad  = freq_trace_guess(knownAmp, knownFreq, knownCount, (float)DIV_COUNTER);
    float fSeed  = (fPower > 0.0f) ? fPower : fQuad;
    if (fPower > 0.0f && fQuad > 0.0f && fabsf(1200.0f * log2f(fQuad / fPower)) > 50.0f) {
      fSeed = fminf(fPower, fQuad);
    }
    if (fSeed <= freqByPair[highestTraced]) fSeed = freqByPair[highestTraced] * ladderRatio;

    float endFreq = find_freq_for_duty50(DIV_COUNTER, fSeed, kTopEndpointWindowRatio, true);
    if (!(endFreq > freqByPair[highestTraced]) && !calibrationCancelRequested) {
      endFreq = find_freq_for_duty50(DIV_COUNTER, freqByPair[highestTraced] * ladderRatio, kBottomEndpointWindowRatio, true);
    }

    if (endFreq > freqByPair[highestTraced]) {
      add_known(endFreq, (float)DIV_COUNTER);
      cal_report_set_pair_from_gap(topPair, g_lastFreqBisectGapUs, endFreq, CAL_SRC_ENDPOINT_FULL);
    } else {
      endFreq = fSeed;
      cal_report_set_pair(topPair, kCalDutyErrUnknown, CAL_SRC_FILLED);
    }
    freqByPair[topPair] = endFreq; ampByPair[topPair] = DIV_COUNTER;
  }

  for (int q = topPair + 1; q < numPairs; ++q) {
    freqByPair[q] = 200000.0f; ampByPair[q] = DIV_COUNTER;
    cal_report_set_pair(q, kCalDutyErrUnknown, CAL_SRC_SENTINEL);
  }

  // 9. Amp-0 Bottom Endpoint
  FreqSearchBounds f0Bounds = amp0_search_band(freqByPair[lowestTraced]);
  float f0Model = amp0_fit_freq(knownAmp, knownFreq, knownCount);
  if (!(f0Model > 0.0f)) f0Model = freq_trace_guess(knownAmp, knownFreq, knownCount, 0.0f);

  float f0Est = f0Model;
  bool  amp0Calc = (autotuneAmp0Mode == AMP0_MODE_CALC) || (calibrationPrecision == CAL_PRECISION_FAST);

  if (amp0Calc) {
    if (!(f0Est > 0.0f)) f0Est = sqrtf(f0Bounds.loHz * f0Bounds.hiHz);
    if (f0Est < kAmp0StoreFloorHz) f0Est = kAmp0StoreFloorHz;
    if (f0Est > f0Bounds.hiHz)     f0Est = f0Bounds.hiHz;
    cal_report_set_pair(0, kCalDutyErrUnknown, CAL_SRC_FILLED);
  } else {
    if (!(f0Est > 0.0f)) f0Est = sqrtf(f0Bounds.loHz * f0Bounds.hiHz);
    if (f0Est < f0Bounds.loHz) f0Est = f0Bounds.loHz;
    if (f0Est > f0Bounds.hiHz) f0Est = f0Bounds.hiHz;

    float found = measure_lowest_freq_at_amp0(f0Est, &f0Bounds);
    float foundErr = duty_err_pct_from_gap(g_lastFreqBisectGapUs, found);
    if (found >= f0Bounds.loHz && found <= f0Bounds.hiHz && fabsf(foundErr) <= kEndpointAcceptDutyPct) {
      cal_report_set_pair_from_gap(0, g_lastFreqBisectGapUs, found, CAL_SRC_ENDPOINT_AMP0);
      f0Est = found;
    } else {
      float stored = (g_lastAmp0ScanSeedHz > 0.0f) ? g_lastAmp0ScanSeedHz : f0Model;
      if (!(stored > 0.0f)) stored = f0Est;
      if (stored < kAmp0StoreFloorHz) stored = kAmp0StoreFloorHz;
      if (stored > f0Bounds.hiHz)     stored = f0Bounds.hiHz;
      f0Est = stored;
      cal_report_set_pair(0, kCalDutyErrUnknown, CAL_SRC_FILLED);
    }
  }

  // Linear Fill Below Lowest Traced Rung
  for (int q = lowestTraced - 1; q >= firstRung; --q) {
    float frac = (float)q / (float)lowestTraced;
    freqByPair[q] = f0Est + (freqByPair[lowestTraced] - f0Est) * frac;
    ampByPair[q]  = (uint16_t)((float)ampByPair[lowestTraced] * frac + 0.5f);
    cal_report_set_pair(q, kCalDutyErrUnknown, CAL_SRC_FILLED);
  }

  // 10. Commit to Buffer & Sanity Check
  ctx.calibrationData[0] = (uint32_t)(f0Est * 100.0f);
  ctx.calibrationData[1] = 0;
  for (int p = 1; p < numPairs; ++p) {
    ctx.calibrationData[2 * p]     = (uint32_t)(freqByPair[p] * 100.0f);
    ctx.calibrationData[2 * p + 1] = ampByPair[p];
  }

  return cal_table_is_monotonic(ctx.calibrationData, numPairs, ctx.dcoIndex, "FREQ_TRACE_ERROR");
}

// =============================================================================
// Calibration Routine C: Fine Table Refinement
// =============================================================================

bool refine_DCO_amp_table(DCOCalibrationContext& ctx) {
  constexpr int numPairs = (int)(chanLevelVoiceDataSize / 2);
  const int base = (int)ctx.dcoIndex * (int)chanLevelVoiceDataSize;

  float    storedFreq[numPairs];
  uint16_t storedAmp[numPairs];
  for (int p = 0; p < numPairs; ++p) {
    int32_t fx100 = freq_to_amp_comp_array[base + 2 * p];
    int32_t a     = freq_to_amp_comp_array[base + 2 * p + 1];
    if (a < 0) a = 0;
    if (a > (int32_t)DIV_COUNTER) a = (int32_t)DIV_COUNTER;
    storedFreq[p] = (fx100 > 0) ? ((float)fx100 / 100.0f) : 0.0f;
    storedAmp[p]  = (uint16_t)a;
  }

  int topPair = -1;
  for (int p = 1; p < numPairs; ++p) {
    if (storedAmp[p] >= DIV_COUNTER) { topPair = p; break; }
  }

  if (topPair < 4) return false;

  float refinedFreq[numPairs];
  for (int p = 0; p < numPairs; ++p) refinedFreq[p] = storedFreq[p];

  for (int p = 0; p <= topPair; ++p) {
    if (calibrationCancelRequested) return false;
    bool isBottomEndpoint = (p == 0 && storedAmp[0] == 0);
    if (isBottomEndpoint && autotuneAmp0Mode == AMP0_MODE_CALC) continue;

    float found = 0.0f;
    if (isBottomEndpoint) {
      FreqSearchBounds bounds = amp0_search_band(storedFreq[1]);
      found = measure_lowest_freq_at_amp0(storedFreq[p], &bounds);
      float err = duty_err_pct_from_gap(g_lastFreqBisectGapUs, found);
      if (!(found >= bounds.loHz && found <= bounds.hiHz) || fabsf(err) > kEndpointAcceptDutyPct) {
        cal_report_set_pair(p, kCalDutyErrUnknown, CAL_SRC_FILLED);
        continue;
      }
    } else {
      found = find_freq_for_duty50(storedAmp[p], storedFreq[p], 1.02f, true);
    }

    if (found <= 0.0f) {
      cal_report_set_pair(p, kCalDutyErrUnknown, CAL_SRC_FILLED);
      continue;
    }

    refinedFreq[p] = found;
    cal_report_set_pair_from_gap(p, g_lastFreqBisectGapUs, found, CAL_SRC_REFINED);
  }

  for (int p = topPair + 1; p < numPairs; ++p) cal_report_set_pair(p, kCalDutyErrUnknown, CAL_SRC_SENTINEL);

  if (storedAmp[0] == 0 && autotuneAmp0Mode == AMP0_MODE_CALC) {
    float fitAmps[kAmp0FitPoints + 3], fitFreqs[kAmp0FitPoints + 3];
    int fitCount = 0;
    for (int p = 1; p <= topPair && fitCount < (int)(sizeof(fitAmps) / sizeof(fitAmps[0])); ++p) {
      if (refinedFreq[p] <= 0.0f || storedAmp[p] == 0) continue;
      fitAmps[fitCount]  = (float)storedAmp[p];
      fitFreqs[fitCount] = refinedFreq[p];
      ++fitCount;
    }
    float f0 = amp0_fit_freq(fitAmps, fitFreqs, fitCount);
    if (!(f0 > 0.0f)) f0 = refinedFreq[0];
    float f0CeilHz = refinedFreq[1] * 0.99f;
    if (f0 < kAmp0StoreFloorHz) f0 = kAmp0StoreFloorHz;
    if (f0 > f0CeilHz) f0 = f0CeilHz;
    refinedFreq[0] = f0;
    cal_report_set_pair(0, kCalDutyErrUnknown, CAL_SRC_FILLED);
  }

  for (int p = 0; p < numPairs; ++p) {
    if (p > topPair) {
      ctx.calibrationData[2 * p]     = (uint32_t)freq_to_amp_comp_array[base + 2 * p];
      ctx.calibrationData[2 * p + 1] = (uint32_t)freq_to_amp_comp_array[base + 2 * p + 1];
    } else {
      ctx.calibrationData[2 * p]     = (uint32_t)(refinedFreq[p] * 100.0f);
      ctx.calibrationData[2 * p + 1] = storedAmp[p];
    }
  }

  return cal_table_is_monotonic(ctx.calibrationData, numPairs, ctx.dcoIndex, "CAL_REFINE_ERROR");
}

#endif  // __AUTOTUNE_SEARCH_IMPL_H__
