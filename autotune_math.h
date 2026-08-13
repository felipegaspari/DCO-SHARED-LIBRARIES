#ifndef __AUTOTUNE_MATH_H__
#define __AUTOTUNE_MATH_H__

#include <math.h>

inline float quadraticInterpolation(float x0, float y0, float x1, float y1, float x2, float y2, float x) {
    float a = ((y2 - (x2 * (y1 - y0) + x1 * y0 - x0 * y1) / (x1 - x0)) / (x2 * (x2 - x0 - x1) + x0 * x1));
    float b = ((y1 - y0) / (x1 - x0) - a * (x0 + x1));
    return a * x * x + b * x + (y0 - x0 * (b + a * x0));
}

inline uint16_t logarithmicInterpolation(float x0, float y0, float x1, float y1, float x) {
    if (x0 <= 0 || x1 <= 0) return 0;
    float a = (y1 - y0) / (log(x1) - log(x0));
    return (uint16_t)round(a * log(x) + (y0 - a * log(x0)));
}

inline float linearInterpolation(float x0, float y0, float x1, float y1, float x) {
    if (x0 == x1) return 0;
    return ((y1 - y0) / (x1 - x0)) * x + (y0 - ((y1 - y0) / (x1 - x0)) * x0);
}

inline float lsq_quadratic(const float* xs, const float* ys, const int* idx, int n, float targetX) {
    double s0 = n, s1 = 0, s2 = 0, s3 = 0, s4 = 0, t0 = 0, t1 = 0, t2 = 0;
    for (int k = 0; k < n; ++k) {
        double u = xs[idx[k]] - targetX, y = ys[idx[k]], u2 = u * u;
        s1 += u; s2 += u2; s3 += u2 * u; s4 += u2 * u2;
        t0 += y; t1 += u * y; t2 += u2 * y;
    }
    double det = s0*(s2*s4 - s3*s3) - s1*(s1*s4 - s3*s2) + s2*(s1*s3 - s2*s2);
    if (det <= 1e-9 * (s2/n)*(s2/n)*(s2/n)) return NAN;
    return (t0*(s2*s4 - s3*s3) - s1*(t1*s4 - s3*t2) + s2*(t1*s3 - s2*t2)) / det;
}

inline double compute_gap_tolerance(double freqHz, double dutyErrorFraction) {
    return (freqHz > 0.0) ? (2.0 * dutyErrorFraction * (1e6 / freqHz)) : 1e6;
}

inline float duty_err_pct_from_gap(float gapUs, float freqHz) {
    return (gapUs == kGapTimeoutSentinel || freqHz <= 0.0f) ? 1e9f : (100.0f * gapUs * freqHz / 2.0e6f);
}

#endif // __AUTOTUNE_MATH_H__