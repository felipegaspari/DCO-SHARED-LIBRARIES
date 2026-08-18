#ifndef __UTILS_H__
#define __UTILS_H__

uint16_t linearToLogarithmic(uint16_t linearValue, float base, uint16_t maxValue);
uint16_t linearToExponential(uint16_t linearValue, float base, uint16_t maxValue);

struct Point {
    float x, y;
  };
  
  // Evaluate a cubic Bézier at parameter t (used by VCA linearize table build).
  Point bezierCubic(const Point& A, const Point& P1, const Point& P2, const Point& B, float t) {
    float one_minus_t = 1.0f - t;
    float one_minus_t_squared = one_minus_t * one_minus_t;
    float t_squared = t * t;
    float x = one_minus_t_squared * one_minus_t * A.x + 3 * one_minus_t_squared * t * P1.x + 3 * one_minus_t * t_squared * P2.x + t_squared * t * B.x;
    float y = one_minus_t_squared * one_minus_t * A.y + 3 * one_minus_t_squared * t * P1.y + 3 * one_minus_t * t_squared * P2.y + t_squared * t * B.y;
    return { x, y };
  }
  
  // Función para encontrar el valor de y dado un valor de x en la curva de Bézier
  float findYForX(const Point& A, const Point& P1, const Point& P2, const Point& B, float xTarget, float tol = 1e-5) {
    float tLow = 0.0f;
    float tHigh = 1.0f;
    float tMid;
  
    while ((tHigh - tLow) > tol) {
      tMid = (tLow + tHigh) / 2.0f;
      Point midPoint = bezierCubic(A, P1, P2, B, tMid);
      if (midPoint.x < xTarget) {
        tLow = tMid;
      } else {
        tHigh = tMid;
      }
    }
  
    Point resultPoint = bezierCubic(A, P1, P2, B, tMid);
    return resultPoint.y;
  }
  
  // Fill AS2164_VCA_linearize_table from Bézier control points (called from setup).
  void generateBezierArray(Point A, Point B, Point P1, Point P2, uint16_t arraySize, uint16_t (&array)[4096]) {
  
    for (int x = 0; x < arraySize; ++x) {
      float yResult = findYForX(A, P1, P2, B, static_cast<float>(x));
  
      array[x] = yResult;
    }
  }
  

// Linear → logarithmic mapping (0..maxValue). Kept for reuse; pitch ADSR no longer calls it.
uint16_t linearToLogarithmic(uint16_t linearValue, float base, uint16_t maxValue) {
    if (linearValue > maxValue) linearValue = maxValue;
  
    float normalizedValue = (float)linearValue / (float)maxValue;
    float logValue = log(normalizedValue * (base - 1) + 1) / log(base);
    float maxLogValue = log(1 + (base - 1)) / log(base);
    uint16_t scaledLogValue = (uint16_t)(logValue * ((float)maxValue / maxLogValue));
  
    return scaledLogValue;
  }
  
  // Linear 0..4095 → exponential 0..maxValue. Same curve the Input board applies to the
  // envelope A/D/R faders before sending them (auxiliary.h), so a MIDI CC
  // lands in the exp domain the 'a'-'c' block frames carry.
  uint16_t linearToExponential(uint16_t linearValue, float base, uint16_t maxValue) {
    if (linearValue > 4095) linearValue = 4095;
  
    float normalizedValue = (float)linearValue / 4095.0f;
    float expValue = pow(base, normalizedValue) - 1.0f;
    float maxExpValue = base - 1.0f;
  
    return (uint16_t)(expValue * ((float)maxValue / maxExpValue));
  }
  
  // Exp curve → float (x^2 / curve). Used by params/LFO drift and related control mapping.
  float expConverterFloat(uint16_t readingValue, uint16_t curve) {
    uint16_t pow3Calc = readingValue;
    float expValOut = (float)pow3Calc * pow3Calc / curve;
    if (expValOut < 0.005) {
      expValOut = 0;
    }
    return expValOut;
  }
  
  // Exp curve → uint16 (x^2 / curve). Used e.g. by apply_param_portamento_time.
  uint16_t expConverter(uint16_t readingValue, uint16_t curve) {
    uint16_t pow3Calc = readingValue;
    uint16_t expValOut = (float)pow3Calc * pow3Calc / curve;
    if (expValOut < 0.1) {
      expValOut = 0;
    }
    return expValOut;
  }
  
  

#endif
