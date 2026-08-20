/**
 * @file utils.h
 * @brief Math and curve transformation utilities (Bézier curves, exponential, and logarithmic mappings).
 */

 #ifndef DCO_UTILS_H
 #define DCO_UTILS_H
 
 #include <stdint.h>
 #include <math.h>
 
 /**
  * @struct Point
  * @brief 2D Point coordinate used for Bézier curve evaluations.
  */
 struct Point {
   float x; ///< X coordinate (input axis).
   float y; ///< Y coordinate (output axis).
 };
 
 /**
  * @brief Evaluates a cubic Bézier curve at parameter t in [0.0, 1.0].
  * @param A Start point.
  * @param P1 First control point.
  * @param P2 Second control point.
  * @param B End point.
  * @param t Interpolation factor [0.0, 1.0].
  * @return Calculated 2D point on the curve.
  */
 inline Point bezierCubic(const Point& A, const Point& P1, const Point& P2, const Point& B, float t) {
   float one_minus_t = 1.0f - t;
   float one_minus_t_squared = one_minus_t * one_minus_t;
   float t_squared = t * t;
   float x = one_minus_t_squared * one_minus_t * A.x + 3 * one_minus_t_squared * t * P1.x + 3 * one_minus_t * t_squared * P2.x + t_squared * t * B.x;
   float y = one_minus_t_squared * one_minus_t * A.y + 3 * one_minus_t_squared * t * P1.y + 3 * one_minus_t * t_squared * P2.y + t_squared * t * B.y;
   return { x, y };
 }
 
 // Función para encontrar el valor de y dado un valor de x en la curva de Bézier
 inline float findYForX(const Point& A, const Point& P1, const Point& P2, const Point& B, float xTarget, float tol = 1e-5f) {
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
 
 /**
  * @brief Populates a 4096-entry lookup table for AS2164 VCA linearization using cubic Bézier curves.
  * @param A Start point.
  * @param B End point.
  * @param P1 First control point.
  * @param P2 Second control point.
  * @param arraySize Size of lookup array (typically 4096).
  * @param array Reference to the destination array.
  */
 inline void generateBezierArray(Point A, Point B, Point P1, Point P2, uint16_t arraySize, uint16_t (&array)[4096]) {
   for (int x = 0; x < arraySize; ++x) {
     float yResult = findYForX(A, P1, P2, B, static_cast<float>(x));
     array[x] = (uint16_t)yResult;
   }
 }
 
 /**
  * @brief Maps a linear input value to a logarithmic curve.
  * @param linearValue Input value [0, maxValue].
  * @param base Logarithmic base curve factor.
  * @param maxValue Maximum input and output ceiling.
  * @return Scaled logarithmic output value.
  */
 inline uint16_t linearToLogarithmic(uint16_t linearValue, float base, uint16_t maxValue) {
   if (linearValue > maxValue) linearValue = maxValue;
 
   float normalizedValue = (float)linearValue / (float)maxValue;
   float logValue = log(normalizedValue * (base - 1.0f) + 1.0f) / log(base);
   float maxLogValue = log(1.0f + (base - 1.0f)) / log(base);
   return (uint16_t)(logValue * ((float)maxValue / maxLogValue));
 }
 
 /**
  * @brief Maps linear 0..4095 input to an exponential curve matching panel faders.
  * @param linearValue Input reading [0, 4095].
  * @param base Exponential base curve factor.
  * @param maxValue Maximum mapped output value.
  * @return Scaled exponential output value.
  */
 inline uint16_t linearToExponential(uint16_t linearValue, float base, uint16_t maxValue) {
   if (linearValue > 4095) linearValue = 4095;
 
   float normalizedValue = (float)linearValue / 4095.0f;
   float expValue = pow(base, normalizedValue) - 1.0f;
   float maxExpValue = base - 1.0f;
 
   return (uint16_t)(expValue * ((float)maxValue / maxExpValue));
 }
 
 /**
  * @brief Quadratic exponential curve mapping returning float: (x^2 / curve).
  * @param readingValue Input integer value.
  * @param curve Scaling denominator.
  * @return Computed float value or 0.0f if below threshold.
  */
 inline float expConverterFloat(uint16_t readingValue, uint16_t curve) {
   uint32_t pow3Calc = readingValue;
   float expValOut = (float)(pow3Calc * pow3Calc) / (float)curve;
   if (expValOut < 0.005f) {
     expValOut = 0.0f;
   }
   return expValOut;
 }
 
 /**
  * @brief Quadratic exponential curve mapping returning integer: (x^2 / curve).
  * @param readingValue Input integer value.
  * @param curve Scaling denominator.
  * @return Scaled uint16_t value.
  */
 inline uint16_t expConverter(uint16_t readingValue, uint16_t curve) {
   uint32_t pow3Calc = readingValue;
   uint16_t expValOut = (uint16_t)((pow3Calc * pow3Calc) / curve);
   if (expValOut < 1) {
     expValOut = 0;
   }
   return expValOut;
 }
 
 /**
  * @brief Fast reciprocal multiplication approximation for expConverterFloat(v, 5000).
  * @param v Input 32-bit value.
  * @return Scaled float output.
  */
 static inline float fast_exp_speed_5000(uint32_t v) {
   if (v <= 4) return 0.0f;
   return (float)(v * v) * 0.0002f;
 }
 
 /**
  * @brief Fast reciprocal multiplication approximation for LFO depth modulation scaling.
  * @param v Input 32-bit value.
  * @return Scaled float modulation depth.
  */
 static inline float fast_lfo_depth_amt(uint32_t v) {
   if (v <= 1) return 0.0f;
   return (float)(v * v) * 7.27272727e-9f;
 }
 
 #endif // DCO_UTILS_H