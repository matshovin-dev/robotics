#ifndef ROBOTICS_MATH_UTILS_H
#define ROBOTICS_MATH_UTILS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717958647692f
#endif

/**
 * @function deg_to_rad
 * @api PUBLIC
 *
 * @input  degrees  float (degrees)
 * @output return   float (radians)
 *
 * Konverterer vinkel fra grader til radianer.
 */
float deg_to_rad(float degrees);

/**
 * @function rad_to_deg
 * @api PUBLIC
 *
 * @input  radians  float (radians)
 * @output return   float (degrees)
 *
 * Konverterer vinkel fra radianer til grader.
 */
float rad_to_deg(float radians);

/**
 * @function clamp
 * @api PUBLIC
 *
 * @input  value       float
 * @input  min         float
 * @input  max         float
 * @input  was_clamped int* (optional, kan være NULL)
 * @output return      float (clamped to [min, max])
 *
 * Begrenser verdi til området [min, max].
 * Hard clamp uten dampening.
 *
 * Hvis was_clamped != NULL, settes *was_clamped = 1 hvis clamping skjedde.
 */
float clamp(float value, float min, float max, int *was_clamped);

/**
 * @function soft_clamp
 * @api PUBLIC
 *
 * @input  value       float
 * @input  min         float
 * @input  max         float
 * @input  margin      float
 * @input  was_clamped int* (optional, kan være NULL)
 * @output return      float (soft clamped)
 *
 * Soft clamp med eksponentiell dampening nær grenser.
 * Innenfor margin-området: smooth dampening.
 * Utenfor margin: hard clamp.
 *
 * Hvis was_clamped != NULL, settes *was_clamped = 1 hvis clamping skjedde.
 */
float soft_clamp(float value, float min, float max, float margin,
		 int *was_clamped);

#endif /* ROBOTICS_MATH_UTILS_H */