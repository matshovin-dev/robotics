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
 * deg_to_rad - Konverter grader til radianer
 * @degrees: vinkel i grader
 *
 * Retur: vinkel i radianer
 */
float deg_to_rad(float degrees);

/**
 * rad_to_deg - Konverter radianer til grader
 * @radians: vinkel i radianer
 *
 * Retur: vinkel i grader
 */
float rad_to_deg(float radians);

#endif /* ROBOTICS_MATH_UTILS_H */