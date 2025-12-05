#include "robotics/math/utils.h"
#include <math.h>

float deg_to_rad(float degrees)
{
	return degrees * (M_PI / 180.0f);
}

float rad_to_deg(float radians)
{
	return radians * (180.0f / M_PI);
}

float normalize_angle(float angle_rad)
{
	/*
	 * Normalize angle to [-π, π] range
	 */
	while (angle_rad > M_PI)
		angle_rad -= M_TWO_PI;
	while (angle_rad < -M_PI)
		angle_rad += M_TWO_PI;

	return angle_rad;
}

float normalize_angle_positive(float angle_rad)
{
	/*
	 * Normalize angle to [0, 2π] range
	 */
	while (angle_rad < 0.0f)
		angle_rad += M_TWO_PI;
	while (angle_rad >= M_TWO_PI)
		angle_rad -= M_TWO_PI;

	return angle_rad;
}

float clamp(float value, float min, float max, int *was_clamped)
{
	if (value < min) {
		if (was_clamped)
			*was_clamped = 1;
		return min;
	}
	if (value > max) {
		if (was_clamped)
			*was_clamped = 1;
		return max;
	}
	return value;
}

float soft_clamp(float value, float min, float max, float margin,
		 int *was_clamped)
{
	/*
	 * Soft clamp med eksponentiell dampening nær grensene.
	 * Innenfor margin-området brukes eksponentiell funksjon for smooth overgang.
	 */
	if (value > max - margin) {
		if (was_clamped)
			*was_clamped = 1;
		/* Eksponentiell tilnærming mot max */
		float excess = value - (max - margin);
		float damped = margin * (1.0f - expf(-excess / margin));
		return (max - margin) + damped;
	}

	if (value < min + margin) {
		if (was_clamped)
			*was_clamped = 1;
		/* Eksponentiell tilnærming mot min */
		float excess = (min + margin) - value;
		float damped = margin * (1.0f - expf(-excess / margin));
		return (min + margin) - damped;
	}

	return value;
}