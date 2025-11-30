#include <math.h>
#include <robotics/math/utils.h>

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