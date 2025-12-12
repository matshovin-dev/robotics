#ifndef ROBOTICS_MATH_ANALYSYS_H
#define ROBOTICS_MATH_ANALYSYS_H

float find_velocity(float samp_cur, float samp_pre, float delta_time_sec);

float find_acceleration(float sampl_next, float samp_cur, float samp_pre,
			float delta_time_sec);

float find_jerk(float sampl_next, float samp_cur, float samp_pre,
		float samp_pre_pre, float delta_time_sec);

float find_max_velocity(float (*func)(float), float t_from, float t_to,
			float t_delta_sec);

float find_max_acceleration(float (*func)(float), float t_from, float t_to,
			    float t_delta_sec);

float find_max_jerk(float (*func)(float), float t_from, float t_to,
		    float t_delta_sec);

float find_mean_velocity(float (*func)(float), float t_from, float t_to,
			 float t_delta_sec);

float find_mean_acceleration(float (*func)(float), float t_from, float t_to,
			     float t_delta_sec);

float find_mean_jerk(float (*func)(float), float t_from, float t_to,
		     float t_delta_sec);

/*
 * fart i x retn
 * fart i y retn
 * fart i z retn
 *
 * fart i xy plan
 * fart i xz plan
 * fart i yz plan
 *
 * fart i 3D
 *
 * samme for acc
 *
 * samme for jerk
 *
 * mean av alt
 * max av alt
 * min av alt
 *
 * alt for prinsipal 1, 2
 *
 */

#endif /* ROBOTICS_MATH_ANALYSYS_H */