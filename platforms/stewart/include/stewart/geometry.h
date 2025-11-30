#ifndef STEWART_GEOMETRY_H
#define STEWART_GEOMETRY_H

#include <robotics/math/vec3.h>

/**
 * struct stewart_geometry - Stewart platform fysisk geometri
 * @base_points: base attachment points (6) i mm, relativt til base center
 * @platform_points: platform attachment points (6) i mm, relativt til
 *                   platform center ved home posisjon
 * @home_height: avstand fra base plan til platform plan i neutral posisjon (mm)
 * @short_foot_length: kort ben lengde (mm) - motor arm
 * @long_foot_length: langt ben lengde (mm) - pushrod
 * @motor_arm_outward: 1 = motor arm peker utover (MX64), 0 = innover (AX18)
 * @max_motor_angle_024_deg: hard limit max vinkel for motor 0,2,4 (grader)
 * @min_motor_angle_024_deg: hard limit min vinkel for motor 0,2,4 (grader)
 * @max_motor_angle_135_deg: hard limit max vinkel for motor 1,3,5 (grader)
 * @min_motor_angle_135_deg: hard limit min vinkel for motor 1,3,5 (grader)
 * @motor_clamp_limit_angle_deg: soft dampening grense (grader)
 * @max_pose_rotation_amplitude: maksimal rotasjon amplitude (grader)
 * @max_pose_rotation_bias: maksimal rotasjon bias/offset (grader)
 * @max_pose_translation_amplitude: maksimal translasjon amplitude (mm)
 * @max_pose_translation_bias: maksimal translasjon bias/offset (mm)
 *
 * Inneholder alle fysiske dimensjoner og kinematiske grenser for en
 * spesifikk Stewart platform konfigurasjon. Robot-spesifikk men
 * motor-agnostisk (fungerer med alle aktuator-typer).
 *
 * Motor layout (sett ovenfra, CCW nummerering):
 *     3     2
 *   4       1
 *      5 0
 */
struct stewart_geometry {
	struct vec3 base_points[6];
	struct vec3 platform_points[6];

	float home_height;
	float short_foot_length;
	float long_foot_length;

	int motor_arm_outward;

	float max_motor_angle_024_deg;
	float min_motor_angle_024_deg;
	float max_motor_angle_135_deg;
	float min_motor_angle_135_deg;
	float motor_clamp_limit_angle_deg;

	float max_pose_rotation_amplitude;
	float max_pose_rotation_bias;
	float max_pose_translation_amplitude;
	float max_pose_translation_bias;
};

/**
 * stewart_geometry_print - Print geometri til stdout
 * @geom: geometri struktur
 *
 * Printer alle geometri-parametere for debugging.
 */
void stewart_geometry_print(const struct stewart_geometry *geom);

/* Forhåndsdefinerte robot-konfigurasjoner */
extern const struct stewart_geometry ROBOT_MX64;
extern const struct stewart_geometry ROBOT_AX18;

#endif /* STEWART_GEOMETRY_H */