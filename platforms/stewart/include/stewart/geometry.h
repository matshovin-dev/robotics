#ifndef STEWART_GEOMETRY_H
#define STEWART_GEOMETRY_H

#include "robotics/math/vec3.h"

/**
 * struct stewart_geometry - Stewart platform fysisk geometri
 *
 * @base_points[6] vec3 (mm) - Base attachment points (y=0)
 * @platform_home_points[6] vec3 (mm) - Platform points at home
 * @home_height float (mm) - Avstand base til platform
 * @short_foot_length float (mm) - Motor arm lengde
 * @long_foot_length float (mm) - Pushrod lengde
 * @motor_arm_outward int - 1=utover (MX64), 0=innover (AX18)
 * @max_motor_angle_024_deg float (deg) - Max vinkel motor 0,2,4
 * @min_motor_angle_024_deg float (deg) - Min vinkel motor 0,2,4
 * @max_motor_angle_135_deg float (deg) - Max vinkel motor 1,3,5
 * @min_motor_angle_135_deg float (deg) - Min vinkel motor 1,3,5
 * @motor_clamp_limit_angle_deg float (deg) - Soft dampening grense
 * @max_pose_rotation_amplitude float - Max rotasjon amplitude
 * @max_pose_rotation_bias float - Max rotasjon bias/offset
 * @max_pose_translation_amplitude float (mm) - Max translasjon amplitude
 * @max_pose_translation_bias float (mm) - Max translasjon bias
 *
 * Inneholder alle fysiske dimensjoner og kinematiske grenser for en
 * spesifikk Stewart platform konfigurasjon. Robot-spesifikk men
 * motor-agnostisk (fungerer med alle aktuator-typer).
 *
 * Origo ligger i platform base ved 6 motoraksler.
 *
 * Motor layout (sett ovenfra, CCW nummerering):
 *    3     2
 *   4       1
 *      5 0
 *
 * @invariant max_motor_angle > min_motor_angle for begge grupper
 * @note platform_home_points kopieres ved bruk (const i struct)
 */
struct stewart_geometry {
	struct vec3 base_points[6];
	struct vec3 platform_home_points[6];

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
 * @function stewart_geometry_print
 * @api PUBLIC
 *
 * @input  geom->base_points[6]                    struct vec3 (mm)
 * @input  geom->platform_home_points[6]           struct vec3 (mm)
 * @input  geom->home_height                       float (mm)
 * @input  geom->short_foot_length                 float (mm)
 * @input  geom->long_foot_length                  float (mm)
 * @input  geom->motor_arm_outward                 int
 * @input  geom->max/min_motor_angle_024/135_deg   float (degrees)
 * @input  geom->motor_clamp_limit_angle_deg       float (degrees)
 * @input  geom->max_pose_rotation_amplitude       float
 * @input  geom->max_pose_rotation_bias            float
 * @input  geom->max_pose_translation_amplitude    float (mm)
 * @input  geom->max_pose_translation_bias         float (mm)
 *
 * @output stdout  Formatted text output
 *
 * Printer alle geometri-parametere til stdout for debugging.
 */
void stewart_geometry_print(const struct stewart_geometry *geom);

/* Forhåndsdefinerte robot-konfigurasjoner */
extern const struct stewart_geometry ROBOT_MX64;
extern const struct stewart_geometry ROBOT_AX18;

#endif /* STEWART_GEOMETRY_H */