#ifndef STEWART_KINEMATICS_H
#define STEWART_KINEMATICS_H

#include "robotics/math/vec3.h"
#include "stewart/geometry.h"
#include "stewart/pose.h"

/**
 * struct stewart_inverse_result - Inverse kinematics resultat
 *
 * @motor_angles_deg[6] float (deg) - Motor vinkler
 * @knee_points[6] vec3 (mm) - Kne posisjoner
 * @platform_points_transformed[6] vec3 (mm) - Transformerte punkter
 * @error int - 0=success, 1=NaN detected
 *
 * Inneholder alle beregnede verdier fra inverse kinematics.
 * Beregnes fra ønsket platform pose.
 *
 * @note Alle arrays indeksert 0-5 for motor nummer
 */
struct stewart_inverse_result {
	float motor_angles_deg[6];
	struct vec3 knee_points[6];
	struct vec3 platform_points_transformed[6];
	int error;
};

/**
 * struct stewart_forward_result - Forward kinematics resultat
 *
 * @leg_force_vectors[6] vec3 - Kraftvektorer for hvert ben
 * @total_force vec3 - Total kraft på platform
 * @total_moment vec3 - Total moment på platform
 * @leg_lengths[6] float (mm) - Deformerte benlengder
 * @leg_length_errors[6] float (mm) - Avvik fra ønsket lengde
 * @pose_result stewart_pose - Beregnet pose
 *
 * pose_result er output fra forward kinematics beregning.
 * Øvrige felt er interne krefter, momenter og debug-info.
 * Beregnes fra knepunkt og platform_points_transformed.
 * Brukes for å verifisere IK og beregne belastning.
 *
 * @note Arrays indeksert 0-5 for motor nummer
 */
struct stewart_forward_result {
	struct vec3 leg_force_vectors[6];
	struct vec3 total_force;
	struct vec3 total_moment;

	float leg_lengths[6];
	float leg_length_errors[6];

	struct stewart_pose pose_result;
};

/**
 * @function stewart_kinematics_inverse
 * @api PUBLIC
 *
 * @input  geom->base_points[6]                      struct vec3 (mm)
 * @input  geom->platform_home_points[6]             struct vec3 (mm)
 * @input  geom->short_foot_length                   float (mm)
 * @input  geom->long_foot_length                    float (mm)
 * @input  geom->motor_arm_outward                   int
 * @input  geom->max/min_motor_angle_024/135_deg     float (degrees)
 * @input  geom->motor_clamp_limit_angle_deg         float (degrees)
 * @input  pose_in->{rx,ry,rz}                       float (degrees)
 * @input  pose_in->{tx,ty,tz}                       float (mm)
 * @input  debug                                     int (1=print, 0=silent)
 *
 * @output result_out->motor_angles_deg[6]           float (degrees)
 * @output result_out->knee_points[6]                struct vec3 (mm)
 * @output result_out->platform_points_transformed[6] struct vec3 (mm)
 * @output result_out->error                         int (0=success, 1=NaN)
 *
 * Beregner motor-vinkler, knepunkt og nye topp platform punkt
 * fra ønsket platform pose.
 *
 * Algoritme:
 * 1. Roterer og translaterer geom.platform_home_points med pose_in, til
 *    result_out->platform_points_transformed
 * 2. For hver motor:
 *    - Projiser platform_points_transformed på motor plan
 *    - Beregn trekant geometri (pythagoras + cosinus-setningen)
 *    - Finn motor vinkel
 *    - Beregn kne posisjon
 * 3. Motor vinkler blir hard eller soft clamped til geometri-grenser.
 */
void stewart_kinematics_inverse(const struct stewart_geometry *geom,
				const struct stewart_pose *pose_in,
				struct stewart_inverse_result *result_out,
				int debug);

/**
 * @function calculate_transformed_platform_points
 * @api PUBLIC
 *
 * @input  geom->platform_home_points[6]             struct vec3 (mm)
 * @input  pose_in->{rx,ry,rz}                       float (degrees)
 * @input  pose_in->{tx,ty,tz}                       float (mm)
 *
 * @output result_out->platform_points_transformed[6] struct vec3 (mm)
 *
 * Transformer platform_home_points med pose.
 * Første steg i inverse kinematics.
 * Kan også brukes av forward_kinematics for å finne transformerte punkter.
 */
void calculate_transformed_platform_points(
	const struct stewart_geometry *geom, const struct stewart_pose *pose_in,
	struct stewart_inverse_result *result_out);

/**
 * @function stewart_kinematics_forward
 * @api PUBLIC
 *
 * @input  geom->base_points[6]                      struct vec3 (mm)
 * @input  geom->short_foot_length                   float (mm)
 * @input  geom->long_foot_length                    float (mm)
 * @input  result_inv->knee_points[6]                struct vec3 (mm)
 * @input  result_inv->platform_points_transformed[6] struct vec3 (mm)
 * @input  pose_calc->{rx,ry,rz,tx,ty,tz}            float (degrees/mm) -
 * MODIFIED
 *
 * @output pose_calc->{rx,ry,rz,tx,ty,tz}            float (degrees/mm) -
 * updated iteratively
 * @output result_forv->leg_force_vectors[6]         struct vec3
 * @output result_forv->total_force                  struct vec3
 * @output result_forv->total_moment                 struct vec3
 * @output result_forv->leg_lengths[6]               float (mm)
 * @output result_forv->leg_length_errors[6]         float (mm)
 * @output result_forv->pose_result                  struct stewart_pose
 *
 * Beregner krefter/momenter fra motor-vinkler. Oppdaterer pose basert
 * på fjær-modell. Kalles etter stewart_kinematics_inverse().
 *
 * @note pose_calc er både input og output (modifiseres iterativt)
 */
void stewart_kinematics_forward(const struct stewart_geometry *geom,
				struct stewart_pose *pose_calc,
				const struct stewart_inverse_result *result_inv,
				struct stewart_forward_result *result_forv);

/**
 * @function stewart_inverse_result_print
 * @api PUBLIC
 *
 * @input  result_in->motor_angles_deg[6]            float (degrees)
 * @input  result_in->knee_points[6]                 struct vec3 (mm)
 * @input  result_in->platform_points_transformed[6] struct vec3 (mm)
 * @input  result_in->error                          int
 *
 * @output stdout  Formatted text output
 *
 * Printer motor vinkler, kne posisjoner og transformerte punkter til stdout.
 */
void stewart_inverse_result_print(
	const struct stewart_inverse_result *result_in);

/**
 * @function stewart_forward_result_print
 * @api PUBLIC
 *
 * @input  result_in->leg_force_vectors[6]  struct vec3
 * @input  result_in->total_force           struct vec3
 * @input  result_in->total_moment          struct vec3
 * @input  result_in->leg_lengths[6]        float (mm)
 * @input  result_in->leg_length_errors[6]  float (mm)
 * @input  result_in->pose_result           struct stewart_pose
 *
 * @output stdout  Formatted text output
 *
 * Printer krefter, momenter, benlengder og beregnet pose til stdout.
 */
void stewart_forward_result_print(
	const struct stewart_forward_result *result_in);

#endif /* STEWART_KINEMATICS_H */