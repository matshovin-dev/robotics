#ifndef STEWART_KINEMATICS_H
#define STEWART_KINEMATICS_H

#include "robotics/math/vec3.h"
#include <stewart/geometry.h>
#include <stewart/pose.h>

/**
 * struct stewart_inverse_result - Inverse kinematics resultat
 * @motor_angles_deg: 	Motor vinkler (6)
 * @knee_points:	Kne-posisjoner (6)
 * @platform_points_transformed: Transformerte platform punkter (6)
 * @error: 		0 = success, 1 = NaN detected
 *
 * Inneholder motor-vinkler og mellomresultater fra inverse kinematics.
 * Beregnes fra ønsket pose til motor-vinkler.
 */
struct stewart_inverse_result {
	float motor_angles_deg[6];
	struct vec3 knee_points[6];
	struct vec3 platform_points_transformed[6];
	int error;
};

/**
 * struct stewart_forward_result - Forward kinematics resultat
 * @leg_force_vectors:	Kraftvektorer for hvert ben (6)
 * @total_force:	Total kraft på platform
 * @total_moment:	Total moment på platform
 * @leg_lengths:	Faktiske benlengder i mm (6)
 * @leg_length_errors:	Avvik fra ønsket lengde i mm (6)
 * @pose_result:	Beregnet pose (rx, ry, rz, tx, ty, tz)
 *
 * pose_result er output fra forward kinematics beregning.
 * Alt annet er interne krefter, momenter og debug-info.
 * Brukes for å verifisere IK-beregning og beregne belastning.
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
 * stewart_kinematics_inverse
 * @param[in] 	geom 	robot geometri
 * @param[in] 	pose_in gitt platform pose, rx ry yz tx ty tz
 * @param[out] 	result 	motor vinkler, kne posisjoner, transformerte punkter
 * @param[in]  	debug	1 = print debug info, 0 = stille
 *
 * Beregner motor-vinkler fra ønsket platform pose.
 *
 * Algoritme:
 * 1. Transformer platform_flat punkter fra *geom opp med
 * pose_in (roter + translater + hever med stewart_geometry.home_height)
 * 2. For hver motor:
 *    - Projiser platform punkt på motor plan
 *    - Beregn trekant geometri (pythagoras + cosinus-setningen)
 *    - Finn motor vinkel
 *    - Beregn kne posisjon
 *
 * Motor vinkler blir hard eller soft clamped til geometri-grenser.
 *
 * NB: ty er offset fra home-posisjon (ty = 0 betyr platform ved home_height).
 */
void stewart_kinematics_inverse(const struct stewart_geometry *geom,
				const struct stewart_pose *pose_in,
				struct stewart_inverse_result *result,
				int debug);

/**
 * calculate_transformed_platform_points - Transform platform punkter med pose
 * @param[in]  geom     robot geometri
 * @param[in]  pose_in  gitt pose (rx, ry, rz, tx, ty, tz)
 * @param[out] result   transformerte platform punkter
 *
 * Roterer platform_points_flat med ZYX Euler angles og translerer med pose_in.
 * platform_points_flat blir også hevet opp med stewart_geometry.home_height.
 * Første steg i inverse kinematics.
 * Kan også brukes av forward kinematics for å oppdatere transformerte punkter.
 *
 * NB: ty er offset fra home-posisjon (ty = 0 betyr platform ved home_height).
 */
void calculate_transformed_platform_points(
	const struct stewart_geometry *geom, const struct stewart_pose *pose_in,
	struct stewart_inverse_result *result);

/**
 * @brief Forward kinematics med iterativ fjær-modell
 *
 * @param[in]     geom        Robot geometri
 * @param[in,out] pose_calc   Pose (modifiseres iterativt)
 * @param[in]     result_inv  Motor vinkler og kne-posisjon
 * @param[out]    result_forv Krefter, momenter, beregnet pose
 *
 * Beregner krefter/momenter fra motor-vinkler. Oppdaterer pose basert
 * på fjær-modell. Kalles etter stewart_kinematics_inverse().
 *
 * NB: ty er offset fra home-posisjon (ty = 0 betyr platform ved home_height).
 */
void stewart_kinematics_forward(const struct stewart_geometry *geom,
				struct stewart_pose *pose_calc,
				const struct stewart_inverse_result *result_inv,
				struct stewart_forward_result *result_forv);

/**
 * stewart_inverse_result_print - Print inverse kinematics resultat
 * @result: resultat struktur
 *
 * Printer motor vinkler, kne posisjoner og transformerte punkter.
 *
 * NB: ty er offset fra home-posisjon (ty = 0 betyr platform ved home_height).
 */
void stewart_inverse_result_print(const struct stewart_inverse_result *result);

/**
 * stewart_forward_result_print - Print forward kinematics resultat
 * @result: resultat struktur
 *
 * Printer krefter, momenter, benlengder og beregnet pose.
 */
void stewart_forward_result_print(const struct stewart_forward_result *result);

#endif /* STEWART_KINEMATICS_H */