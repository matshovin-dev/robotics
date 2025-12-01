#ifndef STEWART_KINEMATICS_H
#define STEWART_KINEMATICS_H

#include "robotics/math/vec3.h"
#include <stewart/geometry.h>
#include <stewart/pose.h>

/**
 * struct stewart_inverse_result - Resultat fra inverse kinematics
 * @motor_angles_deg: motor vinkler i grader (6)
 * @knee_points: kne-posisjoner i world coordinates (6)
 * @platform_points_transformed: transformerte platform punkter (6)
 * @error: 0 = success, 1 = NaN detected
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
 * struct stewart_forward_result - Resultat fra forward kinematics
 * @leg_force_vectors: kraftvektorer for hvert ben (6)
 * @total_force: total kraft på platform
 * @total_moment: total moment på platform
 * @leg_lengths: faktiske benlengder i mm (6)
 * @leg_length_errors: avvik fra ønsket lengde i mm (6)
 * @position: beregnet posisjon (Tx, Ty, Tz) i mm
 * @rotation: beregnet rotasjon (Rx, Ry, Rz) i grader
 *
 * Inneholder krefter, momenter og beregnet pose fra forward kinematics.
 * Brukes for å verifisere IK-beregning og beregne belastning.
 */
struct stewart_forward_result {
	struct vec3 leg_force_vectors[6];
	struct vec3 total_force;
	struct vec3 total_moment;

	float leg_lengths[6];
	float leg_length_errors[6];

	struct vec3 position;
	struct vec3 rotation;
};

/**
 * stewart_kinematics_inverse - Beregn inverse kinematics
 * @geom: robot geometri
 * @pose: ønsket platform pose
 * @result: output - motor vinkler, kne posisjoner, transformerte punkter
 * @debug: 1 = print debug info, 0 = stille
 *
 * Beregner motor-vinkler fra ønsket platform pose.
 *
 * Algoritme:
 * 1. Transformer platform punkter med pose (roter + translater)
 * 2. For hver motor:
 *    - Projiser platform punkt på motor plan
 *    - Beregn trekant geometri (pythagoras + cosinus-setningen)
 *    - Finn motor vinkel
 *    - Beregn kne posisjon
 *
 * Motor vinkler er hard-clamped til geometri-grenser.
 */
void stewart_kinematics_inverse(const struct stewart_geometry *geom,
				const struct stewart_pose *pose,
				struct stewart_inverse_result *result,
				int debug);

/**
 * stewart_kinematics_forward - Beregn forward kinematics
 * @geom: robot geometri
 * @pose_calc: beregnet pose (input/output - modifiseres iterativt)
 * @result_inv: resultat fra inverse kinematics (motor vinkler og kne)
 * @result_forv: output - krefter, momenter, beregnet pose
 *
 * Beregner krefter og momenter fra motor-vinkler og oppdaterer pose
 * iterativt basert på fjær-modell.
 *
 * Bruker fjær-fysikk simulering:
 * - Hver ben er en fjær med avvik fra ønsket lengde
 * - Kraftvektorer summeres til total kraft og moment
 * - Pose oppdateres basert på kraft/moment med demping
 *
 * Kalles typisk etter stewart_kinematics_inverse() for å verifisere
 * løsningen eller beregne belastning.
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