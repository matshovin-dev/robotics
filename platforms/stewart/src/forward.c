#include "robotics/math/vec3.h"
#include <stdio.h>
#include <stewart/kinematics.h>
#include <string.h>

/* Simulerings-parametere for fjær-modell */
static const float SPRING_K = 0.6f; /* Fjærkonstant */
static const float DAMPENING = 0.999f; /* Demping */
static const float TIMESTEP = 0.01f; /* 10ms timestep */

/**
 * calculate_leg_forces - Beregn kraftvektorer fra benlengde-avvik
 * @geom: robot geometri
 * @result_inv: inverse kinematics resultat (motor vinkler, kne)
 * @result_forv: forward kinematics resultat (output)
 *
 * Beregner kraftvektor for hvert ben basert på avvik mellom faktisk
 * og ønsket benlengde. Bruker fjær-modell: F = -k * Δx
 */
static void
calculate_leg_forces(const struct stewart_geometry *geom,
		     const struct stewart_inverse_result *result_inv,
		     struct stewart_forward_result *result_forv)
{
	struct vec3 leg_vector, leg_direction;
	float actual_length, length_error;
	float force_magnitude;
	int i;

	for (i = 0; i < 6; i++) {
		/* Benvektor fra knee til platform */
		vec3_sub(&result_inv->platform_points_transformed[i],
			 &result_inv->knee_points[i], &leg_vector);

		/* Faktisk benlengde */
		actual_length = vec3_length(&leg_vector);

		/* Avvik fra ønsket lengde */
		length_error = actual_length - geom->long_foot_length;

		/* Normaliser benvektor for retning */
		leg_direction = leg_vector;
		vec3_scale(&leg_direction, 1.0f / actual_length);

		/*
		 * Kraftvektor = fjærkraft * retning
		 * Positiv error → kraft skyver inn (mot base)
		 * Negativ error → kraft trekker ut (mot platform)
		 */
		force_magnitude = -SPRING_K * length_error;

		result_forv->leg_force_vectors[i] = leg_direction;
		vec3_scale(&result_forv->leg_force_vectors[i], force_magnitude);

		/* Lagre lengder for debugging */
		result_forv->leg_lengths[i] = actual_length;
		result_forv->leg_length_errors[i] = length_error;
	}
}

/**
 * calculate_total_force_and_moment - Summer krefter og momenter
 * @result_inv: inverse kinematics resultat
 * @result_forv: forward kinematics resultat
 * @total_force: output - total kraft
 * @total_moment: output - total moment
 *
 * Summerer alle ben-kraftvektorer til total kraft.
 * Beregner moment som r × F (kryssprodukt) for hvert ben.
 */
static void calculate_total_force_and_moment(
	const struct stewart_inverse_result *result_inv,
	const struct stewart_forward_result *result_forv,
	struct vec3 *total_force, struct vec3 *total_moment)
{
	struct vec3 moment_contribution;
	int i;

	/* Nullstill totaler */
	*total_force = (struct vec3){ 0.0f, 0.0f, 0.0f };
	*total_moment = (struct vec3){ 0.0f, 0.0f, 0.0f };

	for (i = 0; i < 6; i++) {
		/* Summer kraftvektorer */
		vec3_add(total_force, &result_forv->leg_force_vectors[i],
			 total_force);

		/*
		 * Beregn moment = r × F
		 * r = platform_points_transformed (hvor kraft angriper)
		 * F = leg_force_vectors
		 */
		vec3_cross(&result_inv->platform_points_transformed[i],
			   &result_forv->leg_force_vectors[i],
			   &moment_contribution);

		vec3_add(total_moment, &moment_contribution, total_moment);
	}
}

void stewart_kinematics_forward(const struct stewart_geometry *geom,
				struct stewart_pose *pose_calc,
				const struct stewart_inverse_result *result_inv,
				struct stewart_forward_result *result_forv)
{
	struct vec3 total_force, total_moment;

	if (!geom || !pose_calc || !result_inv || !result_forv)
		return;

	/* Beregn kraftvektorer fra benlengde-avvik */
	calculate_leg_forces(geom, result_inv, result_forv);

	/* Summer til total kraft og moment */
	calculate_total_force_and_moment(result_inv, result_forv, &total_force,
					 &total_moment);

	/*
	 * Integrer krefter til ny pose
	 * F = ma (antar m=1), M = Iα (antar I=1)
	 */
	pose_calc->tx += total_force.x * TIMESTEP;
	pose_calc->ty += total_force.y * TIMESTEP;
	pose_calc->tz += total_force.z * TIMESTEP;

	pose_calc->rx += total_moment.x * TIMESTEP;
	pose_calc->ry += total_moment.y * TIMESTEP;
	pose_calc->rz += total_moment.z * TIMESTEP;

	/* Dempning for å unngå oscillering */
	pose_calc->tx *= DAMPENING;
	pose_calc->ty *= DAMPENING;
	pose_calc->tz *= DAMPENING;

	pose_calc->rx *= DAMPENING;
	pose_calc->ry *= DAMPENING;
	pose_calc->rz *= DAMPENING;

	/* Lagre resultater */
	result_forv->total_force = total_force;
	result_forv->total_moment = total_moment;

	result_forv->position.x = pose_calc->tx;
	result_forv->position.y = pose_calc->ty;
	result_forv->position.z = pose_calc->tz;

	result_forv->rotation.x = pose_calc->rx;
	result_forv->rotation.y = pose_calc->ry;
	result_forv->rotation.z = pose_calc->rz;
}

void stewart_forward_result_print(const struct stewart_forward_result *result)
{
	int i;

	if (!result) {
		printf("stewart_forward_result: NULL\n");
		return;
	}

	printf("stewart_forward_result:\n");
	printf("  Total force: (%.2f, %.2f, %.2f)\n", result->total_force.x,
	       result->total_force.y, result->total_force.z);
	printf("  Total moment: (%.2f, %.2f, %.2f)\n", result->total_moment.x,
	       result->total_moment.y, result->total_moment.z);

	printf("\n  Leg lengths (mm):\n");
	for (i = 0; i < 6; i++) {
		printf("    [%d]: %.2f (error: %.2f)\n", i,
		       result->leg_lengths[i], result->leg_length_errors[i]);
	}

	printf("\n  Calculated position: (%.2f, %.2f, %.2f) mm\n",
	       result->position.x, result->position.y, result->position.z);
	printf("  Calculated rotation: (%.2f, %.2f, %.2f)°\n",
	       result->rotation.x, result->rotation.y, result->rotation.z);
}