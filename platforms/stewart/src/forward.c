#include "stewart/kinematics.h"
#include "robotics/math/vec3.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Simulerings-parametere for fjær-modell */
static const float SPRING_K = 0.6f; /* Fjærkonstant */
static const float DAMPENING = 0.999f; /* Demping */
static const float TIMESTEP = 0.01f; /* 10ms timestep */

/**
 * @function calculate_leg_forces
 * @api STATIC
 *
 * @input  geom->long_foot_length                           float (mm)
 * @input  result_inv_in->knee_points[6]                    struct vec3 (mm)
 * @input  result_inv_in->platform_points_transformed[6]    struct vec3 (mm)
 *
 * @output result_forv_out->leg_force_vectors[6]            struct vec3
 * @output result_forv_out->leg_lengths[6]                  float (mm)
 * @output result_forv_out->leg_length_errors[6]            float (mm)
 *
 * Beregner kraftvektorer for hvert ben basert på avvik mellom faktisk
 * og ønsket benlengde. Bruker fjær-modell: F = -k * Δx
 *
 * Algoritme:
 * 1. Beregn benvektor fra knee til platform
 * 2. Beregn deformert benlengde og avvik fra ønsket lengde
 * 3. Beregn kraftvektor med fjærkonstant: F = -k * length_error * retning
 */
static void
calculate_leg_forces(const struct stewart_geometry *geom,
		     const struct stewart_inverse_result *result_inv_in,
		     struct stewart_forward_result *result_forv_out)
{
	struct vec3 leg_vector, leg_direction;
	float deform_length, length_error;
	float force_magnitude;

	/*
	 * 6 føtter
	 */
	for (int i = 0; i < 6; i++) {
		/* Benvektor fra knee til platform */
		vec3_sub(&result_inv_in->platform_points_transformed[i],
			 &result_inv_in->knee_points[i], &leg_vector);

		/* Deformert benlengde */
		deform_length = vec3_length(&leg_vector);

		/* Avvik fra ønsket lengde */
		length_error = deform_length - geom->long_foot_length;

		/* Normaliser benvektor for retning */
		leg_direction = leg_vector;
		vec3_scale(&leg_direction, 1.0f / deform_length);

		/*
		 * Kraftvektor = fjærkraft * retning
		 * Positiv error → kraft skyver inn (mot base)
		 * Negativ error → kraft trekker ut (mot platform)
		 */
		force_magnitude = -SPRING_K * length_error;

		result_forv_out->leg_force_vectors[i] = leg_direction;
		vec3_scale(&result_forv_out->leg_force_vectors[i],
			   force_magnitude);

		/* Lagre lengder for debugging */
		result_forv_out->leg_lengths[i] = deform_length;
		result_forv_out->leg_length_errors[i] = length_error;
	}
}

/**
 * @function calculate_total_force_and_moment
 * @api STATIC
 *
 * @input  result_inv->platform_points_transformed[6]  struct vec3 (mm)
 * @input  result_forv->leg_force_vectors[6]           struct vec3
 *
 * @output total_force                                 struct vec3
 * @output total_moment                                struct vec3
 *
 * Summerer alle ben-kraftvektorer til total kraft.
 * Beregner moment som r × F (kryssprodukt) for hvert ben.
 *
 * Algoritme:
 * 1. Nullstill total kraft og moment
 * 2. For hvert ben:
 *    - Summer kraftvektor til total
 *    - Beregn moment = r × F (platform_point × force_vector)
 *    - Summer til total moment
 */
static void calculate_total_force_and_moment(
	const struct stewart_inverse_result *result_inv,
	const struct stewart_forward_result *result_forv,
	struct vec3 *total_force, struct vec3 *total_moment)
{
	struct vec3 moment_contribution;

	/* Nullstill totaler */
	*total_force = (struct vec3){ 0.0f, 0.0f, 0.0f };
	*total_moment = (struct vec3){ 0.0f, 0.0f, 0.0f };

	/*
	 * 6 føtter
	 */
	for (int i = 0; i < 6; i++) {
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

/*
 * entry point
 */
void stewart_kinematics_forward(const struct stewart_geometry *geom,
				struct stewart_pose *pose_calc,
				const struct stewart_inverse_result *result_inv,
				struct stewart_forward_result *result_forv)
{
	struct vec3 total_force, total_moment;

	assert(geom != NULL);
	assert(pose_calc != NULL);
	assert(result_inv != NULL);
	assert(result_forv != NULL);

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

	result_forv->pose_result = *pose_calc;
}

/*
 * printings
 */
void stewart_forward_result_print(const struct stewart_forward_result *result)
{
	int i;

	assert(result != NULL);

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
	       result->pose_result.tx, result->pose_result.ty,
	       result->pose_result.tz);
	printf("  Calculated rotation: (%.2f, %.2f, %.2f)°\n",
	       result->pose_result.rx, result->pose_result.ry,
	       result->pose_result.rz);
}