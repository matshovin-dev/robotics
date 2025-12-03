#include "stewart/kinematics.h"

#include "robotics/math/geometry.h"
#include "robotics/math/matrix.h"
#include "robotics/math/utils.h"
#include "robotics/math/vec3.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Motor par for plan-definisjon */
static const int MOTOR_PAIRS[6] = { 1, 0, 3, 2, 5, 4 };

/**
 * soft_clamp - Soft clamp med dampening nær grenser
 * @value: verdi som skal clampes
 * @min: minimum verdi
 * @max: maximum verdi
 * @margin: margin for soft dampening
 *
 * Retur: clamped verdi
 */
static float soft_clamp(float value, float min, float max, float margin)
{
	if (value < min)
		return min;
	if (value > max)
		return max;

	/* Soft dampening nær grenser (ikke implementert enda) */
	return value;
}

/**
 * calculate_transformed_platform_points - Transform platform punkter med pose
 * @geom: robot geometri
 * @pose_in: ønsket pose (absolutt posisjon)
 * @result: output - transformerte platform punkter
 *
 * Roterer platform punkter med ZYX Euler angles og translerer med pose.
 * pose_in inneholder absolutt posisjon i world coordinates.
 * platform_points er definert ved home-posisjon, så vi trekker fra home_height
 * før rotasjon for å få lokale koordinater.
 */
void calculate_transformed_platform_points(
	const struct stewart_geometry *geom, const struct stewart_pose *pose_in,
	struct stewart_inverse_result *result)
{
	struct mat3 rotation;
	struct vec3 translation, local_point;
	int i;

	/* Lag rotasjonsmatrise fra Euler vinkler (ZYX convention) */
	mat3_identity(&rotation);
	mat3_rotate_xyz(&rotation, deg_to_rad(pose_in->rx),
			deg_to_rad(pose_in->ry), deg_to_rad(pose_in->rz));

	/* Translasjon fra pose (absolutt posisjon) */
	translation.x = pose_in->tx;
	translation.y = pose_in->ty;
	translation.z = pose_in->tz;

	/* Transform alle platform punkter */
	for (i = 0; i < 6; i++) {
		/* Konverter til lokalt koordinatsystem (trekk fra home_height) */
		local_point = geom->platform_points[i];
		local_point.y -= geom->home_height;

		/* Roter punkt */
		mat3_transform_vec3(&rotation, &local_point,
				    &result->platform_points_transformed[i]);

		/* Translater punkt */
		vec3_add(&result->platform_points_transformed[i], &translation,
			 &result->platform_points_transformed[i]);
	}
}

/**
 * calculate_motor_angle - Beregn motor vinkel for én motor
 * @motor_no: motor nummer (0-5)
 * @geom: robot geometri
 * @result: inverse kinematics resultat (input/output)
 * @debug: 1 = print debug info
 *
 * Beregner motor vinkel fra transformert platform punkt.
 *
 * 2D plan sett fra utsiden av hver motor (alle vinkler starter ved y- CCW):
 *
 *          top_attachment i 2D (p_proX, p_proY)
 *                /\
 *               /  \
 *              /    \ distance
 *      radius /      \
 *            /        \ target_angle <--\
 *           /    cos_a \.                |
 *    knee  /____________● origin         |
 *            motor_arm  |
 *                       |
 *                       y-
 */
static void calculate_motor_angle(int motor_no,
				  const struct stewart_geometry *geom,
				  struct stewart_inverse_result *result,
				  int debug)
{
	struct vec3 v_x_axis_2d, v_y_axis_2d;
	struct vec3 p_origin, relative;
	struct vec3 normal;
	float p_pro_x, p_pro_y;
	float distance, target_angle_rad;
	float dist_to_plane, radius;
	float cos_angle_rad, motor_angle_rad;
	int motor_pair;

	/*
	 * transformed_platform_points hentes fra result
	 */

	motor_pair = MOTOR_PAIRS[motor_no];
	p_origin = geom->base_points[motor_no];

	/*
	 * Lag 2D plan for å projisere platform punkt på
	 * X-akse peker til høyre (CCW sett utenfra)
	 * Y-akse peker opp
	 */
	if (motor_no & 1) { /* 1, 3, 5 */
		vec3_sub(&geom->base_points[motor_no],
			 &geom->base_points[motor_pair], &v_x_axis_2d);
	} else { /* 0, 2, 4 */
		vec3_sub(&geom->base_points[motor_pair],
			 &geom->base_points[motor_no], &v_x_axis_2d);
	}

	vec3_normalize(&v_x_axis_2d);
	v_y_axis_2d = (struct vec3){ 0.0f, 1.0f, 0.0f };

	/* Projiser platform punkt på 2D plan */
	vec3_sub(&result->platform_points_transformed[motor_no], &p_origin,
		 &relative);

	p_pro_x = vec3_dot(&relative, &v_x_axis_2d);
	p_pro_y = relative.y; /* Enklere enn dot siden Y-akse er (0,1,0) */

	/* Beregn avstand i 2D plan */
	distance = sqrtf(p_pro_x * p_pro_x + p_pro_y * p_pro_y);

	/* Vinkel fra rett ned til vektor fra motor til projeksjon */
	target_angle_rad = atan2f(p_pro_y, p_pro_x);

	/*
	 * Finn avstand fra platform punkt til plan
	 * Brukes for å beregne effektiv radius av servo arm sirkel
	 */
	vec3_cross(&v_x_axis_2d, &v_y_axis_2d, &normal);
	dist_to_plane = distance_point_to_plane(
		&result->platform_points_transformed[motor_no], &p_origin,
		&normal);

	/*
	 * Beregn radius av servo arm sirkel på plan
	 * Pythagoras: radius² + dist_to_plane² = long_foot_length²
	 */
	radius = 0.0f;
	if (dist_to_plane < geom->long_foot_length) {
		radius = sqrtf(geom->long_foot_length * geom->long_foot_length -
			       dist_to_plane * dist_to_plane);
	}

	/*
	 * Beregn vinkel fra cosinus-setningen
	 * cos(α) = (a² + b² - c²) / (2ab)
	 * a = short_foot_length, b = distance, c = radius
	 */
	if (distance > geom->short_foot_length + radius) {
		/* Platform punkt for langt unna - maksimal strekk */
		cos_angle_rad = 0.0f;
	} else if (radius > distance + geom->short_foot_length) {
		/* 180 grader - fullstendig krøkket sammen */
		cos_angle_rad = M_PI;
	} else {
		cos_angle_rad = acosf(
			(geom->short_foot_length * geom->short_foot_length +
			 distance * distance - radius * radius) /
			(2.0f * geom->short_foot_length * distance));
	}

	/*
	 * Beregn motor vinkel basert på motor arm retning
	 * Odd motors (1,3,5) og even motors (0,2,4) har forskjellig
	 * montering
	 */
	if (motor_no & 1) { /* 1, 3, 5 */
		if (geom->motor_arm_outward)
			motor_angle_rad =
				M_PI / 2.0f + target_angle_rad - cos_angle_rad;
		else
			motor_angle_rad =
				M_PI / 2.0f + target_angle_rad + cos_angle_rad;
	} else { /* 0, 2, 4 */
		if (geom->motor_arm_outward)
			motor_angle_rad =
				M_PI / 2.0f + target_angle_rad + cos_angle_rad;
		else
			motor_angle_rad =
				M_PI / 2.0f + target_angle_rad - cos_angle_rad;
	}

	/* Konverter til grader */
	result->motor_angles_deg[motor_no] = rad_to_deg(motor_angle_rad);

	/* Hard clamp til geometri-grenser */
	if (motor_no & 1) { /* 1, 3, 5 */
		result->motor_angles_deg[motor_no] =
			soft_clamp(result->motor_angles_deg[motor_no],
				   geom->min_motor_angle_135_deg,
				   geom->max_motor_angle_135_deg, 10.0f);
	} else { /* 0, 2, 4 */
		result->motor_angles_deg[motor_no] =
			soft_clamp(result->motor_angles_deg[motor_no],
				   geom->min_motor_angle_024_deg,
				   geom->max_motor_angle_024_deg, 10.0f);
	}
}

/**
 * calculate_knee_positions - Beregn kne posisjoner fra motor vinkler
 * @geom: robot geometri
 * @result: inverse kinematics resultat (input/output)
 *
 * Beregner 3D posisjon av hvert kne basert på motor vinkel.
 * Kne er der servo arm møter pushrod.
 */
static void calculate_knee_positions(const struct stewart_geometry *geom,
				     struct stewart_inverse_result *result)
{
	/*
	 * Henter motorvinkler fra result
	 */
	struct mat3 rot_x, rot_y;
	struct vec3 foot;
	float y_angle;
	int motor_no;

	for (motor_no = 0; motor_no < 6; motor_no++) {
		/* Start med foot pekende rett ned i origo */
		foot = (struct vec3){ 0.0f, -geom->short_foot_length, 0.0f };

		/* Roter rundt X-akse med motor vinkel */
		mat3_identity(&rot_x);
		mat3_rotate_x(&rot_x,
			      deg_to_rad(result->motor_angles_deg[motor_no]));
		mat3_transform_vec3(&rot_x, &foot, &foot);

		/* Roter rundt Y-akse til motor posisjon */
		if (motor_no & 1) { /* 1, 3, 5 */
			y_angle = -30.0f + ((motor_no - 1) / 2) * 120.0f;
		} else { /* 0, 2, 4 */
			y_angle = -30.0f + (motor_no / 2) * 120.0f;
		}

		mat3_identity(&rot_y);
		mat3_rotate_y(&rot_y, deg_to_rad(y_angle));
		mat3_transform_vec3(&rot_y, &foot, &foot);

		/* Translater til motor posisjon (world coordinates) */
		vec3_add(&foot, &geom->base_points[motor_no],
			 &result->knee_points[motor_no]);
	}
}

void stewart_kinematics_inverse(const struct stewart_geometry *geom,
				const struct stewart_pose *pose_in,
				struct stewart_inverse_result *result,
				int debug)
{
	int i;

	assert(geom != NULL);
	assert(pose_in != NULL);
	assert(result != NULL);

	/* Nullstill resultat */
	memset(result, 0, sizeof(struct stewart_inverse_result));

	/* Transform alle platform punkter med pose */
	calculate_transformed_platform_points(geom, pose_in, result);

	/* Beregn motor vinkler for alle 6 motorer */
	for (i = 0; i < 6; i++)
		calculate_motor_angle(i, geom, result, debug);

	/* Beregn kne posisjoner */
	calculate_knee_positions(geom, result);
}

void stewart_inverse_result_print(const struct stewart_inverse_result *result)
{
	int i;

	assert(result != NULL);

	printf("stewart_inverse_result:\n");
	printf("  Error: %d\n", result->error);

	printf("\n  Motor angles (deg):\n");
	for (i = 0; i < 6; i++) {
		printf("    [%d]: %7.2f°\n", i, result->motor_angles_deg[i]);
	}

	printf("\n  Knee points (mm):\n");
	for (i = 0; i < 6; i++) {
		printf("    [%d]: (%7.2f, %7.2f, %7.2f)\n", i,
		       result->knee_points[i].x, result->knee_points[i].y,
		       result->knee_points[i].z);
	}

	printf("\n  Platform points transformed (mm):\n");
	for (i = 0; i < 6; i++) {
		printf("    [%d]: (%7.2f, %7.2f, %7.2f)\n", i,
		       result->platform_points_transformed[i].x,
		       result->platform_points_transformed[i].y,
		       result->platform_points_transformed[i].z);
	}
}