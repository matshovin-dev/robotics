#include "stewart/geometry.h"
#include <assert.h>
#include <stdio.h>

void stewart_geometry_print(const struct stewart_geometry *geom)
{
	assert(geom != NULL);

	printf("stewart_geometry:\n");
	printf("  Home height: %.2f mm\n", geom->home_height);
	printf("  Short foot: %.2f mm\n", geom->short_foot_length);
	printf("  Long foot: %.2f mm\n", geom->long_foot_length);
	printf("  Max rotation amp: %.2f deg\n",
	       geom->max_pose_rotation_amplitude);
	printf("  Max translation amp: %.2f mm\n",
	       geom->max_pose_translation_amplitude);

	printf("\n  Base points:\n");
	for (int i = 0; i < 6; i++) {
		printf("    [%d]: (%.2f, %.2f, %.2f)\n", i,
		       geom->base_points[i].x, geom->base_points[i].y,
		       geom->base_points[i].z);
	}

	printf("\n  Platform points (home position):\n");
	for (int i = 0; i < 6; i++) {
		printf("    [%d]: (%.2f, %.2f, %.2f)\n", i,
		       geom->platform_home_points[i].x,
		       geom->platform_home_points[i].y,
		       geom->platform_home_points[i].z);
	}
}

/**
 * ROBOT_MX64 - MX64 Dynamixel motor konfigurasjon
 *
 * Stewart platform med MX64 motorer.
 * Geometri målt fra SolidWorks og verifisert på fysisk robot.
 *
 * Motor nummerering 0-5 i CCW rekkefølge sett ovenfra.
 * Motorer delt i to grupper (024 og 135) pga speil-montering.
 *
 * Origo ligger i platform base i høyde med de 8 motorakslene (base_point)
 *
 * Koordinatsystem:
 *   X+ = Høyre
 *   Y+ = Opp
 *   Z+ = Ut av skjermen (vekk fra betrakter)
 */
const struct stewart_geometry ROBOT_MX64 = {
	/*
	 * Platform (top plate) attachment points ved home-posisjon
	 * Målt fra SolidWorks og rotert 120°/240° CCW (sett nedenfra)
	 * y-koordinat er home_height (205mm)
	 */
	.platform_home_points = {
		{74.91f, 205.0f, 69.65f},   /* 0 - målt fra SolidWorks */
		{97.77f, 205.0f, 30.05f},   /* 1 - point[5] rotert 120° CCW */
		{22.86f, 205.0f, -99.70f},  /* 2 - point[0] rotert 120° CCW */
		{-22.86f, 205.0f, -99.70f}, /* 3 - point[5] rotert 240° CCW */
		{-97.77f, 205.0f, 30.05f},  /* 4 - point[0] rotert 240° CCW */
		{-74.91f, 205.0f, 69.65f}   /* 5 - målt fra SolidWorks */
	},

	/*
	 * Base (bottom plate) attachment points (motor akse posisjoner)
	 * Målt fra SolidWorks og rotert 120°/240° CCW (sett nedenfra)
	 */
	.base_points = {
		{59.24f, 0.0f, 62.49f},   /* 0 - målt fra SolidWorks */
		{83.74f, 0.0f, 20.06f},   /* 1 - point[5] rotert 120° CCW */
		{24.50f, 0.0f, -82.55f},  /* 2 - point[0] rotert 120° CCW */
		{-24.50f, 0.0f, -82.55f}, /* 3 - point[5] rotert 240° CCW */
		{-83.74f, 0.0f, 20.06f},  /* 4 - point[0] rotert 240° CCW */
		{-59.24f, 0.0f, 62.49f}   /* 5 - målt fra SolidWorks */
	},

	/* Hard limits målt på fysisk robot */
	.max_motor_angle_024_deg = 301.348f,
	.min_motor_angle_024_deg = 190.027f,
	.max_motor_angle_135_deg = 169.98f,
	.min_motor_angle_135_deg = 58.45f,
	.motor_clamp_limit_angle_deg = 5.0f,

	/* Mekaniske dimensjoner */
	.home_height = 205.0f,
	.short_foot_length = 70.0f,
	.long_foot_length = 202.42f,
	.motor_arm_outward = 1,

	/* Kinematiske grenser */
	.max_pose_rotation_amplitude = 20.0f,
	.max_pose_rotation_bias = 20.0f,
	.max_pose_translation_amplitude = 20.0f,
	.max_pose_translation_bias = 20.0f
};

/**
 * ROBOT_AX18 - AX18 servo konfigurasjon
 *
 * Mindre Stewart platform med AX18 servos.
 * Koordinatsystem samme som MX64 versjon.
 */
const struct stewart_geometry ROBOT_AX18 = {
	.platform_home_points = { { 5.50f, 140.0f, 74.72f },
				  { 67.46f, 140.0f, -32.60f },
				  { 61.96f, 140.0f, -42.12f },
				  { -61.96f, 140.0f, -42.12f },
				  { -67.46f, 140.0f, -32.60f },
				  { -5.50f, 140.0f, 74.72f } },

	.base_points = { { 33.29f, 0.0f, 74.87f },
			 { 81.48f, 0.00f, -8.61f },
			 { 48.19f, 0.00f, -66.26f },
			 { -48.19f, 0.00f, -66.26f },
			 { -81.48f, 0.00f, -8.61f },
			 { -33.29f, 0.0f, 74.87f } },

	.max_motor_angle_024_deg = 176.484375f,
	.min_motor_angle_024_deg = 73.9453125f,
	.max_motor_angle_135_deg = 286.0546875f,
	.min_motor_angle_135_deg = 183.515625f,
	.motor_clamp_limit_angle_deg = 5.0f,

	.home_height = 140.0f,
	.short_foot_length = 36.0f,
	.long_foot_length = 137.5f,
	.motor_arm_outward = 0,

	.max_pose_rotation_amplitude = 15.0f,
	.max_pose_rotation_bias = 15.0f,
	.max_pose_translation_amplitude = 15.0f,
	.max_pose_translation_bias = 15.0f
};