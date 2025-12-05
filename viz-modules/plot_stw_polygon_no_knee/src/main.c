/**
 * @file main.c
 * @brief Enkel Stewart platform visualizer (polygon-basert, ingen kinematikk)
 *
 * Denne visualizeren tegner Stewart platform som to sekskanter (base og
 * platform) koblet med rette linjer. Den utfører INGEN kinematikk-
 * beregninger - kun geometrisk transformasjon av platform-polygonen.
 *
 * BEGRENSNINGER:
 * - Bena tegnes som rette linjer (ikke realistisk)
 * - Ingen kneledd-visualisering
 * - Ingen motor-vinkel beregninger
 *
 * STØTTER FLERE ROBOTER:
 * - Bytter automatisk geometri basert på robot_type i pose packet
 * - Støtter ROBOT_TYPE_MX64 og ROBOT_TYPE_AX18
 *
 * For realistisk visualisering med kinematikk, se viz-stewart-kinematics.
 */

#include "robotics/math/matrix.h"
#include "robotics/math/utils.h"
#include "robotics/math/vec3.h"
#include "stewart/geometry.h"
#include "viz_protocol.h"
#include "udp.h"
#include <GLFW/glfw3.h>
#include <OpenGL/glu.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Globale variabler */
static struct viz_pose_packet current_pose;
static struct stewart_geometry current_geom;
static int udp_sock = -1;

/* Kamera variabler */
static float camera_azimuth = 45.0f;
static float camera_elevation = 30.0f;
static float ortho_scale = 400.0f;
static float camera_center_y = 100.0f;

/**
 * transform_point - Transformer punkt med pose
 * @x, @y, @z: input punkt (relativ til home)
 * @pose: pose (rotasjon og translasjon, absolutt)
 * @out_x, @out_y, @out_z: output transformert punkt
 *
 * Bruker robotics/math biblioteket for transformasjon.
 * Konverterer fra relativ til absolutt posisjon.
 */
static void transform_point(float x, float y, float z,
			    const struct viz_pose_packet *pose, float *out_x,
			    float *out_y, float *out_z)
{
	struct vec3 point = { x, y, z };
	struct vec3 transformed;
	struct mat3 rotation;

	/* Konverter til relativ posisjon (trekk fra home_height) */
	point.y -= current_geom.home_height;

	/* Lag rotasjonsmatrise fra pose */
	mat3_identity(&rotation);
	mat3_rotate_xyz(&rotation, deg_to_rad(pose->rx), deg_to_rad(pose->ry),
			deg_to_rad(pose->rz));

	/* Roter punkt */
	mat3_transform_vec3(&rotation, &point, &transformed);

	/* Translater til absolutt posisjon */
	*out_x = transformed.x + pose->tx;
	*out_y = transformed.y + pose->ty;
	*out_z = transformed.z + pose->tz;
}

/**
 * key_callback - Håndterer tastaturinput for kameranavigasjon
 */
static void key_callback(GLFWwindow *window, int key, int scancode, int action,
			 int mods)
{
	(void)scancode;
	(void)mods;

	if (action != GLFW_PRESS && action != GLFW_REPEAT)
		return;

	switch (key) {
	case GLFW_KEY_LEFT:
		camera_azimuth -= 5.0f;
		break;
	case GLFW_KEY_RIGHT:
		camera_azimuth += 5.0f;
		break;
	case GLFW_KEY_UP:
		camera_elevation += 5.0f;
		if (camera_elevation > 89.0f)
			camera_elevation = 89.0f;
		break;
	case GLFW_KEY_DOWN:
		camera_elevation -= 5.0f;
		if (camera_elevation < -89.0f)
			camera_elevation = -89.0f;
		break;
	case GLFW_KEY_Q:
		ortho_scale *= 0.9f;
		if (ortho_scale < 50.0f)
			ortho_scale = 50.0f;
		break;
	case GLFW_KEY_W:
		ortho_scale *= 1.1f;
		if (ortho_scale > 2000.0f)
			ortho_scale = 2000.0f;
		break;
	case GLFW_KEY_A:
		camera_center_y -= 10.0f;
		break;
	case GLFW_KEY_S:
		camera_center_y += 10.0f;
		break;
	case GLFW_KEY_R:
		/* Reset kamera */
		camera_azimuth = 45.0f;
		camera_elevation = 30.0f;
		ortho_scale = 400.0f;
		camera_center_y = 100.0f;
		printf("Camera reset\n");
		break;
	case GLFW_KEY_ESCAPE:
		glfwSetWindowShouldClose(window, GLFW_TRUE);
		break;
	}
}

/**
 * plot_stw_polygon_no_knee - Enkel polygon visualisering av Stewart platform
 *
 * Tegner base sekskant, platform sekskant (transformert med pose), og
 * rette ben-linjer fra base til platform. Ingen kinematikk-beregninger,
 * kun geometrisk transformasjon av platform polygon.
 *
 * INGEN KNELEDD - bena er rette linjer (ikke realistisk, men enkelt).
 */
static void plot_stw_polygon_no_knee(void)
{
	float platform_transformed[6][3];
	int i;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* Oppdater projection basert på ortho_scale */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float aspect = 600.0f / 400.0f;
	glOrtho(-ortho_scale * aspect, ortho_scale * aspect, -ortho_scale,
		ortho_scale, -2000.0, 2000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Beregn kamera posisjon fra azimuth og elevation */
	float cam_distance = 800.0f;
	float azimuth_rad = deg_to_rad(camera_azimuth);
	float elevation_rad = deg_to_rad(camera_elevation);

	float eye_x = cam_distance * cosf(elevation_rad) * cosf(azimuth_rad);
	float eye_y = cam_distance * sinf(elevation_rad);
	float eye_z = cam_distance * cosf(elevation_rad) * sinf(azimuth_rad);

	gluLookAt(eye_x, eye_y, eye_z, /* eye */
		  0.0, camera_center_y, 0.0, /* center */
		  0.0, 1.0, 0.0); /* up */

	/* Transform platform punkter med current_pose */
	for (i = 0; i < 6; i++) {
		transform_point(current_geom.platform_home_points[i].x,
				current_geom.platform_home_points[i].y,
				current_geom.platform_home_points[i].z,
				&current_pose, &platform_transformed[i][0],
				&platform_transformed[i][1],
				&platform_transformed[i][2]);
	}

	/* Tegn base sekskant (blå) */
	glColor3f(0.3f, 0.3f, 0.8f);
	glLineWidth(6.0f);
	glBegin(GL_LINE_LOOP);
	for (i = 0; i < 6; i++)
		glVertex3f(current_geom.base_points[i].x,
			   current_geom.base_points[i].y,
			   current_geom.base_points[i].z);
	glEnd();

	/* Tegn platform sekskant (rød) */
	glColor3f(0.8f, 0.3f, 0.3f);
	glLineWidth(6.0f);
	glBegin(GL_LINE_LOOP);
	for (i = 0; i < 6; i++)
		glVertex3f(platform_transformed[i][0],
			   platform_transformed[i][1],
			   platform_transformed[i][2]);
	glEnd();

	/*
	 * Tegn ben (grå linjer fra base til platform)
	 * OBS: Dette er IKKE realistisk - virkelige ben har kneledd!
	 * For realistisk visualisering, se viz-stewart-kinematics.
	 */
	glColor3f(0.5f, 0.5f, 0.5f);
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	for (i = 0; i < 6; i++) {
		glVertex3f(current_geom.base_points[i].x,
			   current_geom.base_points[i].y,
			   current_geom.base_points[i].z);
		glVertex3f(platform_transformed[i][0],
			   platform_transformed[i][1],
			   platform_transformed[i][2]);
	}
	glEnd();

	/* Tegn koordinatsystem i origo */
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	/* X-akse (rød) */
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(100.0f, 0.0f, 0.0f);
	/* Y-akse (grønn) */
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 100.0f, 0.0f);
	/* Z-akse (blå) */
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 100.0f);
	glEnd();
}

/**
 * poll_udp - Poll UDP socket for nye pose packets
 *
 * Sjekker om det er kommet nye pose-data via UDP.
 * Oppdaterer current_pose og bytter geometri basert på robot_type.
 */
static void poll_udp(void)
{
	struct viz_pose_packet packet;
	int n;

	n = udp_receive(udp_sock, &packet, sizeof(packet));

	if (n == sizeof(packet)) {
		/* Valider packet */
		if (packet.magic == VIZ_MAGIC &&
		    packet.type == VIZ_PACKET_POSE) {
			current_pose = packet;

			/* Bytt geometri basert på robot_type */
			if (packet.robot_type == ROBOT_TYPE_AX18) {
				current_geom = ROBOT_AX18;
			} else {
				/* Default: ROBOT_TYPE_MX64 */
				current_geom = ROBOT_MX64;
			}
		}
	}
}

int main(void)
{
	GLFWwindow *window;

	printf("Stewart Platform Visualizer\n");
	printf("============================\n\n");

	/* Initialiser geometri til default (MX64) */
	current_geom = ROBOT_AX18;  // ROBOT_AX18 ROBOT_MX64
	printf("Default geometry: ROBOT_MX64\n");
	printf("Supports dynamic geometry switching via robot_type field\n\n");

	/* Initialiser current_pose til home */
	memset(&current_pose, 0, sizeof(current_pose));

	/* Lag UDP receiver */
	udp_sock = udp_create_receiver(VIZ_PORT);
	if (udp_sock < 0) {
		fprintf(stderr, "Failed to create UDP receiver\n");
		return 1;
	}

	/* Initialiser GLFW */
	if (!glfwInit()) {
		fprintf(stderr, "Failed to initialize GLFW\n");
		return 1;
	}

	/* Lag vindu */
	window = glfwCreateWindow(600, 400, "Stewart Platform", NULL, NULL);
	if (!window) {
		fprintf(stderr, "Failed to create window\n");
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); /* VSync */

	/* Registrer tastaturkallback */
	glfwSetKeyCallback(window, key_callback);

	/* Setup OpenGL */
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

	printf("Window created. Listening for UDP packets...\n");
	printf("Camera controls:\n");
	printf("  Arrow keys: Rotate camera\n");
	printf("  Q/W: Zoom in/out\n");
	printf("  A/S: Lower/raise platform relative to camera\n");
	printf("  R: Reset camera\n");
	printf("  ESC: Exit\n\n");

	/* Main loop */
	while (!glfwWindowShouldClose(window)) {
		poll_udp();
		plot_stw_polygon_no_knee();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	glfwDestroyWindow(window);
	glfwTerminate();
	close(udp_sock);

	return 0;
}