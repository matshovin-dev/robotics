#include "robotics/math/geometry.h"
#include "robotics/math/matrix.h"
#include "robotics/math/utils.h"
#include "robotics/math/vec3.h"
#include "stewart/geometry.h"
#include "stewart/kinematics.h"
#include "stewart/pose.h"
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

/* Global state */
static struct viz_pose_packet current_pose;
static struct stewart_geometry geometry;
static struct stewart_inverse_result inverse_result;
static int udp_sock = -1;
static int has_error = 0;

/* Camera state */
static float camera_azimuth = 45.0f; /* Horizontal rotation (degrees) */
static float camera_elevation = 30.0f; /* Vertical tilt (degrees) */
static float ortho_scale = 400.0f; /* Orthographic view scale */
static float camera_center_y = 100.0f; /* Camera look-at Y position */

/**
 * key_callback - Håndter tastatur-input
 * @window: GLFW vindu
 * @key: tast som ble trykket
 * @scancode: system-spesifikk scancode
 * @action: GLFW_PRESS, GLFW_RELEASE, eller GLFW_REPEAT
 * @mods: modifier bits (shift, ctrl, alt, etc)
 *
 * Piltaster: Roter kamera
 * +/-: Zoom inn/ut
 * R: Reset kamera
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
 * plot_stw_polygon - Render Stewart platform med kinematikk
 *
 * Tegner:
 * - Base sekskant (blå)
 * - Platform sekskant (rød)
 * - Motor arms (grå: base → knee)
 * - Pushrods (grå: knee → platform)
 * - Knee points (små kuler)
 * - Koordinatsystem
 *
 * Viser faktisk kinematikk fra inverse beregning med kneledd.
 */
static void plot_stw_polygon(void)
{
	int i;
	float azimuth_rad, elevation_rad;
	float eye_x, eye_y, eye_z;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* Oppdater orthographic projection basert på ortho_scale */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float aspect = 1024.0f / 768.0f;
	glOrtho(-ortho_scale * aspect, ortho_scale * aspect, -ortho_scale,
		ortho_scale, -2000.0, 2000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Beregn kamera posisjon fra azimuth og elevation */
	float cam_distance = 800.0f;
	azimuth_rad = deg_to_rad(camera_azimuth);
	elevation_rad = deg_to_rad(camera_elevation);

	eye_x = cam_distance * cosf(elevation_rad) * cosf(azimuth_rad);
	eye_y = cam_distance * sinf(elevation_rad);
	eye_z = cam_distance * cosf(elevation_rad) * sinf(azimuth_rad);

	gluLookAt(eye_x, eye_y, eye_z, /* eye */
		  0.0, camera_center_y, 0.0, /* center */
		  0.0, 1.0, 0.0); /* up */

	/* Tegn base sekskant (blå) */
	glColor3f(0.3f, 0.3f, 0.8f);
	glLineWidth(2.0f);
	glBegin(GL_LINE_LOOP);
	for (i = 0; i < 6; i++) {
		struct vec3 *p = &geometry.base_points[i];
		glVertex3f(p->x, p->y, p->z);
	}
	glEnd();

	/* Tegn platform sekskant (rød) */
	glColor3f(0.8f, 0.3f, 0.3f);
	glLineWidth(2.0f);
	glBegin(GL_LINE_LOOP);
	for (i = 0; i < 6; i++) {
		struct vec3 *p = &inverse_result.platform_points_transformed[i];
		glVertex3f(p->x, p->y, p->z);
	}
	glEnd();

	/* Tegn ben (grå linjer: base → knee → platform) */
	glColor3f(0.5f, 0.5f, 0.5f);
	glLineWidth(1.5f);
	glBegin(GL_LINES);
	for (i = 0; i < 6; i++) {
		struct vec3 *base = &geometry.base_points[i];
		struct vec3 *knee = &inverse_result.knee_points[i];
		struct vec3 *platform =
			&inverse_result.platform_points_transformed[i];

		/* Motor arm: base → knee */
		glVertex3f(base->x, base->y, base->z);
		glVertex3f(knee->x, knee->y, knee->z);

		/* Pushrod: knee → platform */
		glVertex3f(knee->x, knee->y, knee->z);
		glVertex3f(platform->x, platform->y, platform->z);
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
 * compute_kinematics - Beregn inverse kinematics for current pose
 *
 * Konverterer UDP pose packet til stewart_pose og kjører inverse
 * kinematics. Oppdaterer inverse_result og has_error flag.
 */
static void compute_kinematics(void)
{
	struct stewart_pose pose;

	/* Konverter UDP packet til stewart pose (absolutte koordinater) */
	pose.rx = current_pose.rx;
	pose.ry = current_pose.ry;
	pose.rz = current_pose.rz;
	pose.tx = current_pose.tx;
	pose.ty = current_pose.ty;
	pose.tz = current_pose.tz;

	/* Kjør inverse kinematics */
	stewart_kinematics_inverse(&geometry, &pose, &inverse_result, 0);

	/* Sjekk for error */
	has_error = inverse_result.error;

	/* Print motor angles (kun ved endring) */
	static float last_angles[6] = { 0 };
	int changed = 0;
	for (int i = 0; i < 6; i++) {
		if (fabsf(inverse_result.motor_angles_deg[i] - last_angles[i]) >
		    0.5f) {
			changed = 1;
			last_angles[i] = inverse_result.motor_angles_deg[i];
		}
	}

	if (changed) {
		printf("Motors: ");
		for (int i = 0; i < 6; i++) {
			printf("[%d]=%.1f° ", i,
			       inverse_result.motor_angles_deg[i]);
		}
		if (has_error)
			printf(" ⚠️  ERROR: Pose unreachable!");
		printf("\n");
	}
}

/**
 * poll_udp - Poll UDP socket for nye pose packets
 *
 * Sjekker om det er kommet nye pose-data via UDP.
 * Oppdaterer current_pose og kjører kinematikk hvis gyldig packet mottas.
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

			/* Oppdater geometri hvis robot type endres */
			if (packet.robot_type == ROBOT_TYPE_MX64) {
				geometry = ROBOT_MX64;
			} else if (packet.robot_type == ROBOT_TYPE_AX18) {
				geometry = ROBOT_AX18;
			}

			/* Beregn kinematikk */
			compute_kinematics();
		}
	}
}

int main(void)
{
	GLFWwindow *window;

	printf("Stewart Platform Visualizer (with kinematics)\n");
	printf("=============================================\n\n");

	/* Initialiser geometry til MX64 (default) */
	geometry = ROBOT_MX64;	// ROBOT_MX64 ROBOT_AX18

	/* Initialiser current_pose til home */
	memset(&current_pose, 0, sizeof(current_pose));
	current_pose.magic = VIZ_MAGIC;
	current_pose.type = VIZ_PACKET_POSE;
	current_pose.robot_type = ROBOT_TYPE_MX64;

	/* Beregn initial kinematikk for home pose */
	compute_kinematics();

	/* Lag UDP receiver */
	udp_sock = udp_create_receiver(VIZ_PORT);
	if (udp_sock < 0) {
		fprintf(stderr, "Failed to create UDP receiver\n");
		return 1;
	}

	printf("Listening on UDP port %d...\n\n", VIZ_PORT);

	/* Initialiser GLFW */
	if (!glfwInit()) {
		fprintf(stderr, "Failed to initialize GLFW\n");
		return 1;
	}

	/* Lag vindu */
	window = glfwCreateWindow(1024, 768, "Stewart Platform", NULL, NULL);
	if (!window) {
		fprintf(stderr, "Failed to create window\n");
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); /* VSync */

	/* Register keyboard callback */
	glfwSetKeyCallback(window, key_callback);

	/* Setup OpenGL */
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.05f, 0.05f, 0.1f, 1.0f);

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
		plot_stw_polygon();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	glfwDestroyWindow(window);
	glfwTerminate();
	close(udp_sock);

	return 0;
}
