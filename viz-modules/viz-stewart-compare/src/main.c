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
static struct viz_pose_packet pose1; /* Reference/Target (port 9001) */
static struct viz_pose_packet pose2; /* Actual/Current (port 9002) */
static struct stewart_geometry geometry;
static struct stewart_inverse_result result1;
static struct stewart_inverse_result result2;
static int udp_sock1 = -1; /* Port 9001 */
static int udp_sock2 = -1; /* Port 9002 */
static int has_error1 = 0;
static int has_error2 = 0;

/* Camera state */
static float camera_azimuth = 45.0f; /* Horizontal rotation (degrees) */
static float camera_elevation = 30.0f; /* Vertical tilt (degrees) */
static float camera_distance = 600.0f; /* Distance from target */
static float ortho_scale = 200.0f; /* Orthographic view scale */

/**
 * draw_sphere - Tegn en sfære på gitt posisjon
 * @x, @y, @z: senterpunkt
 * @radius: radius
 * @slices: antall vertikale segmenter
 * @stacks: antall horisontale segmenter
 */
static void draw_sphere(float x, float y, float z, float radius, int slices,
			int stacks)
{
	GLUquadric *quad;

	quad = gluNewQuadric();
	glPushMatrix();
	glTranslatef(x, y, z);
	gluSphere(quad, radius, slices, stacks);
	glPopMatrix();
	gluDeleteQuadric(quad);
}

/**
 * key_callback - Håndter tastatur-input
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
	case GLFW_KEY_R:
		/* Reset kamera */
		camera_azimuth = 45.0f;
		camera_elevation = 30.0f;
		ortho_scale = 400.0f;
		printf("Camera reset\n");
		break;
	case GLFW_KEY_ESCAPE:
		glfwSetWindowShouldClose(window, GLFW_TRUE);
		break;
	}
}

/**
 * render_pose - Render en pose med gitt farge
 * @result: inverse kinematics result
 * @r, @g, @b: base RGB color (0-1)
 * @label: label for debugging
 */
static void render_pose(const struct stewart_inverse_result *result, float r,
			float g, float b, const char *label)
{
	int i;

	/* Tegn platform sekskant */
	glColor3f(r * 0.8f, g * 0.8f, b * 0.8f);
	glLineWidth(6.0f);
	glBegin(GL_LINE_LOOP);
	for (i = 0; i < 6; i++) {
		const struct vec3 *p = &result->platform_points_transformed[i];
		glVertex3f(p->x, p->y, p->z);
	}
	glEnd();

	/* Tegn motor arms (base → knee) */
	glColor3f(r * 0.9f, g * 0.9f, b * 0.5f);
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	for (i = 0; i < 6; i++) {
		struct vec3 *base = &geometry.base_points[i];
		const struct vec3 *knee = &result->knee_points[i];
		glVertex3f(base->x, base->y, base->z);
		glVertex3f(knee->x, knee->y, knee->z);
	}
	glEnd();

	/* Tegn pushrods (knee → platform) */
	glColor3f(r, g * 0.6f, b * 0.6f);
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	for (i = 0; i < 6; i++) {
		const struct vec3 *knee = &result->knee_points[i];
		const struct vec3 *platform =
			&result->platform_points_transformed[i];
		glVertex3f(knee->x, knee->y, knee->z);
		glVertex3f(platform->x, platform->y, platform->z);
	}
	glEnd();

	/* Tegn knee points (kuler) */
	glColor3f(r, g, b);
	for (i = 0; i < 6; i++) {
		const struct vec3 *knee = &result->knee_points[i];
		draw_sphere(knee->x, knee->y, knee->z, 5.0f, 12, 12);
	}
}

/**
 * render_comparison - Render begge poser side-om-side
 */
static void render_comparison(void)
{
	int i;
	float azimuth_rad, elevation_rad;
	float eye_x, eye_y, eye_z;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* Oppdater orthographic projection */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float aspect = 1024.0f / 768.0f;
	glOrtho(-ortho_scale * aspect, ortho_scale * aspect, -ortho_scale,
		ortho_scale, -2000.0, 2000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Beregn kamera posisjon */
	azimuth_rad = camera_azimuth * M_PI / 180.0f;
	elevation_rad = camera_elevation * M_PI / 180.0f;

	eye_x = camera_distance * cosf(elevation_rad) * cosf(azimuth_rad);
	eye_y = camera_distance * sinf(elevation_rad);
	eye_z = camera_distance * cosf(elevation_rad) * sinf(azimuth_rad);

	gluLookAt(eye_x, eye_y + 100.0f, eye_z, /* eye */
		  0.0, 100.0, 0.0, /* center */
		  0.0, 1.0, 0.0); /* up */

	/* Tegn base sekskant (grå - deles av begge poser) */
	glColor3f(0.4f, 0.4f, 0.4f);
	glLineWidth(6.0f);
	glBegin(GL_LINE_LOOP);
	for (i = 0; i < 6; i++) {
		struct vec3 *p = &geometry.base_points[i];
		glVertex3f(p->x, p->y, p->z);
	}
	glEnd();

	/* Tegn pose 1 (cyan - reference/target) */
	render_pose(&result1, 0.2f, 0.9f, 0.9f, "Pose 1");

	/* Tegn pose 2 (magenta - actual/current) */
	render_pose(&result2, 0.9f, 0.2f, 0.9f, "Pose 2");

	/* Tegn koordinatsystem */
	glLineWidth(4.0f);
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
 * compute_kinematics_for_pose - Beregn IK for en pose
 */
static void compute_kinematics_for_pose(const struct viz_pose_packet *packet,
					struct stewart_inverse_result *result,
					int *has_error, int pose_num)
{
	struct stewart_pose pose;

	/* Konverter UDP packet til stewart pose (absolutte koordinater) */
	pose.rx = packet->rx;
	pose.ry = packet->ry;
	pose.rz = packet->rz;
	pose.tx = packet->tx;
	pose.ty = packet->ty;
	pose.tz = packet->tz;

	stewart_kinematics_inverse(&geometry, &pose, result, 0);
	*has_error = result->error;

	/* Print motor angles ved endring */
	static float last_angles[2][6] = { { 0 }, { 0 } };
	int idx = pose_num - 1;
	int changed = 0;

	for (int i = 0; i < 6; i++) {
		if (fabsf(result->motor_angles_deg[i] - last_angles[idx][i]) >
		    0.5f) {
			changed = 1;
			last_angles[idx][i] = result->motor_angles_deg[i];
		}
	}

	if (changed) {
		// printf("Pose %d Motors: ", pose_num);
		for (int i = 0; i < 6; i++) {
			;  // printf("[%d]=%.1f° ", i,
			   // result->motor_angles_deg[i]);
		}
		if (*has_error)
			printf(" ⚠️  ERROR!");
		printf("\n");
	}
}

/**
 * poll_udp - Poll begge UDP sockets for nye packets
 */
static void poll_udp(void)
{
	struct viz_pose_packet packet;
	int n;

	/* Poll socket 1 (port 9001) */
	n = udp_receive(udp_sock1, &packet, sizeof(packet));
	if (n == sizeof(packet)) {
		if (packet.magic == VIZ_MAGIC &&
		    packet.type == VIZ_PACKET_POSE) {
			pose1 = packet;

			/* Oppdater geometri hvis nødvendig */
			if (packet.robot_type == ROBOT_TYPE_MX64) {
				geometry = ROBOT_MX64;
			} else if (packet.robot_type == ROBOT_TYPE_AX18) {
				geometry = ROBOT_AX18;
			}

			compute_kinematics_for_pose(&pose1, &result1,
						    &has_error1, 1);
		}
	}

	/* Poll socket 2 (port 9002) */
	n = udp_receive(udp_sock2, &packet, sizeof(packet));
	if (n == sizeof(packet)) {
		if (packet.magic == VIZ_MAGIC &&
		    packet.type == VIZ_PACKET_POSE) {
			pose2 = packet;
			compute_kinematics_for_pose(&pose2, &result2,
						    &has_error2, 2);
		}
	}
}

int main(void)
{
	GLFWwindow *window;

	printf("Stewart Platform Comparison Visualizer\n");
	printf("======================================\n\n");

	/* Initialiser geometry */
	geometry = ROBOT_MX64;

	/* Initialiser begge poser til home */
	memset(&pose1, 0, sizeof(pose1));
	pose1.magic = VIZ_MAGIC;
	pose1.type = VIZ_PACKET_POSE;
	pose1.robot_type = ROBOT_TYPE_MX64;

	memset(&pose2, 0, sizeof(pose2));
	pose2.magic = VIZ_MAGIC;
	pose2.type = VIZ_PACKET_POSE;
	pose2.robot_type = ROBOT_TYPE_MX64;

	/* Beregn initial kinematikk */
	compute_kinematics_for_pose(&pose1, &result1, &has_error1, 1);
	compute_kinematics_for_pose(&pose2, &result2, &has_error2, 2);

	/* Lag UDP receivers */
	udp_sock1 = udp_create_receiver(9001);
	if (udp_sock1 < 0) {
		fprintf(stderr, "Failed to create UDP receiver on port 9001\n");
		return 1;
	}

	udp_sock2 = udp_create_receiver(9002);
	if (udp_sock2 < 0) {
		fprintf(stderr, "Failed to create UDP receiver on port 9002\n");
		close(udp_sock1);
		return 1;
	}

	printf("Listening on:\n");
	printf("  Port 9001: Pose 1 (CYAN - reference/target)\n");
	printf("  Port 9002: Pose 2 (MAGENTA - actual/current)\n\n");

	/* Initialiser GLFW */
	if (!glfwInit()) {
		fprintf(stderr, "Failed to initialize GLFW\n");
		return 1;
	}

	/* Lag vindu */
	window = glfwCreateWindow(500, 400, "Stewart Compare", NULL, NULL);
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

	printf("Window created. Ready to compare!\n");
	printf("\nControls:\n");
	printf("  Arrow keys:  Rotate camera\n");
	printf("  Q/W:         Zoom in/out\n");
	printf("  R:           Reset camera\n");
	printf("  ESC:         Exit\n\n");

	/* Main loop */
	while (!glfwWindowShouldClose(window)) {
		poll_udp();
		render_comparison();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	glfwDestroyWindow(window);
	glfwTerminate();
	close(udp_sock1);
	close(udp_sock2);

	return 0;
}
