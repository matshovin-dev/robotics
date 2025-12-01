#include "udp.h"
#include "viz_protocol.h"
#include <GLFW/glfw3.h>
#include <OpenGL/glu.h> /* ← LEGG TIL */
#include <math.h>
#include <robotics/math/matrix.h>
#include <robotics/math/utils.h>
#include <robotics/math/vec3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> /* ← LEGG TIL */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Globale variabler */
static struct viz_pose_packet current_pose;
static int udp_sock = -1;

/* MX64 geometri (hardkoded fra ROBOT_MX64) */
static const float base_points[6][3] = {
	{ 59.24f, 0.0f, 62.49f },  { 83.74f, 0.0f, 20.06f },
	{ 24.50f, 0.0f, -82.55f }, { -24.50f, 0.0f, -82.55f },
	{ -83.74f, 0.0f, 20.06f }, { -59.24f, 0.0f, 62.49f }
};

static const float platform_points[6][3] = {
	{ 74.91f, 0.0f, 69.65f },  { 97.77f, 0.0f, 30.05f },
	{ 22.86f, 0.0f, -99.70f }, { -22.86f, 0.0f, -99.70f },
	{ -97.77f, 0.0f, 30.05f }, { -74.91f, 0.0f, 69.65f }
};

static const float home_height = 205.0f;

/**
 * transform_point - Transformer punkt med pose
 * @x, @y, @z: input punkt
 * @pose: pose (rotasjon og translasjon)
 * @out_x, @out_y, @out_z: output transformert punkt
 *
 * Bruker robotics/math biblioteket for transformasjon.
 */
static void transform_point(float x, float y, float z,
			    const struct viz_pose_packet *pose, float *out_x,
			    float *out_y, float *out_z)
{
	struct vec3 point = { x, y, z };
	struct vec3 transformed;
	struct mat3 rotation;

	/* Lag rotasjonsmatrise fra pose */
	mat3_identity(&rotation);
	mat3_rotate_xyz(&rotation, deg_to_rad(pose->rx), deg_to_rad(pose->ry),
			deg_to_rad(pose->rz));

	/* Roter punkt */
	mat3_transform_vec3(&rotation, &point, &transformed);

	/* Translater (inkluderer home_height) */
	*out_x = transformed.x + pose->tx;
	*out_y = transformed.y + pose->ty + home_height;
	*out_z = transformed.z + pose->tz;
}

/**
 * render_stewart - Render Stewart platform
 *
 * Tegner base sekskant, platform sekskant (transformert), og ben.
 */
static void render_stewart(void)
{
	float platform_transformed[6][3];
	int i;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Kamera posisjon */
	gluLookAt(500.0, 300.0, 500.0, /* eye */
		  0.0, 100.0, 0.0, /* center */
		  0.0, 1.0, 0.0); /* up */

	/* Transform platform punkter med current_pose */
	for (i = 0; i < 6; i++) {
		transform_point(platform_points[i][0], platform_points[i][1],
				platform_points[i][2], &current_pose,
				&platform_transformed[i][0],
				&platform_transformed[i][1],
				&platform_transformed[i][2]);
	}

	/* Tegn base sekskant (blå) */
	glColor3f(0.3f, 0.3f, 0.8f);
	glLineWidth(2.0f);
	glBegin(GL_LINE_LOOP);
	for (i = 0; i < 6; i++)
		glVertex3f(base_points[i][0], base_points[i][1],
			   base_points[i][2]);
	glEnd();

	/* Tegn platform sekskant (rød) */
	glColor3f(0.8f, 0.3f, 0.3f);
	glBegin(GL_LINE_LOOP);
	for (i = 0; i < 6; i++)
		glVertex3f(platform_transformed[i][0],
			   platform_transformed[i][1],
			   platform_transformed[i][2]);
	glEnd();

	/* Tegn ben (grå linjer fra base til platform) */
	glColor3f(0.5f, 0.5f, 0.5f);
	glBegin(GL_LINES);
	for (i = 0; i < 6; i++) {
		glVertex3f(base_points[i][0], base_points[i][1],
			   base_points[i][2]);
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
 * Oppdaterer current_pose hvis gyldig packet mottas.
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
			printf("Pose: rx=%.1f ry=%.1f rz=%.1f "
			       "tx=%.1f ty=%.1f tz=%.1f\n",
			       packet.rx, packet.ry, packet.rz, packet.tx,
			       packet.ty, packet.tz);
		}
	}
}

int main(void)
{
	GLFWwindow *window;

	printf("Stewart Platform Visualizer\n");
	printf("============================\n\n");

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
	window = glfwCreateWindow(800, 600, "Stewart Platform", NULL, NULL);
	if (!window) {
		fprintf(stderr, "Failed to create window\n");
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); /* VSync */

	/* Setup OpenGL */
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

	/* Perspective projection */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, 800.0 / 600.0, 1.0, 2000.0);

	printf("Window created. Listening for UDP packets...\n\n");

	/* Main loop */
	while (!glfwWindowShouldClose(window)) {
		poll_udp();
		render_stewart();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	glfwDestroyWindow(window);
	glfwTerminate();
	close(udp_sock);

	return 0;
}