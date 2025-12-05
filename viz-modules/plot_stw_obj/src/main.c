#include "robotics/math/geometry.h"
#include "robotics/math/matrix.h"
#include "robotics/math/utils.h"
#include "robotics/math/vec3.h"
#include "stewart/geometry.h"
#include "stewart/kinematics.h"
#include "stewart/pose.h"
#include "viz_protocol.h"
#include "udp.h"
#include "obj_loader.h"
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

/* 3D Models */
static ObjModel *model_bunn = NULL;
static ObjModel *model_top = NULL;
static ObjModel *model_legL = NULL;
static ObjModel *model_legR = NULL;
static ObjModel *model_legLong = NULL;

/* Camera state */
static float camera_azimuth = 90.0f; /* Horizontal rotation (degrees) */
static float camera_elevation = 30.0f; /* Vertical tilt (degrees) */
static float ortho_scale = 200.0f; /* Orthographic view scale */
static float camera_center_y = 100.0f; /* Camera look-at Y position */

/**
 * get_robot_folder - Get folder name for robot type
 */
static const char *get_robot_folder(enum stewart_robot_type robot_type)
{
	if (robot_type == ROBOT_TYPE_MX64)
		return "mx64";
	else if (robot_type == ROBOT_TYPE_AX18)
		return "ax18";
	return "mx64"; /* default */
}

/**
 * load_models - Load OBJ models for current robot type
 * @robot_type: robot configuration to load models for
 *
 * Loads all 5 OBJ models (bunn, top, legL, legR, legLong) from the
 * appropriate folder in assets/3d_models/obj/
 *
 * Returns 0 on success, -1 on error.
 */
static int load_models(enum stewart_robot_type robot_type)
{
	const char *folder = get_robot_folder(robot_type);
	char path[512];

	/* Free existing models */
	if (model_bunn)
		obj_free(model_bunn);
	if (model_top)
		obj_free(model_top);
	if (model_legL)
		obj_free(model_legL);
	if (model_legR)
		obj_free(model_legR);
	if (model_legLong)
		obj_free(model_legLong);

	printf("Loading OBJ models for %s...\n", folder);

	/* Load bunn (base) */
	snprintf(path, sizeof(path),
		 "../../assets/3d_models/obj/%s/bunn.obj", folder);
	model_bunn = obj_load(path);
	if (!model_bunn) {
		fprintf(stderr, "Failed to load bunn.obj\n");
		return -1;
	}

	/* Load top (platform) */
	snprintf(path, sizeof(path), "../../assets/3d_models/obj/%s/top.obj",
		 folder);
	model_top = obj_load(path);
	if (!model_top) {
		fprintf(stderr, "Failed to load top.obj\n");
		return -1;
	}

	/* Load legL (left motor arm) */
	snprintf(path, sizeof(path), "../../assets/3d_models/obj/%s/legL.obj",
		 folder);
	model_legL = obj_load(path);
	if (!model_legL) {
		fprintf(stderr, "Failed to load legL.obj\n");
		return -1;
	}

	/* Load legR (right motor arm) */
	snprintf(path, sizeof(path), "../../assets/3d_models/obj/%s/legR.obj",
		 folder);
	model_legR = obj_load(path);
	if (!model_legR) {
		fprintf(stderr, "Failed to load legR.obj\n");
		return -1;
	}

	/* Load legLong (pushrods) */
	snprintf(path, sizeof(path),
		 "../../assets/3d_models/obj/%s/legLong.obj", folder);
	model_legLong = obj_load(path);
	if (!model_legLong) {
		fprintf(stderr, "Failed to load legLong.obj\n");
		return -1;
	}

	printf("All models loaded successfully.\n");
	return 0;
}

/**
 * setup_lighting - Configure OpenGL lighting
 */
static void setup_lighting(void)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);

	GLfloat light_pos[] = { 300.0f, 400.0f, 300.0f, 1.0f };
	GLfloat light_ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat light_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };

	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
}

/**
 * key_callback - Handle keyboard input
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
		camera_elevation -= 5.0f;
		if (camera_elevation > 89.0f)
			camera_elevation = 89.0f;
		break;
	case GLFW_KEY_DOWN:
		camera_elevation += 5.0f;
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
		/* Reset camera */
		camera_azimuth = 90.0f;
		camera_elevation = 30.0f;
		ortho_scale = 200.0f;
		camera_center_y = 100.0f;
		printf("Camera reset\n");
		break;
	case GLFW_KEY_ESCAPE:
		glfwSetWindowShouldClose(window, GLFW_TRUE);
		break;
	}
}

/**
 * draw_coordinate_axes - Draw RGB coordinate system at origin
 */
static void draw_coordinate_axes(void)
{
	glDisable(GL_LIGHTING);
	glLineWidth(3.0f);
	glBegin(GL_LINES);

	/* X-axis (red) */
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(100.0f, 0.0f, 0.0f);

	/* Y-axis (green) */
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 100.0f, 0.0f);

	/* Z-axis (blue) */
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 100.0f);

	glEnd();
	glLineWidth(1.0f);
	glEnable(GL_LIGHTING);
}

/**
 * render_stewart_obj - Render Stewart platform with OBJ models
 *
 * Renders:
 * - Base (bunn) at origin
 * - Platform (top) with pose transformation
 * - 6x motor arms (legL/legR) at base_points with motor angles
 * - 6x pushrods (legLong) from knee to platform
 * - Coordinate axes
 */
static void render_stewart_obj(void)
{
	int i;
	float azimuth_rad, elevation_rad;
	float eye_x, eye_y, eye_z;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* Update orthographic projection based on ortho_scale */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float aspect = 1024.0f / 768.0f;
	glOrtho(-ortho_scale * aspect, ortho_scale * aspect, -ortho_scale,
		ortho_scale, -2000.0, 2000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Calculate camera position from azimuth and elevation */
	float cam_distance = 800.0f;
	azimuth_rad = deg_to_rad(camera_azimuth);
	elevation_rad = deg_to_rad(camera_elevation);

	eye_x = cam_distance * cosf(elevation_rad) * cosf(azimuth_rad);
	eye_y = cam_distance * sinf(elevation_rad);
	eye_z = cam_distance * cosf(elevation_rad) * sinf(azimuth_rad);

	gluLookAt(eye_x, eye_y, eye_z, /* eye */
		  0.0, camera_center_y, 0.0, /* center */
		  0.0, 1.0, 0.0); /* up */

	/* Draw coordinate axes */
	draw_coordinate_axes();

	/* === RENDER BASE (BUNN) === */
	glPushMatrix();
	glColor3f(0.6f, 0.6f, 0.7f); /* Gray base */
	obj_draw(model_bunn);
	glPopMatrix();

	/* === RENDER PLATFORM (TOP) === */
	glPushMatrix();
	/* Apply pose transformation - pose is already absolute */
	glTranslatef(current_pose.tx,
		     current_pose.ty,
		     current_pose.tz);
	glRotatef(current_pose.rz, 0, 0, 1); /* Yaw (Z) */
	glRotatef(current_pose.ry, 0, 1, 0); /* Pitch (Y) */
	glRotatef(current_pose.rx, 1, 0, 0); /* Roll (X) */
	glColor3f(0.8f, 0.8f, 0.9f); /* Light platform */
	obj_draw(model_top);
	glPopMatrix();

	/* === RENDER MOTOR ARMS === */
	/* Pattern: legL at base_points[0], legR at base_points[1], repeated 3x with 120deg rotation */

	/* Motor 0 - legL */
	glPushMatrix();
	glTranslatef(geometry.base_points[0].x, geometry.base_points[0].y,
		     geometry.base_points[0].z);
	glRotatef(90.0f - 30.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(inverse_result.motor_angles_deg[0], 0.0f, 0.0f, 1.0f);
	glColor3f(0.9f, 0.6f, 0.6f);
	obj_draw(model_legL);
	glPopMatrix();

	/* Motor 1 - legR */
	glPushMatrix();
	glTranslatef(geometry.base_points[1].x, geometry.base_points[1].y,
		     geometry.base_points[1].z);
	glRotatef(60.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(inverse_result.motor_angles_deg[1], 0.0f, 0.0f, 1.0f);
	glColor3f(0.6f, 0.6f, 0.9f);
	obj_draw(model_legR);
	glPopMatrix();

	/* Motor 2 - legL */
	glPushMatrix();
	glRotatef(120.0f, 0.0f, 1.0f, 0.0f);
	glTranslatef(geometry.base_points[0].x, geometry.base_points[0].y,
		     geometry.base_points[0].z);
	glRotatef(90.0f - 30.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(inverse_result.motor_angles_deg[2], 0.0f, 0.0f, 1.0f);
	glColor3f(0.9f, 0.6f, 0.6f);
	obj_draw(model_legL);
	glPopMatrix();

	/* Motor 3 - legR */
	glPushMatrix();
	glRotatef(120.0f, 0.0f, 1.0f, 0.0f);
	glTranslatef(geometry.base_points[1].x, geometry.base_points[1].y,
		     geometry.base_points[1].z);
	glRotatef(60.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(inverse_result.motor_angles_deg[3], 0.0f, 0.0f, 1.0f);
	glColor3f(0.6f, 0.6f, 0.9f);
	obj_draw(model_legR);
	glPopMatrix();

	/* Motor 4 - legL */
	glPushMatrix();
	glRotatef(240.0f, 0.0f, 1.0f, 0.0f);
	glTranslatef(geometry.base_points[0].x, geometry.base_points[0].y,
		     geometry.base_points[0].z);
	glRotatef(90.0f - 30.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(inverse_result.motor_angles_deg[4], 0.0f, 0.0f, 1.0f);
	glColor3f(0.9f, 0.6f, 0.6f);
	obj_draw(model_legL);
	glPopMatrix();

	/* Motor 5 - legR */
	glPushMatrix();
	glRotatef(240.0f, 0.0f, 1.0f, 0.0f);
	glTranslatef(geometry.base_points[1].x, geometry.base_points[1].y,
		     geometry.base_points[1].z);
	glRotatef(60.0f, 0.0f, 1.0f, 0.0f);
	glRotatef(inverse_result.motor_angles_deg[5], 0.0f, 0.0f, 1.0f);
	glColor3f(0.6f, 0.6f, 0.9f);
	obj_draw(model_legR);
	glPopMatrix();

	/* === RENDER PUSHRODS (LEGLONG) === */
	glColor3f(0.9f, 0.8f, 0.7f); /* Beige pushrods */
	for (i = 0; i < 6; i++) {
		struct vec3 *knee = &inverse_result.knee_points[i];
		struct vec3 *platform =
			&inverse_result.platform_points_transformed[i];

		/* Calculate direction from knee to platform */
		struct vec3 direction;
		vec3_sub(platform, knee, &direction);
		float length = vec3_length(&direction);

		/* Calculate yaw and pitch angles */
		float yaw = atan2f(direction.x, direction.z) * 180.0f / M_PI;
		float pitch = asinf(direction.y / length) * 180.0f / M_PI -
			      90.0f;

		glPushMatrix();
		glTranslatef(knee->x, knee->y, knee->z);
		glRotatef(yaw, 0.0f, 1.0f, 0.0f);
		glRotatef(-pitch, 1.0f, 0.0f, 0.0f);
		obj_draw(model_legLong);
		glPopMatrix();
	}
}

/**
 * compute_kinematics - Calculate inverse kinematics for current pose
 */
static void compute_kinematics(void)
{
	struct stewart_pose pose;

	/* Convert UDP packet to stewart pose */
	pose.rx = current_pose.rx;
	pose.ry = current_pose.ry;
	pose.rz = current_pose.rz;
	pose.tx = current_pose.tx;
	pose.ty = current_pose.ty;
	pose.tz = current_pose.tz;

	/* Run inverse kinematics */
	stewart_kinematics_inverse(&geometry, &pose, &inverse_result, 0);

	/* Check for error */
	has_error = inverse_result.error;

	/* Print motor angles on change */
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
			printf("[%d]=%.1f ", i,
			       inverse_result.motor_angles_deg[i]);
		}
		if (has_error)
			printf(" ERROR: Pose unreachable!");
		printf("\n");
	}
}

/**
 * poll_udp - Poll UDP socket for new pose packets
 */
static void poll_udp(void)
{
	struct viz_pose_packet packet;
	int n;

	n = udp_receive(udp_sock, &packet, sizeof(packet));

	if (n == sizeof(packet)) {
		/* Validate packet */
		if (packet.magic == VIZ_MAGIC &&
		    packet.type == VIZ_PACKET_POSE) {
			/* Check if robot type changed before updating pose */
			int robot_changed = (packet.robot_type != current_pose.robot_type);

			current_pose = packet;

			/* Update geometry and models if robot type changes */
			if (robot_changed) {
				if (packet.robot_type == ROBOT_TYPE_MX64) {
					geometry = ROBOT_MX64;
					load_models(ROBOT_TYPE_MX64);
					printf("Switched to MX64 models\n");
				} else if (packet.robot_type ==
					   ROBOT_TYPE_AX18) {
					geometry = ROBOT_AX18;
					load_models(ROBOT_TYPE_AX18);
					printf("Switched to AX18 models\n");
				}
			}

			/* Calculate kinematics */
			compute_kinematics();
		}
	}
}

int main(void)
{
	GLFWwindow *window;

	printf("Stewart Platform OBJ Visualizer\n");
	printf("================================\n\n");

	/* Initialize geometry to MX64 (default) */
	geometry = ROBOT_MX64;

	/* Load initial models */
	if (load_models(ROBOT_TYPE_MX64) < 0) {
		fprintf(stderr, "Failed to load OBJ models\n");
		return 1;
	}

	/* Initialize current_pose to home */
	memset(&current_pose, 0, sizeof(current_pose));
	current_pose.magic = VIZ_MAGIC;
	current_pose.type = VIZ_PACKET_POSE;
	current_pose.robot_type = ROBOT_TYPE_MX64;

	/* Calculate initial kinematics for home pose */
	compute_kinematics();

	/* Create UDP receiver */
	udp_sock = udp_create_receiver(VIZ_PORT);
	if (udp_sock < 0) {
		fprintf(stderr, "Failed to create UDP receiver\n");
		return 1;
	}

	printf("Listening on UDP port %d...\n\n", VIZ_PORT);

	/* Initialize GLFW */
	if (!glfwInit()) {
		fprintf(stderr, "Failed to initialize GLFW\n");
		return 1;
	}

	/* Create window */
	window = glfwCreateWindow(1024, 768, "Stewart Platform OBJ Viewer",
				  NULL, NULL);
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
	setup_lighting();

	printf("Window created. Listening for UDP packets...\n");
	printf("Camera controls:\n");
	printf("  Arrow keys: Rotate camera\n");
	printf("  Q/W: Zoom in/out\n");
	printf("  A/S: Lower/raise focus point\n");
	printf("  R: Reset camera\n");
	printf("  ESC: Exit\n\n");

	/* Main loop */
	while (!glfwWindowShouldClose(window)) {
		poll_udp();
		render_stewart_obj();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	obj_free(model_bunn);
	obj_free(model_top);
	obj_free(model_legL);
	obj_free(model_legR);
	obj_free(model_legLong);

	glfwDestroyWindow(window);
	glfwTerminate();
	close(udp_sock);

	return 0;
}
