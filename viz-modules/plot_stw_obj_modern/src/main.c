/**
 * @file main.c
 * @brief Modern OpenGL Stewart Platform Visualizer with Shadow Mapping
 *
 * Uses OpenGL 3.3 Core Profile with:
 * - Shader-based rendering
 * - Shadow mapping with PCF soft shadows
 * - VBO/VAO mesh management
 */

#include <OpenGL/gl3.h>
#include <GLFW/glfw3.h>

#include "gl_math.h"
#include "shader.h"
#include "mesh.h"
#include "ssao.h"
#include "robotics/math/utils.h"
#include "stewart/geometry.h"
#include "stewart/kinematics.h"
#include "stewart/pose.h"
#include "viz_protocol.h"
#include "udp.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Window size */
#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

/* Shadow map resolution */
#define SHADOW_WIDTH 2048
#define SHADOW_HEIGHT 2048

/* Global state */
static struct viz_pose_packet current_pose;
static struct stewart_geometry geometry;
static struct stewart_inverse_result inverse_result;
static int udp_sock = -1;

/* Shaders */
static GLuint scene_shader;
static GLuint shadow_shader;

/* Shadow map */
static GLuint shadow_fbo;
static GLuint shadow_map;

/* Meshes */
static struct mesh *mesh_bunn = NULL;
static struct mesh *mesh_top = NULL;
static struct mesh *mesh_legL = NULL;
static struct mesh *mesh_legR = NULL;
static struct mesh *mesh_legLong = NULL;
static struct mesh *mesh_ground = NULL;

/* Camera */
static float camera_azimuth = 90.0f;
static float camera_elevation = 30.0f;
static float camera_distance = 700.0f;  /* Increased for narrower FOV */
static float camera_center_y = 40.0f;  /* Lower to see ground better */

/* Light positions */
static float light_pos[3] = { 200.0f, 350.0f, 200.0f };
static float fill_light_pos[3] = { -150.0f, 200.0f, -150.0f };  /* Opposite side */

/* SSAO */
static struct ssao_state ssao;

/**
 * setup_shadow_map - Create shadow map framebuffer
 */
static int setup_shadow_map(void)
{
	glGenFramebuffers(1, &shadow_fbo);

	/* Create depth texture */
	glGenTextures(1, &shadow_map);
	glBindTexture(GL_TEXTURE_2D, shadow_map);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH,
		     SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	float border[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

	/* Attach to framebuffer */
	glBindFramebuffer(GL_FRAMEBUFFER, shadow_fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
			       GL_TEXTURE_2D, shadow_map, 0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		fprintf(stderr, "Shadow framebuffer incomplete\n");
		return -1;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return 0;
}

/**
 * load_models - Load OBJ models
 */
static int load_models(enum stewart_robot_type robot_type)
{
	const char *folder = (robot_type == ROBOT_TYPE_MX64) ? "mx64" : "ax18";
	char path[512];

	/* Free existing */
	mesh_free(mesh_bunn);
	mesh_free(mesh_top);
	mesh_free(mesh_legL);
	mesh_free(mesh_legR);
	mesh_free(mesh_legLong);

	printf("Loading OBJ models for %s...\n", folder);

	snprintf(path, sizeof(path), "../../assets/3d_models/obj/%s/bunn.obj", folder);
	mesh_bunn = mesh_create_from_obj(path);

	snprintf(path, sizeof(path), "../../assets/3d_models/obj/%s/top.obj", folder);
	mesh_top = mesh_create_from_obj(path);

	snprintf(path, sizeof(path), "../../assets/3d_models/obj/%s/legL.obj", folder);
	mesh_legL = mesh_create_from_obj(path);

	snprintf(path, sizeof(path), "../../assets/3d_models/obj/%s/legR.obj", folder);
	mesh_legR = mesh_create_from_obj(path);

	snprintf(path, sizeof(path), "../../assets/3d_models/obj/%s/legLong.obj", folder);
	mesh_legLong = mesh_create_from_obj(path);

	if (!mesh_bunn || !mesh_top || !mesh_legL || !mesh_legR || !mesh_legLong) {
		fprintf(stderr, "Failed to load models\n");
		return -1;
	}

	return 0;
}

/**
 * compute_light_space_matrix - Calculate light's view-projection matrix
 */
static void compute_light_space_matrix(glm_mat4 result)
{
	glm_mat4 light_proj, light_view;
	glm_vec3 light = { light_pos[0], light_pos[1], light_pos[2] };
	glm_vec3 center = { 0.0f, 0.0f, 0.0f };  /* Look at origin to include base */
	glm_vec3 up = { 0.0f, 1.0f, 0.0f };

	/* Wider frustum to cover entire scene including base at y≈-60 */
	glm_mat4_ortho(light_proj, -400, 400, -400, 400, 10.0f, 1000.0f);
	glm_mat4_look_at(light_view, light, center, up);
	glm_mat4_multiply(result, light_proj, light_view);
}

/**
 * render_stewart - Render complete Stewart platform
 *
 * Matches original legacy OpenGL transformations exactly.
 */
static void render_stewart(GLuint shader)
{
	int i;
	glm_mat4 model, temp1, temp2;

	/* Base - identity transform (render exactly like other meshes) */
	glm_mat4_identity(model);
	shader_set_mat4(shader, "model", model);
	if (shader == scene_shader)
		shader_set_vec3(shader, "objectColor", 0.6f, 0.6f, 0.7f);
	mesh_draw(mesh_bunn);

	/* Platform - T * Rz * Ry * Rx */
	{
		glm_mat4 trans, rx, ry, rz;
		glm_mat4_translate(trans, current_pose.tx, current_pose.ty, current_pose.tz);
		glm_mat4_rotate_z(rz, current_pose.rz * M_PI / 180.0f);
		glm_mat4_rotate_y(ry, current_pose.ry * M_PI / 180.0f);
		glm_mat4_rotate_x(rx, current_pose.rx * M_PI / 180.0f);

		glm_mat4_multiply(temp1, ry, rx);
		glm_mat4_multiply(temp2, rz, temp1);
		glm_mat4_multiply(model, trans, temp2);

		shader_set_mat4(shader, "model", model);
		if (shader == scene_shader)
			shader_set_vec3(shader, "objectColor", 0.8f, 0.8f, 0.9f);
		mesh_draw(mesh_top);
	}

	/* Motor arms - exact same transforms as legacy code */
	float group_rot_deg[] = { 0.0f, 0.0f, 120.0f, 120.0f, 240.0f, 240.0f };
	int bp_indices[] = { 0, 1, 0, 1, 0, 1 };

	for (i = 0; i < 6; i++) {
		struct mesh *leg = (i % 2 == 0) ? mesh_legL : mesh_legR;
		int bp = bp_indices[i];
		float orient_deg = (i % 2 == 0) ? 60.0f : 60.0f;  /* legL: 60, legR: 60 */

		glm_mat4 group, trans, orient, motor;

		if (i < 2) {
			/* Motor 0-1: T * Ry(orient) * Rz(motor) */
			glm_mat4_translate(trans,
					   geometry.base_points[bp].x,
					   geometry.base_points[bp].y,
					   geometry.base_points[bp].z);
			glm_mat4_rotate_y(orient, orient_deg * M_PI / 180.0f);
			glm_mat4_rotate_z(motor, inverse_result.motor_angles_deg[i] * M_PI / 180.0f);

			glm_mat4_multiply(temp1, orient, motor);
			glm_mat4_multiply(model, trans, temp1);
		} else {
			/* Motor 2-5: Ry(group) * T * Ry(orient) * Rz(motor) */
			glm_mat4_rotate_y(group, group_rot_deg[i] * M_PI / 180.0f);
			glm_mat4_translate(trans,
					   geometry.base_points[bp].x,
					   geometry.base_points[bp].y,
					   geometry.base_points[bp].z);
			glm_mat4_rotate_y(orient, orient_deg * M_PI / 180.0f);
			glm_mat4_rotate_z(motor, inverse_result.motor_angles_deg[i] * M_PI / 180.0f);

			glm_mat4_multiply(temp1, orient, motor);
			glm_mat4_multiply(temp2, trans, temp1);
			glm_mat4_multiply(model, group, temp2);
		}

		shader_set_mat4(shader, "model", model);
		if (shader == scene_shader) {
			shader_set_vec3(shader, "objectColor",
					(i % 2 == 0) ? 0.9f : 0.6f,
					0.6f,
					(i % 2 == 0) ? 0.6f : 0.9f);
		}
		mesh_draw(leg);
	}

	/* Pushrods - T * Ry(yaw) * Rx(-pitch) */
	for (i = 0; i < 6; i++) {
		struct vec3 *knee = &inverse_result.knee_points[i];
		struct vec3 *plat = &inverse_result.platform_points_transformed[i];

		float dx = plat->x - knee->x;
		float dy = plat->y - knee->y;
		float dz = plat->z - knee->z;
		float len = sqrtf(dx * dx + dy * dy + dz * dz);

		float yaw = atan2f(dx, dz) * 180.0f / M_PI;
		float pitch = asinf(dy / len) * 180.0f / M_PI - 90.0f;

		glm_mat4 trans, rot_y, rot_x;
		glm_mat4_translate(trans, knee->x, knee->y, knee->z);
		glm_mat4_rotate_y(rot_y, yaw * M_PI / 180.0f);
		glm_mat4_rotate_x(rot_x, -pitch * M_PI / 180.0f);

		glm_mat4_multiply(temp1, rot_y, rot_x);
		glm_mat4_multiply(model, trans, temp1);

		shader_set_mat4(shader, "model", model);
		if (shader == scene_shader)
			shader_set_vec3(shader, "objectColor", 0.9f, 0.8f, 0.7f);
		mesh_draw(mesh_legLong);
	}
}

/**
 * render_gbuffer - Render scene to G-buffer for SSAO
 */
static void render_gbuffer(glm_mat4 projection, glm_mat4 view, int width, int height)
{
	glBindFramebuffer(GL_FRAMEBUFFER, ssao.gbuffer_fbo);
	glViewport(0, 0, width, height);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(ssao.gbuffer_shader);
	shader_set_mat4(ssao.gbuffer_shader, "projection", projection);
	shader_set_mat4(ssao.gbuffer_shader, "view", view);

	/* Render all geometry to G-buffer */
	render_stewart(ssao.gbuffer_shader);

	glm_mat4 ground_model;
	glm_mat4_translate(ground_model, 0.0f, -65.0f, 0.0f);
	shader_set_mat4(ssao.gbuffer_shader, "model", ground_model);
	mesh_draw(mesh_ground);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/**
 * render_ssao - Compute SSAO from G-buffer
 */
static void render_ssao(glm_mat4 projection, int width, int height)
{
	/* SSAO pass */
	glBindFramebuffer(GL_FRAMEBUFFER, ssao.ssao_fbo);
	glClear(GL_COLOR_BUFFER_BIT);

	glUseProgram(ssao.ssao_shader);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, ssao.gbuffer_position);
	shader_set_int(ssao.ssao_shader, "gPosition", 0);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, ssao.gbuffer_normal);
	shader_set_int(ssao.ssao_shader, "gNormal", 1);

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, ssao.noise_texture);
	shader_set_int(ssao.ssao_shader, "texNoise", 2);

	shader_set_mat4(ssao.ssao_shader, "projection", projection);
	shader_set_float(ssao.ssao_shader, "radius", ssao.radius);
	shader_set_float(ssao.ssao_shader, "bias", ssao.bias);
	shader_set_float(ssao.ssao_shader, "intensity", ssao.intensity);

	float noise_scale[2] = { (float)width / 4.0f, (float)height / 4.0f };
	GLint loc = glGetUniformLocation(ssao.ssao_shader, "noiseScale");
	glUniform2fv(loc, 1, noise_scale);

	ssao_render_quad(&ssao);

	/* Blur pass */
	glBindFramebuffer(GL_FRAMEBUFFER, ssao.ssao_blur_fbo);
	glClear(GL_COLOR_BUFFER_BIT);

	glUseProgram(ssao.blur_shader);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, ssao.ssao_texture);
	shader_set_int(ssao.blur_shader, "ssaoInput", 0);

	ssao_render_quad(&ssao);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/**
 * render_scene - Full render with shadow mapping and optional SSAO
 */
static void render_scene(int width, int height)
{
	glm_mat4 light_space, projection, view;
	float aspect = (float)width / (float)height;

	/* Compute matrices */
	compute_light_space_matrix(light_space);

	float az_rad = camera_azimuth * M_PI / 180.0f;
	float el_rad = camera_elevation * M_PI / 180.0f;
	glm_vec3 eye = {
		camera_distance * cosf(el_rad) * cosf(az_rad),
		camera_distance * sinf(el_rad) + camera_center_y,
		camera_distance * cosf(el_rad) * sinf(az_rad)
	};
	glm_vec3 center = { 0.0f, camera_center_y, 0.0f };
	glm_vec3 up = { 0.0f, 1.0f, 0.0f };

	glm_mat4_perspective(projection, 25.0f * M_PI / 180.0f, aspect, 1.0f, 2000.0f);
	glm_mat4_look_at(view, eye, center, up);

	/* === PASS 1: Shadow map === */
	glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
	glBindFramebuffer(GL_FRAMEBUFFER, shadow_fbo);
	glClear(GL_DEPTH_BUFFER_BIT);

	glUseProgram(shadow_shader);
	shader_set_mat4(shadow_shader, "lightSpaceMatrix", light_space);

	glCullFace(GL_FRONT);  /* Reduce shadow acne */
	render_stewart(shadow_shader);

	/* Ground for shadow pass */
	glm_mat4 ground_model;
	glm_mat4_translate(ground_model, 0.0f, -65.0f, 0.0f);
	shader_set_mat4(shadow_shader, "model", ground_model);
	mesh_draw(mesh_ground);

	glCullFace(GL_BACK);

	/* === PASS 2 & 3: SSAO (if enabled) === */
	if (ssao.enabled) {
		render_gbuffer(projection, view, width, height);
		render_ssao(projection, width, height);
	}

	/* === PASS 4: Final scene render === */
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, width, height);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(scene_shader);
	shader_set_mat4(scene_shader, "projection", projection);
	shader_set_mat4(scene_shader, "view", view);
	shader_set_mat4(scene_shader, "lightSpaceMatrix", light_space);
	shader_set_vec3(scene_shader, "lightPos", light_pos[0], light_pos[1], light_pos[2]);
	shader_set_vec3(scene_shader, "fillLightPos", fill_light_pos[0], fill_light_pos[1], fill_light_pos[2]);
	shader_set_float(scene_shader, "fillLightStrength", 0.3f);
	shader_set_vec3(scene_shader, "viewPos", eye[0], eye[1], eye[2]);
	shader_set_float(scene_shader, "ambientStrength", 0.35f);
	shader_set_float(scene_shader, "shadowSoftness", 1.5f);
	shader_set_float(scene_shader, "flipNormals", 1.0f);
	shader_set_float(scene_shader, "unlit", 0.0f);
	shader_set_float(scene_shader, "checkerboard", 0.0f);
	shader_set_float(scene_shader, "checkerSize", 50.0f);
	shader_set_float(scene_shader, "ssaoEnabled", ssao.enabled ? 1.0f : 0.0f);

	/* Bind shadow map */
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, shadow_map);
	shader_set_int(scene_shader, "shadowMap", 0);

	/* Bind SSAO map */
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, ssao.enabled ? ssao.ssao_blur_texture : 0);
	shader_set_int(scene_shader, "ssaoMap", 1);

	/* Render Stewart */
	render_stewart(scene_shader);

	/* Render ground with checkerboard pattern */
	glm_mat4_translate(ground_model, 0.0f, -65.0f, 0.0f);
	shader_set_mat4(scene_shader, "model", ground_model);
	shader_set_float(scene_shader, "checkerboard", 1.0f);
	mesh_draw(mesh_ground);
	shader_set_float(scene_shader, "checkerboard", 0.0f);
}

/**
 * compute_kinematics - Calculate inverse kinematics
 */
static void compute_kinematics(void)
{
	struct stewart_pose pose = {
		.rx = current_pose.rx,
		.ry = current_pose.ry,
		.rz = current_pose.rz,
		.tx = current_pose.tx,
		.ty = current_pose.ty,
		.tz = current_pose.tz
	};

	stewart_kinematics_inverse(&geometry, &pose, &inverse_result, 0);
}

/**
 * poll_udp - Check for UDP pose updates
 */
static void poll_udp(void)
{
	struct viz_pose_packet packet;
	int n = udp_receive(udp_sock, &packet, sizeof(packet));

	if (n == sizeof(packet) && packet.magic == VIZ_MAGIC &&
	    packet.type == VIZ_PACKET_POSE) {

		int robot_changed = (packet.robot_type != current_pose.robot_type);
		current_pose = packet;

		if (robot_changed) {
			if (packet.robot_type == ROBOT_TYPE_MX64) {
				geometry = ROBOT_MX64;
				load_models(ROBOT_TYPE_MX64);
			} else {
				geometry = ROBOT_AX18;
				load_models(ROBOT_TYPE_AX18);
			}
		}

		compute_kinematics();
	}
}

/**
 * key_callback - Handle keyboard input
 */
static void key_callback(GLFWwindow *window, int key, int scancode,
			 int action, int mods)
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
		if (camera_elevation < -10.0f)
			camera_elevation = -10.0f;
		break;
	case GLFW_KEY_Q:
		camera_distance *= 0.9f;
		if (camera_distance < 100.0f)
			camera_distance = 100.0f;
		break;
	case GLFW_KEY_W:
		camera_distance *= 1.1f;
		if (camera_distance > 1500.0f)
			camera_distance = 1500.0f;
		break;
	case GLFW_KEY_A:
		camera_center_y -= 10.0f;
		break;
	case GLFW_KEY_S:
		camera_center_y += 10.0f;
		break;
	case GLFW_KEY_R:
		camera_azimuth = 90.0f;
		camera_elevation = 30.0f;
		camera_distance = 700.0f;
		camera_center_y = 40.0f;
		break;
	case GLFW_KEY_O:
		ssao.enabled = !ssao.enabled;
		printf("SSAO: %s\n", ssao.enabled ? "ON" : "OFF");
		break;
	case GLFW_KEY_ESCAPE:
		glfwSetWindowShouldClose(window, GLFW_TRUE);
		break;
	}
}

int main(int argc, char *argv[])
{
	int port = VIZ_PORT_OBJ;
	if (argc > 1)
		port = atoi(argv[1]);

	printf("Stewart Platform Visualizer (Modern OpenGL + Shadows)\n");
	printf("======================================================\n\n");

	/* Initialize geometry */
	geometry = ROBOT_MX64;
	memset(&current_pose, 0, sizeof(current_pose));
	current_pose.magic = VIZ_MAGIC;
	current_pose.type = VIZ_PACKET_POSE;
	current_pose.robot_type = ROBOT_TYPE_MX64;
	current_pose.ty = geometry.home_height;

	compute_kinematics();

	/* UDP setup */
	udp_sock = udp_create_receiver(port);
	if (udp_sock < 0) {
		fprintf(stderr, "Failed to create UDP receiver\n");
		return 1;
	}
	printf("Listening on UDP port %d\n\n", port);

	/* GLFW init */
	if (!glfwInit()) {
		fprintf(stderr, "Failed to init GLFW\n");
		return 1;
	}

	/* Request OpenGL 3.3 Core */
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,
					      "Stewart Platform (Shadows)", NULL, NULL);
	if (!window) {
		fprintf(stderr, "Failed to create window\n");
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glfwSetKeyCallback(window, key_callback);

	printf("OpenGL: %s\n", glGetString(GL_VERSION));
	printf("Renderer: %s\n\n", glGetString(GL_RENDERER));

	/* Load shaders */
	scene_shader = shader_load("shaders/scene.vert", "shaders/scene.frag");
	shadow_shader = shader_load("shaders/shadow.vert", "shaders/shadow.frag");

	if (!scene_shader || !shadow_shader) {
		fprintf(stderr, "Failed to load shaders\n");
		return 1;
	}

	/* Setup shadow map */
	if (setup_shadow_map() < 0)
		return 1;

	/* Setup SSAO */
	if (ssao_init(&ssao, WINDOW_WIDTH, WINDOW_HEIGHT) < 0) {
		fprintf(stderr, "Warning: SSAO init failed, continuing without\n");
		ssao.enabled = 0;
	}

	/* Load models */
	if (load_models(ROBOT_TYPE_MX64) < 0)
		return 1;

	mesh_ground = mesh_create_ground(300.0f);

	/* OpenGL setup */
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glClearColor(0.08f, 0.08f, 0.12f, 1.0f);

	printf("Controls:\n");
	printf("  Arrows: Rotate camera\n");
	printf("  Q/W: Zoom in/out\n");
	printf("  A/S: Lower/raise focus\n");
	printf("  O: Toggle SSAO\n");
	printf("  R: Reset camera\n");
	printf("  ESC: Exit\n\n");

	/* Main loop */
	while (!glfwWindowShouldClose(window)) {
		poll_udp();

		int w, h;
		glfwGetFramebufferSize(window, &w, &h);
		render_scene(w, h);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	mesh_free(mesh_bunn);
	mesh_free(mesh_top);
	mesh_free(mesh_legL);
	mesh_free(mesh_legR);
	mesh_free(mesh_legLong);
	mesh_free(mesh_ground);

	ssao_cleanup(&ssao);

	glDeleteFramebuffers(1, &shadow_fbo);
	glDeleteTextures(1, &shadow_map);
	glDeleteProgram(scene_shader);
	glDeleteProgram(shadow_shader);

	glfwDestroyWindow(window);
	glfwTerminate();
	close(udp_sock);

	return 0;
}
