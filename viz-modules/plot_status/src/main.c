/**
 * @file main.c
 * @brief Status display - shows key-value pairs received via UDP
 *
 * Receives status data on UDP port 5556 and displays as text.
 * Uses Liberation Mono font for clean monospace display.
 */

#include "text_renderer.h"
#include "udp.h"
#include "viz_debug.h"
#include "viz_status.h"
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAX_STATUS_LINES 32
#define MAX_LINE_LEN 64

/* Global state */
static int udp_sock = -1;
static char status_lines[MAX_STATUS_LINES][MAX_LINE_LEN];
static int num_lines = 0;
static int window_width = 300;
static int window_height = 400;

/**
 * Parse received UDP data and update status_lines
 */
static void parse_status_data(const char *data, int len)
{
	num_lines = 0;

	const char *p = data;
	const char *end = data + len;

	while (p < end && num_lines < MAX_STATUS_LINES) {
		/* Find end of line */
		const char *eol = p;
		while (eol < end && *eol != '\n')
			eol++;

		int line_len = eol - p;
		if (line_len > 0 && line_len < MAX_LINE_LEN) {
			memcpy(status_lines[num_lines], p, line_len);
			status_lines[num_lines][line_len] = '\0';
			num_lines++;
		}

		p = eol + 1;
	}
}

/**
 * Poll UDP for new status data
 */
static void poll_udp(void)
{
	char buf[1024];
	int n = udp_receive(udp_sock, buf, sizeof(buf) - 1);

	if (n > 0) {
		buf[n] = '\0';
		parse_status_data(buf, n);
	}
}

/**
 * Render status display
 */
static void render(void)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, window_width, window_height, 0, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Draw title */
	text_draw("Status", 10, 10, 0.8f, 0.8f, 0.2f);

	/* Draw separator line */
	glColor3f(0.4f, 0.4f, 0.4f);
	glBegin(GL_LINES);
	glVertex2f(10, 35);
	glVertex2f(window_width - 10, 35);
	glEnd();

	/* Draw status lines */
	float y = 50;
	float line_height = 20;

	for (int i = 0; i < num_lines; i++) {
		/* Parse name:value format */
		char name[32] = "";
		float value = 0;

		if (sscanf(status_lines[i], "%31[^:]:%f", name, &value) == 2) {
			/* Draw name in white */
			char display[64];
			snprintf(display, sizeof(display), "%-12s %8.2f", name,
				 value);
			text_draw(display, 10, y, 0.9f, 0.9f, 0.9f);
		} else {
			/* Raw line */
			text_draw(status_lines[i], 10, y, 0.7f, 0.7f, 0.7f);
		}

		y += line_height;
	}

	/* Show "waiting..." if no data */
	if (num_lines == 0) {
		text_draw("Waiting for data...", 10, 50, 0.5f, 0.5f, 0.5f);
	}
}

/**
 * Window resize callback
 */
static void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
	(void)window;
	window_width = width;
	window_height = height;
	glViewport(0, 0, width, height);
}

/**
 * Key callback
 */
static void key_callback(GLFWwindow *window, int key, int scancode, int action,
			 int mods)
{
	(void)scancode;
	(void)mods;

	if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
		glfwSetWindowShouldClose(window, GLFW_TRUE);
	}
}

int main(void)
{
	GLFWwindow *window;

	viz_printf("Status Display\n");
	viz_printf("==============\n\n");

	/* Create UDP receiver */
	udp_sock = udp_create_receiver(VIZ_STATUS_PORT);
	if (udp_sock < 0) {
		fprintf(stderr, "Failed to create UDP receiver on port %d\n",
			VIZ_STATUS_PORT);
		return 1;
	}
	viz_printf("Listening on UDP port %d...\n\n", VIZ_STATUS_PORT);

	/* Initialize GLFW */
	if (!glfwInit()) {
		fprintf(stderr, "Failed to initialize GLFW\n");
		return 1;
	}

	/* Create window */
	window = glfwCreateWindow(window_width, window_height, "Status", NULL,
				  NULL);
	if (!window) {
		fprintf(stderr, "Failed to create window\n");
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetKeyCallback(window, key_callback);

	/* Initialize text renderer */
	const char *font_path =
		"../../libs/text_renderer/fonts/LiberationMono-Regular.ttf";
	if (!text_init(font_path, 16)) {
		fprintf(stderr, "Failed to initialize text renderer\n");
		fprintf(stderr, "Tried: %s\n", font_path);
		glfwDestroyWindow(window);
		glfwTerminate();
		return 1;
	}

	/* Setup OpenGL */
	glClearColor(0.1f, 0.1f, 0.12f, 1.0f);

	viz_printf("Press ESC to exit\n\n");

	/* Main loop */
	while (!glfwWindowShouldClose(window)) {
		poll_udp();
		render();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	text_cleanup();
	glfwDestroyWindow(window);
	glfwTerminate();
	close(udp_sock);

	return 0;
}
