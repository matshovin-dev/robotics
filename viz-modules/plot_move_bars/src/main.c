/**
 * @file main.c
 * @brief Move parameter bar plot visualizer
 *
 * Displays 42 move parameters as vertical bars (0.0-1.0).
 * Receives data via UDP on port 9010.
 *
 * Usage: ./plot_move_bars [port]
 */

#include "viz_protocol.h"
#include "udp.h"
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Window dimensions */
#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 400

/* Bar layout */
#define NUM_BARS VIZ_MOVE_BARS_COUNT
#define BAR_GROUPS 6        /* 6 DOFs */
#define BARS_PER_GROUP 7    /* 7 params per DOF */

/* Global state */
static struct viz_move_bars_packet current_data;
static int udp_sock = -1;
static GLFWwindow *g_window = NULL;

/* Colors for parameters within each DOF (7 params: amp1,ph1,amp2,ph2,amp3,ph3,bias) */
static const float param_colors[7][3] = {
	{ 1.0f, 0.4f, 0.4f },  /* amp1 - red */
	{ 0.3f, 0.5f, 1.0f },  /* phase1 - blue */
	{ 1.0f, 0.6f, 0.3f },  /* amp2 - orange */
	{ 0.3f, 0.5f, 1.0f },  /* phase2 - blue */
	{ 1.0f, 1.0f, 0.3f },  /* amp3 - yellow */
	{ 0.3f, 0.5f, 1.0f },  /* phase3 - blue */
	{ 0.7f, 0.7f, 0.7f },  /* bias - gray */
};

/* Parameter names for each position in group */
static const char *param_names[7] = {
	"amp", "frq", "phs", "ofs", "dcy", "atk", "wav"
};

/* DOF names */
static const char *dof_names[6] = {
	"RX", "RY", "RZ", "TX", "TY", "TZ"
};

/**
 * draw_bar - Draw a single bar
 */
static void draw_bar(float x, float width, float height, const float *color)
{
	glColor3f(color[0], color[1], color[2]);
	glBegin(GL_QUADS);
	glVertex2f(x, 0.0f);
	glVertex2f(x + width, 0.0f);
	glVertex2f(x + width, height);
	glVertex2f(x, height);
	glEnd();

	/* Border */
	glColor3f(0.2f, 0.2f, 0.2f);
	glLineWidth(1.0f);
	glBegin(GL_LINE_LOOP);
	glVertex2f(x, 0.0f);
	glVertex2f(x + width, 0.0f);
	glVertex2f(x + width, height);
	glVertex2f(x, height);
	glEnd();
}

/**
 * render - Render all bars
 */
static void render(void)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	/* Coordinate system: x=0..1200, y=0..1.2 (with margin for labels) */
	glOrtho(0, WINDOW_WIDTH, -0.15, 1.15, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Draw baseline */
	glColor3f(0.4f, 0.4f, 0.4f);
	glBegin(GL_LINES);
	glVertex2f(0, 0);
	glVertex2f(WINDOW_WIDTH, 0);
	/* Draw 0.5 line */
	glVertex2f(0, 0.5f);
	glVertex2f(WINDOW_WIDTH, 0.5f);
	/* Draw 1.0 line */
	glVertex2f(0, 1.0f);
	glVertex2f(WINDOW_WIDTH, 1.0f);
	glEnd();

	/* Calculate bar dimensions */
	float total_width = WINDOW_WIDTH - 40;  /* margins */
	float group_width = total_width / BAR_GROUPS;
	float bar_width = (group_width - 20) / BARS_PER_GROUP;  /* gap between groups */
	float start_x = 20;

	/* Draw bars grouped by DOF */
	for (int group = 0; group < BAR_GROUPS; group++) {
		float group_x = start_x + group * group_width;

		for (int param = 0; param < BARS_PER_GROUP; param++) {
			int idx = group * BARS_PER_GROUP + param;
			float value = current_data.values[idx];

			/* Clamp to 0-1 */
			if (value < 0.0f) value = 0.0f;
			if (value > 1.0f) value = 1.0f;

			float bar_x = group_x + param * bar_width;
			draw_bar(bar_x, bar_width - 2, value, param_colors[param]);
		}
	}
}

/**
 * poll_udp - Poll UDP socket for new packets
 */
static void poll_udp(void)
{
	struct viz_move_bars_packet packet;
	int n;

	n = udp_receive(udp_sock, &packet, sizeof(packet));

	if (n == sizeof(packet)) {
		if (packet.magic == VIZ_MAGIC &&
		    packet.type == VIZ_PACKET_MOVE_BARS) {
			current_data = packet;

			/* Update window title with move number */
			if (g_window) {
				char title[32];
				snprintf(title, sizeof(title), "B: %d", packet.move_no);
				glfwSetWindowTitle(g_window, title);
			}
		}
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

	if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
		glfwSetWindowShouldClose(window, GLFW_TRUE);
	}
}

int main(int argc, char *argv[])
{
	GLFWwindow *window;
	int port = VIZ_PORT_MOVE_BARS;

	/* Optional port argument */
	if (argc > 1)
		port = atoi(argv[1]);

	printf("Move Parameter Bar Visualizer\n");
	printf("==============================\n");
	printf("42 parameters: 6 DOFs x 7 params\n");
	printf("Groups: RX RY RZ TX TY TZ\n");
	printf("Params: amp frq phs ofs dcy atk wav\n\n");

	/* Initialize data to zero */
	memset(&current_data, 0, sizeof(current_data));
	current_data.magic = VIZ_MAGIC;
	current_data.type = VIZ_PACKET_MOVE_BARS;

	/* Create UDP receiver */
	udp_sock = udp_create_receiver(port);
	if (udp_sock < 0) {
		fprintf(stderr, "Failed to create UDP receiver on port %d\n", port);
		return 1;
	}
	printf("Listening on UDP port %d...\n\n", port);

	/* Initialize GLFW */
	if (!glfwInit()) {
		fprintf(stderr, "Failed to initialize GLFW\n");
		return 1;
	}

	/* Create window */
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "B: -", NULL, NULL);
	if (!window) {
		fprintf(stderr, "Failed to create window\n");
		glfwTerminate();
		return 1;
	}
	g_window = window;

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glfwSetKeyCallback(window, key_callback);

	/* Setup OpenGL */
	glClearColor(0.1f, 0.1f, 0.12f, 1.0f);

	printf("Press ESC to exit\n");

	/* Main loop */
	while (!glfwWindowShouldClose(window)) {
		poll_udp();
		render();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	glfwDestroyWindow(window);
	glfwTerminate();
	close(udp_sock);

	return 0;
}
