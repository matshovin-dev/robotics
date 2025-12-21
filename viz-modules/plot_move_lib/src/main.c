/**
 * @file main.c
 * @brief Static move library visualizer
 *
 * Displays all 100 moves (5 columns x 20 rows) with bar plots.
 * Each move shows 42 parameters (6 DOFs x 7 params).
 * Bar width: 5px, max height: 20px.
 *
 * Usage: ./plot_move_lib [move_lib.json]
 */

#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"

/* Layout constants */
#define NUM_MOVES 100
#define COLS 5
#define ROWS 20
#define DOFS 6
#define PARAMS_PER_DOF 7
#define BARS_PER_MOVE (DOFS * PARAMS_PER_DOF)  /* 42 */

#define BAR_WIDTH 5
#define BAR_MAX_HEIGHT 20
#define DOF_GAP 3
#define MOVE_GAP_X 10
#define MOVE_GAP_Y 5
#define LABEL_HEIGHT 12

/* Move width: 42 bars * 5px + 6 DOF gaps */
#define MOVE_WIDTH (BARS_PER_MOVE * BAR_WIDTH + (DOFS - 1) * DOF_GAP)
/* Cell size including gaps */
#define CELL_WIDTH (MOVE_WIDTH + MOVE_GAP_X)
#define CELL_HEIGHT (BAR_MAX_HEIGHT + MOVE_GAP_Y + LABEL_HEIGHT)

/* Window dimensions */
#define WINDOW_WIDTH (COLS * CELL_WIDTH + MOVE_GAP_X)
#define WINDOW_HEIGHT (ROWS * CELL_HEIGHT + MOVE_GAP_Y)

/* Move data structure */
struct move_params {
	float values[BARS_PER_MOVE];  /* amp1,ph1,amp2,ph2,amp3,ph3,bias for each DOF */
	char name[32];
	int index;
};

static struct move_params moves[NUM_MOVES];
static int num_moves_loaded = 0;
static char current_filename[512] = "../assets/moves/move_lib.json";

/* Colors for parameters */
static const float param_colors[7][3] = {
	{ 0.95f, 0.35f, 0.35f },  /* amp1 - coral red */
	{ 0.4f, 0.65f, 0.95f },   /* phase1 - sky blue */
	{ 0.95f, 0.35f, 0.35f },  /* amp2 - coral red */
	{ 0.4f, 0.65f, 0.95f },   /* phase2 - sky blue */
	{ 0.95f, 0.35f, 0.35f },  /* amp3 - coral red */
	{ 0.4f, 0.65f, 0.95f },   /* phase3 - sky blue */
	{ 0.7f, 0.7f, 0.7f },     /* bias - gray */
};

/**
 * load_move_lib - Load moves from JSON file
 */
static int load_move_lib(const char *filename)
{
	FILE *f = fopen(filename, "rb");
	if (!f) {
		fprintf(stderr, "Could not open %s\n", filename);
		return -1;
	}

	fseek(f, 0, SEEK_END);
	long len = ftell(f);
	fseek(f, 0, SEEK_SET);

	char *data = malloc(len + 1);
	fread(data, 1, len, f);
	data[len] = '\0';
	fclose(f);

	cJSON *root = cJSON_Parse(data);
	free(data);

	if (!root) {
		fprintf(stderr, "JSON parse error\n");
		return -1;
	}

	cJSON *moves_arr = cJSON_GetObjectItem(root, "moves");
	if (!moves_arr) {
		cJSON_Delete(root);
		return -1;
	}

	const char *dof_names[] = { "rx", "ry", "rz", "tx", "ty", "tz" };

	int count = cJSON_GetArraySize(moves_arr);
	if (count > NUM_MOVES)
		count = NUM_MOVES;

	for (int i = 0; i < count; i++) {
		cJSON *move = cJSON_GetArrayItem(moves_arr, i);
		if (!move)
			continue;

		cJSON *idx = cJSON_GetObjectItem(move, "index");
		cJSON *name = cJSON_GetObjectItem(move, "name");
		cJSON *params = cJSON_GetObjectItem(move, "params");

		int move_idx = idx ? idx->valueint : i;
		if (move_idx >= NUM_MOVES)
			continue;

		moves[move_idx].index = move_idx;
		if (name && name->valuestring)
			strncpy(moves[move_idx].name, name->valuestring, 31);

		if (!params)
			continue;

		/* Extract parameters for each DOF */
		for (int d = 0; d < DOFS; d++) {
			cJSON *dof = cJSON_GetObjectItem(params, dof_names[d]);
			if (!dof)
				continue;

			cJSON *h = cJSON_GetObjectItem(dof, "h");
			cJSON *bias = cJSON_GetObjectItem(dof, "bias");

			int base = d * PARAMS_PER_DOF;

			/* 3 harmonics: amp, phase each */
			if (h && cJSON_GetArraySize(h) >= 3) {
				for (int harm = 0; harm < 3; harm++) {
					cJSON *harmonic = cJSON_GetArrayItem(h, harm);
					if (harmonic) {
						cJSON *amp = cJSON_GetObjectItem(harmonic, "amp");
						cJSON *phase = cJSON_GetObjectItem(harmonic, "phase");
						moves[move_idx].values[base + harm * 2] =
							amp ? (float)amp->valuedouble : 0.0f;
						moves[move_idx].values[base + harm * 2 + 1] =
							phase ? (float)phase->valuedouble : 0.0f;
					}
				}
			}

			/* Bias: konverter fra -1..+1 til 0..1 */
			float bias_val = bias ? (float)bias->valuedouble : 0.0f;
			moves[move_idx].values[base + 6] = (bias_val + 1.0f) * 0.5f;
		}

		if (move_idx >= num_moves_loaded)
			num_moves_loaded = move_idx + 1;
	}

	cJSON_Delete(root);
	printf("Loaded %d moves from %s\n", num_moves_loaded, filename);
	return 0;
}

/**
 * draw_move - Draw a single move's bars at given position
 */
static void draw_move(int move_idx, float base_x, float base_y)
{
	struct move_params *m = &moves[move_idx];

	/* Draw bars for each DOF */
	float x = base_x;
	for (int dof = 0; dof < DOFS; dof++) {
		for (int p = 0; p < PARAMS_PER_DOF; p++) {
			int idx = dof * PARAMS_PER_DOF + p;
			float value = m->values[idx];

			/* Clamp to 0-1 */
			if (value < 0.0f) value = 0.0f;
			if (value > 1.0f) value = 1.0f;

			float height = value * BAR_MAX_HEIGHT;

			/* Draw filled bar */
			glColor3f(param_colors[p][0], param_colors[p][1], param_colors[p][2]);
			glBegin(GL_QUADS);
			glVertex2f(x, base_y);
			glVertex2f(x + BAR_WIDTH - 1, base_y);
			glVertex2f(x + BAR_WIDTH - 1, base_y + height);
			glVertex2f(x, base_y + height);
			glEnd();

			x += BAR_WIDTH;
		}
		x += DOF_GAP;  /* Gap between DOFs */
	}

	/* Draw baseline */
	glColor3f(0.3f, 0.3f, 0.3f);
	glBegin(GL_LINES);
	glVertex2f(base_x, base_y);
	glVertex2f(base_x + MOVE_WIDTH, base_y);
	glEnd();
}

/**
 * render - Render all moves
 */
static void render(void)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* Draw all moves in grid */
	for (int row = 0; row < ROWS; row++) {
		for (int col = 0; col < COLS; col++) {
			int move_idx = col * ROWS + row;
			if (move_idx >= NUM_MOVES)
				continue;

			/* Calculate position (row 0 at top) */
			float base_x = MOVE_GAP_X / 2 + col * CELL_WIDTH;
			float base_y = WINDOW_HEIGHT - (row + 1) * CELL_HEIGHT + LABEL_HEIGHT;

			draw_move(move_idx, base_x, base_y);
		}
	}
}

/**
 * reload_moves - Reset and reload move library
 */
static void reload_moves(void)
{
	memset(moves, 0, sizeof(moves));
	for (int i = 0; i < NUM_MOVES; i++)
		moves[i].index = i;
	num_moves_loaded = 0;

	if (load_move_lib(current_filename) == 0)
		printf("Reloaded %s\n", current_filename);
}

/**
 * key_callback - Handle keyboard input
 */
static void key_callback(GLFWwindow *window, int key, int scancode,
			 int action, int mods)
{
	(void)scancode;
	(void)mods;

	if (action != GLFW_PRESS)
		return;

	switch (key) {
	case GLFW_KEY_ESCAPE:
		glfwSetWindowShouldClose(window, GLFW_TRUE);
		break;
	case GLFW_KEY_ENTER:
		reload_moves();
		break;
	}
}

int main(int argc, char *argv[])
{
	GLFWwindow *window;

	if (argc > 1)
		strncpy(current_filename, argv[1], sizeof(current_filename) - 1);

	printf("Move Library Visualizer\n");
	printf("=======================\n");
	printf("Layout: %d columns x %d rows = %d moves\n", COLS, ROWS, NUM_MOVES);
	printf("Window: %d x %d pixels\n", WINDOW_WIDTH, WINDOW_HEIGHT);
	printf("Bar: %dpx wide, %dpx max height\n\n", BAR_WIDTH, BAR_MAX_HEIGHT);

	/* Initialize moves to zero */
	memset(moves, 0, sizeof(moves));
	for (int i = 0; i < NUM_MOVES; i++)
		moves[i].index = i;

	/* Load move library */
	if (load_move_lib(current_filename) < 0) {
		fprintf(stderr, "Warning: Could not load %s\n", current_filename);
	}

	/* Initialize GLFW */
	if (!glfwInit()) {
		fprintf(stderr, "Failed to initialize GLFW\n");
		return 1;
	}

	/* Create window */
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,
				  "Move Library", NULL, NULL);
	if (!window) {
		fprintf(stderr, "Failed to create window\n");
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glfwSetKeyCallback(window, key_callback);

	/* Setup OpenGL */
	glClearColor(0.1f, 0.1f, 0.12f, 1.0f);

	printf("Press ENTER to reload, ESC to exit\n");

	/* Main loop */
	while (!glfwWindowShouldClose(window)) {
		render();
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	/* Cleanup */
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}
