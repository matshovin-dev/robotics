/*
 * wb_plotter_live.c - Enkel y(t) graf-plotter med live vindu
 *
 * Bygg: make wb_plotter_live
 * Kjør:  ./wb_plotter_live
 *
 * Trykk ESC eller lukk vinduet for å avslutte.
 */

#include "move_lib.h"
#include "stewart/pose.h"
#include "stewart/geometry.h"
#include "viz_sender.h"
#include <SDL.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>

// ============ KONFIGURASJON ============

// Tidsintervall
#define T_START 0.0
#define T_END 3.0
#define T_STEP 1.0 / 200.0

// Vindu-størrelse
#define WIDTH 1200
#define HEIGHT 800

// Subplots
#define NO_OF_SUBPLOTS 6
#define SUBPLOT_Y_OFFSET 3.5
#define NO_OF_GRAPHS 7

/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

float f0 = 124.0f / 60.0f;
float ph = 2.0f * M_PI * (3.0f / 4.0f);
float T;
float master_phase = 0.0f;
float moving_phase = 0.0f;

struct stewart_pose pose_graph;
struct stewart_pose pose_a;
struct move_playback pb;
const struct stewart_geometry *geom = &ROBOT_MX64;
struct move m;
int move_no = 21;
double t_current = 1.0;	 // sec
char str[32]; /* div bruk */
int viz_sock = -1;

double y1(double t)
{
	return pose_graph.rx;
}

double y2(double t)
{
	return pose_graph.ry;
}

double y3(double t)
{
	return pose_graph.rz;
}

double y4(double t)
{
	return pose_graph.tx;
}

double y5(double t)
{
	return pose_graph.ty;
}

double y6(double t)
{
	return pose_graph.tz;
}

/* Overskriver - blå */

double y7(double t)
{
	return 0.0 * sin(f0 * 2.0 * M_PI * t + ph + master_phase);
}

/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

struct Graph {
	double (*func)(double);
	Uint8 r, g, b;
	const char *name;
};

int map_t_to_x(double t)
{
	return (int)((t - T_START) / (T_END - T_START) * WIDTH);
}

int map_y_to_screen(double y, int subplot_no)
{
	double y_scale = 0.06;
	// Y-range per subplot
	double y_min = -1.5;
	double y_max = 1.5;

	// Hver subplot tar like mye plass på skjermen
	int subplot_height = HEIGHT / NO_OF_SUBPLOTS;
	int subplot_top = subplot_no * subplot_height;

	// Map y fra [y_min, y_max] til subplot-området (invertert for skjerm)
	double normalized = (y_scale * y - y_min) / (y_max - y_min);
	int local_y = (int)((1.0 - normalized) * subplot_height);

	return subplot_top + local_y;
}

void draw_grid(SDL_Renderer *renderer)
{
	SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);  // Mørk grå grid

	// Vertikale linjer (t-aksen)
	float t = 0.0f;
	while (t < T_END) {
		int x = map_t_to_x(t);
		SDL_RenderDrawLine(renderer, x, 0, x, HEIGHT);
		t = t + T;
	}

	// Horisontal y=0 linje for hver subplot
	for (int i = 0; i < NO_OF_SUBPLOTS; i++) {
		int y0 = map_y_to_screen(0.0, i);
		SDL_RenderDrawLine(renderer, 0, y0, WIDTH, y0);
	}

	// Current time bar
	SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
	SDL_RenderDrawLine(renderer, map_t_to_x(t_current), 0,
			   map_t_to_x(t_current), HEIGHT);
}

void draw_graph(SDL_Renderer *renderer, struct Graph *graph, int graph_no)
{
	// En graf av gangen - graph_no (sub plot nr)
	// Denne kan også kalles flere gang med samme sub plotnr - overskrive
	SDL_SetRenderDrawColor(renderer, graph->r, graph->g, graph->b, 255);

	int prev_x = -1;
	int prev_y1 = -1, prev_y2 = -1, prev_y3 = -1, prev_y4 = -1,
	    prev_y5 = -1, prev_y6 = -1;

	move_playback_reset(&pb);
	for (double t = T_START; t <= T_END; t += T_STEP) {
		move_playback_tick(&pb, T_STEP);
		move_evaluate(&move_lib[move_no], &pb, geom, &pose_graph);
		int x = map_t_to_x(t);
		int y1 = map_y_to_screen(graph->func(t), graph_no);

		if (prev_x >= 0) {
			SDL_RenderDrawLine(renderer, prev_x, prev_y1, x, y1);
		}

		prev_x = x;
		prev_y1 = y1;
	}
}

int main(void)
{
	move_lib_init();
	move_lib_randomize_range(20, 30, 0.5);
	move_playback_set_bpm(&pb, 150);
	T = 1.0f / f0;

	// Opprett viz socket
	viz_sock = viz_sender_create();
	if (viz_sock < 0) {
		printf("Advarsel: Kunne ikke opprette viz socket\n");
	}

	// ============ SETT OPP GRAFENE HER ============
	struct Graph graphs[] = {
		{ y1, 244, 67, 54, "g1" },  // Rød
		{ y2, 244, 67, 54, "g2" },  // Rød
		{ y3, 244, 67, 54, "g3" },  // Rød
		{ y4, 244, 67, 54, "g4" },  // Rød
		{ y5, 244, 67, 54, "g5" },  // Rød
		{ y6, 244, 67, 54, "g6" },  // Rød
		{ y7, 33, 150, 243, "g6" }  // Blå
	};

	// Initialiser SDL
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		printf("SDL init feilet: %s\n", SDL_GetError());
		return 1;
	}

	SDL_Window *window = SDL_CreateWindow(
		"wb_plotter - y(t) Graf", SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);

	if (!window) {
		printf("Vindu-opprettelse feilet: %s\n", SDL_GetError());
		SDL_Quit();
		return 1;
	}

	SDL_Renderer *renderer =
		SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if (!renderer) {
		printf("Renderer-opprettelse feilet: %s\n", SDL_GetError());
		SDL_DestroyWindow(window);
		SDL_Quit();
		return 1;
	}

	// Hovedløkke
	bool running = true;
	SDL_Event event;

	while (running) {
		moving_phase = moving_phase + 0.001;
		// Håndter events
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
			case SDL_QUIT:
				running = false;
				break;
			case SDL_KEYDOWN:
				switch (event.key.keysym.sym) {
				case SDLK_ESCAPE:
					running = false;
					break;
				case SDLK_LEFT:
					t_current -= 0.02;
					if (t_current < T_START)
						t_current = T_START;
					pb.t = t_current;
					move_evaluate(&move_lib[move_no], &pb,
						      geom, &pose_a);
					pose_a.ty += geom->home_height;
					viz_sender_send_pose(viz_sock, &pose_a,
							     ROBOT_TYPE_MX64,
							     9002);
					snprintf(str, sizeof(str),
						 "Move %d : t=%.2f", move_no,
						 t_current);
					SDL_SetWindowTitle(window, str);
					break;
				case SDLK_RIGHT:
					t_current += 0.02;
					if (t_current > T_END)
						t_current = T_END;
					pb.t = t_current;
					move_evaluate(&move_lib[move_no], &pb,
						      geom, &pose_a);
					pose_a.ty += geom->home_height;
					viz_sender_send_pose(viz_sock, &pose_a,
							     ROBOT_TYPE_MX64,
							     9002);
					snprintf(str, sizeof(str),
						 "Move %d : t=%.2f", move_no,
						 t_current);
					SDL_SetWindowTitle(window, str);
					break;
				case SDLK_k:
					// ...
					break;
				}
				break;
			}
		}

		// Tegn
		SDL_SetRenderDrawColor(renderer, 0, 0, 0,
				       255);  // Sort bakgrunn
		SDL_RenderClear(renderer);

		draw_grid(renderer);

		for (int i = 0; i < NO_OF_SUBPLOTS; i++) {
			draw_graph(renderer, &graphs[i], i);
		}
		draw_graph(renderer, &graphs[6], 1);  // overskrive

		SDL_RenderPresent(renderer);
		SDL_Delay(16);	// ~60 FPS
	}

	// Rydd opp
	if (viz_sock >= 0)
		close(viz_sock);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}
