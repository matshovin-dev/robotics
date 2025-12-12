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
#include "robotics/math/utils.h"
#include <SDL.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>

void draw_grid(SDL_Renderer *renderer);

// ============ KONFIGURASJON ============

// Tidsintervall
#define T_START 0.0f
#define T_END 8.0f
#define T_STEP 1.0f / 200.0f
#define T_MIX_START 4.0f
#define T_MIX_END 5.0f

// Vindu-størrelse
#define WIDTH 1200
#define HEIGHT 800

// Subplots
#define NO_OF_SUBPLOTS 6
#define SUBPLOT_Y_OFFSET 3.5f
#define NO_OF_GRAPHS 7

/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

float f0 = 124.0f / 60.0f;
float ph = 2.0f * M_PI * (3.0f / 4.0f);
float T;
float master_phase = 0.0f;
float moving_phase = 0.0f;

struct stewart_pose pose_graph_1;
struct stewart_pose pose_graph_2;
struct stewart_pose pose_graph_mix;
struct stewart_pose pose_a;
struct stewart_pose pose_b;
struct stewart_pose pose_mix;
struct move_playback pb;
const struct stewart_geometry *geom = &ROBOT_MX64;
struct move m;
int move_no = 21;
int move_no_b = 21;
float t_current = 1.0f;	 // sec
int t_is_running = 0;
char str[32]; /* div bruk */
int viz_sock = -1;

float g1_rx(float t)
{
	return pose_graph_1.rx;
}

float g1_ry(float t)
{
	return pose_graph_1.ry;
}

float g1_rz(float t)
{
	return pose_graph_1.rz;
}

float g1_tx(float t)
{
	return pose_graph_1.tx;
}

float g1_ty(float t)
{
	return pose_graph_1.ty;
}

float g1_tz(float t)
{
	return pose_graph_1.tz;
}

/* neste move */

float g2_rx(float t)
{
	return pose_graph_2.rx;
}

float g2_ry(float t)
{
	return pose_graph_2.ry;
}

float g2_rz(float t)
{
	return pose_graph_2.rz;
}

float g2_tx(float t)
{
	return pose_graph_2.tx;
}

float g2_ty(float t)
{
	return pose_graph_2.ty;
}

float g2_tz(float t)
{
	return pose_graph_2.tz;
}

/* mixer ut */

float mix_rx(float t)
{
	return pose_graph_mix.rx;
}

float mix_ry(float t)
{
	return pose_graph_mix.ry;
}

float mix_rz(float t)
{
	return pose_graph_mix.rz;
}

float mix_tx(float t)
{
	return pose_graph_mix.tx;
}

float mix_ty(float t)
{
	return pose_graph_mix.ty;
}

float mix_tz(float t)
{
	return pose_graph_mix.tz;
}

/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

struct Graph {
	float (*func)(float);
	Uint8 r, g, b;
	const char *name;
};

int map_t_to_x(float t)
{
	return (int)((t - T_START) / (T_END - T_START) * WIDTH);
}

int map_y_to_screen(float y, int subplot_no)
{
	float y_scale = 0.06;
	// Y-range per subplot
	float y_min = -1.5;
	float y_max = 1.5;

	// Hver subplot tar like mye plass på skjermen
	int subplot_height = HEIGHT / NO_OF_SUBPLOTS;
	int subplot_top = subplot_no * subplot_height;

	// Map y fra [y_min, y_max] til subplot-området (invertert for skjerm)
	float normalized = (y_scale * y - y_min) / (y_max - y_min);
	int local_y = (int)((1.0f - normalized) * subplot_height);

	return subplot_top + local_y;
}

void draw_graph(SDL_Renderer *renderer, struct Graph *graph, int graph_no)
{
	// En graf av gangen - graph_no (sub plot nr)
	// Denne kan også kalles flere gang med samme sub plotnr - overskrive
	SDL_SetRenderDrawColor(renderer, graph->r, graph->g, graph->b, 255);

	int prev_x = -1;
	int prev_y = -1;

	move_playback_reset(&pb);
	for (float t = T_START; t <= T_END; t += T_STEP) {
		move_playback_tick(&pb, T_STEP);
		move_evaluate(&move_lib[move_no], &pb, geom, &pose_graph_1);
		move_evaluate(&move_lib[move_no_b], &pb, geom, &pose_graph_2);
		move_mixer.deck_a = move_no;
		move_mixer.deck_b = move_no_b;
		move_mixer.crossfader = clampf(t - T_MIX_START, 0.0, 1.0);
		move_evaluate_mixed(&move_mixer, &pb, geom, &pose_graph_mix);
		int x = map_t_to_x(t);
		int y = map_y_to_screen(graph->func(t), graph_no % 6);

		if (prev_x >= 0) {
			SDL_RenderDrawLine(renderer, prev_x, prev_y, x, y);
		}

		prev_x = x;
		prev_y = y;
	}
}

int main(void)
{
	move_lib_init();
	move_lib_randomize_range(20, 30, 0.5f);
	move_playback_set_bpm(&pb, 150);
	T = 1.0f / f0;

	move_mixer.deck_a = move_no;
	move_mixer.deck_b = move_no_b;
	move_mixer.volume_a = 1.0f;
	move_mixer.volume_b = 1.0f;

	// Opprett viz socket
	viz_sock = viz_sender_create();
	if (viz_sock < 0) {
		printf("Advarsel: Kunne ikke opprette viz socket\n");
	}

	// ============ SETT OPP GRAFENE HER ============
	struct Graph graphs[] = {
		{ g1_rx, 244, 67, 54, "g1_rx" },  // Rød
		{ g1_ry, 244, 67, 54, "g1_ry" },  // Rød
		{ g1_rz, 244, 67, 54, "g1_rz" },  // Rød
		{ g1_tx, 244, 67, 54, "g1_tx" },  // Rød
		{ g1_ty, 244, 67, 54, "g1_ty" },  // Rød
		{ g1_tz, 244, 67, 54, "g1_tz" },  // Rød
		{ g2_rx, 33, 150, 243, "g2_rx" },  // Blå
		{ g2_ry, 33, 150, 243, "g2_ry" },  // Blå
		{ g2_rz, 33, 150, 243, "g2_rz" },  // Blå
		{ g2_tx, 33, 150, 243, "g2_tx" },  // Blå
		{ g2_ty, 33, 150, 243, "g2_ty" },  // Blå
		{ g2_tz, 33, 150, 243, "g2_tz" },  // Blå
		{ mix_rx, 200, 200, 200, "mix_rx" },  // Hvit
		{ mix_ry, 200, 200, 200, "mix_ry" },  // Hvit
		{ mix_rz, 200, 200, 200, "mix_rz" },  // Hvit
		{ mix_tx, 200, 200, 200, "mix_tx" },  // Hvit
		{ mix_ty, 200, 200, 200, "mix_ty" },  // Hvit
		{ mix_tz, 200, 200, 200, "mix_tz" }   // Hvit
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
	Uint32 last_time = SDL_GetTicks();
	Uint32 current_time;
	float delta_time;

	while (running) {
		current_time = SDL_GetTicks();
		delta_time = (current_time - last_time) / 1000.0f;
		last_time = current_time;
		moving_phase = moving_phase + 0.001f;
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
				case SDLK_r:
					t_is_running = 1;
					t_current = 0.0f;
					break;
				case SDLK_UP:
					move_no_b += (move_no_b < 98);
					move_mixer.deck_b = move_no_b;
					snprintf(str, sizeof(str),
						 "Move %d/%d : t=%.2f", move_no,
						 move_no_b, t_current);
					SDL_SetWindowTitle(window, str);
					break;
				case SDLK_DOWN:
					move_no_b -= (move_no_b > 0);
					move_mixer.deck_b = move_no_b;
					snprintf(str, sizeof(str),
						 "Move %d/%d : t=%.2f", move_no,
						 move_no_b, t_current);
					SDL_SetWindowTitle(window, str);
					break;
				case SDLK_LEFT:
					t_current -= 0.04f;
					if (t_current < T_START)
						t_current = T_START;
					pb.t = t_current;
					move_mixer.deck_a = move_no;
					move_mixer.deck_b = move_no_b;
					move_mixer.crossfader =
						clampf(t_current - T_MIX_START,
						       0.0f, 1.0f);
					move_evaluate_mixed(&move_mixer, &pb,
							    geom, &pose_mix);
					pose_mix.ty += geom->home_height;
					viz_sender_send_pose(
						viz_sock, &pose_mix,
						ROBOT_TYPE_MX64, 9002);
					snprintf(str, sizeof(str),
						 "Move %d/%d : t=%.2f xf=%.2f",
						 move_no, move_no_b, t_current,
						 move_mixer.crossfader);
					SDL_SetWindowTitle(window, str);
					break;
				case SDLK_RIGHT:
					t_current += 0.04f;
					if (t_current > T_END)
						t_current = T_END;
					pb.t = t_current;
					move_mixer.deck_a = move_no;
					move_mixer.deck_b = move_no_b;
					move_mixer.crossfader =
						clampf(t_current - T_MIX_START,
						       0.0f, 1.0f);
					move_evaluate_mixed(&move_mixer, &pb,
							    geom, &pose_mix);
					pose_mix.ty += geom->home_height;
					viz_sender_send_pose(
						viz_sock, &pose_mix,
						ROBOT_TYPE_MX64, 9002);
					snprintf(str, sizeof(str),
						 "Move %d/%d : t=%.2f xf=%.2f",
						 move_no, move_no_b, t_current,
						 move_mixer.crossfader);
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

		for (int i = 0; i < NO_OF_SUBPLOTS * 3; i++) {
			draw_graph(renderer, &graphs[i], i);
		}

		SDL_RenderPresent(renderer);
		SDL_Delay(16);	// ~60 FPS

		if (t_is_running) {
			pb.t = t_current;
			move_mixer.deck_a = move_no;
			move_mixer.deck_b = move_no_b;
			move_mixer.crossfader =
				clampf(t_current - T_MIX_START, 0.0f, 1.0f);
			move_evaluate_mixed(&move_mixer, &pb, geom, &pose_mix);
			pose_mix.ty += geom->home_height;
			viz_sender_send_pose(viz_sock, &pose_mix,
					     ROBOT_TYPE_MX64, 9002);
			snprintf(str, sizeof(str),
				 "Move %d/%d : t=%.2f xf=%.2f", move_no,
				 move_no_b, t_current, move_mixer.crossfader);
			SDL_SetWindowTitle(window, str);
			t_current += delta_time;
		}
		if (t_current > T_END)
			t_is_running = 0;
	}

	// Rydd opp
	if (viz_sock >= 0)
		close(viz_sock);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}

/* ----------------------------------------------------- */

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
	// for (int i = 0; i < NO_OF_SUBPLOTS; i++) {
	// 	int y0 = map_y_to_screen(0.0, i);
	// 	SDL_RenderDrawLine(renderer, 0, y0, WIDTH, y0);
	// }

	// Current time bar
	SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
	SDL_RenderDrawLine(renderer, map_t_to_x(t_current), 0,
			   map_t_to_x(t_current), HEIGHT);
}