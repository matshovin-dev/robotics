/*
 * wb_plotter_live.c - Enkel y(t) graf-plotter med live vindu
 *
 * Bygg: make wb_plotter_live
 * Kjør:  ./wb_plotter_live
 *
 * Trykk ESC eller lukk vinduet for å avslutte.
 */

#include "move_lib.h"
#include "fade_lib.h"
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
#define T_END 5.0f
#define T_STEP 1.0f / 200.0f

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

float t_mix_start = 0.0f;
float t_mix_end = 0.0f;
int bpm = 150;
float t_inc_manual = 0.01f;

struct move_spline spline;
int spline_active = 0;	// 1 = bruker spline, 0 = bruker fade
int spline_initialized =
	0;  // Har vi initialisert splinen for denne transisjonen?
int current_spline_type = 0;  // 0=C0, 1=C1, 2=C2

// Fade-funksjon (kan byttes med tastatur)
fade_func_t current_fade = fade_linear;
int current_fade_index = 0;
const char *fade_names[] = { "linear", "smoothstep", "ease_in",	 "ease_out",
			     "params", "dip_home",   "via_pose", "hold_rot" };
fade_func_t fade_funcs[] = { fade_linear,   fade_smoothstep, fade_ease_in,
			     fade_ease_out, fade_params,     fade_dip_home,
			     fade_via_pose, fade_hold_rot };
#define NUM_FADES 8

const char *spline_names[] = { "C0", "C1", "C2" };
#define NUM_SPLINES 3

// Audio
#define AUDIO_FREQ 44100
#define AUDIO_SAMPLES 512
int audio_playing = 0;
float audio_phase = 0.0f;

void audio_callback(void *userdata, Uint8 *stream, int len)
{
	float *buf = (float *)stream;
	int samples = len / sizeof(float);
	float freq = 200.0f;

	for (int i = 0; i < samples; i++) {
		if (audio_playing) {
			buf[i] = 0.3f * sinf(audio_phase);
			audio_phase += 2.0f * M_PI * freq / AUDIO_FREQ;
			if (audio_phase > 2.0f * M_PI)
				audio_phase -= 2.0f * M_PI;
		} else {
			buf[i] = 0.0f;
		}
	}
}

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

float get_crossfader(float t)
{
	if (t < t_mix_start)
		return 0.0f;
	if (t > t_mix_end)
		return 1.0f;
	return (t - t_mix_start) / (t_mix_end - t_mix_start);
}

void send_mixed_pose_at_time(float t)
{
	pb.t = t;

	if (spline_active) {
		// Initialiser spline ved transisjon-start
		if (!spline_initialized && t >= t_mix_start) {
			float duration = t_mix_end - t_mix_start;
			pb.t = t_mix_start;  // Sett playback til start
			switch (current_spline_type) {
			case 0:	 // C0
				move_spline_init_c0(&spline, &move_lib[move_no],
						    &move_lib[move_no_b], &pb,
						    geom, duration);
				break;
			case 1:	 // C1
				move_spline_init_c1(&spline, &move_lib[move_no],
						    &move_lib[move_no_b], &pb,
						    geom, duration);
				break;
			case 2:	 // C2
				move_spline_init_c2(&spline, &move_lib[move_no],
						    &move_lib[move_no_b], &pb,
						    geom, duration);
				break;
			}
			spline_initialized = 1;
			pb.t = t;  // Tilbake til nåværende tid
		}

		// Evaluer spline eller bruk move direkte
		if (t < t_mix_start) {
			move_evaluate(&move_lib[move_no], &pb, geom, &pose_mix);
		} else if (t > t_mix_end) {
			move_evaluate(&move_lib[move_no_b], &pb, geom,
				      &pose_mix);
		} else {
			move_spline_evaluate(&spline, &pb, &pose_mix);
		}
	} else {
		// Vanlig fade-funksjon
		float cf = get_crossfader(t);
		current_fade(&move_lib[move_no], &move_lib[move_no_b], cf, geom,
			     &pb, &pose_mix);
	}

	pose_mix.ty += geom->home_height;
	viz_sender_send_pose(viz_sock, &pose_mix, ROBOT_TYPE_MX64, 9002);
}

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

void draw_graph(SDL_Renderer *renderer, struct Graph *graph, int graph_no,
		float t_start, float t_end)
{
	// En graf av gangen - graph_no (sub plot nr)
	// Denne kan også kalles flere gang med samme sub plotnr - overskrive
	SDL_SetRenderDrawColor(renderer, graph->r, graph->g, graph->b, 255);

	int prev_x = -1;
	int prev_y = -1;

	// For spline-plotting: lag lokal spline for denne grafen
	struct move_spline graph_spline;
	int graph_spline_initialized = 0;

	move_playback_reset(&pb);
	pb.t = t_start;
	for (float t = t_start; t <= t_end; t += T_STEP) {
		move_playback_tick(&pb, T_STEP);
		move_evaluate(&move_lib[move_no], &pb, geom, &pose_graph_1);
		move_evaluate(&move_lib[move_no_b], &pb, geom, &pose_graph_2);

		// Mix pose - enten fade eller spline
		if (spline_active) {
			if (!graph_spline_initialized && t >= t_mix_start) {
				float duration = t_mix_end - t_mix_start;
				struct move_playback init_pb = pb;
				init_pb.t = t_mix_start;
				switch (current_spline_type) {
				case 0:
					move_spline_init_c0(
						&graph_spline,
						&move_lib[move_no],
						&move_lib[move_no_b], &init_pb,
						geom, duration);
					break;
				case 1:
					move_spline_init_c1(
						&graph_spline,
						&move_lib[move_no],
						&move_lib[move_no_b], &init_pb,
						geom, duration);
					break;
				case 2:
					move_spline_init_c2(
						&graph_spline,
						&move_lib[move_no],
						&move_lib[move_no_b], &init_pb,
						geom, duration);
					break;
				}
				graph_spline_initialized = 1;
			}

			if (t < t_mix_start) {
				move_evaluate(&move_lib[move_no], &pb, geom,
					      &pose_graph_mix);
			} else if (t > t_mix_end) {
				move_evaluate(&move_lib[move_no_b], &pb, geom,
					      &pose_graph_mix);
			} else {
				move_spline_evaluate(&graph_spline, &pb,
						     &pose_graph_mix);
			}
		} else {
			float cf = get_crossfader(t);
			current_fade(&move_lib[move_no], &move_lib[move_no_b],
				     cf, geom, &pb, &pose_graph_mix);
		}

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
	move_playback_set_bpm(&pb, bpm);
	T = 1.0f / f0;
	float beat_duration = 60.0f / bpm;  // sekunder per beat
	float bar_duration =
		4.0f * beat_duration;  // sekunder per takt (4 beats)
	t_mix_start = 1.0f * bar_duration;  // start ved takt 4
	t_mix_end = t_mix_start + bar_duration / 2.0f;	// varer én takt

	move_mixer.deck_a = move_no;
	move_mixer.deck_b = move_no_b;
	move_mixer.volume_a = 1.0f;
	move_mixer.volume_b = 1.0f;

	// Midtpose for fade_via_pose - hevet posisjon
	fade_mid_pose.rx = 0.0f;
	fade_mid_pose.ry = 0.0f;
	fade_mid_pose.rz = 0.0f;
	fade_mid_pose.tx = 0.0f;
	fade_mid_pose.ty = 18.0f;  // Hevet 15mm
	fade_mid_pose.tz = 0.0f;
	fade_mid_hold = 0.2f;

	// Opprett viz socket
	viz_sock = viz_sender_create();
	if (viz_sock < 0) {
		printf("Advarsel: Kunne ikke opprette viz socket\n");
	}

	// ============ SETT OPP GRAFENE HER ============
	struct Graph graphs[] = {
		{ g1_rx, 244, 67, 54, "g1_rx" },  // Rød A
		{ g1_ry, 244, 67, 54, "g1_ry" },  // Rød A
		{ g1_rz, 244, 67, 54, "g1_rz" },  // Rød A
		{ g1_tx, 244, 67, 54, "g1_tx" },  // Rød A
		{ g1_ty, 244, 67, 54, "g1_ty" },  // Rød A
		{ g1_tz, 244, 67, 54, "g1_tz" },  // Rød A
		{ g2_rx, 33, 150, 243, "g2_rx" },  // Blå B
		{ g2_ry, 33, 150, 243, "g2_ry" },  // Blå B
		{ g2_rz, 33, 150, 243, "g2_rz" },  // Blå B
		{ g2_tx, 33, 150, 243, "g2_tx" },  // Blå B
		{ g2_ty, 33, 150, 243, "g2_ty" },  // Blå B
		{ g2_tz, 33, 150, 243, "g2_tz" },  // Blå B
		{ mix_rx, 200, 200, 200, "mix_rx" },  // Hvit MIX
		{ mix_ry, 200, 200, 200, "mix_ry" },  // Hvit MIX
		{ mix_rz, 200, 200, 200, "mix_rz" },  // Hvit MIX
		{ mix_tx, 200, 200, 200, "mix_tx" },  // Hvit MIX
		{ mix_ty, 200, 200, 200, "mix_ty" },  // Hvit MIX
		{ mix_tz, 200, 200, 200, "mix_tz" }  // Hvit MIX
	};

	// Initialiser SDL
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
		printf("SDL init feilet: %s\n", SDL_GetError());
		return 1;
	}

	// Sett opp audio
	SDL_AudioSpec want, have;
	SDL_AudioDeviceID audio_dev;
	SDL_memset(&want, 0, sizeof(want));
	want.freq = AUDIO_FREQ;
	want.format = AUDIO_F32;
	want.channels = 1;
	want.samples = AUDIO_SAMPLES;
	want.callback = audio_callback;
	audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
	if (audio_dev == 0) {
		printf("Advarsel: Kunne ikke åpne audio: %s\n", SDL_GetError());
	} else {
		SDL_PauseAudioDevice(audio_dev, 0);  // Start audio
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
					spline_initialized = 0;	 // Reset spline
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
					t_current -= t_inc_manual;
					if (t_current < T_START)
						t_current = T_START;
					send_mixed_pose_at_time(t_current);
					snprintf(str, sizeof(str),
						 "Move %d/%d : t=%.2f xf=%.2f",
						 move_no, move_no_b, t_current,
						 move_mixer.crossfader);
					SDL_SetWindowTitle(window, str);
					break;
				case SDLK_RIGHT:
					t_current += t_inc_manual;
					if (t_current > T_END)
						t_current = T_END;
					send_mixed_pose_at_time(t_current);
					snprintf(str, sizeof(str),
						 "Move %d/%d : t=%.2f xf=%.2f",
						 move_no, move_no_b, t_current,
						 move_mixer.crossfader);
					SDL_SetWindowTitle(window, str);
					break;
				case SDLK_f:
					// Bytt fade-funksjon
					if (event.key.keysym.mod & KMOD_SHIFT)
						current_fade_index =
							(current_fade_index -
							 1 + NUM_FADES) %
							NUM_FADES;
					else
						current_fade_index =
							(current_fade_index +
							 1) %
							NUM_FADES;
					current_fade =
						fade_funcs[current_fade_index];
					spline_active =
						0;  // Bytt til fade-modus
					snprintf(
						str, sizeof(str), "Fade: %s",
						fade_names[current_fade_index]);
					SDL_SetWindowTitle(window, str);
					break;
				case SDLK_s:
					// Bytt spline-type
					spline_active = 1;
					if (event.key.keysym.mod & KMOD_SHIFT)
						current_spline_type =
							(current_spline_type -
							 1 + NUM_SPLINES) %
							NUM_SPLINES;
					else
						current_spline_type =
							(current_spline_type +
							 1) %
							NUM_SPLINES;
					spline_initialized =
						0;  // Krever ny init
					snprintf(str, sizeof(str), "Spline: %s",
						 spline_names
							 [current_spline_type]);
					SDL_SetWindowTitle(window, str);
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
			if (i > 11)
				draw_graph(renderer, &graphs[i], i, t_mix_start,
					   t_mix_end);
			else
				draw_graph(renderer, &graphs[i], i, T_START,
					   T_END);
		}

		SDL_RenderPresent(renderer);
		SDL_Delay(16);	// ~60 FPS

		if (t_is_running) {
			send_mixed_pose_at_time(t_current);
			audio_playing = (t_current >= t_mix_start &&
					 t_current <= t_mix_end);
			snprintf(str, sizeof(str),
				 "Move %d/%d : t=%.2f xf=%.2f", move_no,
				 move_no_b, t_current, move_mixer.crossfader);
			SDL_SetWindowTitle(window, str);
			t_current += delta_time;
		} else {
			audio_playing = 0;
		}
		if (t_current > T_END)
			t_is_running = 0;
	}

	// Rydd opp
	if (audio_dev != 0)
		SDL_CloseAudioDevice(audio_dev);
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

	// Vertikale linjer for hver beat
	float beat_duration = 60.0f / bpm;
	float t = 0.0f;
	while (t < T_END) {
		int x = map_t_to_x(t);
		SDL_RenderDrawLine(renderer, x, 0, x, HEIGHT);
		t = t + beat_duration;
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