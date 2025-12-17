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

float f0 = 124.0f / 60.0f;
float ph = 2.0f * M_PI * (3.0f / 4.0f);
float T;
float master_phase = 0.0f;

struct stewart_pose pose_graph_1;
struct stewart_pose pose_graph_2;
struct stewart_pose pose_graph_mix;
struct stewart_pose pose_a;
struct stewart_pose pose_b;
struct stewart_pose pose_mix;
struct move_playback pb;
const struct stewart_geometry *geom = &ROBOT_MX64;
struct move m;
const int move_no_a = 4;  // Hardkodet
int move_no_b = 21;
float t_current = 1.0f;	 // sec
int t_is_running = 0;
char str[32]; /* div bruk */
int viz_sock = -1;

float t_mix_start = 0.0f;
float t_mix_end = 0.0f;
int bpm = 150;
int transition_beats = 4;  // Lengde på transisjon i beats
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

const char *spline_names[] = {
	"C0 (linear)",	"C1 (Hermite)",
	"C2 (quintic)", "Cardinal 0.0", /* tension=0, like C1 */
	"Cardinal 0.5", /* tension=0.5, moderate */
	"Cardinal 0.8", /* tension=0.8, tight */
	"Monotonic", /* no overshoot */
	"B-spline" /* smooth approximation */
};
#define NUM_SPLINES 8

/*
 * Audio for beep ved beat-fase
 */
#define AUDIO_FREQ 44100
#define AUDIO_SAMPLES 128 /* Lavere = mindre latency, men mer CPU */
#define BEEP_DURATION_SEC 0.02f
#define AUDIO_BEEP_VOLUME 0.4f
#define BEEP_FREQ_NORMAL 1000.0f  // Hz - normal beat
#define BEEP_FREQ_TRANSITION 500.0f  // Hz - i transisjon

float audio_phase = 0.0f;
float beep_samples_remaining = 0;
float beep_freq = BEEP_FREQ_NORMAL;
float last_move_phase = 0.0f;

void trigger_beep(bool in_transition)
{
	beep_samples_remaining = BEEP_DURATION_SEC * AUDIO_FREQ;
	beep_freq = in_transition ? BEEP_FREQ_TRANSITION : BEEP_FREQ_NORMAL;
	audio_phase = 0.0f;
}

/* Sjekk fase-crossing og trigger beep ved behov */
void check_beep_at_time(float t)
{
	pb.t = t;
	float current_phase = move_phase_1(&pb);
	float target_phase = 3.0f * M_PI / 2.0f;

	if (last_move_phase < target_phase && current_phase >= target_phase) {
		bool in_transition = (t >= t_mix_start && t <= t_mix_end);
		trigger_beep(in_transition);
	}
	last_move_phase = current_phase;
}

void audio_callback(void *userdata, Uint8 *stream, int len)
{
	float *buf = (float *)stream;
	int samples = len / sizeof(float);

	for (int i = 0; i < samples; i++) {
		if (beep_samples_remaining > 0) {
			buf[i] = AUDIO_BEEP_VOLUME * sinf(audio_phase);
			audio_phase += 2.0f * M_PI * beep_freq / AUDIO_FREQ;
			if (audio_phase > 2.0f * M_PI)
				audio_phase -= 2.0f * M_PI;
			beep_samples_remaining--;
		} else {
			buf[i] = 0.0f;
		}
	}
}

/*
 * 6 grafer for plotting av RX RY RZ TX TY TZ
 * DECK A
 * RØDE
 */
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

/*
 * 6 grafer for plotting av RX RY RZ TX TY TZ
 * DECK B
 * BLÅ
 */

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

/*
 * 6 grafer for plotting av RX RY RZ TX TY TZ
 * MIX DECK A/B, samt splines
 * Disse plottes hvite kun i transisjons området
 */

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

struct Graph {
	float (*func)(float); /* Funksjonene over med farge og navn */
	Uint8 r, g, b;
	const char *name;
};

/*
 * Mixer
 * Ret: 0.0f - 1.0f
 */
float get_crossfader(float t)
{
	if (t < t_mix_start)
		return 0.0f;
	if (t > t_mix_end)
		return 1.0f;
	return (t - t_mix_start) / (t_mix_end - t_mix_start);
}

static void init_spline_by_type(struct move_spline *sp, int type,
				struct move *move_a, struct move *move_b,
				struct move_playback *playback,
				struct stewart_geometry *g, float duration)
{
	switch (type) {
	case 0:
		move_spline_init_c0(sp, move_a, move_b, playback, g, duration);
		break;
	case 1:
		move_spline_init_c1(sp, move_a, move_b, playback, g, duration);
		break;
	case 2:
		move_spline_init_c2(sp, move_a, move_b, playback, g, duration);
		break;
	case 3:
		move_spline_init_cardinal(sp, move_a, move_b, playback, g,
					  duration, 0.0f);
		break;
	case 4:
		move_spline_init_cardinal(sp, move_a, move_b, playback, g,
					  duration, 0.5f);
		break;
	case 5:
		move_spline_init_cardinal(sp, move_a, move_b, playback, g,
					  duration, 0.8f);
		break;
	case 6:
		move_spline_init_monotonic(sp, move_a, move_b, playback, g,
					   duration);
		break;
	case 7:
		move_spline_init_bspline(sp, move_a, move_b, playback, g,
					 duration);
		break;
	}
}

void send_mixed_pose_at_time(float t)
{
	pb.t = t;

	if (!spline_active) {
		float cf = get_crossfader(t);
		current_fade(&move_lib[move_no_a], &move_lib[move_no_b], cf,
			     geom, &pb, &pose_mix);
		goto send;
	}

	if (!spline_initialized && t >= t_mix_start) {
		float duration = t_mix_end - t_mix_start;
		pb.t = t_mix_start;
		init_spline_by_type(&spline, current_spline_type,
				    &move_lib[move_no_a], &move_lib[move_no_b],
				    &pb, geom, duration);
		spline_initialized = 1;
		pb.t = t;
	}

	if (t < t_mix_start)
		move_evaluate(&move_lib[move_no_a], &pb, geom, &pose_mix);
	else if (t > t_mix_end)
		move_evaluate(&move_lib[move_no_b], &pb, geom, &pose_mix);
	else
		move_spline_evaluate(&spline, &pb, &pose_mix);

send:
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
	SDL_SetRenderDrawColor(renderer, graph->r, graph->g, graph->b, 255);

	int prev_x = -1;
	int prev_y = -1;
	struct move_spline graph_spline;
	int graph_spline_initialized = 0;

	move_playback_reset(&pb);
	pb.t = t_start;

	for (float t = t_start; t <= t_end; t += T_STEP) {
		move_playback_tick(&pb, T_STEP);
		move_evaluate(&move_lib[move_no_a], &pb, geom, &pose_graph_1);
		move_evaluate(&move_lib[move_no_b], &pb, geom, &pose_graph_2);

		if (!spline_active) {
			float cf = get_crossfader(t);
			current_fade(&move_lib[move_no_a], &move_lib[move_no_b],
				     cf, geom, &pb, &pose_graph_mix);
			goto draw;
		}

		if (!graph_spline_initialized && t >= t_mix_start) {
			float duration = t_mix_end - t_mix_start;
			struct move_playback init_pb = pb;
			init_pb.t = t_mix_start;
			init_spline_by_type(&graph_spline, current_spline_type,
					    &move_lib[move_no_a],
					    &move_lib[move_no_b], &init_pb,
					    geom, duration);
			graph_spline_initialized = 1;
		}

		if (t < t_mix_start)
			move_evaluate(&move_lib[move_no_a], &pb, geom,
				      &pose_graph_mix);
		else if (t > t_mix_end)
			move_evaluate(&move_lib[move_no_b], &pb, geom,
				      &pose_graph_mix);
		else
			move_spline_evaluate(&graph_spline, &pb,
					     &pose_graph_mix);

	draw:
		int x = map_t_to_x(t);
		int y = map_y_to_screen(graph->func(t), graph_no % 6);

		if (prev_x >= 0)
			SDL_RenderDrawLine(renderer, prev_x, prev_y, x, y);

		prev_x = x;
		prev_y = y;
	}
}

static void init_move_system(void)
{
	move_lib_init();
	move_lib_randomize_range(20, 80, 0.5f);
	move_playback_set_bpm(&pb, bpm);
	pb.master_phase = master_phase;	 // Synk beat-fase
	T = 1.0f / f0;

	float beat_duration = 60.0f / bpm;
	float bar_duration = 4.0f * beat_duration;
	t_mix_start = 1.0f * bar_duration;
	t_mix_end = t_mix_start + transition_beats * beat_duration;

	move_mixer.deck_a = move_no_a;
	move_mixer.deck_b = move_no_b;
	move_mixer.volume_a = 1.0f;
	move_mixer.volume_b = 1.0f;

	fade_mid_pose.rx = 0.0f;
	fade_mid_pose.ry = 0.0f;
	fade_mid_pose.rz = 0.0f;
	fade_mid_pose.tx = 0.0f;
	fade_mid_pose.ty = 18.0f;
	fade_mid_pose.tz = 0.0f;
	fade_mid_hold = 0.2f;

	viz_sock = viz_sender_create();
	if (viz_sock < 0)
		printf("Advarsel: Kunne ikke opprette viz socket\n");
}

static int init_sdl(SDL_Window **window, SDL_Renderer **renderer,
		    SDL_AudioDeviceID *audio_dev)
{
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
		printf("SDL init feilet: %s\n", SDL_GetError());
		return -1;
	}

	SDL_AudioSpec want, have;
	SDL_memset(&want, 0, sizeof(want));
	want.freq = AUDIO_FREQ;
	want.format = AUDIO_F32;
	want.channels = 1;
	want.samples = AUDIO_SAMPLES;
	want.callback = audio_callback;
	*audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
	if (*audio_dev == 0)
		printf("Advarsel: Kunne ikke åpne audio: %s\n", SDL_GetError());
	else
		SDL_PauseAudioDevice(*audio_dev, 0);

	*window = SDL_CreateWindow(
		"C: wb_plotter - y(t) Graf", SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
	if (!*window) {
		printf("Vindu-opprettelse feilet: %s\n", SDL_GetError());
		SDL_Quit();
		return -1;
	}

	*renderer = SDL_CreateRenderer(*window, -1, SDL_RENDERER_ACCELERATED);
	if (!*renderer) {
		printf("Renderer-opprettelse feilet: %s\n", SDL_GetError());
		SDL_DestroyWindow(*window);
		SDL_Quit();
		return -1;
	}

	return 0;
}

static void cleanup(SDL_Window *window, SDL_Renderer *renderer,
		    SDL_AudioDeviceID audio_dev)
{
	if (audio_dev != 0)
		SDL_CloseAudioDevice(audio_dev);
	if (viz_sock >= 0)
		close(viz_sock);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}

static void handle_key_event(SDL_Keysym key, SDL_Window *window, bool *running)
{
	switch (key.sym) {
	case SDLK_ESCAPE:
		*running = false;
		break;
	case SDLK_r:
		t_is_running = 1;
		t_current = 0.0f;
		spline_initialized = 0;
		move_playback_reset(&pb);
		pb.master_phase = master_phase;
		last_move_phase =
			move_phase_1(&pb); /* Synk for riktig første beep */
		break;
	case SDLK_UP:
		move_no_b += (move_no_b < 98);
		move_mixer.deck_b = move_no_b;
		snprintf(str, sizeof(str), "C: Move %d/%d : t=%.2f", move_no_a,
			 move_no_b, t_current);
		SDL_SetWindowTitle(window, str);
		break;
	case SDLK_DOWN:
		move_no_b -= (move_no_b > 0);
		move_mixer.deck_b = move_no_b;
		snprintf(str, sizeof(str), "C: Move %d/%d : t=%.2f", move_no_a,
			 move_no_b, t_current);
		SDL_SetWindowTitle(window, str);
		break;
	case SDLK_LEFT:
		t_current -= t_inc_manual;
		if (t_current < T_START)
			t_current = T_START;
		send_mixed_pose_at_time(t_current);
		check_beep_at_time(t_current);
		snprintf(str, sizeof(str), "C: Move %d/%d : t=%.2f xf=%.2f",
			 move_no_a, move_no_b, t_current,
			 move_mixer.crossfader);
		SDL_SetWindowTitle(window, str);
		break;
	case SDLK_RIGHT:
		t_current += t_inc_manual;
		if (t_current > T_END)
			t_current = T_END;
		send_mixed_pose_at_time(t_current);
		check_beep_at_time(t_current);
		snprintf(str, sizeof(str), "C: Move %d/%d : t=%.2f xf=%.2f",
			 move_no_a, move_no_b, t_current,
			 move_mixer.crossfader);
		SDL_SetWindowTitle(window, str);
		break;
	case SDLK_f:
		if (key.mod & KMOD_SHIFT)
			current_fade_index =
				(current_fade_index - 1 + NUM_FADES) %
				NUM_FADES;
		else
			current_fade_index =
				(current_fade_index + 1) % NUM_FADES;
		current_fade = fade_funcs[current_fade_index];
		spline_active = 0;
		snprintf(str, sizeof(str), "C: Fade: %s",
			 fade_names[current_fade_index]);
		SDL_SetWindowTitle(window, str);
		break;
	case SDLK_s:
		spline_active = 1;
		if (key.mod & KMOD_SHIFT)
			current_spline_type =
				(current_spline_type - 1 + NUM_SPLINES) %
				NUM_SPLINES;
		else
			current_spline_type =
				(current_spline_type + 1) % NUM_SPLINES;
		spline_initialized = 0;
		snprintf(str, sizeof(str), "C: Spline: %s",
			 spline_names[current_spline_type]);
		SDL_SetWindowTitle(window, str);
		break;
	case SDLK_p:
		/* Juster master_phase i steg på 1/8 beat (π/4) */
		if (key.mod & KMOD_SHIFT)
			master_phase -= 0.1f;
		else
			master_phase += 0.1f;
		/* Wrap til [0, 2π) */
		if (master_phase >= 2.0f * M_PI)
			master_phase -= 2.0f * M_PI;
		if (master_phase < 0.0f)
			master_phase += 2.0f * M_PI;
		pb.master_phase = master_phase;
		snprintf(str, sizeof(str), "C: Phase: %.0f deg",
			 master_phase * 180.0f / M_PI);
		SDL_SetWindowTitle(window, str);
		break;
	}
}

static void handle_events(SDL_Window *window, bool *running)
{
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		switch (event.type) {
		case SDL_QUIT:
			*running = false;
			break;
		case SDL_KEYDOWN:
			handle_key_event(event.key.keysym, window, running);
			break;
		}
	}
}

static void render_frame(SDL_Renderer *renderer, struct Graph *graphs)
{
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderClear(renderer);
	draw_grid(renderer);

	for (int i = 0; i < NO_OF_SUBPLOTS * 3; i++) {
		if (i > 11)
			draw_graph(renderer, &graphs[i], i, t_mix_start,
				   t_mix_end);
		else
			draw_graph(renderer, &graphs[i], i, T_START, T_END);
	}

	SDL_RenderPresent(renderer);
}

static void update_playback(float delta_time, SDL_Window *window)
{
	if (!t_is_running)
		return;

	send_mixed_pose_at_time(t_current);
	check_beep_at_time(t_current);

	snprintf(str, sizeof(str), "C: Move %d/%d : t=%.2f xf=%.2f", move_no_a,
		 move_no_b, t_current, move_mixer.crossfader);
	SDL_SetWindowTitle(window, str);
	t_current += delta_time;

	if (t_current > T_END)
		t_is_running = 0;
}

int main(void)
{
	init_move_system();

	struct Graph graphs[] = { { g1_rx, 244, 67, 54, "g1_rx" },
				  { g1_ry, 244, 67, 54, "g1_ry" },
				  { g1_rz, 244, 67, 54, "g1_rz" },
				  { g1_tx, 244, 67, 54, "g1_tx" },
				  { g1_ty, 244, 67, 54, "g1_ty" },
				  { g1_tz, 244, 67, 54, "g1_tz" },
				  { g2_rx, 33, 150, 243, "g2_rx" },
				  { g2_ry, 33, 150, 243, "g2_ry" },
				  { g2_rz, 33, 150, 243, "g2_rz" },
				  { g2_tx, 33, 150, 243, "g2_tx" },
				  { g2_ty, 33, 150, 243, "g2_ty" },
				  { g2_tz, 33, 150, 243, "g2_tz" },
				  { mix_rx, 200, 200, 200, "mix_rx" },
				  { mix_ry, 200, 200, 200, "mix_ry" },
				  { mix_rz, 200, 200, 200, "mix_rz" },
				  { mix_tx, 200, 200, 200, "mix_tx" },
				  { mix_ty, 200, 200, 200, "mix_ty" },
				  { mix_tz, 200, 200, 200, "mix_tz" } };

	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_AudioDeviceID audio_dev;
	if (init_sdl(&window, &renderer, &audio_dev) < 0)
		return 1;

	bool running = true;
	Uint32 last_time = SDL_GetTicks();

	while (running) {
		Uint32 current_time = SDL_GetTicks();
		float delta_time = (current_time - last_time) / 1000.0f;
		last_time = current_time;

		handle_events(window, &running);
		render_frame(renderer, graphs);
		update_playback(delta_time, window);

		SDL_Delay(16);
	}

	cleanup(window, renderer, audio_dev);
	return 0;
}

/* ----------------------------------------------------- */

void draw_grid(SDL_Renderer *renderer)
{
	SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);  // Mørk grå grid

	// Vertikale linjer ved 3π/2 (270°) hvor beep trigger
	float beat_duration = 60.0f / bpm;
	// Konverter master_phase til tidsforskyvning, pluss 3/4 beat for 270°
	float phase_offset = (master_phase / (2.0f * M_PI)) * beat_duration;
	float beep_offset =
		(3.0f / 4.0f) * beat_duration;	// 3π/2 = 3/4 av beat
	float t = -phase_offset + beep_offset;
	// Start fra første synlige beat
	while (t < T_START)
		t += beat_duration;
	while (t > T_START + beat_duration)
		t -= beat_duration;
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