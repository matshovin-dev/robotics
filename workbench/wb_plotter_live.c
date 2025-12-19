/*
 * wb_plotter_live.c - Enkel y(t) graf-plotter med live vindu
 *
 * Bygg: make wb_plotter_live
 * Kjør:  ./wb_plotter_live [start_beat] [end_beat] [music.wav] [choreo.json]
 *
 * Eksempel: ./wb_plotter_live 0 32 track.wav mysong.json
 *
 * Trykk ESC eller lukk vinduet for å avslutte.
 * Trykk W eller NOTE 12 for å lagre segment til koreografi-fil.
 */

#include "move_lib.h"
#include "fade_lib.h"
#include "stewart/pose.h"
#include "stewart/geometry.h"
#include "viz_sender.h"
#include "viz_ports.h"
#include "robotics/math/utils.h"
#include "input_plotter.h"
#include "cJSON.h"
#include <SDL.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>

void draw_grid(SDL_Renderer *renderer);

/*
 * Input
 * master_phase +/-
 * move_a_nr +/-
 * move_b_nr +/-
 * fad_start +/-
 * fad_end +/-
 * fad_type +/-
 * * blend
 * * spline 1 2 cardinal0 cardinal05 cardinal08 monotonic bspline*
 * win_start +/-
 * win_end +/-
 *
 * Titlebar
 * C: master_phase move_a_nr move_b_nr fad_start fad_end fad_type
 *
 * Fil:
 * master_phase move_a_nr move_b_nr fad_start fad_end fad_type
 * master_phase move_a_nr move_b_nr fad_start fad_end fad_type
 */

// Tidsintervall (kan overstyres med kommandolinje)
float t_start = 0.0f;
float t_end = 5.0f;
int start_beat = 0;
int end_beat = 20;
#define T_STEP (1.0f / 200.0f)

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
float master_phase = 314.0f * M_PI / 180.0f;
struct stewart_pose pose_graph_1;
struct stewart_pose pose_graph_2;
struct stewart_pose pose_graph_mix;
struct stewart_pose pose_a;
struct stewart_pose pose_b;
struct stewart_pose pose_mix;
struct move_playback pb;
const struct stewart_geometry *geom = &ROBOT_MX64;
struct move m;
int move_no_a = 4;
int move_no_b = 21;
float t_current = 1.0f;	 // sec
int t_is_running = 0;
int run_repeat = 0;  // Loop playback when reaching end
char str[128]; /* tittelbar */
int viz_sock = -1;

float t_mix_start = 0.0f;
float t_mix_end = 0.0f;
int bpm = 150;
int transition_start_beat = 4;	// Start-beat for transisjon
int transition_beats = 4;  // Lengde på transisjon i beats
float t_inc_manual = 0.01f;

/* Oppdater transisjon-tider fra beat-parametre */
static void update_transition_times(void)
{
	float beat_duration = 60.0f / bpm;
	t_mix_start = transition_start_beat * beat_duration;
	t_mix_end = t_mix_start + transition_beats * beat_duration;
}

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

/* Koreografi-fil */
const char *choreo_filename = "choreo.json";
int segment_count = 0;

/* Lagre segment til koreografi-fil */
static void save_segment(void)
{
	cJSON *root = NULL;
	cJSON *segments = NULL;

	/* Les eksisterende fil eller opprett ny */
	FILE *f = fopen(choreo_filename, "r");
	if (f) {
		fseek(f, 0, SEEK_END);
		long len = ftell(f);
		fseek(f, 0, SEEK_SET);
		char *data = malloc(len + 1);
		fread(data, 1, len, f);
		data[len] = '\0';
		fclose(f);
		root = cJSON_Parse(data);
		free(data);
	}

	if (!root) {
		/* Opprett ny struktur */
		root = cJSON_CreateObject();
		cJSON_AddNumberToObject(root, "bpm", bpm);
		cJSON_AddNumberToObject(root, "master_phase",
					master_phase * 180.0f / M_PI);
		segments = cJSON_CreateArray();
		cJSON_AddItemToObject(root, "segments", segments);
	} else {
		/* Oppdater bpm og phase */
		cJSON *bpm_item = cJSON_GetObjectItem(root, "bpm");
		if (bpm_item)
			bpm_item->valuedouble = bpm;
		cJSON *phase_item = cJSON_GetObjectItem(root, "master_phase");
		if (phase_item)
			phase_item->valuedouble = master_phase * 180.0f / M_PI;
		segments = cJSON_GetObjectItem(root, "segments");
	}

	/* Opprett nytt segment */
	cJSON *seg = cJSON_CreateObject();
	cJSON_AddNumberToObject(seg, "move_a", move_no_a);
	cJSON_AddNumberToObject(seg, "move_b", move_no_b);
	cJSON_AddStringToObject(seg, "trans_type", spline_active ? "S" : "F");
	cJSON_AddNumberToObject(seg, "trans_nr",
				spline_active ? current_spline_type :
						current_fade_index);
	cJSON_AddNumberToObject(seg, "trans_start", transition_start_beat);
	cJSON_AddNumberToObject(seg, "trans_len", transition_beats);

	cJSON_AddItemToArray(segments, seg);
	segment_count = cJSON_GetArraySize(segments);

	/* Skriv til fil */
	char *json_str = cJSON_Print(root);
	f = fopen(choreo_filename, "w");
	if (f) {
		fprintf(f, "%s\n", json_str);
		fclose(f);
		printf("Lagret segment %d til %s\n", segment_count,
		       choreo_filename);
	}
	free(json_str);
	cJSON_Delete(root);
}

/* Oppdater LCD med verdier */
static void update_lcd_values(void)
{
	char buf[8];

	/* Display 0: PHASE grov */
	snprintf(buf, sizeof(buf), "%5.0f", master_phase * 180.0f / M_PI);
	input_plotter_set_lcd(0, LCD_COLOR_GREEN, " GROV", buf);

	/* Display 1: PHASE fin */
	snprintf(buf, sizeof(buf), "%5.0f", master_phase * 180.0f / M_PI);
	input_plotter_set_lcd(1, LCD_COLOR_GREEN, "  FIN", buf);

	/* Display 2: BPM */
	snprintf(buf, sizeof(buf), "  %3d", bpm);
	input_plotter_set_lcd(2, LCD_COLOR_GREEN, "  BPM", buf);

	/* Display 3: MOVE A */
	snprintf(buf, sizeof(buf), "  %3d", move_no_a);
	input_plotter_set_lcd(3, LCD_COLOR_BLUE, "MOVE A", buf);

	/* Display 4: MOVE B */
	snprintf(buf, sizeof(buf), "  %3d", move_no_b);
	input_plotter_set_lcd(4, LCD_COLOR_BLUE, "MOVE B", buf);

	/* Display 5: START beat */
	snprintf(buf, sizeof(buf), "  %3d", transition_start_beat);
	input_plotter_set_lcd(5, LCD_COLOR_YELLOW, " START", buf);

	/* Display 6: LEN beats */
	snprintf(buf, sizeof(buf), "  %3d", transition_beats);
	input_plotter_set_lcd(6, LCD_COLOR_YELLOW, "  LEN", buf);

	/* Display 7: TYPE (Spline or Fade) */
	if (spline_active) {
		snprintf(buf, sizeof(buf), " S%d", current_spline_type);
		input_plotter_set_lcd(7, LCD_COLOR_MAGENTA, "SPLINE", buf);
	} else {
		snprintf(buf, sizeof(buf), " F%d", current_fade_index);
		input_plotter_set_lcd(7, LCD_COLOR_CYAN, " FADE", buf);
	}
}

/* Send move parameters til bar visualizer */
static void send_move_bars(int move_no)
{
	if (viz_sock < 0)
		return;

	float values[42];
	const struct move *m = &move_lib[move_no];
	int idx = 0;

	/* Pack move into 42 floats: 6 DOFs x 7 params */
	for (int dof = 0; dof < 6; dof++) {
		/* 3 harmonics: amp, phase for each */
		for (int h = 0; h < 3; h++) {
			values[idx++] = m->dof[dof].h[h].amplitude;
			values[idx++] = m->dof[dof].h[h].phase;
		}
		/* Bias: convert from -1..+1 to 0..1 */
		values[idx++] = (m->dof[dof].bias + 1.0f) * 0.5f;
	}

	viz_sender_send_move_bars(viz_sock, move_no, values, VIZ_PORT_MOVE_BARS);
}

/* Oppdater tittelbar med alle parametre */
static void update_title(SDL_Window *window)
{
	snprintf(str, sizeof(str), "Phase:%.0f BPM:%d A:%d B:%d %s%d @%d+%d",
		 master_phase * 180.0f / M_PI, bpm, move_no_a, move_no_b,
		 spline_active ? "S" : "F",
		 spline_active ? current_spline_type : current_fade_index,
		 transition_start_beat, transition_beats);
	SDL_SetWindowTitle(window, str);
	update_lcd_values();
}

/*
 * Audio for beep ved beat-fase
 */
#define AUDIO_FREQ 44100
#define AUDIO_SAMPLES 128 /* Lavere = mindre latency, men mer CPU */
#define BEEP_DURATION_SEC 0.02f
#define AUDIO_BEEP_VOLUME 0.12f
#define BEEP_FREQ_NORMAL 1000.0f  // Hz - normal beat
#define BEEP_FREQ_TRANSITION 500.0f  // Hz - i transisjon

float audio_phase = 0.0f;
float beep_samples_remaining = 0;
float beep_freq = BEEP_FREQ_NORMAL;
float last_move_phase = 0.0f;

/* Musikk fra WAV-fil */
Uint8 *music_wav_buffer = NULL; /* Rå WAV-data */
float *music_samples = NULL; /* Konvertert til float */
Uint32 music_wav_length = 0;
int music_sample_count = 0;
int music_channels = 1;
int music_sample_rate = AUDIO_FREQ;
float music_volume = 0.3f;
int music_offset_samples = 0; /* Offset i WAV for start_beat */

/* Last WAV-fil og konverter til float samples */
int load_music(const char *filename)
{
	SDL_AudioSpec wav_spec;
	if (SDL_LoadWAV(filename, &wav_spec, &music_wav_buffer,
			&music_wav_length) == NULL) {
		printf("Kunne ikke laste %s: %s\n", filename, SDL_GetError());
		return -1;
	}

	music_sample_rate = wav_spec.freq;
	music_channels = wav_spec.channels;

	/* Beregn antall samples (avhenger av format) */
	int bytes_per_sample = 2; /* Anta 16-bit */
	if (wav_spec.format == AUDIO_S16LSB || wav_spec.format == AUDIO_S16MSB)
		bytes_per_sample = 2;
	else if (wav_spec.format == AUDIO_F32LSB ||
		 wav_spec.format == AUDIO_F32MSB)
		bytes_per_sample = 4;

	music_sample_count =
		music_wav_length / bytes_per_sample / music_channels;

	/* Konverter til mono float */
	music_samples = malloc(music_sample_count * sizeof(float));
	if (!music_samples) {
		SDL_FreeWAV(music_wav_buffer);
		return -1;
	}

	for (int i = 0; i < music_sample_count; i++) {
		float sample = 0.0f;
		if (wav_spec.format == AUDIO_S16LSB ||
		    wav_spec.format == AUDIO_S16MSB) {
			Sint16 *data = (Sint16 *)music_wav_buffer;
			/* Mix alle kanaler til mono */
			for (int ch = 0; ch < music_channels; ch++) {
				sample += data[i * music_channels + ch] /
					  32768.0f;
			}
			sample /= music_channels;
		} else if (wav_spec.format == AUDIO_F32LSB ||
			   wav_spec.format == AUDIO_F32MSB) {
			float *data = (float *)music_wav_buffer;
			for (int ch = 0; ch < music_channels; ch++) {
				sample += data[i * music_channels + ch];
			}
			sample /= music_channels;
		}
		music_samples[i] = sample;
	}

	printf("Lastet musikk: %s (%d samples, %d Hz, %d kanaler)\n", filename,
	       music_sample_count, music_sample_rate, music_channels);
	return 0;
}

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

/* Global for synkronisering av musikk med visning */
volatile int music_playback_pos = 0; /* Samples spilt, styres av callback */
volatile int music_sync_request = 0; /* Sett til 1 for å synke til audio_time */
volatile int music_playing = 0; /* 1 = musikk spiller, 0 = pauset */
volatile float audio_time = 0.0f; /* Mål-tid ved sync */

void audio_callback(void *userdata, Uint8 *stream, int len)
{
	float *buf = (float *)stream;
	int samples = len / sizeof(float);

	/* Synkroniser musikk-posisjon hvis forespurt (seek/reset) */
	if (music_sync_request) {
		music_playback_pos =
			(int)((audio_time - t_start) * music_sample_rate) +
			music_offset_samples;
		music_sync_request = 0;
	}

	for (int i = 0; i < samples; i++) {
		float beep_out = 0.0f;
		float music_out = 0.0f;

		/* Beep */
		if (beep_samples_remaining > 0) {
			beep_out = AUDIO_BEEP_VOLUME * sinf(audio_phase);
			audio_phase += 2.0f * M_PI * beep_freq / AUDIO_FREQ;
			if (audio_phase > 2.0f * M_PI)
				audio_phase -= 2.0f * M_PI;
			beep_samples_remaining--;
		}

		/* Musikk - kun når running OG playing */
		if (t_is_running && music_playing && music_samples &&
		    music_playback_pos >= 0 &&
		    music_playback_pos < music_sample_count) {
			music_out = music_samples[music_playback_pos] *
				    music_volume;
			music_playback_pos++;
		}

		buf[i] = beep_out + music_out;
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
	return (int)((t - t_start) / (t_end - t_start) * WIDTH);
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

	update_transition_times();

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
	else
		send_move_bars(move_no_a);  /* Send initial move */

	if (input_plotter_init() < 0) {
		printf("Advarsel: MIDI ikke tilgjengelig\n");
	} else {
		/* Initialize LCD displays with current values */
		update_lcd_values();
	}
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
	input_plotter_cleanup();
	if (audio_dev != 0)
		SDL_CloseAudioDevice(audio_dev);
	if (viz_sock >= 0)
		close(viz_sock);
	if (music_samples)
		free(music_samples);
	if (music_wav_buffer)
		SDL_FreeWAV(music_wav_buffer);
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
		t_current = t_start;
		audio_time = t_start;
		music_sync_request = 1;
		spline_initialized = 0;
		move_playback_reset(&pb);
		pb.master_phase = master_phase;
		last_move_phase =
			move_phase_1(&pb); /* Synk for riktig første beep */
		break;
	case SDLK_UP:
		move_no_b += (move_no_b < 98);
		move_mixer.deck_b = move_no_b;
		update_title(window);
		send_move_bars(move_no_b);
		break;
	case SDLK_DOWN:
		move_no_b -= (move_no_b > 0);
		move_mixer.deck_b = move_no_b;
		update_title(window);
		send_move_bars(move_no_b);
		break;
	case SDLK_LEFT:
		t_current -= t_inc_manual;
		if (t_current < t_start)
			t_current = t_start;
		audio_time = t_current;
		send_mixed_pose_at_time(t_current);
		check_beep_at_time(t_current);
		update_title(window);
		break;
	case SDLK_RIGHT:
		t_current += t_inc_manual;
		if (t_current > t_end)
			t_current = t_end;
		audio_time = t_current;
		send_mixed_pose_at_time(t_current);
		check_beep_at_time(t_current);
		update_title(window);
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
		update_title(window);
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
		update_title(window);
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
		update_title(window);
		break;
	case SDLK_o:
		/* O/o = Juster transisjon-lengde i beats */
		if (key.mod & KMOD_SHIFT)
			transition_beats += 1;
		else
			transition_beats -= (transition_beats > 1) ? 1 : 0;
		update_transition_times();
		spline_initialized = 0;
		update_title(window);
		break;
	case SDLK_i:
		/* I/i = Juster transisjon start-beat */
		if (key.mod & KMOD_SHIFT)
			transition_start_beat += 1;
		else
			transition_start_beat -=
				(transition_start_beat > 0) ? 1 : 0;
		update_transition_times();
		spline_initialized = 0;
		update_title(window);
		break;
	case SDLK_w:
		save_segment();
		break;
	}
}

static void handle_midi_event(struct plotter_event *ev, SDL_Window *window)
{
	if (ev->type == PLOTTER_ENCODER) {
		switch (ev->id) {
		case PLOTTER_ID_PHASE_COARSE:
			master_phase += ev->value * 0.2f;
			if (master_phase >= 2.0f * M_PI)
				master_phase -= 2.0f * M_PI;
			if (master_phase < 0.0f)
				master_phase += 2.0f * M_PI;
			pb.master_phase = master_phase;
			update_title(window);
			break;
		case PLOTTER_ID_PHASE_FINE:
			master_phase += ev->value * 0.05f;
			if (master_phase >= 2.0f * M_PI)
				master_phase -= 2.0f * M_PI;
			if (master_phase < 0.0f)
				master_phase += 2.0f * M_PI;
			pb.master_phase = master_phase;
			update_title(window);
			break;
		case PLOTTER_ID_BPM:
			bpm += (int)ev->value;
			if (bpm < 30)
				bpm = 30;
			if (bpm > 300)
				bpm = 300;
			move_playback_set_bpm(&pb, bpm);
			update_transition_times();
			update_title(window);
			break;
		case PLOTTER_ID_MOVE_A:
			move_no_a += (int)ev->value;
			if (move_no_a < 0)
				move_no_a = 0;
			if (move_no_a > 98)
				move_no_a = 98;
			move_mixer.deck_a = move_no_a;
			update_title(window);
			send_move_bars(move_no_a);
			break;
		case PLOTTER_ID_MOVE_B:
			move_no_b += (int)ev->value;
			if (move_no_b < 0)
				move_no_b = 0;
			if (move_no_b > 98)
				move_no_b = 98;
			move_mixer.deck_b = move_no_b;
			update_title(window);
			send_move_bars(move_no_b);
			break;
		case PLOTTER_ID_TRANS_START:
			transition_start_beat += (int)ev->value;
			if (transition_start_beat < 0)
				transition_start_beat = 0;
			update_transition_times();
			spline_initialized = 0;
			update_title(window);
			break;
		case PLOTTER_ID_TRANS_LEN:
			transition_beats += (int)ev->value;
			if (transition_beats < 1)
				transition_beats = 1;
			update_transition_times();
			spline_initialized = 0;
			update_title(window);
			break;
		case PLOTTER_ID_TRANS_TYPE:
			if (ev->value > 0) {
				if (spline_active) {
					current_spline_type =
						(current_spline_type + 1) %
						NUM_SPLINES;
					spline_initialized = 0;
				} else {
					current_fade_index =
						(current_fade_index + 1) %
						NUM_FADES;
					current_fade =
						fade_funcs[current_fade_index];
				}
			} else {
				if (spline_active) {
					current_spline_type =
						(current_spline_type - 1 +
						 NUM_SPLINES) %
						NUM_SPLINES;
					spline_initialized = 0;
				} else {
					current_fade_index =
						(current_fade_index - 1 +
						 NUM_FADES) %
						NUM_FADES;
					current_fade =
						fade_funcs[current_fade_index];
				}
			}
			update_title(window);
			break;
		}
	} else if (ev->type == PLOTTER_BUTTON && ev->value > 0) {
		switch (ev->id) {
		case PLOTTER_ID_RUN:
			t_is_running = 1;
			t_current = t_start;
			audio_time = t_start;
			music_sync_request = 1;
			spline_initialized = 0;
			move_playback_reset(&pb);
			pb.master_phase = master_phase;
			last_move_phase = move_phase_1(&pb);
			break;
		case PLOTTER_ID_SPLINE_MODE:
			spline_active = !spline_active;
			spline_initialized = 0;
			update_title(window);
			break;
		case PLOTTER_ID_TIME_LEFT:
			t_current -= t_inc_manual;
			if (t_current < t_start)
				t_current = t_start;
			audio_time = t_current;
			send_mixed_pose_at_time(t_current);
			check_beep_at_time(t_current);
			update_title(window);
			break;
		case PLOTTER_ID_TIME_RIGHT:
			t_current += t_inc_manual;
			if (t_current > t_end)
				t_current = t_end;
			audio_time = t_current;
			send_mixed_pose_at_time(t_current);
			check_beep_at_time(t_current);
			update_title(window);
			break;
		case PLOTTER_ID_SAVE:
			save_segment();
			break;
		case PLOTTER_ID_TIME_LEFT_FAST:
			t_current -= t_inc_manual * 4.0f;
			if (t_current < t_start)
				t_current = t_start;
			audio_time = t_current;
			send_mixed_pose_at_time(t_current);
			check_beep_at_time(t_current);
			update_title(window);
			break;
		case PLOTTER_ID_TIME_RIGHT_FAST:
			t_current += t_inc_manual * 4.0f;
			if (t_current > t_end)
				t_current = t_end;
			audio_time = t_current;
			send_mixed_pose_at_time(t_current);
			check_beep_at_time(t_current);
			update_title(window);
			break;
		case PLOTTER_ID_REPEAT:
			run_repeat = !run_repeat;
			printf("Repeat: %s\n", run_repeat ? "ON" : "OFF");
			break;
		case PLOTTER_ID_MUSIC_TOGGLE:
			music_playing = !music_playing;
			printf("Music: %s\n", music_playing ? "ON" : "OFF");
			break;
		}
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

	/* Poll MIDI */
	struct plotter_event midi_ev;
	while (input_plotter_poll(&midi_ev)) {
		handle_midi_event(&midi_ev, window);
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
			draw_graph(renderer, &graphs[i], i, t_start, t_end);
	}

	SDL_RenderPresent(renderer);
}

static void update_playback(float delta_time, SDL_Window *window)
{
	if (!t_is_running)
		return;

	send_mixed_pose_at_time(t_current);
	check_beep_at_time(t_current);
	audio_time = t_current; /* Synkroniser musikk med visning */

	update_title(window);
	t_current += delta_time;

	if (t_current > t_end) {
		if (run_repeat) {
			t_current = t_start;
			audio_time = t_start;
			music_sync_request = 1;
			spline_initialized = 0;
			move_playback_reset(&pb);
			pb.master_phase = master_phase;
			last_move_phase = move_phase_1(&pb);
		} else {
			t_is_running = 0;
		}
	}
}

int main(int argc, char *argv[])
{
	/* Kommandolinje-argumenter: start_beat end_beat [music.wav]
	 * [choreo.json] */
	if (argc >= 3) {
		start_beat = atoi(argv[1]);
		end_beat = atoi(argv[2]);
	}
	if (argc >= 5) {
		choreo_filename = argv[4];
	}

	init_move_system();

	/* Beregn tidsintervall fra beats */
	float beat_duration = 60.0f / bpm;
	t_start = start_beat * beat_duration;
	t_end = end_beat * beat_duration;
	t_current = t_start;

	/* Last musikk hvis spesifisert */
	if (argc >= 4) {
		if (load_music(argv[3]) == 0) {
			/* Beregn offset i WAV-filen for start_beat */
			music_offset_samples =
				(int)(t_start * music_sample_rate);
			printf("Musikk starter ved sample %d (beat %d)\n",
			       music_offset_samples, start_beat);
		}
	}

	printf("Kjører fra beat %d til %d (%.2f - %.2f sek)\n", start_beat,
	       end_beat, t_start, t_end);

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
	while (t < t_start)
		t += beat_duration;
	while (t > t_start + beat_duration)
		t -= beat_duration;
	while (t < t_end) {
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