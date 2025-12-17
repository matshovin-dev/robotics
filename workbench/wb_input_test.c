/**
 * @file wb_input_test.c
 * @brief Workbench: Test unified input system
 *
 * Mixer model:
 *   Deck A = move 0 (live output to robot)
 *   Deck B = preview move (select with 0-9)
 *   When happy with B, crossfade to B, copy B->move 0, fade back to A
 *
 * Uses the input library for keyboard control.
 * Run plot_stw_polygon or plot_stw_obj first, then this.
 *
 * Keyboard controls:
 *   q/a     Phase +/-
 *   w/s     BPM +/-
 *   e/d     Crossfade +/-
 *   r/f     Volume A +/- (live)
 *   t/g     Volume B +/- (preview)
 *   0-9     Load move to deck B
 *   Q/Esc   Quit
 */

#include "song_player.h"
#include "song_lib.h"
#include "input.h"
#include "move_lib.h"
#include "stewart/geometry.h"
#include "stewart/pose.h"
#include "viz_sender.h"
#include "viz_status.h"
#include "viz_ports.h"
#include "robotics/math/utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

/* Auto-fade state machine */
#define AUTOFADE_IDLE 0
#define AUTOFADE_WAITING 1
#define AUTOFADE_FADING 2

static int autofade_state = AUTOFADE_IDLE;
static float autofade_start_t; /* time when fading starts */
static float autofade_duration_sec; /* fade duration in seconds */
static float autofade_progress; /* 0.0 to 1.0 */

/* Target phase for auto-fade start: 3π/2 (minimum of sine) */
#define AUTOFADE_TARGET_PHASE (3.0f * M_PI / 2.0f)

/* Click track state */
static int click_enabled = 0;
static float click_last_phase = 0.0f;
static float click_cooldown = 0.0f; /* seconds until next click allowed */

/* Print current status */
static void print_status(void)
{
	printf("\r[A:LIVE v=%.2f] ---(%.2f)--- [B:%d %s v=%.2f]  "
	       "BPM:%.0f  Phase:%.2f   ",
	       move_mixer.volume_a, move_mixer.crossfader, move_mixer.deck_b,
	       move_lib[move_mixer.deck_b].name, move_mixer.volume_b,
	       move_playback.bpm, move_playback.master_phase);
	fflush(stdout);
}

/* Handle input event */
static int handle_event(struct input_event *ev)
{
	/* Quit signal */
	if (ev->id == -1)
		return 0;

	switch (ev->id) {
	case INPUT_ID_PHASE:
		move_playback.master_phase += ev->value;
		break;

	case INPUT_ID_BPM:
		move_playback.bpm =
			clampf(move_playback.bpm + ev->value, 30.0f, 300.0f);
		break;

	case INPUT_ID_CROSSFADE:
		if (ev->type == INPUT_FADER)
			move_mixer_set_crossfade(&move_mixer, ev->value);
		else
			move_mixer_set_crossfade(
				&move_mixer, move_mixer.crossfader + ev->value);
		break;

	case INPUT_ID_VOLUME_A:
		if (ev->type == INPUT_FADER)
			move_mixer.volume_a = ev->value;
		else
			move_mixer.volume_a = clampf(
				move_mixer.volume_a + ev->value, 0.0f, 1.0f);
		break;

	case INPUT_ID_VOLUME_B:
		if (ev->type == INPUT_FADER)
			move_mixer.volume_b = ev->value;
		else
			move_mixer.volume_b = clampf(
				move_mixer.volume_b + ev->value, 0.0f, 1.0f);
		break;

	/* Copy deck B -> move 0 (commit preview to live) */
	case INPUT_ID_COPY:
		move_copy(&move_lib[0], &move_lib[move_mixer.deck_b]);
		printf("\n  [COPY] move %d -> move 0\n", move_mixer.deck_b);
		break;

	/* Auto-fade: wait for phase, fade over 3 periods, copy, reset */
	case INPUT_ID_AUTOFADE:
		if (autofade_state == AUTOFADE_IDLE) {
			autofade_state = AUTOFADE_WAITING;
			/* 3 periods at current BPM: period = 60/bpm seconds */
			autofade_duration_sec =
				3.0f * (60.0f / move_playback.bpm);
			printf("\n  [AUTOFADE] Waiting for phase 3π/2... (%.1fs fade)\n",
			       autofade_duration_sec);
			fflush(stdout);
		}
		break;

	/* Play first song from library, sync with move */
	case INPUT_ID_PLAY_SONG: {
		struct song *s = song_lib_get(0);
		if (s) {
			/* Load and play song from start */
			song_player_load(s->wav_path);
			song_player_rewind();
			song_player_play();

			/* Sync move playback with phase offset */
			move_playback.bpm = s->bpm;
			move_playback.t = 0.0f;
			move_playback.master_phase = s->master_phase;

			printf("\n  [PLAY] %s @ %.0f BPM, phase=%.2f\n",
			       s->name, s->bpm, s->master_phase);
			fflush(stdout);
		} else {
			printf("\n  [PLAY] No songs in library\n");
			fflush(stdout);
		}
		break;
	}

	/* Save current bpm/phase to song library */
	case INPUT_ID_SAVE_SONG: {
		struct song *s = song_lib_get(0);
		if (s) {
			/* Update song with current values */
			s->bpm = move_playback.bpm;
			s->master_phase = move_playback.master_phase;

			/* Save to file */
			if (song_lib_save(
				    "/Users/matsmac/vsCode/robotics/assets/songs/song_lib.json") ==
			    0) {
				printf("\n  [SAVE] %s: BPM=%.0f, phase=%.2f\n",
				       s->name, s->bpm, s->master_phase);
			} else {
				printf("\n  [SAVE] Error saving library\n");
			}
			fflush(stdout);
		} else {
			printf("\n  [SAVE] No songs in library\n");
			fflush(stdout);
		}
		break;
	}

	/* Toggle click track */
	case INPUT_ID_CLICK:
		click_enabled = !click_enabled;
		song_player_click_enable(click_enabled);
		printf("\n  [CLICK] %s\n", click_enabled ? "ON" : "OFF");
		fflush(stdout);
		break;

	/* Move presets -> load to deck B (any move 0-99) */
	default:
		if (ev->id >= INPUT_ID_MOVE_0 &&
		    ev->id < INPUT_ID_MOVE_0 + MOVE_LIB_SIZE)
			move_mixer_set_deck_b(&move_mixer,
					      ev->id - INPUT_ID_MOVE_0);
		break;
	}

	print_status();
	return 1;
}

int main(void)
{
	int sock;
	struct timeval last, now;
	struct stewart_pose pose;
	const struct stewart_geometry *geom_64 = &ROBOT_MX64;
	/* const struct stewart_geometry *geom_18 = &ROBOT_AX18; */
	struct viz_status status;
	struct input_event ev;
	int running = 1;
	song_lib_load(
		"/Users/matsmac/vsCode/robotics/assets/songs/song_lib.json");

	/* Initialize song player */
	if (song_player_init() < 0) {
		fprintf(stderr, "Failed to initialize song player\n");
		return 1;
	}

	/* Initialize keyboard input */
	if (input_keyboard_init() < 0) {
		fprintf(stderr, "Failed to initialize keyboard input\n");
		return 1;
	}

	/* Initialize MIDI input (optional - continues if not found) */
	if (input_midi_init() < 0) {
		fprintf(stderr, "MIDI not available, using keyboard only\n");
	}

	/* Initialize move library */
	move_lib_init();
	move_lib_randomize_range(12, 90, 0.75);
	move_playback.bpm = 120.0f;

	/* Default mixer setup: A=idle(0), B=active move */
	move_mixer.deck_a = 0; /* always */
	move_mixer.deck_b = 4; /* ex. bounce */
	move_mixer.crossfader = 0.0f;
	move_mixer.volume_a = 1.0f;
	move_mixer.volume_b = 1.0f;

	/* Create UDP sender */
	sock = viz_sender_create();
	if (sock < 0) {
		fprintf(stderr, "Failed to create UDP sender\n");
		input_keyboard_cleanup();
		return 1;
	}

	/* Create status sender */
	if (viz_status_init(&status) < 0) {
		fprintf(stderr, "Failed to create status sender\n");
		input_keyboard_cleanup();
		return 1;
	}

	printf("Move Mixer - Keyboard Control\n");
	printf("=============================\n\n");
	input_keyboard_print_help();
	print_status();

	gettimeofday(&last, NULL);

	while (running) {
		/* Poll keyboard */
		while (input_keyboard_poll(&ev)) {
			if (!handle_event(&ev)) {
				running = 0;
				break;
			}
		}

		/* Poll MIDI */
		while (input_midi_poll(&ev)) {
			if (!handle_event(&ev)) {
				running = 0;
				break;
			}
		}

		/* Update time */
		gettimeofday(&now, NULL);
		float dt = (now.tv_sec - last.tv_sec) +
			   (now.tv_usec - last.tv_usec) / 1000000.0f;
		last = now;

		move_playback_tick(&move_playback, dt);

		/* Click track - trigger at 3π/2 phase */
		if (click_enabled) {
			click_cooldown -= dt;
			float phase = move_phase_1(&move_playback);
			float target = (float)AUTOFADE_TARGET_PHASE;

			/* Check if we crossed target phase */
			int crossed = (click_last_phase < target &&
				       phase >= target) ||
				      (click_last_phase > 5.0f && phase < 1.0f);
			click_last_phase = phase;

			if (crossed && click_cooldown <= 0.0f) {
				song_player_click_trigger();
				click_cooldown = 0.2f; /* 200ms cooldown */
			}
		}

		/* Auto-fade state machine */
		if (autofade_state == AUTOFADE_WAITING) {
			/* Get current phase from playback (already [0, 2π]) */
			float phase_norm = move_phase_1(&move_playback);

			/* Check if we crossed target phase (3π/2 ≈ 4.71) */
			float target = (float)AUTOFADE_TARGET_PHASE;
			static float last_phase = 0.0f;
			int crossed =
				(last_phase < target && phase_norm >= target) ||
				(last_phase > 5.0f && phase_norm < 1.0f &&
				 target > 4.0f);

			last_phase = phase_norm;

			if (crossed) {
				autofade_state = AUTOFADE_FADING;
				autofade_start_t = move_playback.t;
				autofade_progress = 0.0f;
				printf("\n  [AUTOFADE] Fading started!\n");
			}
		} else if (autofade_state == AUTOFADE_FADING) {
			/* Calculate progress based on time elapsed */
			float time_elapsed = move_playback.t - autofade_start_t;
			autofade_progress =
				time_elapsed / autofade_duration_sec;

			if (autofade_progress >= 1.0f) {
				/* Fade complete: copy and reset */
				autofade_progress = 1.0f;
				move_mixer.crossfader = 1.0f;

				move_copy(&move_lib[0],
					  &move_lib[move_mixer.deck_b]);
				printf("\n  [AUTOFADE] Complete! move %d -> move 0\n",
				       move_mixer.deck_b);

				/* Reset crossfader and state */
				move_mixer.crossfader = 0.0f;
				autofade_state = AUTOFADE_IDLE;
			} else {
				/* Linear fade */
				move_mixer.crossfader = autofade_progress;
			}
			print_status();
		}

		/* Mixed output (robot) -> port 9010 */
		move_evaluate_mixed(&move_mixer, &move_playback, geom_64,
				    &pose);
		pose.ty += geom_64->home_height;
		viz_sender_send_pose(sock, &pose, ROBOT_TYPE_MX64, 9010);

		/* Deck B preview (ren) -> port 9011 */
		move_evaluate(&move_lib[move_mixer.deck_b], &move_playback,
			      geom_64, &pose);
		/* Apply volume B to preview */
		pose.rx *= move_mixer.volume_b;
		pose.ry *= move_mixer.volume_b;
		pose.rz *= move_mixer.volume_b;
		pose.tx *= move_mixer.volume_b;
		pose.ty *= move_mixer.volume_b;
		pose.tz *= move_mixer.volume_b;
		pose.ty += geom_64->home_height;
		viz_sender_send_pose(sock, &pose, ROBOT_TYPE_MX64, 9011);

		/* Send status */
		viz_status_set(&status, "bpm", move_playback.bpm);
		viz_status_set(&status, "phase", move_playback.master_phase);
		viz_status_set(&status, "crossfader", move_mixer.crossfader);
		viz_status_set(&status, "deckB", move_mixer.deck_b);
		viz_status_set_str(&status, "moveB",
				   move_lib[move_mixer.deck_b].name);
		viz_status_set(&status, "volumeA", move_mixer.volume_a);
		viz_status_set(&status, "volumeB", move_mixer.volume_b);
		viz_status_send(&status);

		usleep(16000);
	}

	printf("\n\nGoodbye!\n");

	song_player_cleanup();
	input_keyboard_cleanup();
	input_midi_cleanup();
	viz_status_close(&status);

	return 0;
}
