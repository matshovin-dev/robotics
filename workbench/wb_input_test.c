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

#include "input.h"
#include "move_lib.h"
#include "stewart/geometry.h"
#include "stewart/pose.h"
#include "viz_sender.h"
#include "viz_status.h"
#include "viz_ports.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

/* Clamp float to range */
static float clampf(float v, float min, float max)
{
	if (v < min)
		return min;
	if (v > max)
		return max;
	return v;
}

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
			move_mixer_set_crossfade(&move_mixer,
						 move_mixer.crossfader + ev->value);
		break;

	case INPUT_ID_VOLUME_A:
		if (ev->type == INPUT_FADER)
			move_mixer.volume_a = ev->value;
		else
			move_mixer.volume_a =
				clampf(move_mixer.volume_a + ev->value, 0.0f, 1.0f);
		break;

	case INPUT_ID_VOLUME_B:
		if (ev->type == INPUT_FADER)
			move_mixer.volume_b = ev->value;
		else
			move_mixer.volume_b =
				clampf(move_mixer.volume_b + ev->value, 0.0f, 1.0f);
		break;

	/* Copy move 0 -> move 99 (backup) */
	case INPUT_ID_COPY:
		move_copy(&move_lib[99], &move_lib[0]);
		printf("\n  [COPY] move 0 -> move 99\n");
		break;

	/* Move presets 0-9 -> load to deck B */
	case INPUT_ID_MOVE_0:
	case INPUT_ID_MOVE_1:
	case INPUT_ID_MOVE_2:
	case INPUT_ID_MOVE_3:
	case INPUT_ID_MOVE_4:
	case INPUT_ID_MOVE_5:
	case INPUT_ID_MOVE_6:
	case INPUT_ID_MOVE_7:
	case INPUT_ID_MOVE_8:
	case INPUT_ID_MOVE_9:
		move_mixer_set_deck_b(&move_mixer, ev->id - INPUT_ID_MOVE_0);
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
	const struct stewart_geometry *geom_18 = &ROBOT_AX18;
	struct viz_status status;
	struct input_event ev;
	int running = 1;

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
	move_playback.bpm = 120.0f;

	/* Default mixer setup: A=idle(0), B=active move */
	move_mixer.deck_a = 0;  /* always idle */
	move_mixer.deck_b = 4;  /* bounce */
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

		/* Evaluate mixer */
		move_evaluate_mixed(&move_mixer, &move_playback, geom_64,
				    &pose);

		/* Add home height to ty */
		pose.ty += geom_64->home_height;

		/* Send pose */
		viz_sender_send_pose(sock, &pose, ROBOT_TYPE_MX64, 9011);

		/* Evaluate mixer */
		move_evaluate_mixed(&move_mixer, &move_playback, geom_18,
				    &pose);

		/* Add home height to ty */
		pose.ty += geom_18->home_height;

		/* Send pose */
		viz_sender_send_pose(sock, &pose, ROBOT_TYPE_AX18, 9010);

		/* Send status */
		viz_status_set(&status, "bpm", move_playback.bpm);
		viz_status_set(&status, "phase", move_playback.master_phase);
		viz_status_set(&status, "crossfader", move_mixer.crossfader);
		viz_status_set(&status, "deckB", move_mixer.deck_b);
		viz_status_set(&status, "volumeA", move_mixer.volume_a);
		viz_status_set(&status, "volumeB", move_mixer.volume_b);
		viz_status_send(&status);

		usleep(16000);
	}

	printf("\n\nGoodbye!\n");

	input_keyboard_cleanup();
	input_midi_cleanup();
	viz_status_close(&status);

	return 0;
}
