/**
 * @file wb_mixer_test.c
 * @brief Workbench: Test DJ-mixer crossfade between moves
 *
 * Demonstrates crossfading between two moves with keyboard control.
 * Run plot_stw_obj or plot_stw_polygon first, then this.
 *
 * Controls (non-blocking, type and press Enter):
 *   a <n>    - Set deck A to move n
 *   b <n>    - Set deck B to move n
 *   x <0-1>  - Set crossfader (0=A, 1=B)
 *   p <0-1>  - Set phase offset for B
 *   bpm <n>  - Set BPM
 *   swap     - Swap decks
 *   list     - List preset moves
 *   q        - Quit
 */

#include "move_lib.h"
#include "stewart/geometry.h"
#include "stewart/pose.h"
#include "viz_sender.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

static int has_input(void)
{
	struct timeval tv = { 0, 0 };
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

static void print_status(void)
{
	printf("\r[A:%d %s] ---(%.2f)--- [B:%d %s]  BPM:%.0f  phase:%.2f   ",
	       move_mixer.deck_a, move_lib[move_mixer.deck_a].name,
	       move_mixer.crossfader,
	       move_mixer.deck_b, move_lib[move_mixer.deck_b].name,
	       move_playback.bpm,
	       move_mixer.phase_offset_b);
	fflush(stdout);
}

static void list_presets(void)
{
	printf("\nPreset moves:\n");
	for (int i = 0; i < 10; i++) {
		if (move_lib[i].name[0])
			printf("  %d: %s\n", i, move_lib[i].name);
	}
	printf("\n");
}

int main(void)
{
	int sock;
	struct timeval last, now;
	struct move_pose pose;
	struct stewart_pose stw_pose;
	const struct stewart_geometry *geom = &ROBOT_MX64;
	char line[256];
	char cmd[32];
	float value;
	int running = 1;

	/* Initialize */
	move_lib_init();
	move_playback.bpm = 120.0f;
	move_playback.master_volume = 1.0f;

	/* Default mixer setup */
	move_mixer.deck_a = 4;  /* bounce */
	move_mixer.deck_b = 7;  /* complex */
	move_mixer.crossfader = 0.0f;
	move_mixer.volume_a = 1.0f;
	move_mixer.volume_b = 1.0f;

	/* Create UDP sender */
	sock = viz_sender_create();
	if (sock < 0) {
		fprintf(stderr, "Failed to create UDP sender\n");
		return 1;
	}

	printf("Move Mixer Test\n");
	printf("===============\n");
	printf("Commands: a <n>, b <n>, x <0-1>, p <0-1>, bpm <n>, swap, list, q\n\n");

	list_presets();
	print_status();

	gettimeofday(&last, NULL);

	while (running) {
		/* Check for input */
		if (has_input()) {
			if (fgets(line, sizeof(line), stdin)) {
				if (sscanf(line, "%s", cmd) == 1) {
					if (strcmp(cmd, "q") == 0 ||
					    strcmp(cmd, "quit") == 0) {
						running = 0;
					} else if (strcmp(cmd, "a") == 0) {
						int n;
						if (sscanf(line, "%*s %d", &n) == 1)
							move_mixer_set_deck_a(&move_mixer, n);
					} else if (strcmp(cmd, "b") == 0) {
						int n;
						if (sscanf(line, "%*s %d", &n) == 1)
							move_mixer_set_deck_b(&move_mixer, n);
					} else if (strcmp(cmd, "x") == 0) {
						if (sscanf(line, "%*s %f", &value) == 1)
							move_mixer_set_crossfade(&move_mixer, value);
					} else if (strcmp(cmd, "p") == 0) {
						if (sscanf(line, "%*s %f", &value) == 1)
							move_mixer_set_phase_offset(&move_mixer, value);
					} else if (strcmp(cmd, "bpm") == 0) {
						if (sscanf(line, "%*s %f", &value) == 1)
							move_playback.bpm = value;
					} else if (strcmp(cmd, "swap") == 0) {
						move_mixer_swap_decks(&move_mixer);
					} else if (strcmp(cmd, "list") == 0) {
						list_presets();
					}
				}
				print_status();
			}
		}

		/* Update time */
		gettimeofday(&now, NULL);
		float dt = (now.tv_sec - last.tv_sec) +
			   (now.tv_usec - last.tv_usec) / 1000000.0f;
		last = now;

		move_playback_tick(&move_playback, dt);

		/* Evaluate mixer */
		move_evaluate_mixer(&pose);

		/* Convert to stewart_pose */
		stw_pose.rx = pose.rx;
		stw_pose.ry = pose.ry;
		stw_pose.rz = pose.rz;
		stw_pose.tx = pose.tx;
		stw_pose.ty = geom->home_height + pose.ty;
		stw_pose.tz = pose.tz;

		/* Send */
		viz_sender_send_pose(sock, &stw_pose, ROBOT_TYPE_MX64, VIZ_PORT);

		usleep(16000);
	}

	printf("\nGoodbye!\n");
	return 0;
}
