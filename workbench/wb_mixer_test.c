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
 *   va <0-1> - Set volume for deck A
 *   vb <0-1> - Set volume for deck B
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
	printf("\r[A:%d %s v=%.1f] ---(%.2f)--- [B:%d %s v=%.1f]  BPM:%.0f   ",
	       move_mixer.deck_a, move_lib[move_mixer.deck_a].name,
	       move_mixer.volume_a,
	       move_mixer.crossfader,
	       move_mixer.deck_b, move_lib[move_mixer.deck_b].name,
	       move_mixer.volume_b,
	       move_playback.bpm);
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
	struct stewart_pose pose;
	const struct stewart_geometry *geom = &ROBOT_MX64;
	char line[256];
	char cmd[32];
	float value;
	int running = 1;

	/* Initialize */
	move_lib_init();
	move_playback.bpm = 120.0f;

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
	printf("Commands: a <n>, b <n>, x <0-1>, va <0-1>, vb <0-1>, bpm <n>, swap, list, q\n\n");

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
					} else if (strcmp(cmd, "va") == 0) {
						if (sscanf(line, "%*s %f", &value) == 1)
							move_mixer.volume_a = value;
					} else if (strcmp(cmd, "vb") == 0) {
						if (sscanf(line, "%*s %f", &value) == 1)
							move_mixer.volume_b = value;
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
		move_evaluate_mixed(&move_mixer, &move_playback, geom, &pose);

		/* Add home height to ty */
		pose.ty += geom->home_height;

		/* Send */
		viz_sender_send_pose(sock, &pose, ROBOT_TYPE_MX64, VIZ_PORT);

		usleep(16000);
	}

	printf("\nGoodbye!\n");
	return 0;
}
