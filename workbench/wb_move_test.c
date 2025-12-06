/**
 * @file wb_move_test.c
 * @brief Workbench: Test move_lib with visualizer
 *
 * Simple test of move_lib sending poses to viz-modules visualizers.
 * Run plot_stw_obj or plot_stw_polygon first, then this.
 *
 * Usage:
 *   ./wb_move_test [move_index] [bpm]
 *
 * Examples:
 *   ./wb_move_test           # Default: move 4 (bounce) at 120 BPM
 *   ./wb_move_test 7         # Move 7 (complex) at 120 BPM
 *   ./wb_move_test 4 140     # Move 4 at 140 BPM
 */

#include "move_lib.h"
#include "stewart/geometry.h"
#include "stewart/pose.h"
#include "viz_sender.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
	int move_index = 4; /* Default: bounce */
	float bpm = 120.0f;
	int sock;
	struct timeval last, now;
	struct stewart_pose pose;
	const struct stewart_geometry *geom = &ROBOT_MX64;

	/* Parse arguments */
	if (argc > 1)
		move_index = atoi(argv[1]);
	if (argc > 2)
		bpm = atof(argv[2]);

	if (move_index < 0 || move_index >= MOVE_LIB_SIZE) {
		fprintf(stderr, "Invalid move index (0-%d)\n",
			MOVE_LIB_SIZE - 1);
		return 1;
	}

	/* Initialize */
	move_lib_init();
	move_playback.bpm = bpm;

	/* Create UDP sender */
	sock = viz_sender_create();
	if (sock < 0) {
		fprintf(stderr, "Failed to create UDP sender\n");
		return 1;
	}

	printf("Move Library Test\n");
	printf("=================\n");
	printf("Move: %d (%s)\n", move_index, move_lib[move_index].name);
	printf("BPM: %.0f\n", bpm);
	printf("Robot: MX64\n");
	printf("Sending to port %d...\n\n", VIZ_PORT);
	printf("Press Ctrl+C to stop\n\n");

	gettimeofday(&last, NULL);

	/* Main loop at ~60 Hz */
	while (1) {
		gettimeofday(&now, NULL);
		float dt = (now.tv_sec - last.tv_sec) +
			   (now.tv_usec - last.tv_usec) / 1000000.0f;
		last = now;

		/* Update playback time */
		move_playback_tick(&move_playback, dt);

		/* Evaluate move */
		move_evaluate(&move_lib[move_index], &move_playback, geom,
			      &pose);

		/* Add home height to ty */
		pose.ty += geom->home_height;

		/* Send to visualizer */
		viz_sender_send_pose(sock, &pose, ROBOT_TYPE_MX64, VIZ_PORT);

		usleep(16000); /* ~60 Hz */
	}

	return 0;
}
