#include "stewart/geometry.h"
#include "stewart/kinematics.h"
#include "stewart/pose.h"
#include "viz_sender.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*
 * Sender 2 poser over 2 UDP porter
 */

static struct stewart_geometry geometry;
static struct stewart_inverse_result inverse_result;
static struct stewart_forward_result forward_result;
struct stewart_pose pose_reference = { 0.0f, 0.0f, 0.0f, 0.0f, 210.0f, 0.0f };
struct stewart_pose pose_calculated = { 0.0f, 0.0f, 0.0f, 0.0f, 210.0f, 0.0f };

static void generate_reference_motion(float time,
				      const struct stewart_geometry *geometry,
				      struct stewart_pose *ref_pose,
				      struct stewart_inverse_result *result_inv)
{
	float off = geometry->home_height;
	/*
	 * Varierende bevegelse - rotasjon og translasjon
	 * Setter det inn i ref_pose, ref_pose skal ikke brukes videre
	 * i iterasjoner, kun visualiseres
	 */
	ref_pose->rx = 0.0f * sinf(time * 0.5f);
	ref_pose->ry = 0.0f * cosf(time * 0.4f);
	ref_pose->rz = 0.0f * sinf(time * 0.3f);

	ref_pose->tx = 0.0f * cosf(time * 0.3f);
	ref_pose->ty = off + 35.0f + 60.0f * sinf(0.5f * time * 0.5f);
	ref_pose->tz = 0.0f * sinf(time * 0.4f);

	/*
	 * Finner nå kneposisjoner som er utgangspunkt for forward_kinematics.
	 * Setter inn kneleddpos i result_inv
	 */
	stewart_kinematics_inverse(geometry, ref_pose, result_inv, 0);
	/*
	 * stewart_kinematics_inverse har så gjort jobben sin og nå skal
	 * snart stewart_kinematics_forward iterere på kneposisjoner mot
	 * ref_pose som også her ble satt
	 */
}

static void
generate_calculated_motion(const struct stewart_geometry *geometry,
			   const struct stewart_inverse_result *inverse_result,
			   struct stewart_forward_result *forward_result,
			   struct stewart_pose *calc_pose)
{
	float off = geometry->home_height;
	/*
	 * Start forward kinematics fra home-posisjon
	 * (simulerer at robot starter fra null og konvergerer)
	 */
	forward_result->pose_result.rx = 0.0f;
	forward_result->pose_result.ry = 0.0f;
	forward_result->pose_result.rz = 0.0f;
	forward_result->pose_result.tx = 0.0f;
	forward_result->pose_result.ty = off;
	forward_result->pose_result.tz = 0.0f;

	/* Iterer forward kinematics for å finne konvergert pose (50
	 * iterasjoner) */
	for (int i = 0; i < 50; i++) {
		stewart_kinematics_forward(geometry, inverse_result,
					   forward_result);
	}

	/* Kopier resultat fra forward_result til calc_pose */
	*calc_pose = forward_result->pose_result;
}

int main(void)
{
	struct stewart_pose pose1, pose2;
	int sock;
	float time = 0.0f;
	float dt = 0.016f; /* ~60 FPS */

	printf("Stewart Platform Comparison Demo\n");
	printf("=================================\n\n");
	printf("Sending poses to:\n");
	printf("  Port 9001: Reference/Target (CYAN)\n");
	printf("  Port 9002: Calculated/Forward (MAGENTA)\n\n");
	printf("Shows difference between reference pose and forward kinematics.\n");
	printf("Press Ctrl+C to stop\n\n");

	/* Initialiser robot geometri */
	geometry = ROBOT_MX64;

	/* Lag UDP sender */
	sock = viz_sender_create();
	if (sock < 0) {
		fprintf(stderr, "Failed to create UDP sender\n");
		return 1;
	}

	/* Main loop */
	while (1) {
		/* Generer begge poser */
		generate_reference_motion(time, &geometry, &pose1,
					  &inverse_result);
		generate_calculated_motion(&geometry, &inverse_result,
					   &forward_result, &pose2);

		/* Send til hver sin port */
		if (viz_sender_send_pose(sock, &pose1, ROBOT_TYPE_MX64, 9001) <
			    0 ||
		    viz_sender_send_pose(sock, &pose2, ROBOT_TYPE_MX64, 9002) <
			    0) {
			fprintf(stderr, "Failed to send poses\n");
			break;
		}

		/* Print current poses (hver 30. frame = ~0.5s) */
		if ((int)(time / dt) % 30 == 0) {
			printf("t=%.1fs\n", time);
			printf("  Reference:  rx=%.1f° ry=%.1f° rz=%.1f°  "
			       "tx=%.1fmm ty=%.1fmm tz=%.1fmm\n",
			       pose1.rx, pose1.ry, pose1.rz, pose1.tx, pose1.ty,
			       pose1.tz);
			printf("  Calculated: rx=%.1f° ry=%.1f° rz=%.1f°  "
			       "tx=%.1fmm ty=%.1fmm tz=%.1fmm\n\n",
			       pose2.rx, pose2.ry, pose2.rz, pose2.tx, pose2.ty,
			       pose2.tz);
		}

		/* Oppdater tid */
		time += dt;

		/* Sleep for å holde ~60 FPS */
		usleep((int)(dt * 1000000));
	}

	close(sock);
	return 0;
}
