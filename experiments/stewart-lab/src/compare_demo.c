#include "viz_protocol.h"
#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stewart/geometry.h>
#include <stewart/kinematics.h>
#include <stewart/pose.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static struct stewart_geometry geometry;
static struct stewart_inverse_result inverse_result;
static struct stewart_forward_result forward_result;

/**
 * udp_create_sender - Lag UDP socket for sending
 */
static int udp_create_sender(void)
{
	int sock;

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("socket");
		return -1;
	}

	return sock;
}

/**
 * send_pose_to_port - Send pose packet via UDP til spesifikk port
 */
static int send_pose_to_port(int sock, const struct viz_pose_packet *pose,
			     int port)
{
	struct sockaddr_in addr;
	ssize_t sent;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	sent = sendto(sock, pose, sizeof(*pose), 0, (struct sockaddr *)&addr,
		      sizeof(addr));

	if (sent < 0) {
		perror("sendto");
		return -1;
	}

	return 0;
}

/**
 * generate_reference_motion - Generer reference pose (målposisjon)
 * @time: tid i sekunder
 * @pose: output pose
 */
static void generate_reference_motion(float time, struct viz_pose_packet *pose)
{
	pose->magic = VIZ_MAGIC;
	pose->type = VIZ_PACKET_POSE;
	pose->robot_type = ROBOT_TYPE_MX64;

	/* Varierende bevegelse - rotasjon og translasjon */
	pose->rx = 5.0f * sinf(time * 0.5f);
	pose->ry = 5.0f * cosf(time * 0.4f);
	pose->rz = 3.0f * sinf(time * 0.3f);

	pose->tx = 10.0f * cosf(time * 0.3f);
	pose->ty = 15.0f * sinf(time * 0.5f);
	pose->tz = 10.0f * sinf(time * 0.4f);
}

/**
 * generate_calculated_motion - Generer forward pose
 * @ref_pose: referanse pose (input)
 * @calc_pose: output - beregnet pose fra forward kinematics
 *
 * Tar en referanse-pose, kjører inverse kinematics for å finne motor-vinkler,
 * deretter itererer forward kinematics for å beregne faktisk pose.
 * Starter forward kinematics fra home-posisjon for å simulere konvergens.
 */
static void generate_calculated_motion(const struct viz_pose_packet *ref_pose,
				       struct viz_pose_packet *calc_pose)
{
	struct stewart_pose pose_ref, pose_calc;
	int i;

	/* Konverter reference pose fra viz_pose_packet til stewart_pose */
	pose_ref.rx = ref_pose->rx;
	pose_ref.ry = ref_pose->ry;
	pose_ref.rz = ref_pose->rz;
	pose_ref.tx = ref_pose->tx;
	pose_ref.ty = ref_pose->ty;
	pose_ref.tz = ref_pose->tz;

	/* Beregn inverse kinematics (reference pose -> motor vinkler) */
	stewart_kinematics_inverse(&geometry, &pose_ref, &inverse_result, 0);

	/*
	 * Start forward kinematics fra home-posisjon
	 * (simulerer at robot starter fra null og konvergerer)
	 */
	pose_calc.rx = 0.0f;
	pose_calc.ry = 0.0f;
	pose_calc.rz = 0.0f;
	pose_calc.tx = 0.0f;
	pose_calc.ty = 0.0f;
	pose_calc.tz = 0.0f;

	/* Iterer forward kinematics for å finne konvergert pose */
	for (i = 0; i < 50; i++) {
		stewart_kinematics_forward(&geometry, &pose_calc,
					   &inverse_result, &forward_result);
	}

	/* Konverter tilbake til viz_pose_packet */
	calc_pose->magic = VIZ_MAGIC;
	calc_pose->type = VIZ_PACKET_POSE;
	calc_pose->robot_type = ROBOT_TYPE_MX64;

	calc_pose->rx = forward_result.rotation.x;
	calc_pose->ry = forward_result.rotation.y;
	calc_pose->rz = forward_result.rotation.z;
	calc_pose->tx = forward_result.position.x;
	calc_pose->ty = forward_result.position.y;
	calc_pose->tz = forward_result.position.z;
}

int main(void)
{
	struct viz_pose_packet pose1, pose2;
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
	sock = udp_create_sender();
	if (sock < 0) {
		fprintf(stderr, "Failed to create UDP sender\n");
		return 1;
	}

	/* Main loop */
	while (1) {
		/* Generer begge poser */
		generate_reference_motion(time, &pose1);
		generate_calculated_motion(&pose1, &pose2);

		/* Send til hver sin port */
		if (send_pose_to_port(sock, &pose1, 9001) < 0 ||
		    send_pose_to_port(sock, &pose2, 9002) < 0) {
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
