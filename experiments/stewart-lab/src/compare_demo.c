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
struct stewart_pose pose_reference = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
struct stewart_pose pose_calculated = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

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
 * send_pose_to_port - Send pose via UDP til spesifikk port
 * @sock: UDP socket
 * @pose: stewart_pose struktur
 * @robot_type: robot konfigurasjon type
 * @port: destinasjons port
 *
 * Konverterer stewart_pose til viz_pose_packet og sender via UDP.
 */
static int send_pose_to_port(int sock, const struct stewart_pose *pose,
			     enum stewart_robot_type robot_type, int port)
{
	struct sockaddr_in addr;
	struct viz_pose_packet packet;
	ssize_t sent;

	/* Pakk stewart_pose inn i viz_pose_packet */
	packet.magic = VIZ_MAGIC;
	packet.type = VIZ_PACKET_POSE;
	packet.robot_type = robot_type;
	packet.rx = pose->rx;
	packet.ry = pose->ry;
	packet.rz = pose->rz;
	packet.tx = pose->tx;
	packet.ty = pose->ty;
	packet.tz = pose->tz;

	/* Sett opp destinasjonsadresse */
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	/* Send pakke */
	sent = sendto(sock, &packet, sizeof(packet), 0,
		      (struct sockaddr *)&addr, sizeof(addr));

	if (sent < 0) {
		perror("sendto");
		return -1;
	}

	return 0;
}

/**
 * generate_reference_motion - Generer reference pose (målposisjon)
 * @time: tid i sekunder
 * @geometry: robot geometri
 * @ref_pose: output pose
 * @inverse_result: output inverse kinematics resultat
 */
static void
generate_reference_motion(float time, const struct stewart_geometry *geometry,
			  struct stewart_pose *ref_pose,
			  struct stewart_inverse_result *inverse_result)
{
	/*
	 * Varierende bevegelse - rotasjon og translasjon
	 * Setter det inn i ref_pose, ref_pose skal ikke brukes videre
	 * i iterasjoner, kun visualiseres
	 */
	ref_pose->rx = 5.0f * sinf(time * 0.5f);
	ref_pose->ry = 5.0f * cosf(time * 0.4f);
	ref_pose->rz = 3.0f * sinf(time * 0.3f);

	ref_pose->tx = 10.0f * cosf(time * 0.3f);
	ref_pose->ty = 15.0f * sinf(time * 0.5f);
	ref_pose->tz = 10.0f * sinf(time * 0.4f);

	/*
	 * Finner nå kneposisjoner som er utgangspunkt for forward_kinematics.
	 * Setter inn kneleddpos i inverse_result
	 */
	stewart_kinematics_inverse(geometry, ref_pose, inverse_result, 0);
	/*
	 * stewart_kinematics_inverse har så gjort jobben sin og nå skal
	 * snart stewart_kinematics_forward iterere på kneposisjoner mot
	 * ref_pose som også her ble satt
	 */
}

/**
 * generate_calculated_motion - Generer forward pose
 * @geometry: robot geometri
 * @inverse_result: inverse kinematics resultat (motor vinkler)
 * @forward_result: forward kinematics resultat buffer
 * @calc_pose: output - beregnet pose fra forward kinematics
 *
 * Tar inverse kinematics resultat (motor vinkler) og itererer forward
 * kinematics for å beregne faktisk pose. Starter fra home-posisjon for å
 * simulere konvergens.
 */
static void
generate_calculated_motion(const struct stewart_geometry *geometry,
			   const struct stewart_inverse_result *inverse_result,
			   struct stewart_forward_result *forward_result,
			   struct stewart_pose *calc_pose)
{

	/*
	 * Itererer, basert på faste kneledd pos (inverse_result) og
	 * fast platform pos (ref_pose)
	 */
	for (int i = 0; i < 20; i++) {
		stewart_kinematics_forward(geometry, &calc_pose,
					   &inverse_result, &forward_result);

		// ++++ result_inv->platform_points_transformed
		calculate_transformed_platform_points(active_robot, &pose_calc,
						      &result_inv);
	}

	int i;

	/*
	 * Start forward kinematics fra home-posisjon
	 * (simulerer at robot starter fra null og konvergerer)
	 */
	calc_pose->rx = 0.0f;
	calc_pose->ry = 0.0f;
	calc_pose->rz = 0.0f;
	calc_pose->tx = 0.0f;
	calc_pose->ty = 0.0f;
	calc_pose->tz = 0.0f;

	/* Iterer forward kinematics for å finne konvergert pose */
	for (i = 0; i < 50; i++) {
		stewart_kinematics_forward(geometry, calc_pose, inverse_result,
					   forward_result);
	}

	/* Kopier resultat fra forward_result til calc_pose */
	calc_pose->rx = forward_result->rotation.x;
	calc_pose->ry = forward_result->rotation.y;
	calc_pose->rz = forward_result->rotation.z;
	calc_pose->tx = forward_result->position.x;
	calc_pose->ty = forward_result->position.y;
	calc_pose->tz = forward_result->position.z;
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
	sock = udp_create_sender();
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
		if (send_pose_to_port(sock, &pose1, ROBOT_TYPE_MX64, 9001) <
			    0 ||
		    send_pose_to_port(sock, &pose2, ROBOT_TYPE_MX64, 9002) <
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
