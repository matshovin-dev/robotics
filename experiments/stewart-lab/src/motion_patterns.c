#include "viz_protocol.h"
#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * udp_create_sender - Lag UDP socket for sending
 *
 * Retur: socket file descriptor, eller -1 ved feil
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
 * send_pose - Send pose packet via UDP
 * @sock: UDP socket
 * @pose: pose packet som skal sendes
 *
 * Sender til localhost:9001 (visualizer)
 *
 * Retur: 0 ved suksess, -1 ved feil
 */
static int send_pose(int sock, const struct viz_pose_packet *pose)
{
	struct sockaddr_in addr;
	ssize_t sent;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(VIZ_PORT);
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
 * generate_circular_motion - Generer sirkulær bevegelse
 * @time: tid i sekunder
 * @pose: output pose
 *
 * Lager en enkel sirkulær bevegelse rundt Z-aksen.
 */
static void generate_circular_motion(float time, struct viz_pose_packet *pose)
{
	float angle = time * 0.5f; /* 0.5 rad/s */

	pose->magic = VIZ_MAGIC;
	pose->type = VIZ_PACKET_POSE;
	pose->robot_type = ROBOT_TYPE_MX64;

	/* Rotasjon rundt Z-akse */
	pose->rx = 0.0f;
	pose->ry = 0.0f;
	pose->rz = angle * 180.0f / M_PI;

	/* Ingen translasjon */
	pose->tx = 0.0f;
	pose->ty = 0.0f;
	pose->tz = 0.0f;
}

/**
 * generate_tilting_motion - Generer vippende bevegelse
 * @time: tid i sekunder
 * @pose: output pose
 *
 * Lager en vippende bevegelse (kombinert RX og RY).
 */
static void generate_tilting_motion(float time, struct viz_pose_packet *pose)
{
	pose->magic = VIZ_MAGIC;
	pose->type = VIZ_PACKET_POSE;
	pose->robot_type = ROBOT_TYPE_MX64;

	/* Vippende bevegelse */
	pose->rx = 10.0f * sinf(time * 1.0f);
	pose->ry = 10.0f * cosf(time * 0.7f);
	pose->rz = 0.0f;

	/* Ingen translasjon */
	pose->tx = 0.0f;
	pose->ty = 0.0f;
	pose->tz = 0.0f;
}

/**
 * generate_combined_motion - Generer kombinert bevegelse
 * @time: tid i sekunder
 * @pose: output pose
 *
 * Kombinerer rotasjon og translasjon for mer kompleks bevegelse.
 */
static void generate_combined_motion(float time, struct viz_pose_packet *pose)
{
	pose->magic = VIZ_MAGIC;
	pose->type = VIZ_PACKET_POSE;
	pose->robot_type = ROBOT_TYPE_MX64;

	/* Rotasjon */
	pose->rx = 5.0f * sinf(time * 1.2f);
	pose->ry = 5.0f * cosf(time * 0.8f);
	pose->rz = 10.0f * sinf(time * 0.5f);

	/* Sirkulær translasjon i XZ-plan */
	pose->tx = 15.0f * cosf(time * 0.6f);
	pose->ty = 0.0f;
	pose->tz = 15.0f * sinf(time * 0.6f);
}

/**
 * print_menu - Skriv ut valgmeny
 */
static void print_menu(void)
{
	printf("\n");
	printf("Stewart Platform Experiment Lab\n");
	printf("================================\n");
	printf("\n");
	printf("Motion patterns:\n");
	printf("  1. Circular (rotation around Z)\n");
	printf("  2. Tilting (RX + RY oscillation)\n");
	printf("  3. Combined (rotation + translation)\n");
	printf("  4. Manual (edit code and recompile)\n");
	printf("\n");
	printf("Select pattern (1-4): ");
}

int main(void)
{
	struct viz_pose_packet pose;
	int sock;
	int choice;
	float time = 0.0f;
	float dt = 0.016f; /* ~60 FPS */

	print_menu();
	if (scanf("%d", &choice) != 1) {
		fprintf(stderr, "Invalid input\n");
		return 1;
	}

	/* Lag UDP sender */
	sock = udp_create_sender();
	if (sock < 0) {
		fprintf(stderr, "Failed to create UDP sender\n");
		return 1;
	}

	printf("\nSending poses to localhost:%d\n", VIZ_PORT);
	printf("Press Ctrl+C to stop\n\n");

	/* Main loop */
	while (1) {
		/* Generer pose basert på valg */
		switch (choice) {
		case 1:
			generate_circular_motion(time, &pose);
			break;
		case 2:
			generate_tilting_motion(time, &pose);
			break;
		case 3:
			generate_combined_motion(time, &pose);
			break;
		case 4:
			/* Manuel pose - edit her og recompile */
			pose.magic = VIZ_MAGIC;
			pose.type = VIZ_PACKET_POSE;
			pose.robot_type = ROBOT_TYPE_MX64;
			pose.rx = 10.0f; /* Endre disse verdiene */
			pose.ry = 5.0f;
			pose.rz = 0.0f;
			pose.tx = 0.0f;
			pose.ty = 0.0f;
			pose.tz = 0.0f;
			break;
		default:
			fprintf(stderr, "Invalid choice\n");
			close(sock);
			return 1;
		}

		/* Send pose */
		if (send_pose(sock, &pose) < 0) {
			fprintf(stderr, "Failed to send pose\n");
			break;
		}

		/* Print current pose (hver 30. frame = ~0.5s) */
		if ((int)(time / dt) % 30 == 0) {
			printf("t=%.1fs  rx=%.1f° ry=%.1f° rz=%.1f°  "
			       "tx=%.1f ty=%.1f tz=%.1f\n",
			       time, pose.rx, pose.ry, pose.rz, pose.tx,
			       pose.ty, pose.tz);
		}

		/* Oppdater tid */
		time += dt;

		/* Sleep for å holde ~60 FPS */
		usleep((int)(dt * 1000000));
	}

	close(sock);
	return 0;
}