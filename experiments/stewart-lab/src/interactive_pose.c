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

/* Current pose state */
static struct viz_pose_packet current_pose;
static int udp_sock = -1;

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
 * send_current_pose - Send current pose via UDP til visualizer
 *
 * Retur: 0 ved suksess, -1 ved feil
 */
static int send_current_pose(void)
{
	struct sockaddr_in addr;
	ssize_t sent;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(VIZ_PORT);
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	sent = sendto(udp_sock, &current_pose, sizeof(current_pose), 0,
		      (struct sockaddr *)&addr, sizeof(addr));

	if (sent < 0) {
		perror("sendto");
		return -1;
	}

	return 0;
}

/**
 * show_current_pose - Print current pose til terminal
 */
static void show_current_pose(void)
{
	printf("\nCurrent pose:\n");
	printf("  Rotation:    rx=%.1f°  ry=%.1f°  rz=%.1f°\n",
	       current_pose.rx, current_pose.ry, current_pose.rz);
	printf("  Translation: tx=%.1f   ty=%.1f   tz=%.1f  (mm)\n\n",
	       current_pose.tx, current_pose.ty, current_pose.tz);
}

/**
 * set_home_pose - Reset pose til home position
 */
static void set_home_pose(void)
{
	current_pose.rx = 0.0f;
	current_pose.ry = 0.0f;
	current_pose.rz = 0.0f;
	current_pose.tx = 0.0f;
	current_pose.ty = 205.0f; /* MX64 home_height */
	current_pose.tz = 0.0f;
	printf("Reset to home position\n");
}

/**
 * print_help - Print kommando-hjelp
 */
static void print_help(void)
{
	printf("\nAvailable commands:\n");
	printf("  rx <degrees>    Set roll (rotation around X-axis)\n");
	printf("  ry <degrees>    Set pitch (rotation around Y-axis)\n");
	printf("  rz <degrees>    Set yaw (rotation around Z-axis)\n");
	printf("  tx <mm>         Set X translation\n");
	printf("  ty <mm>         Set Y translation\n");
	printf("  tz <mm>         Set Z translation\n");
	printf("  home            Reset to home position (all zeros)\n");
	printf("  show            Show current pose\n");
	printf("  help            Show this help\n");
	printf("  quit            Exit program\n");
	printf("\nExamples:\n");
	printf("  rx 10           Tilt forward 10 degrees\n");
	printf("  ry -5           Tilt left 5 degrees\n");
	printf("  tz 20           Move 20mm away from base\n");
	printf("  home            Return to center\n\n");
}

/**
 * parse_and_execute - Parse og eksekvér kommando fra input-linje
 * @line: input line fra bruker
 *
 * Retur: 0 = fortsett, 1 = quit
 */
static int parse_and_execute(const char *line)
{
	char cmd[64];
	float value;
	int n;

	/* Trim leading/trailing whitespace */
	while (*line == ' ' || *line == '\t')
		line++;

	if (strlen(line) == 0)
		return 0;

	/* Try parsing "cmd value" format */
	n = sscanf(line, "%63s %f", cmd, &value);

	if (n == 1) {
		/* Single-word commands */
		if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "q") == 0 ||
		    strcmp(cmd, "exit") == 0) {
			return 1;
		} else if (strcmp(cmd, "home") == 0 || strcmp(cmd, "h") == 0) {
			set_home_pose();
			send_current_pose();
			show_current_pose();
		} else if (strcmp(cmd, "show") == 0 || strcmp(cmd, "s") == 0) {
			show_current_pose();
		} else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
			print_help();
		} else {
			printf("Unknown command: %s (try 'help')\n", cmd);
		}
	} else if (n == 2) {
		/* Commands with value */
		if (strcmp(cmd, "rx") == 0) {
			current_pose.rx = value;
			printf("Set rx = %.1f°\n", value);
		} else if (strcmp(cmd, "ry") == 0) {
			current_pose.ry = value;
			printf("Set ry = %.1f°\n", value);
		} else if (strcmp(cmd, "rz") == 0) {
			current_pose.rz = value;
			printf("Set rz = %.1f°\n", value);
		} else if (strcmp(cmd, "tx") == 0) {
			current_pose.tx = value;
			printf("Set tx = %.1f mm\n", value);
		} else if (strcmp(cmd, "ty") == 0) {
			current_pose.ty = value;
			printf("Set ty = %.1f mm\n", value);
		} else if (strcmp(cmd, "tz") == 0) {
			current_pose.tz = value;
			printf("Set tz = %.1f mm\n", value);
		} else {
			printf("Unknown command: %s (try 'help')\n", cmd);
			return 0;
		}

		/* Send updated pose */
		send_current_pose();
	} else {
		printf("Invalid command format (try 'help')\n");
	}

	return 0;
}

int main(void)
{
	char line[256];

	printf("Stewart Platform Interactive Pose Controller\n");
	printf("=============================================\n\n");

	/* Initialize pose to home */
	memset(&current_pose, 0, sizeof(current_pose));
	current_pose.magic = VIZ_MAGIC;
	current_pose.type = VIZ_PACKET_POSE;
	current_pose.robot_type = ROBOT_TYPE_MX64;

	/* Create UDP sender */
	udp_sock = udp_create_sender();
	if (udp_sock < 0) {
		fprintf(stderr, "Failed to create UDP sender\n");
		return 1;
	}

	printf("Sending poses to localhost:%d\n", VIZ_PORT);
	printf("Type 'help' for commands, 'quit' to exit\n\n");

	/* Send initial home pose */
	send_current_pose();
	show_current_pose();

	/* Main REPL loop */
	while (1) {
		printf("> ");
		fflush(stdout);

		if (fgets(line, sizeof(line), stdin) == NULL)
			break;

		/* Remove trailing newline */
		line[strcspn(line, "\n")] = 0;

		/* Parse and execute command */
		if (parse_and_execute(line))
			break;
	}

	printf("\nExiting...\n");
	close(udp_sock);
	return 0;
}
