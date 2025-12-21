#include "viz_sender.h"
#include "stewart/pose.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

int viz_sender_create(void)
{
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("socket");
		return -1;
	}

	/* Øk send buffer for store pakker (f.eks. move_lib ~17KB) */
	int sndbuf = 65536;
	if (setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
		perror("setsockopt SO_SNDBUF");
	}

	return sock;
}

int viz_sender_send_pose(int sock, const struct stewart_pose *pose,
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

int viz_send_pose(const struct stewart_pose *pose,
		  enum stewart_robot_type robot_type, int port)
{
	int sock;
	int result;

	sock = viz_sender_create();
	if (sock < 0)
		return -1;

	result = viz_sender_send_pose(sock, pose, robot_type, port);

	close(sock);

	return result;
}

int viz_sender_send_move_bars(int sock, int move_no, const float *values,
			      int port)
{
	struct sockaddr_in addr;
	struct viz_move_bars_packet packet;
	ssize_t sent;

	packet.magic = VIZ_MAGIC;
	packet.type = VIZ_PACKET_MOVE_BARS;
	packet.move_no = move_no;
	memcpy(packet.values, values, sizeof(packet.values));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	sent = sendto(sock, &packet, sizeof(packet), 0,
		      (struct sockaddr *)&addr, sizeof(addr));

	if (sent < 0) {
		perror("sendto");
		return -1;
	}

	return 0;
}

int viz_sender_send_move_lib(int sock, int deck_a, int deck_b, int edit_dof,
			     const float *values, int port)
{
	struct sockaddr_in addr;
	struct viz_move_lib_packet packet;
	ssize_t sent;

	packet.magic = VIZ_MAGIC;
	packet.type = VIZ_PACKET_MOVE_LIB;
	packet.deck_a = deck_a;
	packet.deck_b = deck_b;
	packet.edit_dof = edit_dof;
	memcpy(packet.values, values, sizeof(packet.values));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	sent = sendto(sock, &packet, sizeof(packet), 0,
		      (struct sockaddr *)&addr, sizeof(addr));

	if (sent < 0) {
		perror("sendto move_lib");
		return -1;
	}

	return 0;
}
