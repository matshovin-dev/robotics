/**
 * @file viz_status.c
 * @brief Send key-value status data over UDP
 */

#include "viz_status.h"
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int viz_status_init(struct viz_status *st)
{
	st->sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (st->sock < 0) {
		perror("viz_status socket");
		return -1;
	}
	st->count = 0;
	return 0;
}

void viz_status_set(struct viz_status *st, const char *name, float value)
{
	if (st->count >= VIZ_STATUS_MAX_PAIRS)
		return;

	strncpy(st->pairs[st->count].name, name, VIZ_STATUS_NAME_LEN - 1);
	st->pairs[st->count].name[VIZ_STATUS_NAME_LEN - 1] = '\0';
	st->pairs[st->count].value = value;
	st->count++;
}

void viz_status_set_int(struct viz_status *st, const char *name, int value)
{
	viz_status_set(st, name, (float)value);
}

void viz_status_send(struct viz_status *st)
{
	if (st->count == 0)
		return;

	/* Build text message */
	char buf[1024];
	int pos = 0;

	for (int i = 0; i < st->count; i++) {
		int written = snprintf(buf + pos, sizeof(buf) - pos,
				       "%s:%.4f\n",
				       st->pairs[i].name,
				       st->pairs[i].value);
		if (written > 0)
			pos += written;
	}

	/* Send UDP */
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(VIZ_STATUS_PORT);
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	sendto(st->sock, buf, pos, 0, (struct sockaddr *)&addr, sizeof(addr));

	/* Clear buffer for next frame */
	st->count = 0;
}

void viz_status_close(struct viz_status *st)
{
	if (st->sock >= 0) {
		close(st->sock);
		st->sock = -1;
	}
}
