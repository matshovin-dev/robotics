#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

/**
 * udp_create_receiver - Lag UDP socket for mottak
 * @port: port nummer å lytte på
 *
 * Lager non-blocking UDP socket.
 *
 * Retur: socket file descriptor, eller -1 ved feil
 */
int udp_create_receiver(int port)
{
	struct sockaddr_in addr;
	int sock, flags;

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("socket");
		return -1;
	}

	/* Sett non-blocking */
	flags = fcntl(sock, F_GETFL, 0);
	fcntl(sock, F_SETFL, flags | O_NONBLOCK);

	/* Bind til port */
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		close(sock);
		return -1;
	}

	printf("UDP receiver listening on port %d\n", port);

	return sock;
}

/**
 * udp_receive - Motta data fra UDP socket
 * @sock: socket file descriptor
 * @buffer: buffer for mottatte data
 * @buffer_size: størrelse på buffer
 *
 * Non-blocking receive. Returnerer -1 hvis ingen data tilgjengelig.
 *
 * Retur: antall bytes mottatt, 0 hvis ingen data, -1 ved feil
 */
int udp_receive(int sock, void *buffer, size_t buffer_size)
{
	ssize_t n;

	n = recvfrom(sock, buffer, buffer_size, 0, NULL, NULL);

	if (n < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK)
			return 0; /* Ingen data tilgjengelig */
		perror("recvfrom");
		return -1;
	}

	return n;
}