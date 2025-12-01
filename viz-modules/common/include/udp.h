#ifndef VIZ_UDP_H
#define VIZ_UDP_H

#include <stddef.h>

/**
 * udp_create_receiver - Lag UDP socket for mottak
 * @port: port nummer å lytte på
 *
 * Retur: socket file descriptor, eller -1 ved feil
 */
int udp_create_receiver(int port);

/**
 * udp_receive - Motta data fra UDP socket
 * @sock: socket file descriptor
 * @buffer: buffer for mottatte data
 * @buffer_size: størrelse på buffer
 *
 * Retur: antall bytes mottatt, 0 hvis ingen data, -1 ved feil
 */
int udp_receive(int sock, void *buffer, size_t buffer_size);

#endif /* VIZ_UDP_H */