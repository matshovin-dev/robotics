#ifndef VIZ_SENDER_H
#define VIZ_SENDER_H

#include "viz_protocol.h"

struct stewart_pose;

/**
 * viz_sender_create - Lag UDP socket for sending
 *
 * Lager en UDP socket som kan brukes til å sende poser til visualizer.
 * Socket må lukkes med close() når du er ferdig.
 *
 * Retur: socket file descriptor, eller -1 ved feil
 */
int viz_sender_create(void);

/**
 * viz_sender_send_pose - Send pose via UDP til spesifikk port
 * @sock: UDP socket (fra viz_sender_create)
 * @pose: stewart_pose struktur (absolutte koordinater)
 * @robot_type: robot konfigurasjon type (ROBOT_TYPE_MX64/AX18)
 * @port: destinasjons port (f.eks. 9001, 9002)
 *
 * Konverterer stewart_pose til viz_pose_packet og sender via UDP til
 * localhost:port. Posen sendes i absolutte koordinater.
 *
 * Retur: 0 ved suksess, -1 ved feil
 */
int viz_sender_send_pose(int sock, const struct stewart_pose *pose,
			  enum stewart_robot_type robot_type, int port);

/**
 * viz_send_pose - Send pose via UDP (one-shot variant)
 * @pose: stewart_pose struktur (absolutte koordinater)
 * @robot_type: robot konfigurasjon type (ROBOT_TYPE_MX64/AX18)
 * @port: destinasjons port (f.eks. 9001, 9002)
 *
 * Enkel one-shot funksjon som lager socket, sender pose, og lukker socket.
 * Bruk denne hvis du bare skal sende en pose. For kontinuerlig sending,
 * bruk viz_sender_create() + viz_sender_send_pose() i stedet.
 *
 * Retur: 0 ved suksess, -1 ved feil
 */
int viz_send_pose(const struct stewart_pose *pose,
		  enum stewart_robot_type robot_type, int port);

#endif /* VIZ_SENDER_H */
