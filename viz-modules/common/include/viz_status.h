/**
 * @file viz_status.h
 * @brief Send key-value status data over UDP for visualization
 */

#ifndef VIZ_STATUS_H
#define VIZ_STATUS_H

#include "viz_ports.h"

#define VIZ_STATUS_PORT VIZ_PORT_STATUS
#define VIZ_STATUS_MAX_PAIRS 32
#define VIZ_STATUS_NAME_LEN 16

/**
 * Status sender - buffers key-value pairs and sends as UDP packet
 */
struct viz_status {
	int sock;
	int count;
	struct {
		char name[VIZ_STATUS_NAME_LEN];
		float value;
	} pairs[VIZ_STATUS_MAX_PAIRS];
};

/**
 * Initialize status sender
 * Returns 0 on success, -1 on error
 */
int viz_status_init(struct viz_status *st);

/**
 * Add a float value to the buffer
 */
void viz_status_set(struct viz_status *st, const char *name, float value);

/**
 * Add an int value to the buffer
 */
void viz_status_set_int(struct viz_status *st, const char *name, int value);

/**
 * Send all buffered values as UDP packet
 */
void viz_status_send(struct viz_status *st);

/**
 * Cleanup
 */
void viz_status_close(struct viz_status *st);

#endif /* VIZ_STATUS_H */
