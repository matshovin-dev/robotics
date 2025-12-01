#ifndef VIZ_PROTOCOL_H
#define VIZ_PROTOCOL_H

#include <stdint.h>

#define VIZ_PORT 9001
#define VIZ_MAGIC 0x53545750 /* "STWP" */

/**
 * enum viz_packet_type - Packet type identifiers
 * @VIZ_PACKET_POSE: Pose update packet
 * @VIZ_PACKET_GEOMETRY: Geometry configuration packet (future)
 */
enum viz_packet_type {
	VIZ_PACKET_POSE = 1,
	VIZ_PACKET_GEOMETRY = 2,
};

/**
 * enum stewart_robot_type - Robot configuration types
 * @ROBOT_TYPE_MX64: MX64 Dynamixel configuration
 * @ROBOT_TYPE_AX18: AX18 servo configuration
 */
enum stewart_robot_type {
	ROBOT_TYPE_MX64 = 0,
	ROBOT_TYPE_AX18 = 1,
};

/**
 * struct viz_pose_packet - Pose update packet
 * @magic: magic number (VIZ_MAGIC) for validation
 * @type: packet type (VIZ_PACKET_POSE)
 * @robot_type: robot configuration type
 * @rx: roll rotation (degrees)
 * @ry: pitch rotation (degrees)
 * @rz: yaw rotation (degrees)
 * @tx: X translation (mm)
 * @ty: Y translation (mm)
 * @tz: Z translation (mm)
 *
 * Sendes via UDP fra eksperiment-program til visualizer.
 * Packed for å garantere samme layout på alle platformer.
 */
struct viz_pose_packet {
	uint32_t magic;
	uint32_t type;
	uint32_t robot_type;
	float rx, ry, rz;
	float tx, ty, tz;
} __attribute__((packed));

#endif /* VIZ_PROTOCOL_H */