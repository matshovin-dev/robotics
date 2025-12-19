/**
 * @file viz_ports.h
 * @brief UDP port definitions for all visualizers
 *
 * Centralized port assignments to avoid conflicts and make it easy
 * to remember which port goes where.
 *
 * Example - multiple plotters:
 *   ./plot_stw_polygon 9001   # MX64 polygon
 *   ./plot_stw_polygon 9005   # AX18 polygon
 *   ./plot_stw_obj 9002       # MX64 obj
 *   ./plot_stw_obj 9006       # AX18 obj
 *   ./plot_status 9003        # Status
 *
 * In workbench:
 *   viz_sender_send_pose(sock, &pose_mx64, ROBOT_TYPE_MX64, VIZ_PORT_OBJ);
 *   viz_sender_send_pose(sock, &pose_ax18, ROBOT_TYPE_AX18, 9006);
 */

#ifndef VIZ_PORTS_H
#define VIZ_PORTS_H

/* Stewart platform visualizers */
#define VIZ_PORT_POLYGON 9001 /* plot_stw_polygon */
#define VIZ_PORT_OBJ 9002 /* plot_stw_obj */

/* Status and parameter displays */
#define VIZ_PORT_STATUS 9003 /* plot_status */

/* Future plotters */
#define VIZ_PORT_SCOPE 9004 /* Oscilloscope-style DOF plot */
#define VIZ_PORT_PHASE 9005 /* Phase/beat visualization */
#define VIZ_PORT_MOVE_BARS 9010 /* Move parameter bar plot */

#endif /* VIZ_PORTS_H */
