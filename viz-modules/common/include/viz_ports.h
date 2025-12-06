/**
 * @file viz_ports.h
 * @brief UDP port definitions for all visualizers
 *
 * Centralized port assignments to avoid conflicts and make it easy
 * to remember which port goes where.
 */

#ifndef VIZ_PORTS_H
#define VIZ_PORTS_H

/* Stewart platform visualizers */
#define VIZ_PORT_POLYGON      9001   /* plot_stw_polygon */
#define VIZ_PORT_OBJ          9002   /* plot_stw_obj */

/* Status and parameter displays */
#define VIZ_PORT_STATUS       9003   /* plot_status */

/* Future plotters */
#define VIZ_PORT_SCOPE        9004   /* Oscilloscope-style DOF plot */
#define VIZ_PORT_PHASE        9005   /* Phase/beat visualization */

#endif /* VIZ_PORTS_H */
