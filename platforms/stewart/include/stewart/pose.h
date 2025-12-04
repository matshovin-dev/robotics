#ifndef STEWART_POSE_H
#define STEWART_POSE_H

/* Forward declaration */
struct stewart_geometry;

/**
 * struct stewart_pose - Platform pose (6 graders frihet)
 *
 * @rx float (deg) - Roll (rotasjon rundt X-akse)
 * @ry float (deg) - Pitch (rotasjon rundt Y-akse)
 * @rz float (deg) - Yaw (rotasjon rundt Z-akse)
 * @tx float (mm) - X translasjon fra origo
 * @ty float (mm) - Y translasjon (høyde fra origo)
 * @tz float (mm) - Z translasjon fra origo
 *
 * Representerer topp-platformens posisjon og orientering i 3D rom.
 * Origo er midtpunkt på base-platform ved motoraksel-høyde.
 *
 * Koordinatsystem:
 *   X+ = Høyre (rød akse)
 *   Y+ = Opp (grønn akse)
 *   Z+ = Ut av skjermen (blå akse)
 *
 * @note En pose er uavhengig av platformens home posisjon
 */
struct stewart_pose {
	float rx;
	float ry;
	float rz;
	float tx;
	float ty;
	float tz;
};

/**
 * @function stewart_pose_init
 * @api PUBLIC
 *
 * @input  geom->home_height  float (mm)
 *
 * @output pose->{rx,ry,rz}   float (0 degrees)
 * @output pose->{tx,ty,tz}   float (0, home_height, 0 mm)
 *
 * Setter rotasjoner til 0 og ty til home_height for valgt robot.
 * tx og tz settes også til 0 (platform sentrert over base).
 */
void stewart_pose_init(struct stewart_pose *pose,
		       const struct stewart_geometry *geom);

/**
 * @function stewart_pose_set
 * @api PUBLIC
 *
 * @input  rx  float (degrees)
 * @input  ry  float (degrees)
 * @input  rz  float (degrees)
 * @input  tx  float (mm)
 * @input  ty  float (mm)
 * @input  tz  float (mm)
 *
 * @output pose->{rx,ry,rz}  float (degrees)
 * @output pose->{tx,ty,tz}  float (mm)
 *
 * Setter alle pose-verdier manuelt.
 *
 * @note Husk å sette ty høy nok for å unngå kollisjon med base
 */
void stewart_pose_set(struct stewart_pose *pose, float rx, float ry, float rz,
		      float tx, float ty, float tz);

/**
 * @function stewart_pose_copy
 * @api PUBLIC
 *
 * @input  src->{rx,ry,rz}   float (degrees)
 * @input  src->{tx,ty,tz}   float (mm)
 *
 * @output dest->{rx,ry,rz}  float (degrees)
 * @output dest->{tx,ty,tz}  float (mm)
 *
 * Kopierer alle pose-verdier fra src til dest.
 */
void stewart_pose_copy(struct stewart_pose *dest,
		       const struct stewart_pose *src);

/**
 * @function stewart_pose_print
 * @api PUBLIC
 *
 * @input  pose->{rx,ry,rz}  float (degrees)
 * @input  pose->{tx,ty,tz}  float (mm)
 *
 * @output stdout  Formatted text output
 *
 * Printer rotasjon og posisjon til stdout for debugging.
 */
void stewart_pose_print(const struct stewart_pose *pose);

#endif /* STEWART_POSE_H */