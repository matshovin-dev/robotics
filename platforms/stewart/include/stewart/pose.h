#ifndef STEWART_POSE_H
#define STEWART_POSE_H

/**
 * struct stewart_pose - Platform pose (6 graders frihet)
 * @rx: Roll - rotasjon rundt X-akse (grader)
 * @ry: Pitch - rotasjon rundt Y-akse (grader)
 * @rz: Yaw - rotasjon rundt Z-akse (grader)
 * @tx: X translasjon, fra origo (mm)
 * @ty: Y translasjon, høyde fra origo (mm)
 * @tz: Z translasjon, fra origo (mm)
 *
 * Representerer topp-platformens posisjon og orientering i 3D rom.
 * Alle rotasjoner i grader, alle translasjoner i millimeter.
 * Origo er midtpunkt på base-platform i høyde med alle 6 motoraksler.
 *
 * Koordinatsystem:
 *   X+ = Høyre (rød akse)
 *   Y+ = Opp (grønn akse)
 *   Z+ = Ut av skjermen, vekk fra betrakter (blå akse)
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
 * stewart_pose_init - Initialiser til home posisjon
 * @pose: pose struktur som skal initialiseres
 * @geom: robot geometri (for å hente home_height)
 *
 * Setter rotasjoner til 0 og ty til home_height for valgt robot.
 * tx og tz settes også til 0 (platform sentrert over base).
 */
void stewart_pose_init(struct stewart_pose *pose,
		       const struct stewart_geometry *geom);

/**
 * stewart_pose_set - Sett pose-verdier
 * @pose: pose struktur
 * @rx: roll vinkel (grader)
 * @ry: pitch vinkel (grader)
 * @rz: yaw vinkel (grader)
 * @tx: X translasjon (mm)
 * @ty: Y translasjon (mm)
 * @tz: Z translasjon (mm)
 */
void stewart_pose_set(struct stewart_pose *pose, float rx, float ry, float rz,
		      float tx, float ty, float tz);

/**
 * stewart_pose_copy - Kopier pose fra kilde til destinasjon
 * @dest: destinasjon pose
 * @src: kilde pose
 */
void stewart_pose_copy(struct stewart_pose *dest,
		       const struct stewart_pose *src);

/**
 * stewart_pose_print - Print pose til stdout
 * @pose: pose som skal printes
 *
 * Printer rotasjon og posisjon for debugging.
 */
void stewart_pose_print(const struct stewart_pose *pose);

#endif /* STEWART_POSE_H */