#ifndef ROBOTICS_MATH_MATRIX_H
#define ROBOTICS_MATH_MATRIX_H

#include "robotics/math/vec3.h"

/**
 * struct mat3 - 3x3 rotasjonsmatrise
 * @m: matrise-elementer i column-major order
 *
 * Layout:
 * m[0]  m[3]  m[6]
 * m[1]  m[4]  m[7]
 * m[2]  m[5]  m[8]
 */
struct mat3 {
	float m[9];
};

/**
 * mat3_identity - Initialiser til identitetsmatrise
 * @mat: matrise som skal initialiseres
 */
void mat3_identity(struct mat3 *mat);

/**
 * mat3_rotate_x - Roter rundt X-aksen
 * @mat: matrise som skal roteres (modifiseres in-place)
 * @angle_rad: rotasjonsvinkel i radianer
 */
void mat3_rotate_x(struct mat3 *mat, float angle_rad);

/**
 * mat3_rotate_y - Roter rundt Y-aksen
 * @mat: matrise som skal roteres (modifiseres in-place)
 * @angle_rad: rotasjonsvinkel i radianer
 */
void mat3_rotate_y(struct mat3 *mat, float angle_rad);

/**
 * mat3_rotate_z - Roter rundt Z-aksen
 * @mat: matrise som skal roteres (modifiseres in-place)
 * @angle_rad: rotasjonsvinkel i radianer
 */
void mat3_rotate_z(struct mat3 *mat, float angle_rad);

/**
 * mat3_rotate_xyz - Roter rundt alle akser (Euler angles)
 * @mat: matrise som skal roteres (modifiseres in-place)
 * @x_rad: rotasjon rundt X-akse i radianer
 * @y_rad: rotasjon rundt Y-akse i radianer
 * @z_rad: rotasjon rundt Z-akse i radianer
 *
 * Rotasjonsrekkefølge: Z, deretter Y, deretter X (ZYX convention)
 */
void mat3_rotate_xyz(struct mat3 *mat, float x_rad, float y_rad, float z_rad);

/**
 * mat3_transform_vec3 - Roter vektor med matrise
 * @mat: rotasjonsmatrise
 * @in: input vektor
 * @out: output vektor (rotert)
 */
void mat3_transform_vec3(const struct mat3 *mat, const struct vec3 *in,
			 struct vec3 *out);

/**
 * mat3_multiply - Multipliser to matriser
 * @a: første matrise
 * @b: andre matrise
 * @result: resultat-matrise (a * b)
 */
void mat3_multiply(const struct mat3 *a, const struct mat3 *b,
		   struct mat3 *result);

/**
 * mat3_transpose - Transposer matrise
 * @mat: matrise som skal transponeres
 * @result: transponert matrise
 *
 * For rotasjonsmatriser er transpose = inverse
 */
void mat3_transpose(const struct mat3 *mat, struct mat3 *result);

#endif /* ROBOTICS_MATH_MATRIX_H */