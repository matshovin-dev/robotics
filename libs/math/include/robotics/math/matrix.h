#ifndef ROBOTICS_MATH_MATRIX_H
#define ROBOTICS_MATH_MATRIX_H

#include "robotics/math/vec3.h"

/**
 * struct mat3 - 3x3 rotasjonsmatrise
 *
 * @m[9] float - Matrise-elementer i column-major order
 *
 * Representerer 3x3 rotasjonsmatrise for 3D transformasjoner.
 *
 * Layout (column-major):
 *   m[0]  m[3]  m[6]
 *   m[1]  m[4]  m[7]
 *   m[2]  m[5]  m[8]
 *
 * @note Column-major: kolonner sammenhengende i minnet
 */
struct mat3 {
	float m[9];
};

/**
 * @function mat3_identity
 * @api PUBLIC
 *
 * @output mat->m[9]  float (identity matrix)
 *
 * Initialiserer til identitetsmatrise (I).
 */
void mat3_identity(struct mat3 *mat);

/**
 * @function mat3_rotate_x
 * @api PUBLIC
 *
 * @input  mat->m[9]   float
 * @input  angle_rad   float (radians)
 * @output mat->m[9]   float (rotated)
 *
 * Roterer matrise rundt X-aksen (in-place).
 */
void mat3_rotate_x(struct mat3 *mat, float angle_rad);

/**
 * @function mat3_rotate_y
 * @api PUBLIC
 *
 * @input  mat->m[9]   float
 * @input  angle_rad   float (radians)
 * @output mat->m[9]   float (rotated)
 *
 * Roterer matrise rundt Y-aksen (in-place).
 */
void mat3_rotate_y(struct mat3 *mat, float angle_rad);

/**
 * @function mat3_rotate_z
 * @api PUBLIC
 *
 * @input  mat->m[9]   float
 * @input  angle_rad   float (radians)
 * @output mat->m[9]   float (rotated)
 *
 * Roterer matrise rundt Z-aksen (in-place).
 */
void mat3_rotate_z(struct mat3 *mat, float angle_rad);

/**
 * @function mat3_rotate_xyz
 * @api PUBLIC
 *
 * @input  mat->m[9]  float
 * @input  x_rad      float (radians)
 * @input  y_rad      float (radians)
 * @input  z_rad      float (radians)
 * @output mat->m[9]  float (rotated by Euler angles)
 *
 * Roterer matrise rundt alle akser med Euler angles (in-place).
 * Rotasjonsrekkefølge: Z, deretter Y, deretter X (ZYX convention)
 */
void mat3_rotate_xyz(struct mat3 *mat, float x_rad, float y_rad, float z_rad);

/**
 * @function mat3_transform_vec3
 * @api PUBLIC
 *
 * @input  mat->m[9]     float
 * @input  in->{x,y,z}   float
 * @output out->{x,y,z}  float (rotated vector)
 *
 * Roterer vektor med matrise: out = mat * in
 */
void mat3_transform_vec3(const struct mat3 *mat, const struct vec3 *in,
			 struct vec3 *out);

/**
 * @function mat3_multiply
 * @api PUBLIC
 *
 * @input  a->m[9]       float
 * @input  b->m[9]       float
 * @output result->m[9]  float (a * b)
 *
 * Multipliserer to matriser: result = a * b
 */
void mat3_multiply(const struct mat3 *a, const struct mat3 *b,
		   struct mat3 *result);

/**
 * @function mat3_transpose
 * @api PUBLIC
 *
 * @input  mat->m[9]     float
 * @output result->m[9]  float (transposed)
 *
 * Transponerer matrise: result = mat^T
 * For rotasjonsmatriser: transpose = inverse
 */
void mat3_transpose(const struct mat3 *mat, struct mat3 *result);

#endif /* ROBOTICS_MATH_MATRIX_H */