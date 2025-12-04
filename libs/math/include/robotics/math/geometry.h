#ifndef ROBOTICS_MATH_GEOMETRY_H
#define ROBOTICS_MATH_GEOMETRY_H

#include "robotics/math/vec3.h"

/**
 * @function distance_point_to_plane
 * @api PUBLIC
 *
 * @input  point->{x,y,z}        float
 * @input  plane_point->{x,y,z}  float
 * @input  plane_normal->{x,y,z} float (should be normalized)
 * @output return                float (signed distance)
 *
 * Beregner signed distance fra punkt til plan.
 * Positiv avstand: punktet er på siden normalvektoren peker mot.
 * Negativ avstand: punktet er på motsatt side.
 */
float distance_point_to_plane(const struct vec3 *point,
			      const struct vec3 *plane_point,
			      const struct vec3 *plane_normal);

/**
 * @function project_point_to_plane
 * @api PUBLIC
 *
 * @input  point->{x,y,z}        float
 * @input  plane_point->{x,y,z}  float
 * @input  plane_normal->{x,y,z} float (should be normalized)
 * @output result->{x,y,z}       float (projected point on plane)
 *
 * Projiserer punkt ned på plan langs normalvektor.
 */
void project_point_to_plane(const struct vec3 *point,
			    const struct vec3 *plane_point,
			    const struct vec3 *plane_normal,
			    struct vec3 *result);

#endif /* ROBOTICS_MATH_GEOMETRY_H */