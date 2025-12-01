#ifndef ROBOTICS_MATH_GEOMETRY_H
#define ROBOTICS_MATH_GEOMETRY_H

#include "robotics/math/vec3.h"

/**
 * distance_point_to_plane - Beregn avstand fra punkt til plan
 * @point: punktet som skal måles fra
 * @plane_point: et punkt på planet
 * @plane_normal: normalvektor til planet (bør være normalisert)
 *
 * Beregner signed distance. Positiv avstand betyr at punktet er på
 * siden som normalvektoren peker mot.
 *
 * Retur: avstand fra punkt til plan
 */
float distance_point_to_plane(const struct vec3 *point,
			      const struct vec3 *plane_point,
			      const struct vec3 *plane_normal);

/**
 * project_point_to_plane - Projiser punkt ned på plan
 * @point: punktet som skal projiseres
 * @plane_point: et punkt på planet
 * @plane_normal: normalvektor til planet (bør være normalisert)
 * @result: projisert punkt på planet
 */
void project_point_to_plane(const struct vec3 *point,
			    const struct vec3 *plane_point,
			    const struct vec3 *plane_normal,
			    struct vec3 *result);

#endif /* ROBOTICS_MATH_GEOMETRY_H */