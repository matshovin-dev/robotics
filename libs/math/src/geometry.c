#include <robotics/math/geometry.h>

float distance_point_to_plane(const struct vec3 *point,
			      const struct vec3 *plane_point,
			      const struct vec3 *plane_normal)
{
	struct vec3 point_to_plane;
	float distance;

	/*
	 * Calculate vector from plane point to test point
	 */
	vec3_sub(point, plane_point, &point_to_plane);

	/*
	 * Distance is dot product with plane normal
	 * (assumes plane_normal is normalized)
	 */
	distance = vec3_dot(&point_to_plane, plane_normal);

	return distance;
}

void project_point_to_plane(const struct vec3 *point,
			    const struct vec3 *plane_point,
			    const struct vec3 *plane_normal,
			    struct vec3 *result)
{
	float distance;
	struct vec3 offset;

	/*
	 * Get signed distance to plane
	 */
	distance = distance_point_to_plane(point, plane_point, plane_normal);

	/*
	 * Move point along normal by negative distance
	 */
	offset = *plane_normal;
	vec3_scale(&offset, -distance);
	vec3_add(point, &offset, result);
}