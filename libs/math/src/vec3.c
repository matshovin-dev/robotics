#include <math.h>
#include <robotics/math/vec3.h>

float vec3_length(const struct vec3 *v)
{
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

float vec3_length_squared(const struct vec3 *v)
{
	return v->x * v->x + v->y * v->y + v->z * v->z;
}

void vec3_normalize(struct vec3 *v)
{
	float len = vec3_length(v);

	if (len > 0.0f) {
		float inv_len = 1.0f / len;
		v->x *= inv_len;
		v->y *= inv_len;
		v->z *= inv_len;
	}
}

void vec3_scale(struct vec3 *v, float s)
{
	v->x *= s;
	v->y *= s;
	v->z *= s;
}

void vec3_negate(struct vec3 *v)
{
	v->x = -v->x;
	v->y = -v->y;
	v->z = -v->z;
}

void vec3_add(const struct vec3 *a, const struct vec3 *b, struct vec3 *result)
{
	result->x = a->x + b->x;
	result->y = a->y + b->y;
	result->z = a->z + b->z;
}

void vec3_sub(const struct vec3 *a, const struct vec3 *b, struct vec3 *result)
{
	result->x = a->x - b->x;
	result->y = a->y - b->y;
	result->z = a->z - b->z;
}

float vec3_dot(const struct vec3 *a, const struct vec3 *b)
{
	return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vec3_cross(const struct vec3 *a, const struct vec3 *b, struct vec3 *result)
{
	/*
	 * Cross product formula:
	 * result.x = a.y * b.z - a.z * b.y
	 * result.y = a.z * b.x - a.x * b.z
	 * result.z = a.x * b.y - a.y * b.x
	 *
	 * ! VIKTIG: Må bruke temp variabler hvis result == a eller result == b
	 */
	float x = a->y * b->z - a->z * b->y;
	float y = a->z * b->x - a->x * b->z;
	float z = a->x * b->y - a->y * b->x;

	result->x = x;
	result->y = y;
	result->z = z;
}

float vec3_distance(const struct vec3 *a, const struct vec3 *b)
{
	float dx = b->x - a->x;
	float dy = b->y - a->y;
	float dz = b->z - a->z;

	return sqrtf(dx * dx + dy * dy + dz * dz);
}

float vec3_distance_squared(const struct vec3 *a, const struct vec3 *b)
{
	float dx = b->x - a->x;
	float dy = b->y - a->y;
	float dz = b->z - a->z;

	return dx * dx + dy * dy + dz * dz;
}