#include <math.h>
#include <robotics/math/matrix.h>
#include <string.h>

void mat3_identity(struct mat3 *mat)
{
	memset(mat->m, 0, sizeof(mat->m));
	mat->m[0] = 1.0f;
	mat->m[4] = 1.0f;
	mat->m[8] = 1.0f;
}

void mat3_rotate_x(struct mat3 *mat, float angle_rad)
{
	float c = cosf(angle_rad);
	float s = sinf(angle_rad);
	struct mat3 rot;

	mat3_identity(&rot);
	rot.m[4] = c;
	rot.m[5] = s;
	rot.m[7] = -s;
	rot.m[8] = c;

	mat3_multiply(mat, &rot, mat);
}

void mat3_rotate_y(struct mat3 *mat, float angle_rad)
{
	float c = cosf(angle_rad);
	float s = sinf(angle_rad);
	struct mat3 rot;

	mat3_identity(&rot);
	rot.m[0] = c;
	rot.m[2] = -s;
	rot.m[6] = s;
	rot.m[8] = c;

	mat3_multiply(mat, &rot, mat);
}

void mat3_rotate_z(struct mat3 *mat, float angle_rad)
{
	float c = cosf(angle_rad);
	float s = sinf(angle_rad);
	struct mat3 rot;

	mat3_identity(&rot);
	rot.m[0] = c;
	rot.m[1] = s;
	rot.m[3] = -s;
	rot.m[4] = c;

	mat3_multiply(mat, &rot, mat);
}

void mat3_rotate_xyz(struct mat3 *mat, float x_rad, float y_rad, float z_rad)
{
	/*
	 * Apply rotations in ZYX order (standard for robotics)
	 */
	mat3_rotate_z(mat, z_rad);
	mat3_rotate_y(mat, y_rad);
	mat3_rotate_x(mat, x_rad);
}

void mat3_transform_vec3(const struct mat3 *mat, const struct vec3 *in,
			 struct vec3 *out)
{
	float x = in->x;
	float y = in->y;
	float z = in->z;

	out->x = mat->m[0] * x + mat->m[3] * y + mat->m[6] * z;
	out->y = mat->m[1] * x + mat->m[4] * y + mat->m[7] * z;
	out->z = mat->m[2] * x + mat->m[5] * y + mat->m[8] * z;
}

void mat3_multiply(const struct mat3 *a, const struct mat3 *b,
		   struct mat3 *result)
{
	struct mat3 temp;
	int i, j;

	/*
	 * Use temp matrix to handle case where result == a or result == b
	 */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			temp.m[i + j * 3] = a->m[i + 0] * b->m[0 + j * 3] +
					    a->m[i + 3] * b->m[1 + j * 3] +
					    a->m[i + 6] * b->m[2 + j * 3];
		}
	}

	memcpy(result->m, temp.m, sizeof(temp.m));
}

void mat3_transpose(const struct mat3 *mat, struct mat3 *result)
{
	struct mat3 temp;

	temp.m[0] = mat->m[0];
	temp.m[1] = mat->m[3];
	temp.m[2] = mat->m[6];
	temp.m[3] = mat->m[1];
	temp.m[4] = mat->m[4];
	temp.m[5] = mat->m[7];
	temp.m[6] = mat->m[2];
	temp.m[7] = mat->m[5];
	temp.m[8] = mat->m[8];

	memcpy(result->m, temp.m, sizeof(temp.m));
}