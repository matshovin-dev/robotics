/**
 * @file gl_math.h
 * @brief Minimal matrix math for modern OpenGL (header-only)
 *
 * All functions prefixed with glm_ to avoid conflicts with robotics/math
 */

#ifndef GL_MATH_H
#define GL_MATH_H

#include <math.h>
#include <string.h>

typedef float glm_mat4[16];
typedef float glm_vec3[3];

static inline void glm_mat4_identity(glm_mat4 m)
{
	memset(m, 0, sizeof(glm_mat4));
	m[0] = m[5] = m[10] = m[15] = 1.0f;
}

static inline void glm_mat4_multiply(glm_mat4 result, const glm_mat4 a, const glm_mat4 b)
{
	glm_mat4 temp;
	/* Column-major matrix multiply: C = A * B
	 * C[col][row] = sum_k A[k][row] * B[col][k]
	 * In flat index: C[col*4+row] = sum_k A[k*4+row] * B[col*4+k]
	 */
	for (int col = 0; col < 4; col++) {
		for (int row = 0; row < 4; row++) {
			temp[col * 4 + row] = 0.0f;
			for (int k = 0; k < 4; k++) {
				temp[col * 4 + row] += a[k * 4 + row] * b[col * 4 + k];
			}
		}
	}
	memcpy(result, temp, sizeof(glm_mat4));
}

static inline void glm_mat4_translate(glm_mat4 m, float x, float y, float z)
{
	glm_mat4_identity(m);
	m[12] = x;
	m[13] = y;
	m[14] = z;
}

static inline void glm_mat4_scale(glm_mat4 m, float x, float y, float z)
{
	glm_mat4_identity(m);
	m[0] = x;
	m[5] = y;
	m[10] = z;
}

static inline void glm_mat4_rotate_x(glm_mat4 m, float angle_rad)
{
	glm_mat4_identity(m);
	float c = cosf(angle_rad);
	float s = sinf(angle_rad);
	m[5] = c;
	m[6] = s;
	m[9] = -s;
	m[10] = c;
}

static inline void glm_mat4_rotate_y(glm_mat4 m, float angle_rad)
{
	glm_mat4_identity(m);
	float c = cosf(angle_rad);
	float s = sinf(angle_rad);
	m[0] = c;
	m[2] = -s;
	m[8] = s;
	m[10] = c;
}

static inline void glm_mat4_rotate_z(glm_mat4 m, float angle_rad)
{
	glm_mat4_identity(m);
	float c = cosf(angle_rad);
	float s = sinf(angle_rad);
	m[0] = c;
	m[1] = s;
	m[4] = -s;
	m[5] = c;
}

static inline void glm_mat4_perspective(glm_mat4 m, float fov_rad, float aspect,
					float near, float far)
{
	memset(m, 0, sizeof(glm_mat4));
	float tan_half_fov = tanf(fov_rad / 2.0f);
	m[0] = 1.0f / (aspect * tan_half_fov);
	m[5] = 1.0f / tan_half_fov;
	m[10] = -(far + near) / (far - near);
	m[11] = -1.0f;
	m[14] = -(2.0f * far * near) / (far - near);
}

static inline void glm_mat4_ortho(glm_mat4 m, float left, float right, float bottom,
				  float top, float near, float far)
{
	memset(m, 0, sizeof(glm_mat4));
	m[0] = 2.0f / (right - left);
	m[5] = 2.0f / (top - bottom);
	m[10] = -2.0f / (far - near);
	m[12] = -(right + left) / (right - left);
	m[13] = -(top + bottom) / (top - bottom);
	m[14] = -(far + near) / (far - near);
	m[15] = 1.0f;
}

static inline void glm_vec3_normalize(glm_vec3 v)
{
	float len = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (len > 0.0001f) {
		v[0] /= len;
		v[1] /= len;
		v[2] /= len;
	}
}

static inline void glm_vec3_cross(glm_vec3 result, const glm_vec3 a, const glm_vec3 b)
{
	result[0] = a[1] * b[2] - a[2] * b[1];
	result[1] = a[2] * b[0] - a[0] * b[2];
	result[2] = a[0] * b[1] - a[1] * b[0];
}

static inline float glm_vec3_dot(const glm_vec3 a, const glm_vec3 b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static inline void glm_mat4_look_at(glm_mat4 m, const glm_vec3 eye, const glm_vec3 center,
				    const glm_vec3 up)
{
	glm_vec3 f = { center[0] - eye[0], center[1] - eye[1], center[2] - eye[2] };
	glm_vec3_normalize(f);

	glm_vec3 s;
	glm_vec3_cross(s, f, up);
	glm_vec3_normalize(s);

	glm_vec3 u;
	glm_vec3_cross(u, s, f);

	glm_mat4_identity(m);
	m[0] = s[0];
	m[4] = s[1];
	m[8] = s[2];
	m[1] = u[0];
	m[5] = u[1];
	m[9] = u[2];
	m[2] = -f[0];
	m[6] = -f[1];
	m[10] = -f[2];
	m[12] = -glm_vec3_dot(s, eye);
	m[13] = -glm_vec3_dot(u, eye);
	m[14] = glm_vec3_dot(f, eye);
}

#endif /* GL_MATH_H */
