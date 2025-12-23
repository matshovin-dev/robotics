/**
 * @file shader.h
 * @brief OpenGL shader loading and management
 */

#ifndef SHADER_H
#define SHADER_H

#include <OpenGL/gl3.h>

/**
 * shader_load - Load and compile a shader program
 * @vert_path: Path to vertex shader file
 * @frag_path: Path to fragment shader file
 *
 * Returns shader program ID, or 0 on error.
 */
GLuint shader_load(const char *vert_path, const char *frag_path);

/**
 * shader_set_mat4 - Set a mat4 uniform
 */
void shader_set_mat4(GLuint program, const char *name, const float *matrix);

/**
 * shader_set_vec3 - Set a vec3 uniform
 */
void shader_set_vec3(GLuint program, const char *name, float x, float y,
		     float z);

/**
 * shader_set_float - Set a float uniform
 */
void shader_set_float(GLuint program, const char *name, float value);

/**
 * shader_set_int - Set an int uniform
 */
void shader_set_int(GLuint program, const char *name, int value);

#endif /* SHADER_H */
