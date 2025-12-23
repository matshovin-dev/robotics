/**
 * @file shader.c
 * @brief OpenGL shader loading and management
 */

#include "shader.h"
#include <stdio.h>
#include <stdlib.h>

static char *read_file(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		fprintf(stderr, "Failed to open: %s\n", path);
		return NULL;
	}

	fseek(f, 0, SEEK_END);
	long len = ftell(f);
	fseek(f, 0, SEEK_SET);

	char *buf = malloc(len + 1);
	if (!buf) {
		fclose(f);
		return NULL;
	}

	fread(buf, 1, len, f);
	buf[len] = '\0';
	fclose(f);

	return buf;
}

static GLuint compile_shader(const char *source, GLenum type)
{
	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &source, NULL);
	glCompileShader(shader);

	GLint success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		char log[512];
		glGetShaderInfoLog(shader, sizeof(log), NULL, log);
		fprintf(stderr, "Shader compile error: %s\n", log);
		glDeleteShader(shader);
		return 0;
	}

	return shader;
}

GLuint shader_load(const char *vert_path, const char *frag_path)
{
	char *vert_src = read_file(vert_path);
	char *frag_src = read_file(frag_path);

	if (!vert_src || !frag_src) {
		free(vert_src);
		free(frag_src);
		return 0;
	}

	GLuint vert = compile_shader(vert_src, GL_VERTEX_SHADER);
	GLuint frag = compile_shader(frag_src, GL_FRAGMENT_SHADER);

	free(vert_src);
	free(frag_src);

	if (!vert || !frag) {
		glDeleteShader(vert);
		glDeleteShader(frag);
		return 0;
	}

	GLuint program = glCreateProgram();
	glAttachShader(program, vert);
	glAttachShader(program, frag);
	glLinkProgram(program);

	GLint success;
	glGetProgramiv(program, GL_LINK_STATUS, &success);
	if (!success) {
		char log[512];
		glGetProgramInfoLog(program, sizeof(log), NULL, log);
		fprintf(stderr, "Shader link error: %s\n", log);
		glDeleteProgram(program);
		program = 0;
	}

	glDeleteShader(vert);
	glDeleteShader(frag);

	return program;
}

void shader_set_mat4(GLuint program, const char *name, const float *matrix)
{
	GLint loc = glGetUniformLocation(program, name);
	glUniformMatrix4fv(loc, 1, GL_FALSE, matrix);
}

void shader_set_vec3(GLuint program, const char *name, float x, float y,
		     float z)
{
	GLint loc = glGetUniformLocation(program, name);
	glUniform3f(loc, x, y, z);
}

void shader_set_float(GLuint program, const char *name, float value)
{
	GLint loc = glGetUniformLocation(program, name);
	glUniform1f(loc, value);
}

void shader_set_int(GLuint program, const char *name, int value)
{
	GLint loc = glGetUniformLocation(program, name);
	glUniform1i(loc, value);
}
