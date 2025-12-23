/**
 * @file ssao.c
 * @brief Screen-Space Ambient Occlusion implementation
 */

#include "ssao.h"
#include "shader.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Simple random float [0, 1] */
static float randf(void)
{
	return (float)rand() / (float)RAND_MAX;
}

/* Linear interpolation */
static float lerp(float a, float b, float t)
{
	return a + t * (b - a);
}

/**
 * Generate hemisphere kernel samples
 */
static void generate_kernel(float *kernel, int count)
{
	for (int i = 0; i < count; i++) {
		/* Random point in hemisphere */
		float x = randf() * 2.0f - 1.0f;
		float y = randf() * 2.0f - 1.0f;
		float z = randf();  /* Only positive Z (hemisphere) */

		/* Normalize */
		float len = sqrtf(x*x + y*y + z*z);
		x /= len;
		y /= len;
		z /= len;

		/* Scale to be within hemisphere */
		float scale = (float)i / (float)count;
		scale = lerp(0.1f, 1.0f, scale * scale);

		kernel[i * 3 + 0] = x * scale;
		kernel[i * 3 + 1] = y * scale;
		kernel[i * 3 + 2] = z * scale;
	}
}

/**
 * Generate noise texture (4x4 random rotation vectors)
 */
static GLuint generate_noise_texture(void)
{
	float noise[16 * 3];  /* 4x4 texture, RGB */

	for (int i = 0; i < 16; i++) {
		/* Random rotation around Z axis */
		noise[i * 3 + 0] = randf() * 2.0f - 1.0f;
		noise[i * 3 + 1] = randf() * 2.0f - 1.0f;
		noise[i * 3 + 2] = 0.0f;
	}

	GLuint tex;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 4, 4, 0, GL_RGB, GL_FLOAT, noise);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	return tex;
}

/**
 * Create fullscreen quad
 */
static void create_quad(GLuint *vao, GLuint *vbo)
{
	float vertices[] = {
		/* pos.xy, tex.xy */
		-1.0f,  1.0f, 0.0f, 1.0f,
		-1.0f, -1.0f, 0.0f, 0.0f,
		 1.0f,  1.0f, 1.0f, 1.0f,
		 1.0f, -1.0f, 1.0f, 0.0f,
	};

	glGenVertexArrays(1, vao);
	glGenBuffers(1, vbo);
	glBindVertexArray(*vao);
	glBindBuffer(GL_ARRAY_BUFFER, *vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

	glBindVertexArray(0);
}

/**
 * Create G-buffer framebuffer
 */
static int create_gbuffer(struct ssao_state *state, int width, int height)
{
	glGenFramebuffers(1, &state->gbuffer_fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, state->gbuffer_fbo);

	/* Position buffer (view-space) */
	glGenTextures(1, &state->gbuffer_position);
	glBindTexture(GL_TEXTURE_2D, state->gbuffer_position);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, state->gbuffer_position, 0);

	/* Normal buffer (view-space) */
	glGenTextures(1, &state->gbuffer_normal);
	glBindTexture(GL_TEXTURE_2D, state->gbuffer_normal);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, state->gbuffer_normal, 0);

	/* Depth renderbuffer */
	glGenRenderbuffers(1, &state->gbuffer_depth);
	glBindRenderbuffer(GL_RENDERBUFFER, state->gbuffer_depth);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, state->gbuffer_depth);

	/* Tell OpenGL which color attachments to use */
	GLenum attachments[2] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
	glDrawBuffers(2, attachments);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		fprintf(stderr, "G-buffer framebuffer incomplete\n");
		return -1;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return 0;
}

/**
 * Create SSAO framebuffer
 */
static int create_ssao_fbo(GLuint *fbo, GLuint *texture, int width, int height)
{
	glGenFramebuffers(1, fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, *fbo);

	glGenTextures(1, texture);
	glBindTexture(GL_TEXTURE_2D, *texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, *texture, 0);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		fprintf(stderr, "SSAO framebuffer incomplete\n");
		return -1;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return 0;
}

int ssao_init(struct ssao_state *state, int width, int height)
{
	/* Default settings */
	state->enabled = 1;
	state->radius = 15.0f;
	state->bias = 0.5f;
	state->intensity = 1.5f;

	/* Generate kernel samples */
	generate_kernel(state->kernel, 64);

	/* Generate noise texture */
	state->noise_texture = generate_noise_texture();

	/* Create framebuffers */
	if (create_gbuffer(state, width, height) < 0)
		return -1;

	if (create_ssao_fbo(&state->ssao_fbo, &state->ssao_texture, width, height) < 0)
		return -1;

	if (create_ssao_fbo(&state->ssao_blur_fbo, &state->ssao_blur_texture, width, height) < 0)
		return -1;

	/* Create fullscreen quad */
	create_quad(&state->quad_vao, &state->quad_vbo);

	/* Load shaders */
	state->gbuffer_shader = shader_load("shaders/gbuffer.vert", "shaders/gbuffer.frag");
	state->ssao_shader = shader_load("shaders/ssao.vert", "shaders/ssao.frag");
	state->blur_shader = shader_load("shaders/ssao.vert", "shaders/ssao_blur.frag");

	if (!state->gbuffer_shader || !state->ssao_shader || !state->blur_shader) {
		fprintf(stderr, "Failed to load SSAO shaders\n");
		return -1;
	}

	/* Set kernel samples in SSAO shader */
	glUseProgram(state->ssao_shader);
	for (int i = 0; i < 64; i++) {
		char name[32];
		snprintf(name, sizeof(name), "samples[%d]", i);
		GLint loc = glGetUniformLocation(state->ssao_shader, name);
		glUniform3f(loc, state->kernel[i*3], state->kernel[i*3+1], state->kernel[i*3+2]);
	}

	printf("SSAO initialized (press 'O' to toggle)\n");
	return 0;
}

void ssao_cleanup(struct ssao_state *state)
{
	glDeleteFramebuffers(1, &state->gbuffer_fbo);
	glDeleteTextures(1, &state->gbuffer_position);
	glDeleteTextures(1, &state->gbuffer_normal);
	glDeleteRenderbuffers(1, &state->gbuffer_depth);

	glDeleteFramebuffers(1, &state->ssao_fbo);
	glDeleteTextures(1, &state->ssao_texture);

	glDeleteFramebuffers(1, &state->ssao_blur_fbo);
	glDeleteTextures(1, &state->ssao_blur_texture);

	glDeleteTextures(1, &state->noise_texture);

	glDeleteVertexArrays(1, &state->quad_vao);
	glDeleteBuffers(1, &state->quad_vbo);

	glDeleteProgram(state->gbuffer_shader);
	glDeleteProgram(state->ssao_shader);
	glDeleteProgram(state->blur_shader);
}

void ssao_resize(struct ssao_state *state, int width, int height)
{
	/* Resize G-buffer textures */
	glBindTexture(GL_TEXTURE_2D, state->gbuffer_position);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, NULL);

	glBindTexture(GL_TEXTURE_2D, state->gbuffer_normal);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, NULL);

	glBindRenderbuffer(GL_RENDERBUFFER, state->gbuffer_depth);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);

	/* Resize SSAO textures */
	glBindTexture(GL_TEXTURE_2D, state->ssao_texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, NULL);

	glBindTexture(GL_TEXTURE_2D, state->ssao_blur_texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, NULL);
}

void ssao_render_quad(struct ssao_state *state)
{
	glBindVertexArray(state->quad_vao);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);
}
