/**
 * @file ssao.h
 * @brief Screen-Space Ambient Occlusion
 */

#ifndef SSAO_H
#define SSAO_H

#include <OpenGL/gl3.h>

/**
 * SSAO configuration and state
 */
struct ssao_state {
	/* G-buffer */
	GLuint gbuffer_fbo;
	GLuint gbuffer_position;  /* View-space positions */
	GLuint gbuffer_normal;    /* View-space normals */
	GLuint gbuffer_depth;     /* Depth renderbuffer */

	/* SSAO */
	GLuint ssao_fbo;
	GLuint ssao_texture;

	/* SSAO blur */
	GLuint ssao_blur_fbo;
	GLuint ssao_blur_texture;

	/* Noise texture */
	GLuint noise_texture;

	/* Kernel samples */
	float kernel[64 * 3];  /* 64 samples, xyz each */

	/* Shaders */
	GLuint gbuffer_shader;
	GLuint ssao_shader;
	GLuint blur_shader;

	/* Fullscreen quad */
	GLuint quad_vao;
	GLuint quad_vbo;

	/* Settings */
	int enabled;
	float radius;
	float bias;
	float intensity;
};

/**
 * Initialize SSAO system
 */
int ssao_init(struct ssao_state *state, int width, int height);

/**
 * Cleanup SSAO resources
 */
void ssao_cleanup(struct ssao_state *state);

/**
 * Resize SSAO buffers
 */
void ssao_resize(struct ssao_state *state, int width, int height);

/**
 * Render fullscreen quad
 */
void ssao_render_quad(struct ssao_state *state);

#endif /* SSAO_H */
