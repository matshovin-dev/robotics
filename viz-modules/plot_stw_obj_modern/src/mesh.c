/**
 * @file mesh.c
 * @brief Modern OpenGL mesh (VBO/VAO) management
 */

#include "mesh.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct mesh *mesh_create(const float *vertices, int vertex_count,
			 const unsigned int *indices, int index_count)
{
	struct mesh *m = malloc(sizeof(*m));
	if (!m)
		return NULL;

	m->vertex_count = vertex_count;
	m->index_count = index_count;

	glGenVertexArrays(1, &m->vao);
	glGenBuffers(1, &m->vbo);

	glBindVertexArray(m->vao);

	/* Upload vertex data */
	glBindBuffer(GL_ARRAY_BUFFER, m->vbo);
	glBufferData(GL_ARRAY_BUFFER, vertex_count * 6 * sizeof(float),
		     vertices, GL_STATIC_DRAW);

	/* Position attribute (location = 0) */
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
			      (void *)0);
	glEnableVertexAttribArray(0);

	/* Normal attribute (location = 1) */
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
			      (void *)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	/* Optional index buffer */
	if (indices && index_count > 0) {
		glGenBuffers(1, &m->ebo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER,
			     index_count * sizeof(unsigned int), indices,
			     GL_STATIC_DRAW);
	} else {
		m->ebo = 0;
	}

	glBindVertexArray(0);

	return m;
}

/**
 * Simple OBJ loader - same approach as legacy obj_loader.c
 * Stores vertices/normals/faces separately, then converts to VBO
 */
struct mesh *mesh_create_from_obj(const char *path)
{
	FILE *f = fopen(path, "r");
	if (!f) {
		fprintf(stderr, "Failed to open OBJ: %s\n", path);
		return NULL;
	}

	/* Dynamic arrays - start with initial capacity */
	int v_cap = 1000, vn_cap = 1000, f_cap = 1000;
	int v_count = 0, vn_count = 0, f_count = 0;

	float *verts = malloc(v_cap * 3 * sizeof(float));   /* x,y,z per vertex */
	float *norms = malloc(vn_cap * 3 * sizeof(float));  /* x,y,z per normal */
	int *faces = malloc(f_cap * 6 * sizeof(int));       /* v0,vn0,v1,vn1,v2,vn2 per face */

	if (!verts || !norms || !faces) {
		free(verts);
		free(norms);
		free(faces);
		fclose(f);
		return NULL;
	}

	char line[256];
	while (fgets(line, sizeof(line), f)) {
		if (line[0] == '#' || line[0] == '\n')
			continue;

		/* Parse vertex: v x y z */
		if (line[0] == 'v' && line[1] == ' ') {
			float x, y, z;
			if (sscanf(line, "v %f %f %f", &x, &y, &z) == 3) {
				if (v_count >= v_cap) {
					v_cap *= 2;
					verts = realloc(verts, v_cap * 3 * sizeof(float));
				}
				verts[v_count * 3 + 0] = x;
				verts[v_count * 3 + 1] = y;
				verts[v_count * 3 + 2] = z;
				v_count++;
			}
		}
		/* Parse normal: vn x y z */
		else if (line[0] == 'v' && line[1] == 'n') {
			float x, y, z;
			if (sscanf(line, "vn %f %f %f", &x, &y, &z) == 3) {
				if (vn_count >= vn_cap) {
					vn_cap *= 2;
					norms = realloc(norms, vn_cap * 3 * sizeof(float));
				}
				norms[vn_count * 3 + 0] = x;
				norms[vn_count * 3 + 1] = y;
				norms[vn_count * 3 + 2] = z;
				vn_count++;
			}
		}
		/* Parse face: f v//vn v//vn v//vn */
		else if (line[0] == 'f' && line[1] == ' ') {
			int v0, vn0, v1, vn1, v2, vn2;
			if (sscanf(line, "f %d//%d %d//%d %d//%d",
				   &v0, &vn0, &v1, &vn1, &v2, &vn2) == 6) {
				if (f_count >= f_cap) {
					f_cap *= 2;
					faces = realloc(faces, f_cap * 6 * sizeof(int));
				}
				/* Convert to 0-based indices */
				faces[f_count * 6 + 0] = v0 - 1;
				faces[f_count * 6 + 1] = vn0 - 1;
				faces[f_count * 6 + 2] = v1 - 1;
				faces[f_count * 6 + 3] = vn1 - 1;
				faces[f_count * 6 + 4] = v2 - 1;
				faces[f_count * 6 + 5] = vn2 - 1;
				f_count++;
			}
		}
	}
	fclose(f);

	printf("OBJ parsed: %d vertices, %d normals, %d faces\n",
	       v_count, vn_count, f_count);

	/* Now convert to interleaved VBO format */
	int total_verts = f_count * 3;
	float *vbo_data = malloc(total_verts * 6 * sizeof(float));
	if (!vbo_data) {
		free(verts);
		free(norms);
		free(faces);
		return NULL;
	}

	int idx = 0;
	for (int i = 0; i < f_count; i++) {
		for (int j = 0; j < 3; j++) {
			int vi = faces[i * 6 + j * 2];      /* vertex index */
			int ni = faces[i * 6 + j * 2 + 1];  /* normal index */

			/* Position */
			if (vi >= 0 && vi < v_count) {
				vbo_data[idx++] = verts[vi * 3 + 0];
				vbo_data[idx++] = verts[vi * 3 + 1];
				vbo_data[idx++] = verts[vi * 3 + 2];
			} else {
				vbo_data[idx++] = 0;
				vbo_data[idx++] = 0;
				vbo_data[idx++] = 0;
			}

			/* Normal */
			if (ni >= 0 && ni < vn_count) {
				vbo_data[idx++] = norms[ni * 3 + 0];
				vbo_data[idx++] = norms[ni * 3 + 1];
				vbo_data[idx++] = norms[ni * 3 + 2];
			} else {
				vbo_data[idx++] = 0;
				vbo_data[idx++] = 1;
				vbo_data[idx++] = 0;
			}
		}
	}

	free(verts);
	free(norms);
	free(faces);

	struct mesh *m = mesh_create(vbo_data, total_verts, NULL, 0);
	free(vbo_data);

	if (m)
		printf("Loaded OBJ: %s (%d triangles)\n", path, f_count);

	return m;
}

void mesh_draw(const struct mesh *m)
{
	if (!m)
		return;

	glBindVertexArray(m->vao);

	if (m->ebo && m->index_count > 0) {
		glDrawElements(GL_TRIANGLES, m->index_count, GL_UNSIGNED_INT, 0);
	} else {
		glDrawArrays(GL_TRIANGLES, 0, m->vertex_count);
	}

	glBindVertexArray(0);
}

void mesh_free(struct mesh *m)
{
	if (!m)
		return;

	glDeleteVertexArrays(1, &m->vao);
	glDeleteBuffers(1, &m->vbo);
	if (m->ebo)
		glDeleteBuffers(1, &m->ebo);

	free(m);
}

struct mesh *mesh_create_ground(float size)
{
	/* Ground plane with normal pointing up - CCW winding for top visibility */
	float vertices[] = {
		/* pos.xyz, normal.xyz */
		-size, 0.0f, -size, 0.0f, 1.0f, 0.0f,
		 size, 0.0f,  size, 0.0f, 1.0f, 0.0f,
		 size, 0.0f, -size, 0.0f, 1.0f, 0.0f,

		-size, 0.0f, -size, 0.0f, 1.0f, 0.0f,
		-size, 0.0f,  size, 0.0f, 1.0f, 0.0f,
		 size, 0.0f,  size, 0.0f, 1.0f, 0.0f,
	};

	return mesh_create(vertices, 6, NULL, 0);
}
