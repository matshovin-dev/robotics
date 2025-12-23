/**
 * @file mesh.h
 * @brief Modern OpenGL mesh (VBO/VAO) management
 */

#ifndef MESH_H
#define MESH_H

#include <OpenGL/gl3.h>

/**
 * struct mesh - GPU-ready mesh with VBO/VAO
 */
struct mesh {
	GLuint vao;
	GLuint vbo;
	GLuint ebo;
	int vertex_count;
	int index_count;
};

/**
 * mesh_create - Create mesh from vertex data
 * @vertices: Interleaved vertex data (pos.xyz + normal.xyz = 6 floats per vertex)
 * @vertex_count: Number of vertices
 * @indices: Optional index buffer (NULL for non-indexed)
 * @index_count: Number of indices (0 if no indices)
 */
struct mesh *mesh_create(const float *vertices, int vertex_count,
			 const unsigned int *indices, int index_count);

/**
 * mesh_create_from_obj - Load OBJ and create mesh
 * @path: Path to OBJ file
 */
struct mesh *mesh_create_from_obj(const char *path);

/**
 * mesh_draw - Draw mesh
 */
void mesh_draw(const struct mesh *m);

/**
 * mesh_free - Free mesh resources
 */
void mesh_free(struct mesh *m);

/**
 * mesh_create_ground - Create a ground plane mesh
 * @size: Half-size of the plane
 */
struct mesh *mesh_create_ground(float size);

#endif /* MESH_H */
