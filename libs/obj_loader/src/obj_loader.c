#include "obj_loader.h"
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 256
#define INITIAL_CAPACITY 1000

ObjModel *obj_load(const char *filename) {
  FILE *file = fopen(filename, "r");
  if (!file) {
    fprintf(stderr, "ERROR: Could not open OBJ file: %s\n", filename);
    return NULL;
  }

  ObjModel *model = (ObjModel *)malloc(sizeof(ObjModel));
  if (!model) {
    fclose(file);
    return NULL;
  }

  // Initialiser med kapasitet
  int vertex_capacity = INITIAL_CAPACITY;
  int normal_capacity = INITIAL_CAPACITY;
  int face_capacity = INITIAL_CAPACITY;

  model->vertices = (Vec3 *)malloc(vertex_capacity * sizeof(Vec3));
  model->normals = (Vec3 *)malloc(normal_capacity * sizeof(Vec3));
  model->faces = (Face *)malloc(face_capacity * sizeof(Face));

  model->num_vertices = 0;
  model->num_normals = 0;
  model->num_faces = 0;

  char line[MAX_LINE];
  while (fgets(line, sizeof(line), file)) {
    // Skip comments og tomme linjer
    if (line[0] == '#' || line[0] == '\n') {
      continue;
    }

    // Parse vertex: v x y z
    if (line[0] == 'v' && line[1] == ' ') {
      Vec3 v;
      if (sscanf(line, "v %f %f %f", &v.x, &v.y, &v.z) == 3) {
        // Utvid array om nødvendig
        if (model->num_vertices >= vertex_capacity) {
          vertex_capacity *= 2;
          model->vertices = (Vec3 *)realloc(model->vertices, vertex_capacity * sizeof(Vec3));
        }
        model->vertices[model->num_vertices++] = v;
      }
    }
    // Parse normal: vn x y z
    else if (line[0] == 'v' && line[1] == 'n') {
      Vec3 vn;
      if (sscanf(line, "vn %f %f %f", &vn.x, &vn.y, &vn.z) == 3) {
        if (model->num_normals >= normal_capacity) {
          normal_capacity *= 2;
          model->normals = (Vec3 *)realloc(model->normals, normal_capacity * sizeof(Vec3));
        }
        model->normals[model->num_normals++] = vn;
      }
    }
    // Parse face: f v1//vn1 v2//vn2 v3//vn3
    else if (line[0] == 'f' && line[1] == ' ') {
      Face f;
      // OBJ bruker 1-basert indeksering, vi konverterer til 0-basert
      if (sscanf(line, "f %d//%d %d//%d %d//%d", &f.v[0], &f.vn[0], &f.v[1], &f.vn[1], &f.v[2], &f.vn[2]) == 6) {
        // Konverter til 0-basert indeksering
        f.v[0]--;
        f.v[1]--;
        f.v[2]--;
        f.vn[0]--;
        f.vn[1]--;
        f.vn[2]--;

        if (model->num_faces >= face_capacity) {
          face_capacity *= 2;
          model->faces = (Face *)realloc(model->faces, face_capacity * sizeof(Face));
        }
        model->faces[model->num_faces++] = f;
      }
    }
  }

  fclose(file);

  // Trim arrays til faktisk størrelse
  model->vertices = (Vec3 *)realloc(model->vertices, model->num_vertices * sizeof(Vec3));
  model->normals = (Vec3 *)realloc(model->normals, model->num_normals * sizeof(Vec3));
  model->faces = (Face *)realloc(model->faces, model->num_faces * sizeof(Face));

  printf("Loaded OBJ: %d vertices, %d normals, %d faces\n", model->num_vertices, model->num_normals, model->num_faces);

  return model;
}

void obj_free(ObjModel *model) {
  if (model) {
    free(model->vertices);
    free(model->normals);
    free(model->faces);
    free(model);
  }
}

void obj_draw(const ObjModel *model) {
  if (!model) {
    return;
  }

  glBegin(GL_TRIANGLES);

  for (int i = 0; i < model->num_faces; i++) {
    Face face = model->faces[i];

    for (int j = 0; j < 3; j++) {
      // Sett normal
      if (face.vn[j] >= 0 && face.vn[j] < model->num_normals) {
        Vec3 n = model->normals[face.vn[j]];
        glNormal3f(n.x, n.y, n.z);
      }

      // Sett vertex
      if (face.v[j] >= 0 && face.v[j] < model->num_vertices) {
        Vec3 v = model->vertices[face.v[j]];
        glVertex3f(v.x, v.y, v.z);
      }
    }
  }

  glEnd();
}

void obj_print_info(const ObjModel *model) {
  if (!model) {
    printf("Model is NULL\n");
    return;
  }

  printf("OBJ Model Info:\n");
  printf("  Vertices: %d\n", model->num_vertices);
  printf("  Normals:  %d\n", model->num_normals);
  printf("  Faces:    %d\n", model->num_faces);

  if (model->num_vertices > 0) {
    printf("  First vertex: (%.2f, %.2f, %.2f)\n", model->vertices[0].x, model->vertices[0].y, model->vertices[0].z);
  }
}