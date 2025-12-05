#ifndef OBJ_LOADER_H
#define OBJ_LOADER_H

// Struktur for 3D vertex
typedef struct {
  float x, y, z;
} Vec3;

// Struktur for face (trekant)
typedef struct {
  int v[3];  // Vertex indices
  int vn[3]; // Normal indices
} Face;

// Struktur for hele OBJ-modellen
typedef struct {
  Vec3 *vertices;   // Array av vertices
  Vec3 *normals;    // Array av normaler
  Face *faces;      // Array av faces
  int num_vertices; // Antall vertices
  int num_normals;  // Antall normaler
  int num_faces;    // Antall faces
} ObjModel;

// Last OBJ-fil
// Returnerer peker til ObjModel, eller NULL ved feil
ObjModel *obj_load(const char *filename);

// Frigjør minne
void obj_free(ObjModel *model);

// Tegn modellen med OpenGL
void obj_draw(const ObjModel *model);

// Debug: Print info om modellen
void obj_print_info(const ObjModel *model);

#endif // OBJ_LOADER_H