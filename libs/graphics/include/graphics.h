#ifndef GRAPHICS_H
#define GRAPHICS_H

// Initialiser grafikk-system
void graphics_init(void);

// Setup 2D orthographic projection for tekst
void graphics_setup_2d(int width, int height);

// Setup 3D projection for grafikk
void graphics_setup_3d(void);

// Tegn en roterende trekant
void graphics_draw_rotating_triangle(float time);
void graphics_draw_triangle_outline(float time);

void graphics_test_line_polygon_rendering(float time);

// Tegn en sirkel
void graphics_draw_circle(float x, float y, float radius, float r, float g, float b);

// Linje
void graphics_draw_line(float x1, float y1, float z1, float x2, float y2, float z2);

#endif // GRAPHICS_H