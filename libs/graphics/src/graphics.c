#include "graphics.h"
#include <GLFW/glfw3.h>
#include <math.h>

void graphics_init(void) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void graphics_setup_2d(int width, int height) {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, width, height, 0, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void graphics_setup_3d(void) {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-1, 1, -1, 1, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void graphics_draw_rotating_triangle(float time) {
  glPushMatrix();
  glRotatef(time * 50.0f, 3.0f, 3.0f, 1.0f); // <-- FIKSET

  glBegin(GL_TRIANGLES);
  glColor3f(1.0f, 0.5f, 0.0f);
  glVertex2f(0.0f, 0.3f);
  glVertex2f(-1.25f, -0.15f);
  glVertex2f(1.25f, -1.15f);
  glEnd();

  glPopMatrix();
}

void graphics_draw_triangle_outline(float time) {
  glLineWidth(3.0f); // Litt tykkere

  glPushMatrix();
  glRotatef(time * 50.0f, 3.0f, 3.0f, 1.0f); // <-- FIKSET

  glBegin(GL_LINE_LOOP);
  glColor3f(1.0f, 0.5f, 1.0f);
  glVertex2f(0.0f, 0.3f);
  glVertex2f(-1.25f, -0.15f);
  glVertex2f(1.25f, -1.15f);
  glEnd();

  glPopMatrix();

  glLineWidth(1.0f); // Reset
}

void graphics_test_line_polygon_rendering(float time) {
  // Test 1: Polygon først, så linjer (vanlig rekkefølge)
  glPushMatrix();
  glTranslatef(-0.5f, 0.5f, 0.0f);

  // Tegn omriss oppå
  glLineWidth(3.0f);
  glBegin(GL_LINE_LOOP);
  glColor3f(1.0f, 1.0f, 0.0f); // Gul omriss
  glVertex2f(0.0f, 0.2f);
  glVertex2f(-0.15f, -0.1f);
  glVertex2f(0.15f, -0.1f);
  glEnd();
  glLineWidth(1.0f);

  glPopMatrix();

  // Test 2: Linjer først, så polygon (motsatt rekkefølge)
  glPushMatrix();
  glTranslatef(0.5f, 0.5f, 0.0f);

  // Tegn omriss først
  glLineWidth(3.0f);
  glBegin(GL_LINE_LOOP);
  glColor3f(1.0f, 1.0f, 0.0f); // Gul omriss
  glVertex2f(0.0f, 0.2f);
  glVertex2f(-0.15f, -0.1f);
  glVertex2f(0.15f, -0.1f);
  glEnd();
  glLineWidth(1.0f);

  // Tegn fyllt trekant etterpå
  glBegin(GL_TRIANGLES);
  glColor3f(0.0f, 0.5f, 1.0f); // Blå
  glVertex2f(0.0f, 0.2f);
  glVertex2f(-0.15f, -0.1f);
  glVertex2f(0.15f, -0.1f);
  glEnd();

  glPopMatrix();

  // Test 3: Overlappende polygoner og linjer
  glPushMatrix();
  glTranslatef(-0.5f, -0.3f, 0.0f);
  glRotatef(time * 30.0f, 0.0f, 0.0f, 1.0f);

  // Stor blå trekant (bakgrunn)
  glBegin(GL_TRIANGLES);
  glColor3f(0.2f, 0.3f, 0.8f);
  glVertex2f(0.0f, 0.25f);
  glVertex2f(-0.2f, -0.15f);
  glVertex2f(0.2f, -0.15f);
  glEnd();

  // Mindre rød trekant oppå
  glBegin(GL_TRIANGLES);
  glColor3f(0.8f, 0.2f, 0.2f);
  glVertex2f(0.0f, 0.15f);
  glVertex2f(-0.1f, -0.05f);
  glVertex2f(0.1f, -0.05f);
  glEnd();

  // Hvit omriss rundt alt
  glLineWidth(5.0f);
  glBegin(GL_LINE_LOOP);
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex2f(0.0f, 0.25f);
  glVertex2f(-0.2f, -0.15f);
  glVertex2f(0.2f, -0.15f);
  glEnd();
  glLineWidth(1.0f);

  glPopMatrix();

  // Test 4: Med depth testing (Z-buffer)
  glPushMatrix();
  glTranslatef(0.5f, -0.3f, 0.0f);

  glEnable(GL_DEPTH_TEST);

  // Trekant med depth 0.5
  glBegin(GL_TRIANGLES);
  glColor3f(0.2f, 0.8f, 0.2f);  // Grønn
  glVertex3f(0.0f, 0.2f, 0.5f); // Z = 0.5
  glVertex3f(-0.15f, -0.1f, 0.5f);
  glVertex3f(0.15f, -0.1f, 0.5f);
  glEnd();

  // Linjer med depth 0.0 (nærmere kamera)
  glLineWidth(3.0f);
  glBegin(GL_LINE_LOOP);
  glColor3f(1.0f, 0.0f, 1.0f);   // Magenta
  glVertex3f(0.0f, 0.15f, 0.0f); // Z = 0.0 (nærmere)
  glVertex3f(-0.1f, -0.05f, 0.0f);
  glVertex3f(0.1f, -0.05f, 0.0f);
  glEnd();
  glLineWidth(1.0f);

  glDisable(GL_DEPTH_TEST);

  glPopMatrix();
}

void graphics_draw_circle(float x, float y, float radius, float r, float g, float b) {
  const int segments = 32;

  glBegin(GL_TRIANGLE_FAN);
  glColor3f(r, g, b);
  glVertex2f(x, y);

  for (int i = 0; i <= segments; i++) {
    float angle = 2.0f * 3.14159f * i / segments;
    float dx = radius * cosf(angle);
    float dy = radius * sinf(angle);
    glVertex2f(x + dx, y + dy);
  }
  glEnd();
}

void graphics_draw_line(float x1, float y1, float z1, float x2, float y2, float z2) {
  glDisable(GL_LIGHTING); // Hvis du vil ha fast farge
  glLineWidth(2.0f);      // Tykkelse

  glBegin(GL_LINES);
  glColor3f(1.0f, 1.0f, 0.0f); // Gul farge
  glVertex3f(x1, y1, z1);      // Startpunkt
  glVertex3f(x2, y2, z2);      // Sluttpunkt
  glEnd();

  glLineWidth(1.0f);
  glEnable(GL_LIGHTING);
}