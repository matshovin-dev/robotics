#include "text_renderer.h"
#include <GLFW/glfw3.h>
#include <ft2build.h>
#include <stdio.h>
#include FT_FREETYPE_H

// Privat struktur (skjult fra brukere av biblioteket)
struct Character {
  unsigned int textureID; // OpenGL texture ID for denne bokstavens bitmap
  int width;              // Bredde på bokstav-bitmap i pixels
  int height;             // Høyde på bokstav-bitmap i pixels
  int bearingX;           // Horizontal offset fra cursor til venstre kant av glyph
  int bearingY;           // Vertical offset fra baseline til topp av glyph
  int advance;            // Horizontal avstand til neste bokstav (i 1/64 pixels)
};

// Private globale variabler
static struct Character characters[128];
static FT_Library ft;
static FT_Face face;
static int initialized = 0;

int text_init(const char *font_path, int font_size) {
  if (initialized) {
    fprintf(stderr, "Text renderer already initialized\n");
    return 0;
  }

  if (FT_Init_FreeType(&ft)) {
    fprintf(stderr, "ERROR: Could not init FreeType Library\n");
    return 0;
  }

  if (FT_New_Face(ft, font_path, 0, &face)) {
    fprintf(stderr, "ERROR: Failed to load font: %s\n", font_path);
    FT_Done_FreeType(ft);
    return 0;
  }

  FT_Set_Pixel_Sizes(face, 0, font_size);

  // Generer teksturer for alle ASCII tegn
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  for (unsigned char c = 0; c < 128; c++) {
    if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
      fprintf(stderr, "WARNING: Failed to load Glyph %c\n", c);
      continue;
    }

    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, face->glyph->bitmap.width, face->glyph->bitmap.rows, 0, GL_ALPHA, GL_UNSIGNED_BYTE,
                 face->glyph->bitmap.buffer);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    struct Character character = {texture,
                                  face->glyph->bitmap.width,
                                  face->glyph->bitmap.rows,
                                  face->glyph->bitmap_left,
                                  face->glyph->bitmap_top,
                                  face->glyph->advance.x};
    characters[c] = character;
  }

  initialized = 1;
  printf("Text renderer initialized with font: %s\n", font_path);
  return 1;
}

void text_draw(const char *text, float x, float y, float r, float g, float b) {
  if (!initialized) {
    fprintf(stderr, "ERROR: Text renderer not initialized\n");
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glColor3f(r, g, b);

  // Finn maksimal bearingY for baseline
  int max_bearingY = 0;
  for (const char *p = text; *p; p++) {
    struct Character ch = characters[(int)*p];
    if (ch.bearingY > max_bearingY) {
      max_bearingY = ch.bearingY;
    }
  }

  // Tegn hver bokstav
  for (const char *p = text; *p; p++) {
    struct Character ch = characters[(int)*p];

    float xpos = x + ch.bearingX;
    float ypos = y + (max_bearingY - ch.bearingY);

    float w = ch.width;
    float h = ch.height;

    glBindTexture(GL_TEXTURE_2D, ch.textureID);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(xpos, ypos + h);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(xpos + w, ypos + h);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(xpos + w, ypos);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(xpos, ypos);
    glEnd();

    x += (ch.advance >> 6);
  }

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_BLEND);
}

void text_cleanup(void) {
  if (!initialized) {
    return;
  }

  // Slett alle teksturer
  for (int i = 0; i < 128; i++) {
    if (characters[i].textureID != 0) {
      glDeleteTextures(1, &characters[i].textureID);
    }
  }

  FT_Done_Face(face);
  FT_Done_FreeType(ft);
  initialized = 0;

  printf("Text renderer cleaned up\n");
}