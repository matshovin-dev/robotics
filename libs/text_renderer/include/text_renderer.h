#ifndef TEXT_RENDERER_H
#define TEXT_RENDERER_H

// Initialiser tekst-systemet med en font
// Returnerer 1 ved suksess, 0 ved feil
int text_init(const char* font_path, int font_size);

// Tegn tekst på skjermen
void text_draw(const char* text, float x, float y, float r, float g, float b);

// Rydd opp ressurser
void text_cleanup(void);

#endif // TEXT_RENDERER_H