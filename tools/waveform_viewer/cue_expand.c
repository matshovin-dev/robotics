#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    int beat;
    char text[256];
} Cue;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <cue.txt> <coreo.txt>\n", argv[0]);
        fprintf(stderr, "\nExpands cue file to full beat list.\n");
        fprintf(stderr, "Example input (cue.txt):\n");
        fprintf(stderr, "  11 .\n");
        fprintf(stderr, "  27 start\n");
        fprintf(stderr, "  43 .\n");
        fprintf(stderr, "\nExample output (coreo.txt):\n");
        fprintf(stderr, "  1\n");
        fprintf(stderr, "  2\n");
        fprintf(stderr, "  ...\n");
        fprintf(stderr, "  11 .\n");
        fprintf(stderr, "  ...\n");
        fprintf(stderr, "  27 start\n");
        fprintf(stderr, "  ...\n");
        return 1;
    }

    const char* input_path = argv[1];
    const char* output_path = argv[2];

    // First pass: read cues and find max beat
    FILE* in = fopen(input_path, "r");
    if (!in) {
        fprintf(stderr, "Could not open input file: %s\n", input_path);
        return 1;
    }

    Cue* cues = NULL;
    int num_cues = 0;
    int max_beat = 0;
    char line[512];

    while (fgets(line, sizeof(line), in)) {
        if (line[0] == '\n' || line[0] == '\0') continue;

        // Skip leading spaces (indent)
        char* p = line;
        while (*p == ' ') p++;

        // Parse beat number
        int beat = atoi(p);
        if (beat <= 0) continue;

        // Skip past the number
        while (*p >= '0' && *p <= '9') p++;

        // Skip whitespace after number
        while (*p == ' ' || *p == '\t') p++;

        // Rest is the text (strip newline)
        char* nl = strchr(p, '\n');
        if (nl) *nl = '\0';

        // Add to cues array
        cues = realloc(cues, (num_cues + 1) * sizeof(Cue));
        cues[num_cues].beat = beat;
        strncpy(cues[num_cues].text, p, sizeof(cues[num_cues].text) - 1);
        cues[num_cues].text[sizeof(cues[num_cues].text) - 1] = '\0';
        num_cues++;

        if (beat > max_beat) max_beat = beat;
    }
    fclose(in);

    if (num_cues == 0) {
        fprintf(stderr, "No cues found in input file\n");
        return 1;
    }

    // Write output
    FILE* out = fopen(output_path, "w");
    if (!out) {
        fprintf(stderr, "Could not open output file: %s\n", output_path);
        free(cues);
        return 1;
    }

    for (int beat = 1; beat <= max_beat; beat++) {
        // Check if this beat has a cue
        const char* text = NULL;
        for (int i = 0; i < num_cues; i++) {
            if (cues[i].beat == beat) {
                text = cues[i].text;
                break;
            }
        }

        if (text && text[0] != '\0') {
            fprintf(out, "%d %s\n", beat, text);
        } else {
            fprintf(out, "%d \n", beat);  // Empty space after beat number
        }
    }

    fclose(out);
    free(cues);

    fprintf(stderr, "Expanded %d cues to %d beats -> %s\n", num_cues, max_beat, output_path);
    return 0;
}
