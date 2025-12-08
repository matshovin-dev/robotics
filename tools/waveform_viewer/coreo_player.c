#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#define MAX_BEATS 2000
#define MAX_NAME_LEN 64

// Coreo line data
typedef struct {
    char comment[MAX_NAME_LEN];
    char move[MAX_NAME_LEN];
    int fader;  // 0 = no fade, 4 = f4, etc.
} CoreoLine;

// Global state
CoreoLine coreo[MAX_BEATS];
int max_beat = 0;
double bpm = 120.0;
double master_phase = 0.0;

// Current playback state
char current_move[MAX_NAME_LEN] = "";
char next_move[MAX_NAME_LEN] = "";
int fade_start_beat = 0;
int fade_length = 0;

// Parse fader string like "f4" -> 4, "f8" -> 8
int parse_fader(const char* str) {
    if (!str || str[0] == '\0') return 0;
    if (str[0] == 'f' || str[0] == 's' || str[0] == 'i' || str[0] == 'o') {
        return atoi(str + 1);
    }
    return 0;
}

// Load coreo file
int load_coreo(const char* filename) {
    FILE* f = fopen(filename, "r");
    if (!f) {
        fprintf(stderr, "Could not open coreo file: %s\n", filename);
        return 0;
    }

    // Initialize all entries
    for (int i = 0; i < MAX_BEATS; i++) {
        coreo[i].comment[0] = '\0';
        coreo[i].move[0] = '\0';
        coreo[i].fader = 0;
    }

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        // Parse: <beat><TAB><comment><TAB><move><TAB><fader>
        // Move is after 2nd tab, fader is after 3rd tab
        int beat = 0;
        char move[MAX_NAME_LEN] = "";
        char fader[MAX_NAME_LEN] = "";

        // Parse beat number
        beat = atoi(line);
        if (beat <= 0 || beat >= MAX_BEATS) continue;

        // Find tabs
        char* tab1 = strchr(line, '\t');
        char* tab2 = tab1 ? strchr(tab1 + 1, '\t') : NULL;
        char* tab3 = tab2 ? strchr(tab2 + 1, '\t') : NULL;

        // Move is after 2nd tab
        if (tab2) {
            char* start = tab2 + 1;
            char* end = tab3 ? tab3 : start + strlen(start);
            // Trim newline
            while (end > start && (*(end-1) == '\n' || *(end-1) == '\r')) end--;
            int len = end - start;
            if (len > 0 && len < MAX_NAME_LEN) {
                strncpy(move, start, len);
                move[len] = '\0';
            }
        }

        // Fader is after 3rd tab
        if (tab3) {
            char* start = tab3 + 1;
            char* end = start + strlen(start);
            // Trim newline
            while (end > start && (*(end-1) == '\n' || *(end-1) == '\r')) end--;
            int len = end - start;
            if (len > 0 && len < MAX_NAME_LEN) {
                strncpy(fader, start, len);
                fader[len] = '\0';
            }
        }

        // Store
        strcpy(coreo[beat].move, move);
        coreo[beat].fader = parse_fader(fader);

        if (beat > max_beat) max_beat = beat;
    }

    fclose(f);
    return max_beat;
}

// Find next different move starting from beat
const char* find_next_move(int from_beat) {
    const char* current = coreo[from_beat].move;
    for (int b = from_beat + 1; b <= max_beat; b++) {
        if (coreo[b].move[0] != '\0' && strcmp(coreo[b].move, current) != 0) {
            return coreo[b].move;
        }
    }
    return NULL;
}

// Smooth interpolation (ease in-out)
float smooth(float t) {
    return t * t * (3.0f - 2.0f * t);
}

// Get current time in seconds
double get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

// Convert time to beat
int time_to_beat(double t, double start_time) {
    double elapsed = t - start_time;
    double seconds_per_beat = 60.0 / bpm;
    int beat = (int)(elapsed / seconds_per_beat + master_phase) + 1;
    return beat < 1 ? 1 : beat;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <coreo.txt> [bpm] [phase]\n", argv[0]);
        fprintf(stderr, "\nPlays through coreo file and shows move/fade state.\n");
        fprintf(stderr, "Press Ctrl+C to quit.\n");
        return 1;
    }

    const char* coreo_file = argv[1];
    if (argc >= 3) bpm = atof(argv[2]);
    if (argc >= 4) master_phase = atof(argv[3]);

    if (!load_coreo(coreo_file)) return 1;

    fprintf(stderr, "Loaded coreo: %d beats, BPM: %.1f\n", max_beat, bpm);
    fprintf(stderr, "Starting playback...\n\n");

    double start_time = get_time();
    int last_beat = 0;

    // Set initial move
    for (int b = 1; b <= max_beat; b++) {
        if (coreo[b].move[0] != '\0') {
            strcpy(current_move, coreo[b].move);
            break;
        }
    }

    while (1) {
        double t = get_time();
        int beat = time_to_beat(t, start_time);

        if (beat > max_beat) {
            printf("\n[End of coreo]\n");
            break;
        }

        // New beat?
        if (beat != last_beat) {
            last_beat = beat;
            CoreoLine* line = &coreo[beat];

            // Start fade on this beat?
            if (line->fader > 0 && fade_length == 0) {
                const char* nm = find_next_move(beat);
                if (nm) {
                    strcpy(next_move, nm);
                    fade_start_beat = beat;
                    fade_length = line->fader;
                }
            }

            // Fade finished?
            if (fade_length > 0 && beat >= fade_start_beat + fade_length) {
                strcpy(current_move, next_move);
                next_move[0] = '\0';
                fade_length = 0;
            }

            // Hard switch (new move without fade)?
            if (line->move[0] != '\0' &&
                strcmp(line->move, current_move) != 0 &&
                fade_length == 0) {
                strcpy(current_move, line->move);
            }

            // Calculate fade progress
            float fade_progress = 0.0f;
            if (fade_length > 0) {
                fade_progress = (float)(beat - fade_start_beat) / fade_length;
                fade_progress = smooth(fade_progress);
            }

            // Print status
            if (fade_length > 0) {
                printf("Beat %3d: %-12s -> %-12s (fade %.0f%%)\n",
                       beat, current_move, next_move, fade_progress * 100);
            } else {
                printf("Beat %3d: %-12s\n", beat, current_move);
            }
            fflush(stdout);
        }

        // Sleep a bit
        usleep(10000);  // 10ms
    }

    return 0;
}
