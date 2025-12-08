/**
 * @file wb_coreo_player.c
 * @brief Workbench: Play choreography from coreo.txt file
 *
 * Reads a coreo.txt file with format:
 *   <beat><TAB><comment><TAB><move><TAB><fader>
 *
 * Plays moves synchronized to BPM, with crossfade support.
 */

#include "song_player.h"
#include "song_lib.h"
#include "move_lib.h"
#include "stewart/geometry.h"
#include "stewart/pose.h"
#include "viz_sender.h"
#include "viz_status.h"
#include "viz_ports.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>

#define MAX_BEATS 2000
#define MAX_NAME_LEN 64

/* Coreo line data */
typedef struct {
    char move[MAX_NAME_LEN];
    int fader;  /* 0 = no fade, 4 = f4, etc. */
} CoreoLine;

/* Global state */
static CoreoLine coreo[MAX_BEATS];
static int max_beat = 0;

/* Fade state */
static int current_move_idx = 0;
static int next_move_idx = 0;
static int fade_start_beat = 0;
static int fade_length = 0;

/* Terminal raw mode */
static struct termios orig_termios;

static void disable_raw_mode(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

static void enable_raw_mode(void)
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disable_raw_mode);

    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

/* Parse fader string like "f4" -> 4 */
static int parse_fader(const char *str)
{
    if (!str || str[0] == '\0')
        return 0;
    if (str[0] == 'f' || str[0] == 's' || str[0] == 'i' || str[0] == 'o')
        return atoi(str + 1);
    return 0;
}

/* Find move index by name */
static int find_move_by_name(const char *name)
{
    if (!name || name[0] == '\0')
        return -1;

    for (int i = 0; i < MOVE_LIB_SIZE; i++) {
        if (strcmp(move_lib[i].name, name) == 0)
            return i;
    }
    return -1;
}

/* Load coreo file */
static int load_coreo(const char *filename)
{
    FILE *f = fopen(filename, "r");
    if (!f) {
        fprintf(stderr, "Could not open coreo file: %s\n", filename);
        return 0;
    }

    /* Initialize all entries */
    for (int i = 0; i < MAX_BEATS; i++) {
        coreo[i].move[0] = '\0';
        coreo[i].fader = 0;
    }

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        int beat = atoi(line);
        if (beat <= 0 || beat >= MAX_BEATS)
            continue;

        /* Find tabs: move after 2nd tab, fader after 3rd */
        char *tab1 = strchr(line, '\t');
        char *tab2 = tab1 ? strchr(tab1 + 1, '\t') : NULL;
        char *tab3 = tab2 ? strchr(tab2 + 1, '\t') : NULL;

        /* Move is after 2nd tab */
        if (tab2) {
            char *start = tab2 + 1;
            char *end = tab3 ? tab3 : start + strlen(start);
            while (end > start && (*(end - 1) == '\n' || *(end - 1) == '\r'))
                end--;
            int len = end - start;
            if (len > 0 && len < MAX_NAME_LEN) {
                strncpy(coreo[beat].move, start, len);
                coreo[beat].move[len] = '\0';
            }
        }

        /* Fader is after 3rd tab */
        if (tab3) {
            char *start = tab3 + 1;
            char *end = start + strlen(start);
            while (end > start && (*(end - 1) == '\n' || *(end - 1) == '\r'))
                end--;
            int len = end - start;
            if (len > 0) {
                char fader_str[MAX_NAME_LEN];
                strncpy(fader_str, start, len);
                fader_str[len] = '\0';
                coreo[beat].fader = parse_fader(fader_str);
            }
        }

        if (beat > max_beat)
            max_beat = beat;
    }

    fclose(f);
    return max_beat;
}

/* Find next different move starting from beat */
static const char *find_next_move(int from_beat)
{
    const char *current = coreo[from_beat].move;
    for (int b = from_beat + 1; b <= max_beat; b++) {
        if (coreo[b].move[0] != '\0' && strcmp(coreo[b].move, current) != 0)
            return coreo[b].move;
    }
    return NULL;
}

/* Smooth interpolation (ease in-out) */
static float smooth(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

/* Convert playback time to beat */
static int time_to_beat(float t, float bpm, float phase)
{
    float seconds_per_beat = 60.0f / bpm;
    int beat = (int)(t / seconds_per_beat + phase) + 1;
    return beat < 1 ? 1 : beat;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <coreo.txt> [song_index]\n", argv[0]);
        fprintf(stderr, "\nPlays choreography synchronized to music.\n");
        fprintf(stderr, "Press q to quit, SPACE to pause.\n");
        return 1;
    }

    const char *coreo_file = argv[1];
    int song_index = (argc >= 3) ? atoi(argv[2]) : 0;

    /* Load song library */
    song_lib_load("/Users/matsmac/vsCode/robotics/assets/songs/song_lib.json");

    /* Initialize song player */
    if (song_player_init() < 0) {
        fprintf(stderr, "Failed to initialize song player\n");
        return 1;
    }

    /* Initialize move library */
    move_lib_init();

    /* Load coreo */
    if (!load_coreo(coreo_file)) {
        song_player_cleanup();
        return 1;
    }

    fprintf(stderr, "Loaded coreo: %d beats\n", max_beat);

    /* Get song */
    struct song *s = song_lib_get(song_index);
    if (!s) {
        fprintf(stderr, "No song at index %d\n", song_index);
        song_player_cleanup();
        return 1;
    }

    /* Setup playback */
    move_playback.bpm = s->bpm;
    move_playback.master_phase = s->master_phase;
    move_playback.t = 0.0f;

    fprintf(stderr, "Song: %s @ %.0f BPM, phase=%.2f\n",
            s->name, s->bpm, s->master_phase);

    /* Find initial move */
    for (int b = 1; b <= max_beat; b++) {
        if (coreo[b].move[0] != '\0') {
            current_move_idx = find_move_by_name(coreo[b].move);
            if (current_move_idx < 0) {
                fprintf(stderr, "Warning: move '%s' not found in library\n",
                        coreo[b].move);
                current_move_idx = 0;
            }
            break;
        }
    }

    /* Setup mixer */
    move_mixer.deck_a = current_move_idx;
    move_mixer.deck_b = current_move_idx;
    move_mixer.crossfader = 0.0f;
    move_mixer.volume_a = 1.0f;
    move_mixer.volume_b = 1.0f;

    /* Create UDP sender */
    int sock = viz_sender_create();
    if (sock < 0) {
        fprintf(stderr, "Failed to create UDP sender\n");
        song_player_cleanup();
        return 1;
    }

    /* Create status sender */
    struct viz_status status;
    if (viz_status_init(&status) < 0) {
        fprintf(stderr, "Failed to create status sender\n");
        song_player_cleanup();
        return 1;
    }

    const struct stewart_geometry *geom = &ROBOT_MX64;
    struct stewart_pose pose;
    struct timeval last, now;
    int last_beat = 0;
    int running = 1;
    int paused = 0;

    /* Load and play song */
    song_player_load(s->wav_path);
    song_player_rewind();
    song_player_play();

    enable_raw_mode();

    fprintf(stderr, "Playing! Press SPACE to pause, q to quit\n");

    gettimeofday(&last, NULL);

    while (running) {
        /* Check keyboard */
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == 'q' || c == 27) {
                running = 0;
            } else if (c == ' ') {
                if (paused) {
                    song_player_play();
                    paused = 0;
                    fprintf(stderr, "\r[Playing]          \n");
                } else {
                    song_player_stop();
                    paused = 1;
                    fprintf(stderr, "\r[Paused]           \n");
                }
            }
        }

        /* Update time */
        gettimeofday(&now, NULL);
        float dt = (now.tv_sec - last.tv_sec) +
                   (now.tv_usec - last.tv_usec) / 1000000.0f;
        last = now;

        if (!paused) {
            move_playback_tick(&move_playback, dt);
        }

        int beat = time_to_beat(move_playback.t, move_playback.bpm,
                                move_playback.master_phase);

        /* End of coreo? */
        if (beat > max_beat) {
            fprintf(stderr, "\n[End of coreo]\n");
            break;
        }

        /* New beat? */
        if (beat != last_beat) {
            last_beat = beat;
            CoreoLine *line = &coreo[beat];

            /* Start fade on this beat? */
            if (line->fader > 0 && fade_length == 0) {
                const char *nm = find_next_move(beat);
                if (nm) {
                    next_move_idx = find_move_by_name(nm);
                    if (next_move_idx >= 0) {
                        move_mixer.deck_b = next_move_idx;
                        fade_start_beat = beat;
                        fade_length = line->fader;
                    }
                }
            }

            /* Fade finished? */
            if (fade_length > 0 && beat >= fade_start_beat + fade_length) {
                current_move_idx = next_move_idx;
                move_mixer.deck_a = current_move_idx;
                move_mixer.crossfader = 0.0f;
                fade_length = 0;
            }

            /* Hard switch (new move without fade)? */
            if (line->move[0] != '\0' && fade_length == 0) {
                int idx = find_move_by_name(line->move);
                if (idx >= 0 && idx != current_move_idx) {
                    current_move_idx = idx;
                    move_mixer.deck_a = current_move_idx;
                }
            }

            /* Calculate fade progress */
            float fade_progress = 0.0f;
            if (fade_length > 0) {
                fade_progress = (float)(beat - fade_start_beat) / fade_length;
                fade_progress = smooth(fade_progress);
                move_mixer.crossfader = fade_progress;
            }

            /* Print status */
            if (fade_length > 0) {
                fprintf(stderr, "\rBeat %3d: %-12s -> %-12s (fade %.0f%%)   ",
                        beat, move_lib[current_move_idx].name,
                        move_lib[next_move_idx].name,
                        fade_progress * 100);
            } else {
                fprintf(stderr, "\rBeat %3d: %-12s                          ",
                        beat, move_lib[current_move_idx].name);
            }
            fflush(stderr);
        }

        /* Evaluate and send pose */
        move_evaluate_mixed(&move_mixer, &move_playback, geom, &pose);
        pose.ty += geom->home_height;
        viz_sender_send_pose(sock, &pose, ROBOT_TYPE_MX64, 9010);

        /* Send status */
        viz_status_set(&status, "beat", (float)beat);
        viz_status_set(&status, "crossfader", move_mixer.crossfader);
        viz_status_set_str(&status, "moveA", move_lib[move_mixer.deck_a].name);
        viz_status_set_str(&status, "moveB", move_lib[move_mixer.deck_b].name);
        viz_status_send(&status);

        usleep(16000);  /* ~60 Hz */
    }

    fprintf(stderr, "\n\nGoodbye!\n");

    disable_raw_mode();
    song_player_cleanup();
    viz_status_close(&status);

    return 0;
}
