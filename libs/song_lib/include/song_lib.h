#ifndef SONG_LIB_H
#define SONG_LIB_H

#define SONG_LIB_SIZE 64
#define SONG_NAME_MAX 64
#define SONG_PATH_MAX 256

/**
 * struct song - Song metadata for move synchronization
 * @name: Display name / title
 * @wav_path: Path to wav file
 * @bpm: Beats per minute (constant for song)
 * @master_phase: Phase offset to align move_downbeat with music_downbeat
 *
 * master_phase shifts the move so that move_downbeat (h[0] minimum at 3π/2)
 * aligns with the music's downbeat when song starts from 0.
 */
struct song {
	char name[SONG_NAME_MAX];
	char wav_path[SONG_PATH_MAX];
	float bpm;
	float master_phase;
};

/* Global song library */
extern struct song song_lib[SONG_LIB_SIZE];
extern int song_lib_count;

/**
 * song_lib_init - Initialize song library
 * Clears all songs.
 */
void song_lib_init(void);

/**
 * song_lib_add - Add a song to the library
 * @name: Display name
 * @wav_path: Path to wav file
 * @bpm: Beats per minute
 * @master_phase: Phase offset for move synchronization
 *
 * Returns index of added song, or -1 if library is full.
 */
int song_lib_add(const char *name, const char *wav_path, float bpm,
		 float master_phase);

/**
 * song_lib_get - Get song by index
 * @index: Song index (0 to song_lib_count-1)
 *
 * Returns pointer to song, or NULL if index invalid.
 */
struct song *song_lib_get(int index);

/**
 * song_lib_find - Find song by name
 * @name: Song name to search for
 *
 * Returns index of song, or -1 if not found.
 */
int song_lib_find(const char *name);

/**
 * song_lib_save - Save song library to JSON file
 * @path: Path to JSON file
 *
 * Returns 0 on success, -1 on error.
 */
int song_lib_save(const char *path);

/**
 * song_lib_load - Load song library from JSON file
 * @path: Path to JSON file
 *
 * Clears existing library before loading.
 * Returns number of songs loaded, or -1 on error.
 */
int song_lib_load(const char *path);

#endif /* SONG_LIB_H */
