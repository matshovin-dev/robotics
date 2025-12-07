#ifndef SONG_PLAYER_H
#define SONG_PLAYER_H

/**
 * song_player - WAV playback with consistent latency
 *
 * Uses CoreAudio on macOS for low-latency, consistent playback.
 * Fixed latency from play command to audio output (~10-20ms typical).
 */

/**
 * song_player_init - Initialize audio system
 *
 * Returns 0 on success, -1 on error.
 */
int song_player_init(void);

/**
 * song_player_cleanup - Shutdown audio system
 */
void song_player_cleanup(void);

/**
 * song_player_load - Load a WAV file
 * @path: Path to WAV file (16-bit PCM, any sample rate)
 *
 * Returns 0 on success, -1 on error.
 * Only one file can be loaded at a time.
 */
int song_player_load(const char *path);

/**
 * song_player_unload - Unload current WAV file
 */
void song_player_unload(void);

/**
 * song_player_play - Start playback
 *
 * Starts from current position.
 */
void song_player_play(void);

/**
 * song_player_stop - Stop playback
 *
 * Keeps current position.
 */
void song_player_stop(void);

/**
 * song_player_seek - Seek to position
 * @time_sec: Position in seconds
 */
void song_player_seek(float time_sec);

/**
 * song_player_rewind - Seek to start
 */
void song_player_rewind(void);

/**
 * song_player_is_playing - Check if playing
 *
 * Returns 1 if playing, 0 if stopped.
 */
int song_player_is_playing(void);

/**
 * song_player_get_position - Get current playback position
 *
 * Returns position in seconds.
 */
float song_player_get_position(void);

/**
 * song_player_get_duration - Get total duration
 *
 * Returns duration in seconds, or 0 if no file loaded.
 */
float song_player_get_duration(void);

#endif /* SONG_PLAYER_H */
