/**
 * @file song_player.c
 * @brief WAV playback using CoreAudio (macOS)
 *
 * Provides consistent-latency audio playback for move synchronization.
 * Uses AudioQueue for buffered playback with predictable timing.
 */

#include "song_player.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <AudioToolbox/AudioToolbox.h>

/* Audio queue configuration */
#define NUM_BUFFERS 3
#define BUFFER_SIZE 4096

/* WAV file header */
struct wav_header {
	char riff[4];           /* "RIFF" */
	uint32_t file_size;
	char wave[4];           /* "WAVE" */
	char fmt[4];            /* "fmt " */
	uint32_t fmt_size;
	uint16_t audio_format;  /* 1 = PCM */
	uint16_t num_channels;
	uint32_t sample_rate;
	uint32_t byte_rate;
	uint16_t block_align;
	uint16_t bits_per_sample;
};

/* Player state */
static struct {
	AudioQueueRef queue;
	AudioQueueBufferRef buffers[NUM_BUFFERS];
	AudioStreamBasicDescription format;

	int16_t *samples;       /* Interleaved sample data */
	uint32_t total_frames;
	uint32_t current_frame;
	uint32_t sample_rate;
	uint16_t num_channels;

	int playing;
	int initialized;
	int loaded;
} player;

/* Audio queue callback - fills buffer with samples */
static void audio_callback(void *user_data, AudioQueueRef queue,
			   AudioQueueBufferRef buffer)
{
	(void)user_data;

	if (!player.playing || !player.loaded) {
		/* Fill with silence */
		memset(buffer->mAudioData, 0, buffer->mAudioDataBytesCapacity);
		buffer->mAudioDataByteSize = buffer->mAudioDataBytesCapacity;
		AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
		return;
	}

	uint32_t frames_to_copy = buffer->mAudioDataBytesCapacity /
				  (player.num_channels * sizeof(int16_t));
	uint32_t frames_left = player.total_frames - player.current_frame;

	if (frames_to_copy > frames_left)
		frames_to_copy = frames_left;

	if (frames_to_copy > 0) {
		uint32_t samples_to_copy = frames_to_copy * player.num_channels;
		memcpy(buffer->mAudioData,
		       &player.samples[player.current_frame * player.num_channels],
		       samples_to_copy * sizeof(int16_t));
		buffer->mAudioDataByteSize = samples_to_copy * sizeof(int16_t);
		player.current_frame += frames_to_copy;
	} else {
		/* End of file - fill with silence and stop */
		memset(buffer->mAudioData, 0, buffer->mAudioDataBytesCapacity);
		buffer->mAudioDataByteSize = buffer->mAudioDataBytesCapacity;
		player.playing = 0;
	}

	AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
}

int song_player_init(void)
{
	if (player.initialized)
		return 0;

	memset(&player, 0, sizeof(player));
	player.initialized = 1;
	return 0;
}

void song_player_cleanup(void)
{
	if (!player.initialized)
		return;

	song_player_stop();
	song_player_unload();

	if (player.queue) {
		AudioQueueDispose(player.queue, true);
		player.queue = NULL;
	}

	player.initialized = 0;
}

int song_player_load(const char *path)
{
	if (!player.initialized)
		return -1;

	song_player_unload();

	FILE *f = fopen(path, "rb");
	if (!f) {
		fprintf(stderr, "song_player: Cannot open %s\n", path);
		return -1;
	}

	/* Read WAV header */
	struct wav_header hdr;
	if (fread(&hdr, sizeof(hdr), 1, f) != 1) {
		fprintf(stderr, "song_player: Cannot read header\n");
		fclose(f);
		return -1;
	}

	/* Validate */
	if (memcmp(hdr.riff, "RIFF", 4) != 0 ||
	    memcmp(hdr.wave, "WAVE", 4) != 0) {
		fprintf(stderr, "song_player: Not a WAV file\n");
		fclose(f);
		return -1;
	}

	if (hdr.audio_format != 1) {
		fprintf(stderr, "song_player: Only PCM supported\n");
		fclose(f);
		return -1;
	}

	if (hdr.bits_per_sample != 16) {
		fprintf(stderr, "song_player: Only 16-bit supported\n");
		fclose(f);
		return -1;
	}

	/* Find data chunk */
	char chunk_id[4];
	uint32_t chunk_size;
	while (fread(chunk_id, 4, 1, f) == 1) {
		if (fread(&chunk_size, 4, 1, f) != 1)
			break;
		if (memcmp(chunk_id, "data", 4) == 0)
			break;
		fseek(f, chunk_size, SEEK_CUR);
	}

	if (memcmp(chunk_id, "data", 4) != 0) {
		fprintf(stderr, "song_player: No data chunk\n");
		fclose(f);
		return -1;
	}

	/* Read samples */
	uint32_t num_samples = chunk_size / sizeof(int16_t);
	player.samples = malloc(chunk_size);
	if (!player.samples) {
		fprintf(stderr, "song_player: Out of memory\n");
		fclose(f);
		return -1;
	}

	if (fread(player.samples, sizeof(int16_t), num_samples, f) != num_samples) {
		fprintf(stderr, "song_player: Cannot read samples\n");
		free(player.samples);
		player.samples = NULL;
		fclose(f);
		return -1;
	}

	fclose(f);

	player.num_channels = hdr.num_channels;
	player.sample_rate = hdr.sample_rate;
	player.total_frames = num_samples / hdr.num_channels;
	player.current_frame = 0;

	/* Setup audio format */
	player.format.mSampleRate = hdr.sample_rate;
	player.format.mFormatID = kAudioFormatLinearPCM;
	player.format.mFormatFlags = kLinearPCMFormatFlagIsSignedInteger |
				     kLinearPCMFormatFlagIsPacked;
	player.format.mBytesPerPacket = hdr.num_channels * sizeof(int16_t);
	player.format.mFramesPerPacket = 1;
	player.format.mBytesPerFrame = hdr.num_channels * sizeof(int16_t);
	player.format.mChannelsPerFrame = hdr.num_channels;
	player.format.mBitsPerChannel = 16;

	/* Create audio queue */
	OSStatus status = AudioQueueNewOutput(&player.format, audio_callback,
					      NULL, NULL, NULL, 0,
					      &player.queue);
	if (status != noErr) {
		fprintf(stderr, "song_player: Cannot create queue (%d)\n",
			(int)status);
		free(player.samples);
		player.samples = NULL;
		return -1;
	}

	/* Allocate buffers */
	for (int i = 0; i < NUM_BUFFERS; i++) {
		AudioQueueAllocateBuffer(player.queue, BUFFER_SIZE,
					 &player.buffers[i]);
	}

	player.loaded = 1;

	printf("song_player: Loaded %s (%u Hz, %u ch, %.1f sec)\n",
	       path, player.sample_rate, player.num_channels,
	       song_player_get_duration());

	return 0;
}

void song_player_unload(void)
{
	if (!player.loaded)
		return;

	song_player_stop();

	if (player.queue) {
		AudioQueueDispose(player.queue, true);
		player.queue = NULL;
	}

	if (player.samples) {
		free(player.samples);
		player.samples = NULL;
	}

	player.loaded = 0;
	player.total_frames = 0;
	player.current_frame = 0;
}

void song_player_play(void)
{
	if (!player.loaded || player.playing)
		return;

	/* Prime buffers */
	for (int i = 0; i < NUM_BUFFERS; i++) {
		audio_callback(NULL, player.queue, player.buffers[i]);
	}

	player.playing = 1;
	AudioQueueStart(player.queue, NULL);
}

void song_player_stop(void)
{
	if (!player.loaded || !player.playing)
		return;

	player.playing = 0;
	AudioQueueStop(player.queue, true);
}

void song_player_seek(float time_sec)
{
	if (!player.loaded)
		return;

	uint32_t frame = (uint32_t)(time_sec * player.sample_rate);
	if (frame > player.total_frames)
		frame = player.total_frames;

	player.current_frame = frame;
}

void song_player_rewind(void)
{
	song_player_seek(0.0f);
}

int song_player_is_playing(void)
{
	return player.playing;
}

float song_player_get_position(void)
{
	if (!player.loaded || player.sample_rate == 0)
		return 0.0f;
	return (float)player.current_frame / (float)player.sample_rate;
}

float song_player_get_duration(void)
{
	if (!player.loaded || player.sample_rate == 0)
		return 0.0f;
	return (float)player.total_frames / (float)player.sample_rate;
}
