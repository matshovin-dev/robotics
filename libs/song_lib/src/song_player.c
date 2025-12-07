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
#include <math.h>
#include <AudioToolbox/AudioToolbox.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Audio queue configuration */
#define NUM_BUFFERS 3
#define BUFFER_SIZE 4096

/* Click track configuration */
#define CLICK_FREQ 1000.0f       /* Hz */
#define CLICK_DURATION 0.05f    /* seconds (50ms) */
#define CLICK_SAMPLE_RATE 44100
#define CLICK_AMPLITUDE 0.6f

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

/* Click track state */
static struct {
	AudioQueueRef queue;
	AudioQueueBufferRef buffer;
	AudioStreamBasicDescription format;
	int16_t *samples;
	uint32_t num_samples;
	uint32_t current_sample;
	int enabled;
	int initialized;
	int triggered;
} click;

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

/* Click track callback */
static void click_callback(void *user_data, AudioQueueRef queue,
			   AudioQueueBufferRef buffer)
{
	(void)user_data;

	if (!click.triggered || !click.enabled) {
		/* Fill with silence */
		memset(buffer->mAudioData, 0, buffer->mAudioDataBytesCapacity);
		buffer->mAudioDataByteSize = buffer->mAudioDataBytesCapacity;
		AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
		return;
	}

	uint32_t frames_to_copy = buffer->mAudioDataBytesCapacity / sizeof(int16_t);
	uint32_t frames_left = click.num_samples - click.current_sample;

	if (frames_to_copy > frames_left)
		frames_to_copy = frames_left;

	if (frames_to_copy > 0) {
		memcpy(buffer->mAudioData,
		       &click.samples[click.current_sample],
		       frames_to_copy * sizeof(int16_t));
		buffer->mAudioDataByteSize = frames_to_copy * sizeof(int16_t);
		click.current_sample += frames_to_copy;
	} else {
		/* End of click - fill with silence */
		memset(buffer->mAudioData, 0, buffer->mAudioDataBytesCapacity);
		buffer->mAudioDataByteSize = buffer->mAudioDataBytesCapacity;
		click.triggered = 0;
	}

	AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
}

/* Initialize click track */
static int click_init(void)
{
	if (click.initialized)
		return 0;

	/* Generate sine wave samples */
	click.num_samples = (uint32_t)(CLICK_SAMPLE_RATE * CLICK_DURATION);
	click.samples = malloc(click.num_samples * sizeof(int16_t));
	if (!click.samples)
		return -1;

	for (uint32_t i = 0; i < click.num_samples; i++) {
		float t = (float)i / CLICK_SAMPLE_RATE;
		float envelope = 1.0f;
		/* Quick fade out at end to avoid click */
		if (i > click.num_samples - 200)
			envelope = (float)(click.num_samples - i) / 200.0f;
		click.samples[i] = (int16_t)(32767.0f * CLICK_AMPLITUDE * envelope *
					      sinf(2.0f * M_PI * CLICK_FREQ * t));
	}

	/* Setup audio format (mono) */
	click.format.mSampleRate = CLICK_SAMPLE_RATE;
	click.format.mFormatID = kAudioFormatLinearPCM;
	click.format.mFormatFlags = kLinearPCMFormatFlagIsSignedInteger |
				    kLinearPCMFormatFlagIsPacked;
	click.format.mBytesPerPacket = sizeof(int16_t);
	click.format.mFramesPerPacket = 1;
	click.format.mBytesPerFrame = sizeof(int16_t);
	click.format.mChannelsPerFrame = 1;
	click.format.mBitsPerChannel = 16;

	/* Create audio queue */
	OSStatus status = AudioQueueNewOutput(&click.format, click_callback,
					      NULL, NULL, NULL, 0,
					      &click.queue);
	if (status != noErr) {
		free(click.samples);
		click.samples = NULL;
		return -1;
	}

	/* Allocate buffer */
	AudioQueueAllocateBuffer(click.queue, BUFFER_SIZE, &click.buffer);

	/* Prime buffer with silence and start queue */
	memset(click.buffer->mAudioData, 0, click.buffer->mAudioDataBytesCapacity);
	click.buffer->mAudioDataByteSize = click.buffer->mAudioDataBytesCapacity;
	AudioQueueEnqueueBuffer(click.queue, click.buffer, 0, NULL);
	AudioQueueStart(click.queue, NULL);

	click.initialized = 1;
	click.enabled = 0;
	click.triggered = 0;

	return 0;
}

void song_player_click_enable(int enable)
{
	if (!click.initialized)
		click_init();
	click.enabled = enable;
	if (!enable)
		click.triggered = 0;
}

void song_player_click_trigger(void)
{
	if (!click.initialized)
		click_init();
	if (!click.enabled)
		return;

	/* Reset to start of click sound */
	click.current_sample = 0;
	click.triggered = 1;
}
