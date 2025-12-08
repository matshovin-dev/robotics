#include <AudioToolbox/AudioToolbox.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// WAV header structure
typedef struct {
    char riff[4];
    uint32_t file_size;
    char wave[4];
    char fmt[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
} WavHeader;

// Global state
float* samples = NULL;
int num_samples = 0;
int sample_rate = 44100;
int num_channels_wav = 1;

// Beat settings
double bpm = 120.0;
double master_phase = 0.0;

// Audio playback
AudioQueueRef audio_queue = NULL;
int playback_position = 0;
int is_playing = 0;

// Output file
FILE* output_file = NULL;

#define AUDIO_BUFFER_SIZE 4096

void audio_callback(void* user_data, AudioQueueRef queue, AudioQueueBufferRef buffer) {
    if (!is_playing) {
        memset(buffer->mAudioData, 0, buffer->mAudioDataByteSize);
        AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
        return;
    }

    int16_t* out = (int16_t*)buffer->mAudioData;
    int frames_to_fill = AUDIO_BUFFER_SIZE;
    int samples_written = 0;

    for (int i = 0; i < frames_to_fill; i++) {
        if (playback_position >= num_samples) {
            is_playing = 0;
            out[samples_written++] = 0;
            out[samples_written++] = 0;
        } else {
            int idx = playback_position * num_channels_wav;
            int16_t left = (int16_t)(samples[idx] * 32767.0f);
            int16_t right = (num_channels_wav == 2) ?
                (int16_t)(samples[idx + 1] * 32767.0f) : left;
            out[samples_written++] = left;
            out[samples_written++] = right;
            playback_position++;
        }
    }

    buffer->mAudioDataByteSize = samples_written * sizeof(int16_t);
    AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
}

float* load_wav(const char* filename) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        fprintf(stderr, "Could not open file: %s\n", filename);
        return NULL;
    }

    WavHeader header;
    fread(&header, sizeof(WavHeader), 1, file);

    if (strncmp(header.riff, "RIFF", 4) != 0 || strncmp(header.wave, "WAVE", 4) != 0) {
        fprintf(stderr, "Not a valid WAV file\n");
        fclose(file);
        return NULL;
    }

    fprintf(stderr, "WAV: %uHz, %uch, %ubit\n", header.sample_rate, header.num_channels, header.bits_per_sample);

    sample_rate = header.sample_rate;
    num_channels_wav = header.num_channels;

    // Find data chunk
    char chunk_id[4];
    uint32_t chunk_size;
    while (fread(chunk_id, 4, 1, file) == 1) {
        fread(&chunk_size, 4, 1, file);
        if (strncmp(chunk_id, "data", 4) == 0) break;
        fseek(file, chunk_size, SEEK_CUR);
    }

    int total_samples = chunk_size / (header.bits_per_sample / 8);
    num_samples = total_samples / header.num_channels;

    float duration = (float)num_samples / sample_rate;
    fprintf(stderr, "Duration: %.1f sec\n", duration);

    float* data = malloc(total_samples * sizeof(float));

    if (header.bits_per_sample == 16) {
        int16_t* raw = malloc(total_samples * sizeof(int16_t));
        fread(raw, sizeof(int16_t), total_samples, file);
        for (int i = 0; i < total_samples; i++) {
            data[i] = raw[i] / 32768.0f;
        }
        free(raw);
    }

    fclose(file);
    return data;
}

int get_current_beat(void) {
    double seconds_per_beat = 60.0 / bpm;
    double samples_per_beat = seconds_per_beat * sample_rate;
    double phase_offset_samples = master_phase * samples_per_beat;
    int nearest_beat = (int)((playback_position - phase_offset_samples) / samples_per_beat + 0.5);
    if (nearest_beat < 0) nearest_beat = 0;
    return nearest_beat + 1;
}

void write_cue(int indent, int beat) {
    char line[64];
    if (indent == 0) {
        snprintf(line, sizeof(line), "%d", beat);
    } else if (indent == 1) {
        snprintf(line, sizeof(line), " %d", beat);
    } else {
        snprintf(line, sizeof(line), "  %d", beat);
    }

    // Write to terminal
    printf("%s\n", line);
    fflush(stdout);

    // Write to file
    if (output_file) {
        fprintf(output_file, "%s\n", line);
        fflush(output_file);
    }
}

// Terminal raw mode for non-blocking input
struct termios orig_termios;

void disable_raw_mode(void) {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enable_raw_mode(void) {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disable_raw_mode);

    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <wavfile.wav> <output.txt> [bpm] [phase]\n", argv[0]);
        fprintf(stderr, "\nControls:\n");
        fprintf(stderr, "  SPACE     Play/Pause\n");
        fprintf(stderr, "  0         Return to start\n");
        fprintf(stderr, "  1         Record cue (no indent)\n");
        fprintf(stderr, "  2         Record cue (1 space indent)\n");
        fprintf(stderr, "  3         Record cue (2 space indent)\n");
        fprintf(stderr, "  q/ESC     Quit\n");
        return 1;
    }

    // Parse arguments
    const char* wav_file = argv[1];
    const char* output_path = argv[2];

    if (argc >= 4) {
        bpm = atof(argv[3]);
        if (bpm < 20.0) bpm = 20.0;
        if (bpm > 300.0) bpm = 300.0;
    }
    if (argc >= 5) {
        master_phase = atof(argv[4]);
        while (master_phase < 0.0) master_phase += 1.0;
        while (master_phase >= 1.0) master_phase -= 1.0;
    }

    // Open output file
    output_file = fopen(output_path, "w");
    if (!output_file) {
        fprintf(stderr, "Could not open output file: %s\n", output_path);
        return 1;
    }

    // Load WAV
    samples = load_wav(wav_file);
    if (!samples) {
        fclose(output_file);
        return 1;
    }

    // Initialize Audio Queue
    AudioStreamBasicDescription audio_format = {0};
    audio_format.mSampleRate = sample_rate;
    audio_format.mFormatID = kAudioFormatLinearPCM;
    audio_format.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
    audio_format.mBitsPerChannel = 16;
    audio_format.mChannelsPerFrame = 2;
    audio_format.mBytesPerFrame = 4;
    audio_format.mFramesPerPacket = 1;
    audio_format.mBytesPerPacket = 4;

    OSStatus status = AudioQueueNewOutput(&audio_format, audio_callback, NULL,
                                          CFRunLoopGetCurrent(), kCFRunLoopCommonModes,
                                          0, &audio_queue);
    if (status != 0) {
        fprintf(stderr, "Failed to create audio queue\n");
        fclose(output_file);
        free(samples);
        return 1;
    }

    for (int i = 0; i < 3; i++) {
        AudioQueueBufferRef buffer;
        AudioQueueAllocateBuffer(audio_queue, AUDIO_BUFFER_SIZE * 4, &buffer);
        buffer->mAudioDataByteSize = AUDIO_BUFFER_SIZE * 4;
        memset(buffer->mAudioData, 0, buffer->mAudioDataByteSize);
        AudioQueueEnqueueBuffer(audio_queue, buffer, 0, NULL);
    }

    fprintf(stderr, "\nBPM: %.1f, Phase: %.3f\n", bpm, master_phase);
    fprintf(stderr, "Output: %s\n", output_path);
    fprintf(stderr, "Press SPACE to play, 1/2/3 for cues, q to quit\n\n");

    enable_raw_mode();

    int running = 1;
    while (running) {
        // Process audio
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.01, false);

        // Check for input
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            switch (c) {
                case ' ':
                    if (is_playing) {
                        is_playing = 0;
                        AudioQueuePause(audio_queue);
                        fprintf(stderr, "[Paused at beat %d]\n", get_current_beat());
                    } else {
                        is_playing = 1;
                        AudioQueueStart(audio_queue, NULL);
                        fprintf(stderr, "[Playing]\n");
                    }
                    break;
                case '0':
                    playback_position = 0;
                    fprintf(stderr, "[Reset]\n");
                    break;
                case '1':
                    write_cue(0, get_current_beat());
                    break;
                case '2':
                    write_cue(1, get_current_beat());
                    break;
                case '3':
                    write_cue(2, get_current_beat());
                    break;
                case 'q':
                case 27:  // ESC
                    running = 0;
                    break;
            }
        }

        // Check if playback finished
        if (!is_playing && playback_position >= num_samples) {
            fprintf(stderr, "[End of file]\n");
            playback_position = 0;
        }
    }

    // Cleanup
    disable_raw_mode();
    AudioQueueStop(audio_queue, true);
    AudioQueueDispose(audio_queue, true);
    fclose(output_file);
    free(samples);

    fprintf(stderr, "\nCues saved to %s\n", output_path);
    return 0;
}
