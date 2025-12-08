#include <AudioToolbox/AudioToolbox.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

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

// Cue entry
typedef struct {
    int beat;
    int indent;  // 0, 1, or 2
    char text[128];  // Optional text after beat number
} Cue;

// Global state
float* samples = NULL;
int num_samples = 0;
int sample_rate = 44100;
int num_channels_wav = 1;

// Beat settings
double bpm = 120.0;
double master_phase = 0.0;

// Playback range
int start_beat = 1;
int end_beat = -1;  // -1 means end of file

// Audio playback
AudioQueueRef audio_queue = NULL;
volatile double playback_position = 0;  // Position in source file
volatile int is_playing = 0;
volatile int should_stop = 0;
volatile int half_speed = 0;  // Toggle for half speed playback

// Cues
Cue* cues = NULL;
int num_cues = 0;
int next_cue_index = 0;
int last_printed_beat = -1;

#define AUDIO_BUFFER_SIZE 4096

int beat_to_sample(int beat) {
    double seconds_per_beat = 60.0 / bpm;
    double samples_per_beat = seconds_per_beat * sample_rate;
    double phase_offset_samples = master_phase * samples_per_beat;
    return (int)(phase_offset_samples + (beat - 1) * samples_per_beat);
}

int sample_to_beat(double sample_pos) {
    double seconds_per_beat = 60.0 / bpm;
    double samples_per_beat = seconds_per_beat * sample_rate;
    double phase_offset_samples = master_phase * samples_per_beat;
    // Use round() to match cue_recorder behavior
    int beat = (int)((sample_pos - phase_offset_samples) / samples_per_beat + 0.5) + 1;
    return beat < 1 ? 1 : beat;
}

double get_effective_bpm(void) {
    return half_speed ? bpm * 0.5 : bpm;
}

void audio_callback(void* user_data, AudioQueueRef queue, AudioQueueBufferRef buffer) {
    if (!is_playing || should_stop) {
        memset(buffer->mAudioData, 0, buffer->mAudioDataByteSize);
        AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
        return;
    }

    double speed = half_speed ? 0.5 : 1.0;
    int end_sample = (end_beat > 0) ? beat_to_sample(end_beat + 1) : num_samples;
    if (end_sample > num_samples) end_sample = num_samples;

    int16_t* out = (int16_t*)buffer->mAudioData;
    int frames_to_fill = AUDIO_BUFFER_SIZE;
    int samples_written = 0;

    for (int i = 0; i < frames_to_fill; i++) {
        int pos = (int)playback_position;
        if (pos >= end_sample) {
            is_playing = 0;
            should_stop = 1;
            out[samples_written++] = 0;
            out[samples_written++] = 0;
        } else {
            int idx = pos * num_channels_wav;
            int16_t left = (int16_t)(samples[idx] * 32767.0f);
            int16_t right = (num_channels_wav == 2) ?
                (int16_t)(samples[idx + 1] * 32767.0f) : left;
            out[samples_written++] = left;
            out[samples_written++] = right;
            playback_position += speed;
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

int load_cues(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Could not open cue file: %s\n", filename);
        return 0;
    }

    // Count lines first
    int count = 0;
    char line[256];
    while (fgets(line, sizeof(line), file)) {
        if (line[0] != '\n' && line[0] != '\0') count++;
    }

    cues = malloc(count * sizeof(Cue));
    num_cues = 0;

    rewind(file);
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '\n' || line[0] == '\0') continue;

        // Count leading spaces for indent
        int indent = 0;
        char* p = line;
        while (*p == ' ') { indent++; p++; }

        // Parse beat number
        int beat = atoi(p);
        if (beat > 0) {
            cues[num_cues].beat = beat;
            cues[num_cues].indent = (indent >= 2) ? 2 : indent;

            // Skip past the number to get any text
            while (*p >= '0' && *p <= '9') p++;
            while (*p == ' ' || *p == '\t') p++;  // Skip whitespace after number

            // Copy remaining text (strip newline)
            strncpy(cues[num_cues].text, p, sizeof(cues[num_cues].text) - 1);
            cues[num_cues].text[sizeof(cues[num_cues].text) - 1] = '\0';
            char* nl = strchr(cues[num_cues].text, '\n');
            if (nl) *nl = '\0';

            num_cues++;
        }
    }

    fclose(file);
    return num_cues;
}

void check_and_print_cues(int current_beat) {
    // Check all cues that match current beat
    for (int i = 0; i < num_cues; i++) {
        if (cues[i].beat == current_beat && cues[i].beat != last_printed_beat) {
            int indent = cues[i].indent;
            const char* text = cues[i].text;
            int has_text = (text[0] != '\0');

            if (indent == 0) {
                printf("%d%s%s\n", cues[i].beat, has_text ? " " : "", text);
            } else if (indent == 1) {
                printf(" %d%s%s\n", cues[i].beat, has_text ? " " : "", text);
            } else {
                printf("  %d%s%s\n", cues[i].beat, has_text ? " " : "", text);
            }
            fflush(stdout);
        }
    }
    last_printed_beat = current_beat;
}

// Terminal raw mode
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
        fprintf(stderr, "Usage: %s <wavfile.wav> <cues.txt> [bpm] [phase] [start_beat] [end_beat]\n", argv[0]);
        fprintf(stderr, "\nPlays audio and prints cues at matching beats.\n");
        fprintf(stderr, "Press q or ESC to quit, SPACE to pause/resume.\n");
        return 1;
    }

    const char* wav_file = argv[1];
    const char* cue_file = argv[2];

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
    if (argc >= 6) {
        start_beat = atoi(argv[5]);
        if (start_beat < 1) start_beat = 1;
    }
    if (argc >= 7) {
        end_beat = atoi(argv[6]);
    }

    // Load WAV
    samples = load_wav(wav_file);
    if (!samples) return 1;

    // Load cues
    if (!load_cues(cue_file)) {
        free(samples);
        return 1;
    }

    // Find first cue at or after start_beat
    next_cue_index = 0;
    while (next_cue_index < num_cues && cues[next_cue_index].beat < start_beat) {
        next_cue_index++;
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
        free(samples);
        free(cues);
        return 1;
    }

    for (int i = 0; i < 3; i++) {
        AudioQueueBufferRef buffer;
        AudioQueueAllocateBuffer(audio_queue, AUDIO_BUFFER_SIZE * 4, &buffer);
        buffer->mAudioDataByteSize = AUDIO_BUFFER_SIZE * 4;
        memset(buffer->mAudioData, 0, buffer->mAudioDataByteSize);
        AudioQueueEnqueueBuffer(audio_queue, buffer, 0, NULL);
    }

    // Set start position
    playback_position = beat_to_sample(start_beat);
    if (playback_position < 0) playback_position = 0;

    fprintf(stderr, "BPM: %.1f, Phase: %.3f\n", bpm, master_phase);
    fprintf(stderr, "Playing beats %d to %d\n", start_beat, end_beat > 0 ? end_beat : sample_to_beat(num_samples));
    fprintf(stderr, "Loaded %d cues: ", num_cues);
    for (int i = 0; i < num_cues; i++) {
        fprintf(stderr, "%s%d", (cues[i].indent > 0) ? " " : "", cues[i].beat);
        if (i < num_cues - 1) fprintf(stderr, ", ");
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Press SPACE to pause, h for half speed, q to quit\n\n");

    enable_raw_mode();

    // Start playback
    is_playing = 1;
    AudioQueueStart(audio_queue, NULL);

    while (!should_stop) {
        // Process audio
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.01, false);

        // Check and print cues
        int current_beat = sample_to_beat(playback_position);
        check_and_print_cues(current_beat);

        // Check for input
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == 'q' || c == 27) {  // q or ESC
                should_stop = 1;
            } else if (c == ' ') {
                if (is_playing) {
                    is_playing = 0;
                    AudioQueuePause(audio_queue);
                    fprintf(stderr, "[Paused at beat %d]\n", current_beat);
                } else {
                    is_playing = 1;
                    AudioQueueStart(audio_queue, NULL);
                    fprintf(stderr, "[Playing]\n");
                }
            } else if (c == 'h') {
                half_speed = !half_speed;
                fprintf(stderr, "[Speed: %s, effective BPM: %.1f]\n",
                        half_speed ? "0.5x" : "1.0x", get_effective_bpm());
            }
        }
    }

    // Cleanup
    disable_raw_mode();
    AudioQueueStop(audio_queue, true);
    AudioQueueDispose(audio_queue, true);
    free(samples);
    free(cues);

    fprintf(stderr, "\nDone.\n");
    return 0;
}
