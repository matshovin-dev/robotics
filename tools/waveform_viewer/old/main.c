#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <AudioToolbox/AudioToolbox.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "text_renderer.h"

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
double zoom = 1.0;
double pan = 0.0;  // 0.0 = start, 1.0 = end
int window_width = 1200;
int window_height = 600;

// Beat markers
double bpm = 120.0;
double master_phase = 0.0;  // Phase offset in beats (0.0 to 1.0)
int show_beats = 1;
int show_downbeats = 1;  // Every 4th beat (measure start)

// Audio playback
AudioQueueRef audio_queue = NULL;
int playback_position = 0;  // Current sample position
int is_playing = 0;
int num_channels_wav = 1;
float* stereo_samples = NULL;  // Original stereo data for playback
int stereo_num_samples = 0;

#define AUDIO_BUFFER_SIZE 4096

void audio_callback(void* user_data, AudioQueueRef queue, AudioQueueBufferRef buffer) {
    if (!is_playing) {
        // Fill with silence
        memset(buffer->mAudioData, 0, buffer->mAudioDataByteSize);
        AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
        return;
    }

    int16_t* out = (int16_t*)buffer->mAudioData;
    int frames_to_fill = AUDIO_BUFFER_SIZE;
    int samples_written = 0;

    for (int i = 0; i < frames_to_fill; i++) {
        if (playback_position >= stereo_num_samples) {
            // End of file - stop and fill rest with silence
            is_playing = 0;
            out[samples_written++] = 0;
            out[samples_written++] = 0;
        } else {
            // Convert float to int16
            int idx = playback_position * num_channels_wav;
            int16_t left = (int16_t)(stereo_samples[idx] * 32767.0f);
            int16_t right = (num_channels_wav == 2) ?
                (int16_t)(stereo_samples[idx + 1] * 32767.0f) : left;
            out[samples_written++] = left;
            out[samples_written++] = right;
            playback_position++;
        }
    }

    buffer->mAudioDataByteSize = samples_written * sizeof(int16_t);
    AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
}

// Font path for text renderer
static const char* font_path = NULL;

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    double old_zoom = zoom;

    // Zoom in/out
    if (yoffset > 0) {
        zoom *= 1.2;
    } else {
        zoom /= 1.2;
        if (zoom < 1.0) zoom = 1.0;
    }

    // Get cursor position for zoom-to-cursor
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    double cursor_ratio = xpos / window_width;

    // Adjust pan to zoom towards cursor
    double visible_before = 1.0 / old_zoom;
    double visible_after = 1.0 / zoom;
    double cursor_pos = pan + cursor_ratio * visible_before;
    pan = cursor_pos - cursor_ratio * visible_after;

    // Clamp pan
    if (pan < 0.0) pan = 0.0;
    if (pan + visible_after > 1.0) pan = 1.0 - visible_after;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        double visible = 1.0 / zoom;
        double pan_step = visible * 0.1;

        if (key == GLFW_KEY_LEFT) {
            pan -= pan_step;
            if (pan < 0.0) pan = 0.0;
        }
        if (key == GLFW_KEY_RIGHT) {
            pan += pan_step;
            if (pan + visible > 1.0) pan = 1.0 - visible;
        }
        if (key == GLFW_KEY_HOME) {
            pan = 0.0;
        }
        if (key == GLFW_KEY_END) {
            pan = 1.0 - visible;
        }
        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        if (key == GLFW_KEY_R) {
            zoom = 1.0;
            pan = 0.0;
        }
        // BPM adjustment
        if (key == GLFW_KEY_UP) {
            bpm += (mods & GLFW_MOD_SHIFT) ? 10.0 : 1.0;
            printf("BPM: %.1f\n", bpm);
        }
        if (key == GLFW_KEY_DOWN) {
            bpm -= (mods & GLFW_MOD_SHIFT) ? 10.0 : 1.0;
            if (bpm < 20.0) bpm = 20.0;
            printf("BPM: %.1f\n", bpm);
        }
        // Phase adjustment
        if (key == GLFW_KEY_COMMA) {  // <
            master_phase -= (mods & GLFW_MOD_SHIFT) ? 0.1 : 0.01;
            if (master_phase < 0.0) master_phase += 1.0;
            printf("Phase: %.3f\n", master_phase);
        }
        if (key == GLFW_KEY_PERIOD) {  // >
            master_phase += (mods & GLFW_MOD_SHIFT) ? 0.1 : 0.01;
            if (master_phase >= 1.0) master_phase -= 1.0;
            printf("Phase: %.3f\n", master_phase);
        }
        // Toggle beat display
        if (key == GLFW_KEY_B) {
            show_beats = !show_beats;
            printf("Beats: %s\n", show_beats ? "ON" : "OFF");
        }
        if (key == GLFW_KEY_D) {
            show_downbeats = !show_downbeats;
            printf("Downbeats: %s\n", show_downbeats ? "ON" : "OFF");
        }
        // Playback control
        if (key == GLFW_KEY_SPACE) {
            if (is_playing) {
                is_playing = 0;
                AudioQueuePause(audio_queue);
                printf("Paused at sample %d\n", playback_position);
            } else {
                is_playing = 1;
                AudioQueueStart(audio_queue, NULL);
                printf("Playing from sample %d\n", playback_position);
            }
        }
        if (key == GLFW_KEY_0) {
            // Return to start
            playback_position = 0;
            printf("Reset to start\n");
        }
        // Cue markers: 1, 2, 3 keys for choreography notes
        if (key == GLFW_KEY_1 || key == GLFW_KEY_2 || key == GLFW_KEY_3) {
            // Calculate nearest beat to current playback position
            double seconds_per_beat = 60.0 / bpm;
            double samples_per_beat = seconds_per_beat * sample_rate;
            double phase_offset_samples = master_phase * samples_per_beat;
            int nearest_beat = (int)round((playback_position - phase_offset_samples) / samples_per_beat);
            if (nearest_beat < 0) nearest_beat = 0;
            int beat_number = nearest_beat + 1;

            // Print with indentation based on key
            if (key == GLFW_KEY_1) {
                printf("%d\n", beat_number);
            } else if (key == GLFW_KEY_2) {
                printf(" %d\n", beat_number);
            } else if (key == GLFW_KEY_3) {
                printf("  %d\n", beat_number);
            }
        }
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    window_width = width;
    window_height = height;
    glViewport(0, 0, width, height);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        // Get mouse position
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);

        // Convert to sample position
        double visible = 1.0 / zoom;
        double click_ratio = xpos / window_width;
        double click_pos = pan + click_ratio * visible;
        int click_sample = (int)(click_pos * num_samples);

        // Find nearest beat
        double seconds_per_beat = 60.0 / bpm;
        double samples_per_beat = seconds_per_beat * sample_rate;
        double phase_offset_samples = master_phase * samples_per_beat;

        // Calculate which beat is closest
        int nearest_beat = (int)round((click_sample - phase_offset_samples) / samples_per_beat);
        if (nearest_beat < 0) nearest_beat = 0;

        // Snap to that beat
        int snap_sample = (int)(phase_offset_samples + nearest_beat * samples_per_beat);
        if (snap_sample < 0) snap_sample = 0;
        if (snap_sample >= num_samples) snap_sample = num_samples - 1;

        playback_position = snap_sample;
        printf("Jumped to beat %d (sample %d)\n", nearest_beat + 1, snap_sample);
    }
}

float* load_wav(const char* filename, int* out_num_samples, int* out_sample_rate) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        fprintf(stderr, "Could not open file: %s\n", filename);
        return NULL;
    }

    WavHeader header;
    fread(&header, sizeof(WavHeader), 1, file);

    // Validate
    if (strncmp(header.riff, "RIFF", 4) != 0 || strncmp(header.wave, "WAVE", 4) != 0) {
        fprintf(stderr, "Not a valid WAV file\n");
        fclose(file);
        return NULL;
    }

    printf("WAV Info:\n");
    printf("  Sample rate: %u Hz\n", header.sample_rate);
    printf("  Channels: %u\n", header.num_channels);
    printf("  Bits per sample: %u\n", header.bits_per_sample);

    *out_sample_rate = header.sample_rate;
    num_channels_wav = header.num_channels;

    // Find data chunk
    char chunk_id[4];
    uint32_t chunk_size;
    while (fread(chunk_id, 4, 1, file) == 1) {
        fread(&chunk_size, 4, 1, file);
        if (strncmp(chunk_id, "data", 4) == 0) {
            break;
        }
        fseek(file, chunk_size, SEEK_CUR);
    }

    int total_samples = chunk_size / (header.bits_per_sample / 8);
    int mono_samples = total_samples / header.num_channels;

    printf("  Total samples: %d\n", mono_samples);
    printf("  Duration: %.2f seconds\n", (float)mono_samples / header.sample_rate);

    float* data = malloc(mono_samples * sizeof(float));

    // Also allocate stereo buffer for playback
    stereo_samples = malloc(total_samples * sizeof(float));
    stereo_num_samples = mono_samples;

    if (header.bits_per_sample == 16) {
        int16_t* raw = malloc(total_samples * sizeof(int16_t));
        fread(raw, sizeof(int16_t), total_samples, file);

        // Store stereo data for playback
        for (int i = 0; i < total_samples; i++) {
            stereo_samples[i] = raw[i] / 32768.0f;
        }

        // Convert to mono float normalized to [-1, 1] for display
        for (int i = 0; i < mono_samples; i++) {
            if (header.num_channels == 2) {
                data[i] = (raw[i*2] + raw[i*2+1]) / (2.0f * 32768.0f);
            } else {
                data[i] = raw[i] / 32768.0f;
            }
        }
        free(raw);
    } else if (header.bits_per_sample == 24) {
        uint8_t* raw = malloc(total_samples * 3);
        fread(raw, 3, total_samples, file);

        for (int i = 0; i < mono_samples; i++) {
            int idx = i * header.num_channels * 3;
            int32_t sample = (raw[idx] | (raw[idx+1] << 8) | (raw[idx+2] << 16));
            if (sample & 0x800000) sample |= 0xFF000000;  // Sign extend
            data[i] = sample / 8388608.0f;
            // For 24-bit, just use mono for playback too
            stereo_samples[i] = data[i];
        }
        free(raw);
    }

    fclose(file);
    *out_num_samples = mono_samples;
    return data;
}

void render_waveform(void) {
    glClear(GL_COLOR_BUFFER_BIT);

    // Calculate visible range
    double visible = 1.0 / zoom;
    int start_sample = (int)(pan * num_samples);
    int end_sample = (int)((pan + visible) * num_samples);
    if (end_sample > num_samples) end_sample = num_samples;

    int visible_samples = end_sample - start_sample;

    // Draw beat markers BEHIND the waveform
    if (show_beats || show_downbeats) {
        double seconds_per_beat = 60.0 / bpm;
        double samples_per_beat = seconds_per_beat * sample_rate;

        // Calculate first beat position considering phase
        double phase_offset_samples = master_phase * samples_per_beat;

        // Find first visible beat
        int first_beat = (int)ceil((start_sample - phase_offset_samples) / samples_per_beat);
        if (first_beat < 0) first_beat = 0;

        for (int beat = first_beat; ; beat++) {
            double beat_sample = phase_offset_samples + beat * samples_per_beat;
            if (beat_sample > end_sample) break;
            if (beat_sample < start_sample) continue;

            // Convert sample position to screen X coordinate
            float x = 2.0f * (beat_sample - start_sample) / (float)visible_samples - 1.0f;

            int is_downbeat = (beat % 4 == 0);

            // Continuous beat number (1, 2, 3, 4, 5, 6, ...)
            int beat_number = beat + 1;

            if (is_downbeat && show_downbeats) {
                // Downbeat (every 4th beat) - bright red, full height
                glColor4f(1.0f, 0.2f, 0.2f, 0.8f);
                glLineWidth(2.0f);
                glBegin(GL_LINES);
                glVertex2f(x, -1.0f);
                glVertex2f(x, 1.0f);
                glEnd();

                // Draw beat number using text_renderer
                // Convert NDC x to pixel coordinates, centered
                char num_str[16];
                snprintf(num_str, sizeof(num_str), "%d", beat_number);
                float px = (x + 1.0f) * 0.5f * window_width;
                float py = window_height - 50;  // Near bottom
                // Estimate text width for centering (approx 10px per char at size 20)
                float text_width = strlen(num_str) * 10.0f;

                // Set up orthographic projection for text
                glMatrixMode(GL_PROJECTION);
                glPushMatrix();
                glLoadIdentity();
                glOrtho(0, window_width, window_height, 0, -1, 1);
                glMatrixMode(GL_MODELVIEW);
                glPushMatrix();
                glLoadIdentity();

                text_draw(num_str, px - text_width * 0.5f, py, 1.0f, 0.5f, 0.5f);

                // Restore projection
                glMatrixMode(GL_PROJECTION);
                glPopMatrix();
                glMatrixMode(GL_MODELVIEW);
                glPopMatrix();
            } else if (show_beats && !is_downbeat) {
                // Regular beat - dimmer, shorter, no number
                glColor4f(0.6f, 0.6f, 0.2f, 0.5f);
                glLineWidth(1.0f);
                glBegin(GL_LINES);
                glVertex2f(x, -0.7f);
                glVertex2f(x, 0.7f);
                glEnd();
            }
        }
        glLineWidth(1.0f);
    }

    // Draw center line
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_LINES);
    glVertex2f(-1.0f, 0.0f);
    glVertex2f(1.0f, 0.0f);
    glEnd();

    // Draw waveform
    glColor3f(0.216f, 0.216f, 0.216f);  // rgb(55,55,55)

    if (visible_samples > window_width * 2) {
        // Too many samples - draw min/max envelope per pixel
        glBegin(GL_LINES);
        for (int px = 0; px < window_width; px++) {
            int s0 = start_sample + (px * visible_samples) / window_width;
            int s1 = start_sample + ((px + 1) * visible_samples) / window_width;
            if (s1 > end_sample) s1 = end_sample;

            float min_val = 1.0f, max_val = -1.0f;
            for (int s = s0; s < s1; s++) {
                if (samples[s] < min_val) min_val = samples[s];
                if (samples[s] > max_val) max_val = samples[s];
            }

            float x = (2.0f * px / window_width) - 1.0f;
            glVertex2f(x, min_val * 0.45f);
            glVertex2f(x, max_val * 0.45f);
        }
        glEnd();
    } else {
        // Few enough samples - draw as line strip
        glBegin(GL_LINE_STRIP);
        for (int i = start_sample; i < end_sample; i++) {
            float x = 2.0f * (i - start_sample) / (float)visible_samples - 1.0f;
            glVertex2f(x, samples[i] * 0.45f);
        }
        glEnd();
    }

    // Draw playhead (blue vertical line)
    if (playback_position >= start_sample && playback_position <= end_sample) {
        float playhead_x = 2.0f * (playback_position - start_sample) / (float)visible_samples - 1.0f;
        glColor4f(0.3f, 0.5f, 1.0f, 0.9f);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glVertex2f(playhead_x, -1.0f);
        glVertex2f(playhead_x, 1.0f);
        glEnd();
        glLineWidth(1.0f);
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <wavfile.wav> [bpm] [phase]\n", argv[0]);
        fprintf(stderr, "  bpm: beats per minute (default: 120)\n");
        fprintf(stderr, "  phase: master phase offset 0.0-1.0 (default: 0.0)\n");
        return 1;
    }

    // Parse optional BPM and phase
    if (argc >= 3) {
        bpm = atof(argv[2]);
        if (bpm < 20.0) bpm = 20.0;
        if (bpm > 300.0) bpm = 300.0;
    }
    if (argc >= 4) {
        master_phase = atof(argv[3]);
        while (master_phase < 0.0) master_phase += 1.0;
        while (master_phase >= 1.0) master_phase -= 1.0;
    }

    // Determine font path (relative to executable or absolute)
    // Try to find font relative to this source file's location
    font_path = "/Users/matsmac/vsCode/robotics/libs/text_renderer/fonts/LiberationMono-Bold.ttf";

    // Load WAV file
    samples = load_wav(argv[1], &num_samples, &sample_rate);
    if (!samples) {
        return 1;
    }

    // Initialize Audio Queue for playback
    AudioStreamBasicDescription audio_format = {0};
    audio_format.mSampleRate = sample_rate;
    audio_format.mFormatID = kAudioFormatLinearPCM;
    audio_format.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
    audio_format.mBitsPerChannel = 16;
    audio_format.mChannelsPerFrame = 2;  // Always output stereo
    audio_format.mBytesPerFrame = 4;     // 2 channels * 2 bytes
    audio_format.mFramesPerPacket = 1;
    audio_format.mBytesPerPacket = 4;

    OSStatus status = AudioQueueNewOutput(&audio_format, audio_callback, NULL,
                                          CFRunLoopGetCurrent(), kCFRunLoopCommonModes,
                                          0, &audio_queue);
    if (status != 0) {
        fprintf(stderr, "Failed to create audio queue: %d\n", (int)status);
        return 1;
    }

    // Allocate and enqueue buffers
    for (int i = 0; i < 3; i++) {
        AudioQueueBufferRef buffer;
        AudioQueueAllocateBuffer(audio_queue, AUDIO_BUFFER_SIZE * 4, &buffer);
        buffer->mAudioDataByteSize = AUDIO_BUFFER_SIZE * 4;
        memset(buffer->mAudioData, 0, buffer->mAudioDataByteSize);
        AudioQueueEnqueueBuffer(audio_queue, buffer, 0, NULL);
    }

    // Initialize GLFW
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(window_width, window_height, "Waveform Viewer", NULL, NULL);
    if (!window) {
        fprintf(stderr, "Failed to create window\n");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Initialize text renderer
    if (!text_init(font_path, 20)) {
        fprintf(stderr, "Warning: Could not initialize text renderer\n");
    }

    printf("\nBeat settings: BPM=%.1f, Phase=%.3f, Sample rate=%d\n", bpm, master_phase, sample_rate);
    printf("Samples per beat: %.1f\n", (60.0 / bpm) * sample_rate);
    printf("\nControls:\n");
    printf("  Scroll:       Zoom in/out\n");
    printf("  Left/Right:   Pan\n");
    printf("  Home/End:     Jump to start/end\n");
    printf("  R:            Reset zoom\n");
    printf("  Up/Down:      Adjust BPM (+/-1, Shift for +/-10)\n");
    printf("  </> :         Adjust phase (+/-0.01, Shift for +/-0.1)\n");
    printf("  B:            Toggle beat markers\n");
    printf("  D:            Toggle downbeat markers\n");
    printf("  SPACE:        Play/Pause\n");
    printf("  0:            Return to start\n");
    printf("  Click:        Jump to nearest beat\n");
    printf("  ESC:          Quit\n");

    while (!glfwWindowShouldClose(window)) {
        render_waveform();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    text_cleanup();
    AudioQueueStop(audio_queue, true);
    AudioQueueDispose(audio_queue, true);
    free(samples);
    free(stereo_samples);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
