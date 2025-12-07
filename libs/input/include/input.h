/**
 * @file input.h
 * @brief Unified input handling for keyboard, MIDI, and serial
 *
 * Abstracts input sources so workbench code doesn't care where
 * events come from. All sources produce the same event format.
 *
 * Event types:
 *   INPUT_ENCODER - Relative, value is delta (-1.0 to +1.0 typical)
 *   INPUT_FADER   - Absolute, value is position (0.0 to 1.0)
 *   INPUT_BUTTON  - Momentary, value is 1.0 (pressed) or 0.0 (released)
 */

#ifndef INPUT_H
#define INPUT_H

/* Event types */
#define INPUT_ENCODER  0
#define INPUT_FADER    1
#define INPUT_BUTTON   2

/* Control IDs - shared across all sources
 *
 * Mixer model:
 *   Deck A = always move 0 (live output to robot)
 *   Deck B = preview move (select with 0-9)
 *   Crossfader blends between A and B
 *   COPY button: save move 0 -> move 99 (backup before transition)
 */
#define INPUT_ID_PHASE      0   /* Master phase (encoder) */
#define INPUT_ID_BPM        1   /* BPM (encoder) */
#define INPUT_ID_CROSSFADE  2   /* Crossfader A<->B (fader 0-1) */
#define INPUT_ID_VOLUME_A   3   /* Volume/intensity deck A - live (fader 0-1) */
#define INPUT_ID_VOLUME_B   4   /* Volume/intensity deck B - preview (fader 0-1) */
#define INPUT_ID_COPY       5   /* Copy move 0 -> move 99 (button) */
#define INPUT_ID_AUTOFADE   6   /* Auto-fade: wait for phase, fade, copy, reset */
#define INPUT_ID_PLAY_SONG  7   /* Play first song from library, sync with move */
#define INPUT_ID_SAVE_SONG  8   /* Save current bpm/phase to song library */
#define INPUT_ID_MOVE_0     10  /* Move presets 0-9 -> load to deck B */
#define INPUT_ID_MOVE_1     11
#define INPUT_ID_MOVE_2     12
#define INPUT_ID_MOVE_3     13
#define INPUT_ID_MOVE_4     14
#define INPUT_ID_MOVE_5     15
#define INPUT_ID_MOVE_6     16
#define INPUT_ID_MOVE_7     17
#define INPUT_ID_MOVE_8     18
#define INPUT_ID_MOVE_9     19

/**
 * struct input_event - Unified input event
 * @type: INPUT_ENCODER, INPUT_FADER, or INPUT_BUTTON
 * @id: Control ID (INPUT_ID_*)
 * @value: Event value (meaning depends on type)
 */
struct input_event {
	int type;
	int id;
	float value;
};

/**
 * input_keyboard_init - Initialize keyboard input
 * Sets terminal to raw mode for single-key reading.
 * Returns 0 on success, -1 on error.
 */
int input_keyboard_init(void);

/**
 * input_keyboard_poll - Check for keyboard input
 * @ev: Output event (only valid if return is 1)
 * Returns 1 if event available, 0 if no input.
 * Non-blocking.
 */
int input_keyboard_poll(struct input_event *ev);

/**
 * input_keyboard_cleanup - Restore terminal settings
 */
void input_keyboard_cleanup(void);

/**
 * input_keyboard_print_help - Print key mappings
 */
void input_keyboard_print_help(void);

/**
 * input_midi_init - Initialize MIDI input (X-Touch Extender)
 * Returns 0 on success, -1 on error.
 */
int input_midi_init(void);

/**
 * input_midi_poll - Check for MIDI input
 * @ev: Output event (only valid if return is 1)
 * Returns 1 if event available, 0 if no input.
 * Non-blocking.
 */
int input_midi_poll(struct input_event *ev);

/**
 * input_midi_cleanup - Close MIDI connection
 */
void input_midi_cleanup(void);

/**
 * input_midi_get_current_move - Get current move selection for deck B
 * Returns move index (0-98)
 */
int input_midi_get_current_move(void);

#endif /* INPUT_H */
