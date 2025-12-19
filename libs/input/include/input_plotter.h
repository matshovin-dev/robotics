/**
 * @file input_plotter.h
 * @brief MIDI input for wb_plotter_live (Behringer X-Touch Mini)
 *
 * Behringer X-Touch Mini layout:
 *   CC 80-87: Top row encoders
 *   CC 90-97: Bottom row encoders/faders
 *   NOTE 8-15, 16-23, 24-31, 32-39: Button rows
 */

#ifndef INPUT_PLOTTER_H
#define INPUT_PLOTTER_H

/* Event types */
#define PLOTTER_ENCODER  0
#define PLOTTER_FADER    1
#define PLOTTER_BUTTON   2

/* Control IDs for plotter */
#define PLOTTER_ID_PHASE_COARSE    0   /* master_phase 0.2 rad (encoder) */
#define PLOTTER_ID_PHASE_FINE      1   /* master_phase 0.05 rad (encoder) */
#define PLOTTER_ID_BPM             2   /* bpm (encoder) */
#define PLOTTER_ID_MOVE_A          3   /* move_a_nr (encoder) */
#define PLOTTER_ID_MOVE_B          4   /* move_b_nr (encoder) */
#define PLOTTER_ID_TRANS_START     5   /* transition_start_beat (encoder) */
#define PLOTTER_ID_TRANS_LEN       6   /* transition_beats (encoder) */
#define PLOTTER_ID_TRANS_TYPE      7   /* transition type - spline/fade (encoder) */

#define PLOTTER_ID_RUN             10  /* Run/play animation (button) */
#define PLOTTER_ID_SPLINE_MODE     11  /* Toggle spline/fade mode (button) */
#define PLOTTER_ID_TIME_LEFT       12  /* Step time left (button) */
#define PLOTTER_ID_TIME_RIGHT      13  /* Step time right (button) */
#define PLOTTER_ID_SAVE            14  /* Save current segment (button) */
#define PLOTTER_ID_TIME_LEFT_FAST  15  /* Step time left 4x (button) */
#define PLOTTER_ID_TIME_RIGHT_FAST 16  /* Step time right 4x (button) */

/**
 * struct plotter_event - Input event for plotter
 * @type: PLOTTER_ENCODER, PLOTTER_FADER, or PLOTTER_BUTTON
 * @id: Control ID (PLOTTER_ID_*)
 * @value: Event value (delta for encoder, 0-1 for fader, 1/0 for button)
 */
struct plotter_event {
	int type;
	int id;
	float value;
};

/**
 * input_plotter_init - Initialize MIDI input (Behringer X-Touch Mini)
 * Returns 0 on success, -1 on error.
 */
int input_plotter_init(void);

/**
 * input_plotter_poll - Check for MIDI input
 * @ev: Output event (only valid if return is 1)
 * Returns 1 if event available, 0 if no input.
 * Non-blocking.
 */
int input_plotter_poll(struct plotter_event *ev);

/**
 * input_plotter_cleanup - Close MIDI connection
 */
void input_plotter_cleanup(void);

/**
 * LCD colors for X-Touch Extender
 */
#define LCD_COLOR_OFF      0
#define LCD_COLOR_RED      1
#define LCD_COLOR_GREEN    2
#define LCD_COLOR_YELLOW   3
#define LCD_COLOR_BLUE     4
#define LCD_COLOR_MAGENTA  5
#define LCD_COLOR_CYAN     6
#define LCD_COLOR_WHITE    7

/**
 * input_plotter_set_lcd - Set LCD display text and color
 * @display: Display number 0-7
 * @color: LCD_COLOR_* constant
 * @top: Top line text (max 7 chars, will be padded/truncated)
 * @bottom: Bottom line text (max 7 chars, will be padded/truncated)
 * Returns 0 on success, -1 on error.
 */
int input_plotter_set_lcd(int display, int color,
			  const char *top, const char *bottom);

/**
 * input_plotter_clear_all_lcd - Clear all 8 LCD displays
 */
void input_plotter_clear_all_lcd(void);

#endif /* INPUT_PLOTTER_H */
