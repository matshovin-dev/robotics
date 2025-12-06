/**
 * @file input_keyboard.c
 * @brief Keyboard input source for macOS/Linux
 *
 * Mixer model:
 *   Deck A = always move 0 (live output)
 *   Deck B = preview move (select with 0-9)
 *
 * Key mappings:
 *   q/a     - Phase up/down (encoder)
 *   w/s     - BPM up/down (encoder)
 *   e/d     - Crossfade up/down (encoder)
 *   r/f     - Volume A up/down (live)
 *   t/g     - Volume B up/down (preview)
 *   c       - Copy move 0 -> move 99 (backup)
 *   0-9     - Load move preset to deck B
 *   escape  - Quit signal
 */

#include "input.h"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

static struct termios orig_termios;
static int initialized = 0;

/* Encoder step size */
#define ENCODER_STEP 0.02f
#define ENCODER_STEP_FINE 0.005f

int input_keyboard_init(void)
{
	struct termios raw;

	if (initialized)
		return 0;

	if (tcgetattr(STDIN_FILENO, &orig_termios) < 0)
		return -1;

	raw = orig_termios;
	raw.c_lflag &= ~(ICANON | ECHO);
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;

	if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0)
		return -1;

	initialized = 1;
	return 0;
}

void input_keyboard_cleanup(void)
{
	if (initialized) {
		tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
		initialized = 0;
	}
}

static int has_input(void)
{
	struct timeval tv = { 0, 0 };
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

int input_keyboard_poll(struct input_event *ev)
{
	char c;

	if (!has_input())
		return 0;

	if (read(STDIN_FILENO, &c, 1) != 1)
		return 0;

	/* Default: no event */
	ev->type = -1;
	ev->id = -1;
	ev->value = 0;

	switch (c) {
	/* Phase encoder */
	case 'q':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_PHASE;
		ev->value = ENCODER_STEP;
		break;
	case 'a':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_PHASE;
		ev->value = -ENCODER_STEP;
		break;

	/* BPM encoder */
	case 'w':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_BPM;
		ev->value = 1.0f;  /* +1 BPM */
		break;
	case 's':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_BPM;
		ev->value = -1.0f;  /* -1 BPM */
		break;

	/* Crossfade encoder */
	case 'e':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_CROSSFADE;
		ev->value = ENCODER_STEP;
		break;
	case 'd':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_CROSSFADE;
		ev->value = -ENCODER_STEP;
		break;

	/* Volume A encoder (live) */
	case 'r':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_VOLUME_A;
		ev->value = ENCODER_STEP;
		break;
	case 'f':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_VOLUME_A;
		ev->value = -ENCODER_STEP;
		break;

	/* Volume B encoder (preview) */
	case 't':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_VOLUME_B;
		ev->value = ENCODER_STEP;
		break;
	case 'g':
		ev->type = INPUT_ENCODER;
		ev->id = INPUT_ID_VOLUME_B;
		ev->value = -ENCODER_STEP;
		break;

	/* Copy move 0 -> move 99 */
	case 'c':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_COPY;
		ev->value = 1.0f;
		break;

	/* Move presets -> deck B */
	case '0':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_0;
		ev->value = 1.0f;
		break;
	case '1':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_1;
		ev->value = 1.0f;
		break;
	case '2':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_2;
		ev->value = 1.0f;
		break;
	case '3':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_3;
		ev->value = 1.0f;
		break;
	case '4':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_4;
		ev->value = 1.0f;
		break;
	case '5':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_5;
		ev->value = 1.0f;
		break;
	case '6':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_6;
		ev->value = 1.0f;
		break;
	case '7':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_7;
		ev->value = 1.0f;
		break;
	case '8':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_8;
		ev->value = 1.0f;
		break;
	case '9':
		ev->type = INPUT_BUTTON;
		ev->id = INPUT_ID_MOVE_9;
		ev->value = 1.0f;
		break;

	/* Quit */
	case 27:  /* Escape */
	case 'Q':
		ev->type = INPUT_BUTTON;
		ev->id = -1;  /* Special: quit signal */
		ev->value = 1.0f;
		break;

	default:
		return 0;  /* Unknown key, no event */
	}

	return (ev->type >= 0) ? 1 : 0;
}

void input_keyboard_print_help(void)
{
	printf("Keyboard controls:\n");
	printf("  q/a     Phase +/-\n");
	printf("  w/s     BPM +/-\n");
	printf("  e/d     Crossfade +/-\n");
	printf("  r/f     Volume A +/- (live)\n");
	printf("  t/g     Volume B +/- (preview)\n");
	printf("  c       Copy move 0 -> 99 (backup)\n");
	printf("  0-9     Load move to deck B\n");
	printf("  Q/Esc   Quit\n");
	printf("\n");
}
