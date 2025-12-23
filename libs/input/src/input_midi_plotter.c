/**
 * @file input_midi_plotter.c
 * @brief MIDI input for wb_plotter_live (Behringer X-Touch Mini)
 *
 * Behringer X-Touch Mini mappings:
 *   Top encoders (CC 80-87):
 *     80: master_phase (0.2 rad increments)
 *     81: master_phase (0.05 rad fine)
 *     82: bpm
 *     83: move_a_nr
 *     84: move_b_nr
 *     85: transition_start_beat
 *     86: transition_beats
 *     87: transition type (spline/fade)
 *
 *   Buttons row 1 (NOTE 8-15):
 *     8: Run animation
 *     9: Toggle spline/fade mode
 *     10: Step time left
 *     11: Step time right
 *
 * Uses CoreMIDI on macOS.
 */

#include "input_plotter.h"
#include <stdio.h>
#include <string.h>
#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>

/* MIDI CC mappings - top row encoders */
#define MIDI_CC_PHASE_COARSE  80
#define MIDI_CC_PHASE_FINE    81
#define MIDI_CC_BPM           82
#define MIDI_CC_MOVE_A        83
#define MIDI_CC_MOVE_B        84
#define MIDI_CC_TRANS_START   85
#define MIDI_CC_TRANS_LEN     86
#define MIDI_CC_TRANS_TYPE    87

/* MIDI CC mappings - bottom row faders */
#define MIDI_CC_FADER_0       70
#define MIDI_CC_FADER_1       71
#define MIDI_CC_FADER_2       72
#define MIDI_CC_FADER_3       73
#define MIDI_CC_FADER_4       74
#define MIDI_CC_FADER_5       75
#define MIDI_CC_FADER_6       76
#define MIDI_CC_DOF_SELECT    77  /* DOF selector fader (rightmost) */

/* MIDI NOTE mappings - button row 1 */
#define MIDI_NOTE_RUN            8
#define MIDI_NOTE_SPLINE_MODE    9
#define MIDI_NOTE_TIME_LEFT      10
#define MIDI_NOTE_TIME_RIGHT     11
#define MIDI_NOTE_SAVE           12
#define MIDI_NOTE_CYCLE_DOF      13
#define MIDI_NOTE_WIN_START_UP   14   /* Increase window start beat */
#define MIDI_NOTE_WIN_END_UP     15   /* Increase window end beat */

/* MIDI NOTE mappings - button row 2 (notes 16-23) */
#define MIDI_NOTE_REPEAT          16
#define MIDI_NOTE_WIN_START_DOWN  22   /* Decrease window start beat */
#define MIDI_NOTE_WIN_END_DOWN    23   /* Decrease window end beat */
#define MIDI_NOTE_TIME_LEFT_FAST  18
#define MIDI_NOTE_TIME_RIGHT_FAST 19
#define MIDI_NOTE_SAVE_MOVE_LIB   20

/* MIDI NOTE mappings - button row 3 (notes 24-31) */
#define MIDI_NOTE_MUSIC_TOGGLE    24
#define MIDI_NOTE_PASTE_MOVE      27  /* Paste clipboard move to deck_b */
#define MIDI_NOTE_CLEAR_DECK_B    28
#define MIDI_NOTE_SEGMENT_DOWN    38  /* Decrease current segment */
#define MIDI_NOTE_PHASE_SHIFT_B   31

/* MIDI NOTE mappings - button row 4 (notes 32-39) */
#define MIDI_NOTE_RUN_4_BEATS     32
#define MIDI_NOTE_COPY_MOVE_NR    35  /* Copy deck_b move number to clipboard */
#define MIDI_NOTE_RANDOM_DECK_B   36
#define MIDI_NOTE_SEGMENT_UP      30  /* Increase current segment */

/* Event queue */
#define EVENT_QUEUE_SIZE 32
static struct plotter_event event_queue[EVENT_QUEUE_SIZE];
static int queue_head = 0;
static int queue_tail = 0;

/* CoreMIDI handles */
static MIDIClientRef midi_client = 0;
static MIDIPortRef midi_input_port = 0;
static MIDIPortRef midi_output_port = 0;
static MIDIEndpointRef midi_source = 0;
static MIDIEndpointRef midi_dest = 0;
static int initialized = 0;

/* SysEx header for Behringer X-Touch Extender LCD */
static const Byte SYSEX_LCD_HEADER[] = {
	0xF0,  /* SysEx start */
	0x00,  /* Manufacturer ID byte 1 */
	0x20,  /* Manufacturer ID byte 2 */
	0x32,  /* Manufacturer ID byte 3 (Behringer) */
	0x15,  /* Device ID */
	0x4C   /* LCD command */
};

/* Queue helpers */
static int queue_empty(void)
{
	return queue_head == queue_tail;
}

static int queue_full(void)
{
	return ((queue_tail + 1) % EVENT_QUEUE_SIZE) == queue_head;
}

static void queue_push(struct plotter_event *ev)
{
	if (queue_full())
		return;
	event_queue[queue_tail] = *ev;
	queue_tail = (queue_tail + 1) % EVENT_QUEUE_SIZE;
}

static int queue_pop(struct plotter_event *ev)
{
	if (queue_empty())
		return 0;
	*ev = event_queue[queue_head];
	queue_head = (queue_head + 1) % EVENT_QUEUE_SIZE;
	return 1;
}

/* Convert relative encoder value to delta
 * X-Touch Mini sends: 1 = CW, 65 = CCW (relative mode) */
static float encoder_delta(int value)
{
	if (value == 1)
		return -1.0f;  /* clockwise */
	if (value == 65)
		return 1.0f;   /* counter-clockwise */
	return 0.0f;
}

/* MIDI callback */
static void midi_read_callback(const MIDIPacketList *pktlist,
			       void *readProcRefCon,
			       void *srcConnRefCon)
{
	(void)readProcRefCon;
	(void)srcConnRefCon;

	const MIDIPacket *packet = &pktlist->packet[0];

	for (UInt32 i = 0; i < pktlist->numPackets; i++) {
		if (packet->length >= 3) {
			int status = packet->data[0];
			int data1 = packet->data[1];
			int data2 = packet->data[2];
			int msg_type = status & 0xF0;

			struct plotter_event ev = { -1, -1, 0 };

			/* Control Change (0xB0) - encoders */
			if (msg_type == 0xB0) {
				int cc = data1;
				int value = data2;

				ev.type = PLOTTER_ENCODER;

				switch (cc) {
				case MIDI_CC_PHASE_COARSE:
					ev.id = PLOTTER_ID_PHASE_COARSE;
					ev.value = encoder_delta(value);
					break;
				case MIDI_CC_PHASE_FINE:
					ev.id = PLOTTER_ID_PHASE_FINE;
					ev.value = encoder_delta(value);
					break;
				case MIDI_CC_BPM:
					ev.id = PLOTTER_ID_BPM;
					ev.value = encoder_delta(value);
					break;
				case MIDI_CC_MOVE_A:
					ev.id = PLOTTER_ID_MOVE_A;
					ev.value = encoder_delta(value);
					break;
				case MIDI_CC_MOVE_B:
					ev.id = PLOTTER_ID_MOVE_B;
					ev.value = encoder_delta(value);
					break;
				case MIDI_CC_TRANS_START:
					ev.id = PLOTTER_ID_TRANS_START;
					ev.value = encoder_delta(value);
					break;
				case MIDI_CC_TRANS_LEN:
					ev.id = PLOTTER_ID_TRANS_LEN;
					ev.value = encoder_delta(value);
					break;
				case MIDI_CC_TRANS_TYPE:
					ev.id = PLOTTER_ID_TRANS_TYPE;
					ev.value = encoder_delta(value);
					break;
				/* Faders - absolute value 0-127 → 0.0-1.0 */
				case MIDI_CC_FADER_0:
					ev.type = PLOTTER_FADER;
					ev.id = PLOTTER_ID_FADER_0;
					ev.value = value / 127.0f;
					break;
				case MIDI_CC_FADER_1:
					ev.type = PLOTTER_FADER;
					ev.id = PLOTTER_ID_FADER_1;
					ev.value = value / 127.0f;
					break;
				case MIDI_CC_FADER_2:
					ev.type = PLOTTER_FADER;
					ev.id = PLOTTER_ID_FADER_2;
					ev.value = value / 127.0f;
					break;
				case MIDI_CC_FADER_3:
					ev.type = PLOTTER_FADER;
					ev.id = PLOTTER_ID_FADER_3;
					ev.value = value / 127.0f;
					break;
				case MIDI_CC_FADER_4:
					ev.type = PLOTTER_FADER;
					ev.id = PLOTTER_ID_FADER_4;
					ev.value = value / 127.0f;
					break;
				case MIDI_CC_FADER_5:
					ev.type = PLOTTER_FADER;
					ev.id = PLOTTER_ID_FADER_5;
					ev.value = value / 127.0f;
					break;
				case MIDI_CC_FADER_6:
					ev.type = PLOTTER_FADER;
					ev.id = PLOTTER_ID_FADER_6;
					ev.value = value / 127.0f;
					break;
				case MIDI_CC_DOF_SELECT:
					ev.type = PLOTTER_FADER;
					ev.id = PLOTTER_ID_DOF_SELECT;
					ev.value = value / 127.0f;
					break;
				default:
					ev.type = -1;  /* Unknown CC */
					break;
				}
			}
			/* Note On (0x90) - buttons */
			else if (msg_type == 0x90) {
				int note = data1;
				int velocity = data2;

				if (velocity > 0) {  /* Button press */
					ev.type = PLOTTER_BUTTON;
					ev.value = 1.0f;

					switch (note) {
					case MIDI_NOTE_RUN:
						ev.id = PLOTTER_ID_RUN;
						break;
					case MIDI_NOTE_SPLINE_MODE:
						ev.id = PLOTTER_ID_SPLINE_MODE;
						break;
					case MIDI_NOTE_TIME_LEFT:
						ev.id = PLOTTER_ID_TIME_LEFT;
						break;
					case MIDI_NOTE_TIME_RIGHT:
						ev.id = PLOTTER_ID_TIME_RIGHT;
						break;
					case MIDI_NOTE_SAVE:
						ev.id = PLOTTER_ID_SAVE;
						break;
					case MIDI_NOTE_CYCLE_DOF:
						ev.id = PLOTTER_ID_CYCLE_DOF;
						break;
					case MIDI_NOTE_TIME_LEFT_FAST:
						ev.id = PLOTTER_ID_TIME_LEFT_FAST;
						break;
					case MIDI_NOTE_TIME_RIGHT_FAST:
						ev.id = PLOTTER_ID_TIME_RIGHT_FAST;
						break;
					case MIDI_NOTE_REPEAT:
						ev.id = PLOTTER_ID_REPEAT;
						break;
					case MIDI_NOTE_MUSIC_TOGGLE:
						ev.id = PLOTTER_ID_MUSIC_TOGGLE;
						break;
					case MIDI_NOTE_SAVE_MOVE_LIB:
						ev.id = PLOTTER_ID_SAVE_MOVE_LIB;
						break;
					case MIDI_NOTE_CLEAR_DECK_B:
						ev.id = PLOTTER_ID_CLEAR_DECK_B;
						break;
					case MIDI_NOTE_RUN_4_BEATS:
						ev.id = PLOTTER_ID_RUN_4_BEATS;
						break;
					case MIDI_NOTE_RANDOM_DECK_B:
						ev.id = PLOTTER_ID_RANDOM_DECK_B;
						break;
					case MIDI_NOTE_PHASE_SHIFT_B:
						ev.id = PLOTTER_ID_PHASE_SHIFT_B;
						break;
					case MIDI_NOTE_SEGMENT_DOWN:
						ev.id = PLOTTER_ID_SEGMENT_DOWN;
						break;
					case MIDI_NOTE_SEGMENT_UP:
						ev.id = PLOTTER_ID_SEGMENT_UP;
						break;
					case MIDI_NOTE_WIN_START_UP:
						ev.id = PLOTTER_ID_WIN_START_UP;
						break;
					case MIDI_NOTE_WIN_START_DOWN:
						ev.id = PLOTTER_ID_WIN_START_DOWN;
						break;
					case MIDI_NOTE_WIN_END_UP:
						ev.id = PLOTTER_ID_WIN_END_UP;
						break;
					case MIDI_NOTE_WIN_END_DOWN:
						ev.id = PLOTTER_ID_WIN_END_DOWN;
						break;
					case MIDI_NOTE_COPY_MOVE_NR:
						ev.id = PLOTTER_ID_COPY_MOVE_NR;
						break;
					case MIDI_NOTE_PASTE_MOVE:
						ev.id = PLOTTER_ID_PASTE_MOVE;
						break;
					default:
						ev.type = -1;  /* Unknown button */
						break;
					}
				}
			}
			/* Note Off (0x80) - button release */
			else if (msg_type == 0x80) {
				int note = data1;
				ev.type = PLOTTER_BUTTON;
				ev.value = 0.0f;

				switch (note) {
				case MIDI_NOTE_RUN:
					ev.id = PLOTTER_ID_RUN;
					break;
				case MIDI_NOTE_SPLINE_MODE:
					ev.id = PLOTTER_ID_SPLINE_MODE;
					break;
				case MIDI_NOTE_TIME_LEFT:
					ev.id = PLOTTER_ID_TIME_LEFT;
					break;
				case MIDI_NOTE_TIME_RIGHT:
					ev.id = PLOTTER_ID_TIME_RIGHT;
					break;
				case MIDI_NOTE_SAVE:
					ev.id = PLOTTER_ID_SAVE;
					break;
				case MIDI_NOTE_CYCLE_DOF:
					ev.id = PLOTTER_ID_CYCLE_DOF;
					break;
				case MIDI_NOTE_TIME_LEFT_FAST:
					ev.id = PLOTTER_ID_TIME_LEFT_FAST;
					break;
				case MIDI_NOTE_TIME_RIGHT_FAST:
					ev.id = PLOTTER_ID_TIME_RIGHT_FAST;
					break;
				case MIDI_NOTE_REPEAT:
					ev.id = PLOTTER_ID_REPEAT;
					break;
				case MIDI_NOTE_MUSIC_TOGGLE:
					ev.id = PLOTTER_ID_MUSIC_TOGGLE;
					break;
				case MIDI_NOTE_SAVE_MOVE_LIB:
					ev.id = PLOTTER_ID_SAVE_MOVE_LIB;
					break;
				case MIDI_NOTE_CLEAR_DECK_B:
					ev.id = PLOTTER_ID_CLEAR_DECK_B;
					break;
				case MIDI_NOTE_RUN_4_BEATS:
					ev.id = PLOTTER_ID_RUN_4_BEATS;
					break;
				case MIDI_NOTE_RANDOM_DECK_B:
					ev.id = PLOTTER_ID_RANDOM_DECK_B;
					break;
				case MIDI_NOTE_PHASE_SHIFT_B:
					ev.id = PLOTTER_ID_PHASE_SHIFT_B;
					break;
				case MIDI_NOTE_SEGMENT_DOWN:
					ev.id = PLOTTER_ID_SEGMENT_DOWN;
					break;
				case MIDI_NOTE_SEGMENT_UP:
					ev.id = PLOTTER_ID_SEGMENT_UP;
					break;
				case MIDI_NOTE_WIN_START_UP:
					ev.id = PLOTTER_ID_WIN_START_UP;
					break;
				case MIDI_NOTE_WIN_START_DOWN:
					ev.id = PLOTTER_ID_WIN_START_DOWN;
					break;
				case MIDI_NOTE_WIN_END_UP:
					ev.id = PLOTTER_ID_WIN_END_UP;
					break;
				case MIDI_NOTE_WIN_END_DOWN:
					ev.id = PLOTTER_ID_WIN_END_DOWN;
					break;
				case MIDI_NOTE_COPY_MOVE_NR:
					ev.id = PLOTTER_ID_COPY_MOVE_NR;
					break;
				case MIDI_NOTE_PASTE_MOVE:
					ev.id = PLOTTER_ID_PASTE_MOVE;
					break;
				default:
					ev.type = -1;
					break;
				}
			}

			if (ev.type >= 0)
				queue_push(&ev);
		}
		packet = MIDIPacketNext(packet);
	}
}

/* Find Behringer MIDI source (input) */
static MIDIEndpointRef find_behringer_source(void)
{
	ItemCount num_sources = MIDIGetNumberOfSources();

	for (ItemCount i = 0; i < num_sources; i++) {
		MIDIEndpointRef src = MIDIGetSource(i);
		CFStringRef name = NULL;

		MIDIObjectGetStringProperty(src, kMIDIPropertyName, &name);
		if (name) {
			char buf[256];
			CFStringGetCString(name, buf, sizeof(buf),
					   kCFStringEncodingUTF8);
			CFRelease(name);

			/* Match X-Touch Mini or generic Behringer */
			if (strstr(buf, "X-Touch") ||
			    strstr(buf, "X-TOUCH") ||
			    strstr(buf, "Behringer") ||
			    strstr(buf, "BEHRINGER")) {
				printf("Found MIDI source: %s\n", buf);
				return src;
			}
		}
	}
	return 0;
}

/* Find Behringer MIDI destination (output) */
static MIDIEndpointRef find_behringer_dest(void)
{
	ItemCount num_dests = MIDIGetNumberOfDestinations();

	for (ItemCount i = 0; i < num_dests; i++) {
		MIDIEndpointRef dest = MIDIGetDestination(i);
		CFStringRef name = NULL;

		MIDIObjectGetStringProperty(dest, kMIDIPropertyName, &name);
		if (name) {
			char buf[256];
			CFStringGetCString(name, buf, sizeof(buf),
					   kCFStringEncodingUTF8);
			CFRelease(name);

			/* Match X-Touch or Behringer */
			if (strstr(buf, "X-Touch") ||
			    strstr(buf, "X-TOUCH") ||
			    strstr(buf, "Behringer") ||
			    strstr(buf, "BEHRINGER")) {
				printf("Found MIDI destination: %s\n", buf);
				return dest;
			}
		}
	}
	return 0;
}

int input_plotter_init(void)
{
	OSStatus status;

	if (initialized)
		return 0;

	/* Create MIDI client */
	status = MIDIClientCreate(CFSTR("PlotterInput"),
				  NULL, NULL, &midi_client);
	if (status != noErr) {
		fprintf(stderr, "Failed to create MIDI client: %d\n",
			(int)status);
		return -1;
	}

	/* Create input port */
	status = MIDIInputPortCreate(midi_client, CFSTR("PlotterInputPort"),
				     midi_read_callback, NULL, &midi_input_port);
	if (status != noErr) {
		fprintf(stderr, "Failed to create MIDI input port: %d\n",
			(int)status);
		MIDIClientDispose(midi_client);
		return -1;
	}

	/* Create output port */
	status = MIDIOutputPortCreate(midi_client, CFSTR("PlotterOutputPort"),
				      &midi_output_port);
	if (status != noErr) {
		fprintf(stderr, "Failed to create MIDI output port: %d\n",
			(int)status);
		MIDIPortDispose(midi_input_port);
		MIDIClientDispose(midi_client);
		return -1;
	}

	/* Find Behringer source (input) */
	midi_source = find_behringer_source();
	if (!midi_source) {
		fprintf(stderr, "Behringer source not found. Available:\n");
		ItemCount n = MIDIGetNumberOfSources();
		for (ItemCount i = 0; i < n; i++) {
			MIDIEndpointRef src = MIDIGetSource(i);
			CFStringRef name = NULL;
			MIDIObjectGetStringProperty(src, kMIDIPropertyName, &name);
			if (name) {
				char buf[256];
				CFStringGetCString(name, buf, sizeof(buf),
						   kCFStringEncodingUTF8);
				CFRelease(name);
				fprintf(stderr, "  [%lu] %s\n",
					(unsigned long)i, buf);
			}
		}
		MIDIPortDispose(midi_output_port);
		MIDIPortDispose(midi_input_port);
		MIDIClientDispose(midi_client);
		return -1;
	}

	/* Find Behringer destination (output) */
	midi_dest = find_behringer_dest();
	if (!midi_dest) {
		fprintf(stderr, "Warning: Behringer output not found, "
			"LCD functions disabled\n");
		/* Continue anyway - input still works */
	}

	/* Connect source to input port */
	status = MIDIPortConnectSource(midi_input_port, midi_source, NULL);
	if (status != noErr) {
		fprintf(stderr, "Failed to connect MIDI source: %d\n",
			(int)status);
		MIDIPortDispose(midi_output_port);
		MIDIPortDispose(midi_input_port);
		MIDIClientDispose(midi_client);
		return -1;
	}

	initialized = 1;
	printf("Plotter MIDI initialized (Behringer)\n");
	return 0;
}

int input_plotter_poll(struct plotter_event *ev)
{
	if (!initialized)
		return 0;
	return queue_pop(ev);
}

void input_plotter_cleanup(void)
{
	if (initialized) {
		if (midi_source)
			MIDIPortDisconnectSource(midi_input_port, midi_source);
		if (midi_output_port)
			MIDIPortDispose(midi_output_port);
		if (midi_input_port)
			MIDIPortDispose(midi_input_port);
		if (midi_client)
			MIDIClientDispose(midi_client);
		initialized = 0;
		printf("Plotter MIDI closed\n");
	}
}

int input_plotter_set_lcd(int display, int color,
			  const char *top, const char *bottom)
{
	if (!initialized || !midi_dest)
		return -1;

	if (display < 0 || display > 7)
		return -1;

	/*
	 * SysEx format for X-Touch Extender LCD:
	 * F0 00 20 32 15 4C [nr] [color] [7 chars top] [7 chars bottom] F7
	 * Total: 23 bytes
	 */
	Byte sysex[23];
	int idx = 0;

	/* Header */
	for (int i = 0; i < 6; i++)
		sysex[idx++] = SYSEX_LCD_HEADER[i];

	/* Display number and color */
	sysex[idx++] = (Byte)display;
	sysex[idx++] = (Byte)color;

	/* Top line - 7 characters, pad with spaces */
	for (int i = 0; i < 7; i++) {
		if (top && top[i] != '\0')
			sysex[idx++] = (Byte)top[i];
		else
			sysex[idx++] = ' ';
		if (top && top[i] == '\0')
			top = NULL;  /* Stop reading after null */
	}

	/* Bottom line - 7 characters, pad with spaces */
	for (int i = 0; i < 7; i++) {
		if (bottom && bottom[i] != '\0')
			sysex[idx++] = (Byte)bottom[i];
		else
			sysex[idx++] = ' ';
		if (bottom && bottom[i] == '\0')
			bottom = NULL;
	}

	/* SysEx end */
	sysex[idx++] = 0xF7;

	/* Send via CoreMIDI */
	Byte buffer[256];
	MIDIPacketList *pktlist = (MIDIPacketList *)buffer;
	MIDIPacket *pkt = MIDIPacketListInit(pktlist);
	pkt = MIDIPacketListAdd(pktlist, sizeof(buffer), pkt, 0, 23, sysex);

	if (!pkt) {
		fprintf(stderr, "Failed to create MIDI packet\n");
		return -1;
	}

	OSStatus status = MIDISend(midi_output_port, midi_dest, pktlist);
	if (status != noErr) {
		fprintf(stderr, "Failed to send MIDI: %d\n", (int)status);
		return -1;
	}

	return 0;
}

void input_plotter_clear_all_lcd(void)
{
	for (int i = 0; i < 8; i++) {
		input_plotter_set_lcd(i, LCD_COLOR_OFF, "", "");
	}
}

int input_plotter_set_fader(int fader, float value)
{
	if (!initialized || !midi_dest)
		return -1;

	if (fader < 0 || fader > 6)
		return -1;

	/* Clamp value to 0-1 and convert to 0-127 */
	if (value < 0.0f) value = 0.0f;
	if (value > 1.0f) value = 1.0f;
	Byte midi_value = (Byte)(value * 127.0f);

	/* CC message: status, cc number, value */
	Byte cc_msg[3] = {
		0xB0,  /* Control Change, channel 0 */
		(Byte)(MIDI_CC_FADER_0 + fader),
		midi_value
	};

	/* Send via CoreMIDI */
	Byte buffer[64];
	MIDIPacketList *pktlist = (MIDIPacketList *)buffer;
	MIDIPacket *pkt = MIDIPacketListInit(pktlist);
	pkt = MIDIPacketListAdd(pktlist, sizeof(buffer), pkt, 0, 3, cc_msg);

	if (!pkt)
		return -1;

	OSStatus status = MIDISend(midi_output_port, midi_dest, pktlist);
	return (status == noErr) ? 0 : -1;
}

void input_plotter_set_all_faders(const float *values)
{
	for (int i = 0; i < 7; i++) {
		input_plotter_set_fader(i, values[i]);
	}
}
