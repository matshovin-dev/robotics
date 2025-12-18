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

/* MIDI NOTE mappings - button row 1 */
#define MIDI_NOTE_RUN         8
#define MIDI_NOTE_SPLINE_MODE 9
#define MIDI_NOTE_TIME_LEFT   10
#define MIDI_NOTE_TIME_RIGHT  11
#define MIDI_NOTE_SAVE        12

/* Event queue */
#define EVENT_QUEUE_SIZE 32
static struct plotter_event event_queue[EVENT_QUEUE_SIZE];
static int queue_head = 0;
static int queue_tail = 0;

/* CoreMIDI handles */
static MIDIClientRef midi_client = 0;
static MIDIPortRef midi_port = 0;
static MIDIEndpointRef midi_source = 0;
static int initialized = 0;

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
		return 1.0f;   /* clockwise */
	if (value == 65)
		return -1.0f;  /* counter-clockwise */
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

/* Find Behringer source */
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
	status = MIDIInputPortCreate(midi_client, CFSTR("PlotterPort"),
				     midi_read_callback, NULL, &midi_port);
	if (status != noErr) {
		fprintf(stderr, "Failed to create MIDI port: %d\n",
			(int)status);
		MIDIClientDispose(midi_client);
		return -1;
	}

	/* Find Behringer */
	midi_source = find_behringer_source();
	if (!midi_source) {
		fprintf(stderr, "Behringer not found. Available sources:\n");
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
		MIDIPortDispose(midi_port);
		MIDIClientDispose(midi_client);
		return -1;
	}

	/* Connect source to port */
	status = MIDIPortConnectSource(midi_port, midi_source, NULL);
	if (status != noErr) {
		fprintf(stderr, "Failed to connect MIDI source: %d\n",
			(int)status);
		MIDIPortDispose(midi_port);
		MIDIClientDispose(midi_client);
		return -1;
	}

	initialized = 1;
	printf("Plotter MIDI input initialized (Behringer)\n");
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
			MIDIPortDisconnectSource(midi_port, midi_source);
		if (midi_port)
			MIDIPortDispose(midi_port);
		if (midi_client)
			MIDIClientDispose(midi_client);
		initialized = 0;
		printf("Plotter MIDI input closed\n");
	}
}
