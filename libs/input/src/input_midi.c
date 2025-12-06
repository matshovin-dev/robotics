/**
 * @file input_midi.c
 * @brief MIDI input source for Behringer X-Touch Extender
 *
 * X-Touch Extender mappings (all on channel 1):
 *   Fader note 70  - Volume A (0-127)
 *   Fader note 71  - Volume B (0-127)
 *   Fader note 72  - Crossfade (0-127)
 *   Encoder note 80 - BPM (relative: 1=up, 65=down)
 *   Encoder note 81 - Phase (relative: 1=up, 65=down)
 *   Encoder note 82 - Move selector for deck B (relative)
 *   Button note 8   - Copy move 0 -> 99
 *
 * Uses CoreMIDI on macOS.
 */

#include "input.h"
#include <stdio.h>
#include <string.h>
#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>

/* MIDI note mappings */
#define MIDI_NOTE_VOLUME_A    70
#define MIDI_NOTE_VOLUME_B    71
#define MIDI_NOTE_CROSSFADE   72
#define MIDI_NOTE_BPM         80
#define MIDI_NOTE_PHASE       81
#define MIDI_NOTE_MOVE_SEL    82
#define MIDI_NOTE_COPY        8

/* Encoder step sizes */
#define PHASE_STEP    0.1f    /* radians */
#define BPM_STEP      1.0f    /* BPM */

/* Event queue */
#define EVENT_QUEUE_SIZE 32
static struct input_event event_queue[EVENT_QUEUE_SIZE];
static int queue_head = 0;
static int queue_tail = 0;

/* Current move selection for deck B */
static int current_move_b = 1;

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

static void queue_push(struct input_event *ev)
{
	if (queue_full())
		return;
	event_queue[queue_tail] = *ev;
	queue_tail = (queue_tail + 1) % EVENT_QUEUE_SIZE;
}

static int queue_pop(struct input_event *ev)
{
	if (queue_empty())
		return 0;
	*ev = event_queue[queue_head];
	queue_head = (queue_head + 1) % EVENT_QUEUE_SIZE;
	return 1;
}

/* Convert relative encoder value to delta */
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
			int note = packet->data[1];
			int value = packet->data[2];
			int channel = (status & 0x0F) + 1;
			int msg_type = status & 0xF0;

			struct input_event ev = { -1, -1, 0 };
			int cc = note;  /* For CC messages, data[1] is CC number */

			/* Pitch Bend (0xE0) - faders (14-bit), uses channel for fader # */
			if (msg_type == 0xE0) {
				int lsb = packet->data[1];
				int msb = packet->data[2];
				float fader_val = ((msb << 7) | lsb) / 16383.0f;
				int fader_num = status & 0x0F;  /* channel 0-7 = fader 1-8 */

				switch (fader_num) {
				case 0:  /* Fader 1 = Volume A */
					ev.type = INPUT_FADER;
					ev.id = INPUT_ID_VOLUME_A;
					ev.value = fader_val;
					break;
				case 1:  /* Fader 2 = Volume B */
					ev.type = INPUT_FADER;
					ev.id = INPUT_ID_VOLUME_B;
					ev.value = fader_val;
					break;
				case 2:  /* Fader 3 = Crossfade */
					ev.type = INPUT_FADER;
					ev.id = INPUT_ID_CROSSFADE;
					ev.value = fader_val;
					break;
				}
			}
			/* Handle channel 1 for CC and Note messages */
			else if (channel == 1) {
				/* Control Change (0xB0) - encoders (and fallback faders) */
				if (msg_type == 0xB0) {
					switch (cc) {
					case MIDI_NOTE_VOLUME_A:
						ev.type = INPUT_FADER;
						ev.id = INPUT_ID_VOLUME_A;
						ev.value = value / 127.0f;
						break;

					case MIDI_NOTE_VOLUME_B:
						ev.type = INPUT_FADER;
						ev.id = INPUT_ID_VOLUME_B;
						ev.value = value / 127.0f;
						break;

					case MIDI_NOTE_CROSSFADE:
						ev.type = INPUT_FADER;
						ev.id = INPUT_ID_CROSSFADE;
						ev.value = value / 127.0f;
						break;

					case MIDI_NOTE_BPM:
						ev.type = INPUT_ENCODER;
						ev.id = INPUT_ID_BPM;
						ev.value = encoder_delta(value) * BPM_STEP;
						break;

					case MIDI_NOTE_PHASE:
						ev.type = INPUT_ENCODER;
						ev.id = INPUT_ID_PHASE;
						ev.value = encoder_delta(value) * PHASE_STEP;
						break;

					case MIDI_NOTE_MOVE_SEL:
						/* Encoder to select move for deck B */
						current_move_b += (int)encoder_delta(value);
						if (current_move_b < 0)
							current_move_b = 0;
						if (current_move_b > 98)
							current_move_b = 98;
						ev.type = INPUT_BUTTON;
						ev.id = INPUT_ID_MOVE_0 + current_move_b;
						ev.value = 1.0f;
						break;
					}
				}
				/* Note On (0x90) - buttons */
				else if (msg_type == 0x90) {
					switch (note) {
					case MIDI_NOTE_COPY:
						if (value > 0) {  /* button press */
							ev.type = INPUT_BUTTON;
							ev.id = INPUT_ID_COPY;
							ev.value = 1.0f;
						}
						break;
					}
				}
			}

			if (ev.type >= 0)
				queue_push(&ev);
		}
		packet = MIDIPacketNext(packet);
	}
}

/* Find X-Touch source */
static MIDIEndpointRef find_xtouch_source(void)
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

			if (strstr(buf, "X-Touch") || strstr(buf, "X-TOUCH")) {
				printf("Found MIDI source: %s\n", buf);
				return src;
			}
		}
	}
	return 0;
}

int input_midi_init(void)
{
	OSStatus status;

	if (initialized)
		return 0;

	/* Create MIDI client */
	status = MIDIClientCreate(CFSTR("RoboticsInput"),
				  NULL, NULL, &midi_client);
	if (status != noErr) {
		fprintf(stderr, "Failed to create MIDI client: %d\n",
			(int)status);
		return -1;
	}

	/* Create input port */
	status = MIDIInputPortCreate(midi_client, CFSTR("Input"),
				     midi_read_callback, NULL, &midi_port);
	if (status != noErr) {
		fprintf(stderr, "Failed to create MIDI port: %d\n",
			(int)status);
		MIDIClientDispose(midi_client);
		return -1;
	}

	/* Find X-Touch */
	midi_source = find_xtouch_source();
	if (!midi_source) {
		fprintf(stderr, "X-Touch not found. Available sources:\n");
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
	current_move_b = 1;
	printf("MIDI input initialized (X-Touch)\n");
	return 0;
}

int input_midi_poll(struct input_event *ev)
{
	if (!initialized)
		return 0;
	return queue_pop(ev);
}

void input_midi_cleanup(void)
{
	if (initialized) {
		if (midi_source)
			MIDIPortDisconnectSource(midi_port, midi_source);
		if (midi_port)
			MIDIPortDispose(midi_port);
		if (midi_client)
			MIDIClientDispose(midi_client);
		initialized = 0;
		printf("MIDI input closed\n");
	}
}

int input_midi_get_current_move(void)
{
	return current_move_b;
}
