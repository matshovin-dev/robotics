#!/usr/bin/env python3
"""
MIDI Scanner for Behringer X-Touch Extender
Prints CC numbers and values as you move controls.

Usage: python3 midi_scan.py

Requires: pip3 install rtmidi
"""

import rtmidi
import sys

def list_ports(midi_in):
    """List available MIDI input ports"""
    ports = midi_in.get_ports()
    if not ports:
        print("No MIDI input ports found!")
        return None

    print("Available MIDI ports:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port}")
    return ports

def find_xtouch(ports):
    """Find X-Touch Extender port"""
    for i, port in enumerate(ports):
        if 'X-Touch' in port or 'XTOUCH' in port.upper():
            return i
    return None

def main():
    midi_in = rtmidi.MidiIn()

    ports = list_ports(midi_in)
    if not ports:
        return 1

    # Try to find X-Touch automatically
    port_num = find_xtouch(ports)

    if port_num is None:
        print("\nX-Touch not found automatically.")
        try:
            port_num = int(input("Enter port number: "))
        except (ValueError, EOFError):
            return 1
    else:
        print(f"\nFound X-Touch at port {port_num}")

    try:
        midi_in.open_port(port_num)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return 1

    print(f"\nListening on: {ports[port_num]}")
    print("Move controls to see MIDI messages...")
    print("Press Ctrl+C to quit\n")
    print("-" * 50)
    print(f"{'Type':<12} {'Channel':<8} {'CC/Note':<8} {'Value':<8}")
    print("-" * 50)

    try:
        while True:
            msg = midi_in.get_message()
            if msg:
                data, delta = msg
                if len(data) >= 3:
                    status = data[0]
                    channel = (status & 0x0F) + 1
                    msg_type = status & 0xF0

                    if msg_type == 0xB0:  # Control Change
                        cc = data[1]
                        value = data[2]
                        print(f"{'CC':<12} {channel:<8} {cc:<8} {value:<8}")
                    elif msg_type == 0x90:  # Note On
                        note = data[1]
                        velocity = data[2]
                        if velocity > 0:
                            print(f"{'Note ON':<12} {channel:<8} {note:<8} {velocity:<8}")
                        else:
                            print(f"{'Note OFF':<12} {channel:<8} {note:<8} {velocity:<8}")
                    elif msg_type == 0x80:  # Note Off
                        note = data[1]
                        velocity = data[2]
                        print(f"{'Note OFF':<12} {channel:<8} {note:<8} {velocity:<8}")
                    elif msg_type == 0xE0:  # Pitch Bend (faders often use this)
                        lsb = data[1]
                        msb = data[2]
                        value = (msb << 7) | lsb
                        print(f"{'Pitch/Fader':<12} {channel:<8} {'-':<8} {value:<8} (14-bit)")
                    else:
                        print(f"{'Unknown':<12} {hex(status):<8} {data[1]:<8} {data[2]:<8}")
    except KeyboardInterrupt:
        print("\n\nDone!")

    midi_in.close_port()
    return 0

if __name__ == '__main__':
    sys.exit(main())
