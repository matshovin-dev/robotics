CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 -I include

SRC_DIR = src
BUILD_DIR = build

# Keyboard is always built, MIDI only on macOS
OBJS = $(BUILD_DIR)/input_keyboard.o

# Check for macOS (CoreMIDI)
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    OBJS += $(BUILD_DIR)/input_midi.o
    MIDI_FRAMEWORKS = -framework CoreMIDI -framework CoreFoundation
endif

.PHONY: all clean

all: $(BUILD_DIR) $(OBJS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/input_keyboard.o: $(SRC_DIR)/input_keyboard.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/input_midi.o: $(SRC_DIR)/input_midi.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
