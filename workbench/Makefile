CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 \
	-I ../libs/move_lib/include \
	-I ../libs/input/include \
	-I ../libs/math/include \
	-I ../libs/song_lib/include \
	-I ../platforms/stewart/include \
	-I ../viz-modules/common/include

LDFLAGS = -lm

# Check for macOS (CoreMIDI + AudioToolbox)
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
    LDFLAGS += -framework CoreMIDI -framework CoreFoundation -framework AudioToolbox
    INPUT_MIDI = ../libs/input/build/input_midi.o
endif

# Object files from other libraries
MOVE_LIB = ../libs/move_lib/build/move_lib.o
INPUT_LIB = ../libs/input/build/input_keyboard.o $(INPUT_MIDI)
STEWART_GEOM = ../platforms/stewart/build/geometry.o
MATH_UTILS = ../libs/math/build/utils.o
SONG_LIB = ../libs/song_lib/build/song_lib.o \
           ../libs/song_lib/build/song_player.o \
           ../libs/song_lib/build/cJSON.o
VIZ_COMMON = ../viz-modules/common/src/viz_sender.c \
             ../viz-modules/common/src/viz_status.c \
             ../viz-modules/common/src/udp.c

DEPS = $(MOVE_LIB) $(STEWART_GEOM) $(MATH_UTILS) $(SONG_LIB)

.PHONY: all clean deps

all: deps wb_move_test wb_mixer_test wb_input_test wb_coreo_player wb_plotter

deps:
	$(MAKE) -C ../libs/math
	$(MAKE) -C ../libs/move_lib
	$(MAKE) -C ../libs/input
	$(MAKE) -C ../libs/song_lib
	$(MAKE) -C ../platforms/stewart

wb_move_test: wb_move_test.c $(DEPS)
	$(CC) $(CFLAGS) $< $(DEPS) $(VIZ_COMMON) -o $@ $(LDFLAGS)

wb_mixer_test: wb_mixer_test.c $(DEPS)
	$(CC) $(CFLAGS) $< $(DEPS) $(VIZ_COMMON) -o $@ $(LDFLAGS)

wb_input_test: wb_input_test.c $(DEPS) $(INPUT_LIB)
	$(CC) $(CFLAGS) $< $(DEPS) $(INPUT_LIB) $(VIZ_COMMON) -o $@ $(LDFLAGS)

wb_coreo_player: wb_coreo_player.c $(DEPS)
	$(CC) $(CFLAGS) $< $(DEPS) $(VIZ_COMMON) -o $@ $(LDFLAGS)

wb_plotter: wb_plotter.c
	$(CC) -Wall -std=c11 -O2 $< -o $@ -lm

CJSON = ../libs/song_lib/build/cJSON.o

wb_plotter_live: wb_plotter_live.c $(MOVE_LIB) $(CJSON) $(STEWART_GEOM) $(MATH_UTILS)
	$(CC) -Wall -std=c11 -O2 -I ../libs/move_lib/include -I ../platforms/stewart/include -I ../libs/math/include -I ../viz-modules/common/include $< $(MOVE_LIB) $(CJSON) $(STEWART_GEOM) $(MATH_UTILS) $(VIZ_COMMON) -o $@ -lm $(shell sdl2-config --cflags --libs)

clean:
	rm -f wb_move_test wb_mixer_test wb_input_test wb_coreo_player wb_plotter wb_plotter_live
