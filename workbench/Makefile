CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 \
	-I ../libs/move_lib/include \
	-I ../libs/math/include \
	-I ../platforms/stewart/include \
	-I ../viz-modules/common/include

LDFLAGS = -lm

# Object files from other libraries
MOVE_LIB = ../libs/move_lib/build/move_lib.o
STEWART_GEOM = ../platforms/stewart/build/geometry.o
VIZ_SENDER = ../viz-modules/common/src/viz_sender.c \
             ../viz-modules/common/src/udp.c

DEPS = $(MOVE_LIB) $(STEWART_GEOM)

.PHONY: all clean deps

all: deps wb_move_test wb_mixer_test

deps:
	$(MAKE) -C ../libs/move_lib
	$(MAKE) -C ../platforms/stewart

wb_move_test: wb_move_test.c $(DEPS)
	$(CC) $(CFLAGS) $< $(DEPS) $(VIZ_SENDER) -o $@ $(LDFLAGS)

wb_mixer_test: wb_mixer_test.c $(DEPS)
	$(CC) $(CFLAGS) $< $(DEPS) $(VIZ_SENDER) -o $@ $(LDFLAGS)

clean:
	rm -f wb_move_test wb_mixer_test
