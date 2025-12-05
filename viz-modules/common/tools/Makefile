CC = gcc
CFLAGS = -Wall -Wextra -std=c11 \
	 -I../include \
	 -I../../../libs/math/include \
	 -I../../../platforms/stewart/include

LDFLAGS = -lm

STEWART_LIB = ../../../platforms/stewart
STEWART_OBJ = $(STEWART_LIB)/build/geometry.o

VIZ_COMMON = ../src/viz_sender.c ../src/udp.c

all: interactive_pose

interactive_pose: interactive_pose.c $(VIZ_COMMON) $(STEWART_OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f interactive_pose

.PHONY: all clean
