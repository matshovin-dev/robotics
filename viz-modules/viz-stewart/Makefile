CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I../common/include -I../../libs/math/include -I/opt/homebrew/include -DGL_SILENCE_DEPRECATION
LDFLAGS = -L/opt/homebrew/lib -lglfw -lm -framework OpenGL -framework Cocoa -framework IOKit

MATH_LIB = ../../libs/math
MATH_SRC = $(MATH_LIB)/src/vec3.c $(MATH_LIB)/src/matrix.c $(MATH_LIB)/src/utils.c
MATH_OBJ = $(MATH_SRC:$(MATH_LIB)/src/%.c=build/math_%.o)

VIZ_SRC = src/main.c ../common/src/udp.c
VIZ_OBJ = $(VIZ_SRC:.c=.o)

OBJ = $(VIZ_OBJ) $(MATH_OBJ)

viz-stewart: $(OBJ) | build
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

%.o: %.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/math_%.o: $(MATH_LIB)/src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build:
	mkdir -p build

clean:
	rm -f $(VIZ_OBJ) $(MATH_OBJ) viz-stewart
	rm -rf build

run: viz-stewart
	./viz-stewart

.PHONY: clean run
