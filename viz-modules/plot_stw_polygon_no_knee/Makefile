CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I../common/include -I../../libs/math/include -I../../platforms/stewart/include -I/opt/homebrew/include -DGL_SILENCE_DEPRECATION
LDFLAGS = -L/opt/homebrew/lib -lglfw -lm -framework OpenGL -framework Cocoa -framework IOKit

MATH_LIB = ../../libs/math
MATH_SRC = $(MATH_LIB)/src/vec3.c $(MATH_LIB)/src/matrix.c $(MATH_LIB)/src/utils.c
MATH_OBJ = $(MATH_SRC:$(MATH_LIB)/src/%.c=build/math_%.o)

STEWART_LIB = ../../platforms/stewart
STEWART_SRC = $(STEWART_LIB)/src/geometry.c
STEWART_OBJ = $(STEWART_SRC:$(STEWART_LIB)/src/%.c=build/stewart_%.o)

VIZ_SRC = src/main.c ../common/src/udp.c
VIZ_OBJ = $(VIZ_SRC:.c=.o)

OBJ = $(VIZ_OBJ) $(MATH_OBJ) $(STEWART_OBJ)

plot_stw_polygon_no_knee: $(OBJ) | build
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

test_animation: test_animation.c ../common/src/viz_sender.c ../common/src/udp.c $(STEWART_OBJ) | build
	$(CC) $(CFLAGS) $^ -o $@ -lm

%.o: %.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/math_%.o: $(MATH_LIB)/src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/stewart_%.o: $(STEWART_LIB)/src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build:
	mkdir -p build

clean:
	rm -f $(VIZ_OBJ) $(MATH_OBJ) $(STEWART_OBJ) plot_stw_polygon_no_knee test_animation
	rm -rf build

run: plot_stw_polygon_no_knee
	./plot_stw_polygon_no_knee

test: test_animation
	./test_animation

.PHONY: clean run test
