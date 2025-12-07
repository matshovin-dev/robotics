CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -Iinclude

SRC = src/vec3.c src/matrix.c src/geometry.c src/utils.c
OBJ = $(SRC:src/%.c=build/%.o)

.PHONY: all clean test

all: $(OBJ)

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build:
	mkdir -p build

clean:
	rm -rf build

test: build/test_utils build/test_vec3 build/test_matrix build/test_geometry
	./build/test_utils
	./build/test_vec3
	./build/test_matrix
	./build/test_geometry

build/test_%: tests/test_%.c $(OBJ) | build
	$(CC) $(CFLAGS) -o $@ $< $(OBJ) -lm
