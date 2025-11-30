CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -Iinclude -I../../libs/math/include
LDFLAGS = -lm

MATH_LIB = ../../libs/math
MATH_SRC = $(MATH_LIB)/src/vec3.c $(MATH_LIB)/src/matrix.c $(MATH_LIB)/src/geometry.c $(MATH_LIB)/src/utils.c
MATH_OBJ = $(MATH_SRC:$(MATH_LIB)/src/%.c=build/math_%.o)

STEWART_SRC = src/geometry.c src/pose.c src/inverse.c src/forward.c
STEWART_OBJ = $(STEWART_SRC:src/%.c=build/%.o)

OBJ = $(STEWART_OBJ) $(MATH_OBJ)

test: build/test_kinematics
	@echo "================================"
	@echo "Running Stewart kinematics tests"
	@echo "================================"
	./build/test_kinematics
	@echo "All tests complete!"

build/test_kinematics: tests/test_kinematics.c $(OBJ) | build
	$(CC) $(CFLAGS) -o $@ tests/test_kinematics.c $(OBJ) $(LDFLAGS)

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/math_%.o: $(MATH_LIB)/src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: clean test
