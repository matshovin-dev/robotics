CC = gcc
CFLAGS = -Wall -Wextra -std=c11 \
	 -I../common/include \
	 -I../../libs/math/include \
	 -I../../platforms/stewart/include \
	 -I/opt/homebrew/include \
	 -DGL_SILENCE_DEPRECATION
LDFLAGS = -L/opt/homebrew/lib -lglfw -lm -framework OpenGL -framework Cocoa -framework IOKit

BUILD_DIR = build

# Math library sources
MATH_LIB = ../../libs/math
MATH_SRC = $(MATH_LIB)/src/vec3.c \
	   $(MATH_LIB)/src/matrix.c \
	   $(MATH_LIB)/src/utils.c \
	   $(MATH_LIB)/src/geometry.c
MATH_OBJ = $(MATH_SRC:$(MATH_LIB)/src/%.c=$(BUILD_DIR)/math_%.o)

# Stewart platform sources
STEWART_LIB = ../../platforms/stewart
STEWART_SRC = $(STEWART_LIB)/src/geometry.c \
	      $(STEWART_LIB)/src/pose.c \
	      $(STEWART_LIB)/src/inverse.c \
	      $(STEWART_LIB)/src/forward.c
STEWART_OBJ = $(STEWART_SRC:$(STEWART_LIB)/src/%.c=$(BUILD_DIR)/stewart_%.o)

# Viz common sources
VIZ_COMMON_SRC = ../common/src/udp.c
VIZ_COMMON_OBJ = $(BUILD_DIR)/udp.o

# Main visualizer source
VIZ_SRC = src/main.c
VIZ_OBJ = $(BUILD_DIR)/main.o

# All objects
OBJ = $(MATH_OBJ) $(STEWART_OBJ) $(VIZ_COMMON_OBJ) $(VIZ_OBJ)

# Default target
all: viz-stewart-kinematics

viz-stewart-kinematics: $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

# Build math library objects
$(BUILD_DIR)/math_%.o: $(MATH_LIB)/src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build stewart platform objects
$(BUILD_DIR)/stewart_%.o: $(STEWART_LIB)/src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build viz common objects
$(BUILD_DIR)/udp.o: ../common/src/udp.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build main visualizer object
$(BUILD_DIR)/main.o: src/main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) viz-stewart-kinematics

run: viz-stewart-kinematics
	./viz-stewart-kinematics

.PHONY: all clean run
