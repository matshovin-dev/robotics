CC = gcc
CFLAGS = -Wall -Wextra -std=c11 \
	 -I../common/include \
	 -I../../libs/math/include \
	 -I../../libs/obj_loader/include \
	 -I../../libs/graphics/include \
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

# OBJ loader sources
OBJ_LOADER_LIB = ../../libs/obj_loader
OBJ_LOADER_SRC = $(OBJ_LOADER_LIB)/src/obj_loader.c
OBJ_LOADER_OBJ = $(BUILD_DIR)/obj_loader.o

# Graphics library sources
GRAPHICS_LIB = ../../libs/graphics
GRAPHICS_SRC = $(GRAPHICS_LIB)/src/graphics.c
GRAPHICS_OBJ = $(BUILD_DIR)/graphics.o

# Viz common sources
VIZ_COMMON_SRC = ../common/src/udp.c
VIZ_COMMON_OBJ = $(BUILD_DIR)/udp.o

# Main visualizer source
VIZ_SRC = src/main.c
VIZ_OBJ = $(BUILD_DIR)/main.o

# All objects
OBJ = $(MATH_OBJ) $(STEWART_OBJ) $(OBJ_LOADER_OBJ) $(GRAPHICS_OBJ) $(VIZ_COMMON_OBJ) $(VIZ_OBJ)

# Default target
all: plot_stw_obj

plot_stw_obj: $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

test_animation: test_animation.c ../common/src/viz_sender.c ../common/src/udp.c $(BUILD_DIR)/stewart_geometry.o | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ -lm

# Build math library objects
$(BUILD_DIR)/math_%.o: $(MATH_LIB)/src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build stewart platform objects
$(BUILD_DIR)/stewart_%.o: $(STEWART_LIB)/src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build OBJ loader objects
$(BUILD_DIR)/obj_loader.o: $(OBJ_LOADER_LIB)/src/obj_loader.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build graphics library objects
$(BUILD_DIR)/graphics.o: $(GRAPHICS_LIB)/src/graphics.c | $(BUILD_DIR)
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
	rm -rf $(BUILD_DIR) plot_stw_obj test_animation

run: plot_stw_obj
	./plot_stw_obj

test: test_animation
	./test_animation

.PHONY: all clean run test
