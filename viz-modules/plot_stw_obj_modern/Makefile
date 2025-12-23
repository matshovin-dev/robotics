CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 \
	 -Iinclude \
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

# Local sources
LOCAL_SRC = src/main.c src/shader.c src/mesh.c
LOCAL_OBJ = $(BUILD_DIR)/main.o $(BUILD_DIR)/shader.o $(BUILD_DIR)/mesh.o

# All objects
OBJ = $(MATH_OBJ) $(STEWART_OBJ) $(VIZ_COMMON_OBJ) $(LOCAL_OBJ)

# Default target
all: plot_stw_obj_modern

plot_stw_obj_modern: $(OBJ)
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

# Build local objects
$(BUILD_DIR)/main.o: src/main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/shader.o: src/shader.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/mesh.o: src/mesh.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) plot_stw_obj_modern

run: plot_stw_obj_modern
	./plot_stw_obj_modern

.PHONY: all clean run
