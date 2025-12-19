CC = gcc
CFLAGS = -Wall -Wextra -std=c11 \
	 -I../common/include \
	 -I/opt/homebrew/include \
	 -DGL_SILENCE_DEPRECATION
LDFLAGS = -L/opt/homebrew/lib -lglfw -lm -framework OpenGL -framework Cocoa -framework IOKit

BUILD_DIR = build

# Viz common sources
VIZ_COMMON_SRC = ../common/src/udp.c
VIZ_COMMON_OBJ = $(BUILD_DIR)/udp.o

# Main visualizer source
VIZ_SRC = src/main.c
VIZ_OBJ = $(BUILD_DIR)/main.o

# All objects
OBJ = $(VIZ_COMMON_OBJ) $(VIZ_OBJ)

# Default target
all: plot_move_bars

plot_move_bars: $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

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
	rm -rf $(BUILD_DIR) plot_move_bars

run: plot_move_bars
	./plot_move_bars

.PHONY: all clean run
