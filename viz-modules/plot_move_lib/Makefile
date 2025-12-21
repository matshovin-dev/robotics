CC = gcc
CFLAGS = -Wall -Wextra -std=c11 \
	 -I../common/include \
	 -I../../libs/song_lib/vendor \
	 -I/opt/homebrew/include \
	 -DGL_SILENCE_DEPRECATION
LDFLAGS = -L/opt/homebrew/lib -lglfw -lm -framework OpenGL -framework Cocoa -framework IOKit

BUILD_DIR = build

# cJSON source
CJSON_SRC = ../../libs/song_lib/vendor/cJSON.c
CJSON_OBJ = $(BUILD_DIR)/cJSON.o

# UDP source
UDP_SRC = ../common/src/udp.c
UDP_OBJ = $(BUILD_DIR)/udp.o

# Main visualizer source
VIZ_SRC = src/main.c
VIZ_OBJ = $(BUILD_DIR)/main.o

# All objects
OBJ = $(CJSON_OBJ) $(UDP_OBJ) $(VIZ_OBJ)

# Default target
all: plot_move_lib

plot_move_lib: $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

# Build cJSON object
$(BUILD_DIR)/cJSON.o: $(CJSON_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build UDP object
$(BUILD_DIR)/udp.o: $(UDP_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build main visualizer object
$(BUILD_DIR)/main.o: src/main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) plot_move_lib

run: plot_move_lib
	./plot_move_lib

.PHONY: all clean run
