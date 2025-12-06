CC = gcc
CFLAGS = -Wall -Wextra -std=c11 \
	 -I../common/include \
	 -I../../libs/text_renderer/include \
	 -I/opt/homebrew/include \
	 -DGL_SILENCE_DEPRECATION
LDFLAGS = -L/opt/homebrew/lib -lglfw -lfreetype -lm \
	  -framework OpenGL -framework Cocoa -framework IOKit

BUILD_DIR = build

# Text renderer
TEXT_LIB = ../../libs/text_renderer
TEXT_SRC = $(TEXT_LIB)/src/text_renderer.c
TEXT_OBJ = $(BUILD_DIR)/text_renderer.o

# Viz common sources
VIZ_COMMON_SRC = ../common/src/udp.c
VIZ_COMMON_OBJ = $(BUILD_DIR)/udp.o

# Main source
MAIN_SRC = src/main.c
MAIN_OBJ = $(BUILD_DIR)/main.o

# All objects
OBJ = $(TEXT_OBJ) $(VIZ_COMMON_OBJ) $(MAIN_OBJ)

# Default target
all: plot_status

plot_status: $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

# Build text renderer
$(BUILD_DIR)/text_renderer.o: $(TEXT_LIB)/src/text_renderer.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I/opt/homebrew/include/freetype2 -c $< -o $@

# Build viz common objects
$(BUILD_DIR)/udp.o: ../common/src/udp.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Build main object
$(BUILD_DIR)/main.o: src/main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I/opt/homebrew/include/freetype2 -c $< -o $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) plot_status

run: plot_status
	./plot_status

.PHONY: all clean run
