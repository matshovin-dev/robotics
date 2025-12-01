CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I../../viz-modules/common/include
LDFLAGS = -lm

BUILD_DIR = build

# List of all experiment executables
EXECUTABLES = motion_patterns

# Default target builds all experiments
all: $(addprefix $(BUILD_DIR)/,$(EXECUTABLES))

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Pattern rule for building executables from src/*.c
$(BUILD_DIR)/%: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)

# Run the motion_patterns experiment by default
run: $(BUILD_DIR)/motion_patterns
	$(BUILD_DIR)/motion_patterns

.PHONY: all clean run
