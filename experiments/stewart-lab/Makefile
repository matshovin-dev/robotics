CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I../../viz-modules/common/include
LDFLAGS = -lm

# Stewart platform paths
STEWART_DIR = ../../platforms/stewart
STEWART_INCLUDE = $(STEWART_DIR)/include
MATH_INCLUDE = ../../libs/math/include

# Stewart platform object files
STEWART_OBJS = $(STEWART_DIR)/build/geometry.o \
               $(STEWART_DIR)/build/pose.o \
               $(STEWART_DIR)/build/inverse.o \
               $(STEWART_DIR)/build/forward.o \
               $(STEWART_DIR)/build/math_vec3.o \
               $(STEWART_DIR)/build/math_matrix.o \
               $(STEWART_DIR)/build/math_geometry.o \
               $(STEWART_DIR)/build/math_utils.o

# Viz common directory
VIZ_COMMON_DIR = ../../viz-modules/common

BUILD_DIR = build

# List of all experiment executables
EXECUTABLES = motion_patterns interactive_pose compare_demo

# Default target builds all experiments
all: $(addprefix $(BUILD_DIR)/,$(EXECUTABLES))

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Pattern rule for simple executables (no Stewart dependency)
$(BUILD_DIR)/motion_patterns: src/motion_patterns.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

$(BUILD_DIR)/interactive_pose: src/interactive_pose.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

# Build viz_sender object
$(BUILD_DIR)/viz_sender.o: $(VIZ_COMMON_DIR)/src/viz_sender.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STEWART_INCLUDE) -I$(MATH_INCLUDE) -c $< -o $@

# compare_demo needs Stewart platform and viz_sender
$(BUILD_DIR)/compare_demo: src/compare_demo.c $(STEWART_OBJS) $(BUILD_DIR)/viz_sender.o | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STEWART_INCLUDE) -I$(MATH_INCLUDE) $< $(STEWART_OBJS) $(BUILD_DIR)/viz_sender.o -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)

# Run the motion_patterns experiment by default
run: $(BUILD_DIR)/motion_patterns
	$(BUILD_DIR)/motion_patterns

.PHONY: all clean run
