CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 -I include \
	-I ../../platforms/stewart/include \
	-I ../../libs/math/include

SRC_DIR = src
BUILD_DIR = build

OBJS = $(BUILD_DIR)/move_lib.o

.PHONY: all clean

all: $(BUILD_DIR) $(OBJS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/move_lib.o: $(SRC_DIR)/move_lib.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
