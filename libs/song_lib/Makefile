CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -Iinclude -Ivendor
LDFLAGS = -framework AudioToolbox

OBJ = build/song_lib.o build/song_player.o build/cJSON.o

.PHONY: all clean test

all: $(OBJ)

build/song_lib.o: src/song_lib.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/song_player.o: src/song_player.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/cJSON.o: vendor/cJSON.c | build
	$(CC) $(CFLAGS) -w -c $< -o $@

build/test_player: tests/test_player.c $(OBJ) | build
	$(CC) $(CFLAGS) -o $@ $< $(OBJ) $(LDFLAGS)

test: build/test_player
	@echo "Run: ./build/test_player <wav_file>"

build:
	mkdir -p build

clean:
	rm -rf build
