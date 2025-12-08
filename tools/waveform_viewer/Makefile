CC = clang
LDFLAGS_CUE = -framework AudioToolbox -framework CoreFoundation

all: cue_recorder cue_player cue_expand coreo_player

cue_recorder: cue_recorder.c
	$(CC) -Wall -O2 -o $@ $< $(LDFLAGS_CUE)

cue_player: cue_player.c
	$(CC) -Wall -O2 -o $@ $< $(LDFLAGS_CUE)

cue_expand: cue_expand.c
	$(CC) -Wall -O2 -o $@ $<

coreo_player: coreo_player.c
	$(CC) -Wall -O2 -o $@ $<

clean:
	rm -f cue_recorder cue_player cue_expand coreo_player

.PHONY: all clean
