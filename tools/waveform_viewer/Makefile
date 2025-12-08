CC = clang
CFLAGS = -Wall -O2 -I/opt/homebrew/include -I/opt/homebrew/include/freetype2 -I../../libs/text_renderer/include
LDFLAGS_VIEWER = -L/opt/homebrew/lib -lglfw -lfreetype -framework OpenGL -framework Cocoa -framework IOKit -framework AudioToolbox -framework CoreFoundation
LDFLAGS_CUE = -framework AudioToolbox -framework CoreFoundation

TEXT_RENDERER_SRC = ../../libs/text_renderer/src/text_renderer.c

all: waveform_viewer cue_recorder cue_player

waveform_viewer: main.c $(TEXT_RENDERER_SRC)
	$(CC) $(CFLAGS) -o $@ main.c $(TEXT_RENDERER_SRC) $(LDFLAGS_VIEWER)

cue_recorder: cue_recorder.c
	$(CC) -Wall -O2 -o $@ $< $(LDFLAGS_CUE)

cue_player: cue_player.c
	$(CC) -Wall -O2 -o $@ $< $(LDFLAGS_CUE)

clean:
	rm -f waveform_viewer cue_recorder cue_player

.PHONY: all clean
