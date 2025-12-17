CC = clang
CFLAGS = -Wall -O2
FRAMEWORKS = -framework ApplicationServices -framework Carbon

TARGET = focus
SRC = focus_wind.c

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(FRAMEWORKS)

clean:
	rm -f $(TARGET)

.PHONY: clean
