CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I../../viz-modules/common/include
LDFLAGS = -lm

SRC = src/main.c
OBJ = $(SRC:.c=.o)

stewart-lab: $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) stewart-lab

run: stewart-lab
	./stewart-lab

.PHONY: clean run
