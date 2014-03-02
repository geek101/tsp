CC=clang++
CFLAGS=-I. -g -std=c++11 -Wall -Wextra -Wpedantic
DEPS =
OBJ = tsp.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

%.o: %.cc $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

tsp: $(OBJ)
	$(CC) -g -o $@ $^

check-syntax:
	$(CC) $(CFLAGS) -Wall -Wextra -0pedantic -fsyntax-only -o nul -S ${CHK_SOURCES}

.PHONY: clean

clean:
	rm -f tsp $(OBJ) *~
