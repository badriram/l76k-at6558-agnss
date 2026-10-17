CC ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror
CPPFLAGS += -Iinclude

all: test

build/test_casic: src/casic_agnss.c tests/test_casic.c include/casic_agnss.h
	@mkdir -p build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/casic_agnss.c tests/test_casic.c -lm -o $@

build/example_assist: src/casic_agnss.c examples/assist_cold_start.c include/casic_agnss.h
	@mkdir -p build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DCASIC_EXAMPLE_HOST src/casic_agnss.c examples/assist_cold_start.c -o $@

test: build/test_casic build/example_assist
	./build/test_casic

clean:
	rm -rf build

.PHONY: all test clean
