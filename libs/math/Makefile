CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -Iinclude -I../../libs/math/include
LDFLAGS = -lm

# Math library
MATH_LIB = ../../libs/math
MATH_SRC = $(MATH_LIB)/src/vec3.c \
           $(MATH_LIB)/src/matrix.c \
           $(MATH_LIB)/src/geometry.c \
           $(MATH_LIB)/src/utils.c
MATH_OBJ = $(MATH_SRC:$(MATH_LIB)/src/%.c=build/math_%.o)

# Stewart platform sources
STEWART_SRC = src/geometry.c src/pose.c src/inverse.c src/forward.c
STEWART_OBJ = $(STEWART_SRC:src/%.c=build/%.o)

# All objects
OBJ = $(STEWART_OBJ) $(MATH_OBJ)

# Test programs
TESTS = build/test_kinematics

# Default target: run all tests
test: $(TESTS)
	@echo "================================"
	@echo "Running Stewart kinematics tests"
	@echo "================================"
	@echo ""
	./build/test_kinematics
	@echo ""
	@echo "================================"
	@echo "All tests passed!"
	@echo "================================"

# Build test executable
build/test_kinematics: tests/test_kinematics.c $(OBJ) | build
	$(CC) $(CFLAGS) -o $@ tests/test_kinematics.c $(OBJ) $(LDFLAGS)

# Compile Stewart platform objects
build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

# Compile math library objects
build/math_%.o: $(MATH_LIB)/src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

# Create build directory
build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: clean test
```

**VIKTIG:** Linjene som starter med `$(CC)`, `@echo`, `./build`, og `mkdir` **MÅ** ha en TAB først, ikke spaces!

---

## **Også: Du har `reverse.c` i stedet for `inverse.c`!**

Fra tree output:
```
└── src
    ├── forward.c
    ├── geometry.c
    ├── pose.c
    └── reverse.c    ← Skal være inverse.c!