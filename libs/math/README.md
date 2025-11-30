# Robotics Math Library

Cross-platform matematikk-bibliotek for robotikk.
Kompilerer på desktop (macOS/Linux) og Teensy 4.1.

## Design Philosophy

- **Kernel-style C**: Ren C11, ingen C++
- **Zero dependencies**: Kun standard library (`math.h`)
- **Cross-platform**: Desktop og embedded
- **Explicit APIs**: Tydelige input/output parametere
- **No dynamic allocation**: Alt stack-allocated

## Structure
└── robotics
    ├── libs
    │   └── math
    │       ├── build
    │       │   ├── geometry.o
    │       │   ├── matrix.o
    │       │   ├── test_geometry
    │       │   ├── test_matrix
    │       │   ├── test_utils
    │       │   ├── test_vec3
    │       │   ├── utils.o
    │       │   └── vec3.o
    │       ├── DESIGN.md
    │       ├── include
    │       │   └── robotics
    │       │       └── math
    │       │           ├── geometry.h
    │       │           ├── matrix.h
    │       │           ├── utils.h
    │       │           └── vec3.h
    │       ├── Makefile
    │       ├── README.md
    │       ├── src
    │       │   ├── geometry.c
    │       │   ├── matrix.c
    │       │   ├── utils.c
    │       │   └── vec3.c
    │       ├── STRUCTURE.txt
    │       └── tests
    │           ├── test_geometry.c
    │           ├── test_matrix.c
    │           ├── test_utils.c
    │           └── test_vec3.c
    └── platforms
        └── stewart
            ├── include
            │   └── stewart
            │       ├── geometry.h
            │       ├── kinematics.h
            │       └── pose.h
            ├── src
            │   ├── forward.c
            │   ├── geometry.c
            │   ├── inverse.c
            │   └── pose.c
            └── tests
                └── test_kinematics.c

## Modules

### vec3 - 3D Vectors

3D vektor-operasjoner for posisjon, retning, og hastighet.

**Operations:**
- Single-vector: `length`, `normalize`, `scale`, `negate`
- Two-vector: `add`, `sub`, `dot`, `cross`, `distance`

## Usage

### Basic Example
```c
#include <robotics/math/vec3.h>
#include 

int main(void)
{
	struct vec3 position = {10.0f, 20.0f, 30.0f};
	struct vec3 velocity = {1.0f, 0.0f, 0.0f};
	struct vec3 new_position;

	/* Update position */
	vec3_add(&position, &velocity, &new_position);

	/* Calculate distance traveled */
	float dist = vec3_distance(&position, &new_position);

	printf("Moved %.2f units\n", dist);

	return 0;
}
```

### Vector Operations
```c
struct vec3 a = {1.0f, 0.0f, 0.0f};
struct vec3 b = {0.0f, 1.0f, 0.0f};
struct vec3 result;

/* Addition */
vec3_add(&a, &b, &result);
/* result = (1.0, 1.0, 0.0) */

/* Cross product */
vec3_cross(&a, &b, &result);
/* result = (0.0, 0.0, 1.0) - perpendicular to both */

/* Dot product */
float dot = vec3_dot(&a, &b);
/* dot = 0.0 - vectors are perpendicular */

/* Normalize */
struct vec3 v = {3.0f, 4.0f, 0.0f};
vec3_normalize(&v);
/* v = (0.6, 0.8, 0.0) - unit vector */
```

### Robotics Example
```c
/* Calculate vector from base to platform */
struct vec3 base_point = {0.0f, 0.0f, 0.0f};
struct vec3 platform_point = {10.0f, 5.0f, 15.0f};
struct vec3 leg_vector;

vec3_sub(&platform_point, &base_point, &leg_vector);

/* Check if within reach */
float leg_length = vec3_length(&leg_vector);
if (leg_length > MAX_LEG_LENGTH) {
	printf("Position unreachable!\n");
	return -1;
}

/* Get unit direction */
struct vec3 leg_direction = leg_vector;
vec3_normalize(&leg_direction);
```

## Building

### Desktop (macOS/Linux)
```bash
# Compile and run tests
make test

# Clean build artifacts
make clean
```

### Integrating into Your Project

**Option 1: Copy files**
```bash
cp -r include/robotics /your/project/include/
cp src/vec3.c /your/project/src/
```

**Option 2: Use as library**
```makefile
# In your Makefile
CFLAGS += -I/path/to/math/include
LDFLAGS += -L/path/to/math/build -lmath
```

### Teensy / PlatformIO
```ini
; platformio.ini
[env:teensy41]
platform = teensy
board = teensy41
lib_deps = 
    file:///path/to/robotics/libs/math
build_flags = 
    -I/path/to/robotics/libs/math/include
```

## API Reference

### struct vec3
```c
struct vec3 {
	float x;
	float y;
	float z;
};
```

### Single-Vector Operations
```c
/* Get length of vector */
float vec3_length(const struct vec3 *v);

/* Get squared length (faster, no sqrt) */
float vec3_length_squared(const struct vec3 *v);

/* Normalize to unit length (modifies in-place) */
void vec3_normalize(struct vec3 *v);

/* Scale by factor (modifies in-place) */
void vec3_scale(struct vec3 *v, float s);

/* Negate vector (modifies in-place) */
void vec3_negate(struct vec3 *v);
```

### Two-Vector Operations
```c
/* Add vectors: result = a + b */
void vec3_add(const struct vec3 *a, const struct vec3 *b, 
              struct vec3 *result);

/* Subtract vectors: result = a - b */
void vec3_sub(const struct vec3 *a, const struct vec3 *b,
              struct vec3 *result);

/* Dot product: returns scalar */
float vec3_dot(const struct vec3 *a, const struct vec3 *b);

/* Cross product: result perpendicular to a and b */
void vec3_cross(const struct vec3 *a, const struct vec3 *b,
                struct vec3 *result);

/* Distance between points */
float vec3_distance(const struct vec3 *a, const struct vec3 *b);

/* Squared distance (faster, no sqrt) */
float vec3_distance_squared(const struct vec3 *a, const struct vec3 *b);
```

## Performance Notes

### When to use `_squared` variants

Use `vec3_length_squared()` and `vec3_distance_squared()` when:
- Comparing distances (sqrt not needed)
- Checking if within radius
- Magnitude comparisons
```c
/* ✅ Efficient - no sqrt */
if (vec3_distance_squared(&a, &b) < radius * radius) {
	/* Within radius */
}

/* ❌ Slower - unnecessary sqrt */
if (vec3_distance(&a, &b) < radius) {
	/* Within radius */
}
```

### In-place vs Output Parameter

Some functions modify in-place:
```c
vec3_normalize(&v);  /* Modifies v */
vec3_scale(&v, 2.0f);  /* Modifies v */
```

Others require output parameter:
```c
vec3_add(&a, &b, &result);  /* Does not modify a or b */
```

## Testing
```bash
cd /path/to/math
make test
```

Expected output:
```
a = (1.00, 0.00, 0.00)
b = (0.00, 1.00, 0.00)
a + b = (1.00, 1.00, 0.00)
a × b = (0.00, 0.00, 1.00)
a · b = 0.00
|a| = 1.00
```

## Future Modules

- **matrix.h**: 4x4 transformation matrices
- **quaternion.h**: Rotation representation
- **geometry.h**: Geometric primitives and tests

## License

MIT License - Free to use in personal and commercial projects.

## Author

Mats - Part of the robotics library ecosystem