# plot_stw_obj - Stewart Platform OBJ Visualizer

A real-time 3D visualizer for Stewart platform motion using OBJ models. Renders realistic 3D models of the base, platform, motor arms, and pushrods with lighting and accurate kinematics.

## Features

- **Realistic 3D rendering** using OBJ models from SolidWorks
- **Real-time kinematics** via inverse kinematics calculations
- **UDP control protocol** for receiving pose commands
- **Dynamic geometry switching** between MX64 and AX18 configurations
- **Interactive camera controls** with zoom, rotation, and focus adjustment
- **Lighting effects** with OpenGL lighting and material properties

## Architecture

```
plot_stw_obj/
├── src/
│   └── main.c           # Main visualizer with OBJ rendering
├── Makefile             # Build configuration
└── README.md            # This file
```

## Dependencies

### Required Libraries
- **GLFW3** - Window and input handling
- **OpenGL** - 3D rendering
- **obj_loader** - OBJ file parsing (`../../libs/obj_loader/`)
- **graphics** - Graphics utilities (`../../libs/graphics/`)
- **math** - Vector/matrix math (`../../libs/math/`)
- **stewart** - Kinematics library (`../../platforms/stewart/`)
- **viz_common** - UDP protocol (`../common/`)

### 3D Model Assets
Models located in `../../assets/3d_models/obj/`:
- `mx64/` - Models for MX64 Dynamixel configuration
  - `bunn.obj` - Base platform
  - `top.obj` - Top platform
  - `legL.obj` - Left motor arm
  - `legR.obj` - Right motor arm
  - `legLong.obj` - Pushrod linkage
- `ax18/` - Models for AX18 servo configuration (same structure)

## Building

```bash
make              # Build the visualizer
make run          # Build and run
make clean        # Clean build artifacts
```

## Usage

### Running the Visualizer

```bash
./plot_stw_obj
```

The visualizer will:
1. Load OBJ models for the current robot type (default: MX64)
2. Start listening on UDP port 9001
3. Open a window showing the Stewart platform at home position
4. Update in real-time as pose commands are received

### Camera Controls

| Key | Action |
|-----|--------|
| Arrow Keys | Rotate camera around platform |
| Q / W | Zoom in / out |
| A / S | Lower / raise camera focus point |
| R | Reset camera to default view |
| ESC | Exit visualizer |

### UDP Protocol

The visualizer receives pose updates via UDP on port 9001 using the `viz_protocol.h` format:

```c
struct viz_pose_packet {
    uint32_t magic;        // VIZ_MAGIC (0x53545750)
    uint32_t type;         // VIZ_PACKET_POSE (1)
    uint32_t robot_type;   // ROBOT_TYPE_MX64 or ROBOT_TYPE_AX18
    float rx, ry, rz;      // Rotation in degrees (roll, pitch, yaw)
    float tx, ty, tz;      // Translation in mm
};
```

### Sending Pose Commands

Use the `viz_sender` library from `../common/` to send commands:

```c
#include "viz_sender.h"

int sock = viz_sender_create();
viz_sender_send_pose(sock, rx, ry, rz, tx, ty, tz, ROBOT_TYPE_MX64);
```

Or use the test animation program from `plot_stw_polygon`:
```bash
cd ../plot_stw_polygon
make test_animation
./test_animation
```

## Rendering Details

### Model Coordinate System
- **Origin**: Base center at (0, 0, 0)
- **Y-up**: Positive Y is up (vertical)
- **Right-handed**: X-right, Y-up, Z-forward

### Rendering Order
1. **Base (bunn)** - Rendered at origin with gray color
2. **Platform (top)** - Rendered with pose transformation (light gray)
3. **Motor arms** - 6x legL/legR at base points with motor angles (red/blue)
4. **Pushrods** - 6x legLong from knee to platform (beige)
5. **Coordinate axes** - RGB axes at origin

### Motor Arm Placement
- Motors 0, 2, 4 use `legL` (left arms)
- Motors 1, 3, 5 use `legR` (right arms)
- Each pair placed at 0°, 120°, 240° around base
- Motor angles from inverse kinematics applied as Z-axis rotations

### Lighting
- Single directional light (GL_LIGHT0)
- Position: (300, 400, 300)
- Ambient: 0.3, Diffuse: 0.8
- Color material enabled for per-object colors

## Differences from plot_stw_polygon

| Feature | plot_stw_polygon | plot_stw_obj |
|---------|------------------|--------------|
| Rendering | Line-based wireframe | 3D OBJ models |
| Lighting | None | OpenGL lighting |
| Visuals | Abstract geometry | Realistic models |
| Performance | Very fast | Moderate (model complexity) |
| Models | None | 5 OBJ files per robot type |

## Differences from stwMacC (Legacy)

This is a modernized version of the old visualizer with improvements:

### New Features
- Uses new `stewart_kinematics_inverse()` API
- Uses new geometry structs (`ROBOT_MX64`, `ROBOT_AX18`)
- Uses `viz_protocol.h` for standardized UDP
- Cleaner code structure
- Better error handling
- Dynamic model loading on robot type change

### Removed Features
- Text renderer (UI overlay)
- Forward kinematics visualization
- UDP connection status display
- Sphere/test model rendering
- 2D debug overlays

## Troubleshooting

### Models not loading
Check that model paths are correct:
```bash
ls ../../assets/3d_models/obj/mx64/
ls ../../assets/3d_models/obj/ax18/
```

### Black screen
- Ensure lighting is enabled (should be automatic)
- Check that models loaded successfully (see console output)
- Try resetting camera with 'R' key

### UDP not receiving
- Check that port 9001 is not in use by another process
- Verify sender is using correct protocol (`VIZ_MAGIC`, `VIZ_PACKET_POSE`)
- Check firewall settings

### Compilation errors
- Ensure all dependencies are built (math, stewart, obj_loader, graphics)
- Check that GLFW3 is installed: `brew install glfw`
- Verify include paths in Makefile match your directory structure

## Performance

Typical performance on Apple Silicon Mac:
- 60 FPS at 1024x768 resolution
- ~10-15ms per frame for MX64 models
- ~5-8ms per frame for AX18 models (simpler geometry)

Model complexity:
- MX64 base: ~100k vertices
- AX18 base: ~300k vertices
- Leg models: ~5k-20k vertices each

## Related Tools

- `plot_stw_polygon` - Lightweight wireframe visualizer
- `test_animation` - UDP pose sender for testing
- `viz_sender` - Library for sending pose commands

## License

Part of the Stewart platform robotics project.
