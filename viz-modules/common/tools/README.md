# Visualizer Tools

Felles verktøy for Stewart platform visualizere.

## interactive_pose

Interaktiv kommandolinje-verktøy for å sende custom poses til visualizere.

### Bygging

```bash
make
```

### Bruk

1. Start en visualizer (plot_stw_polygon eller plot_stw_polygon_no_knee)
2. I et annet terminal:

```bash
./interactive_pose
```

### Kommandoer

```
rx <deg>         - Set rotation X (degrees)
ry <deg>         - Set rotation Y (degrees)
rz <deg>         - Set rotation Z (degrees)
tx <mm>          - Set translation X (millimeters)
ty <mm>          - Set translation Y (millimeters)
tz <mm>          - Set translation Z (millimeters)
home             - Reset to home position
robot mx64|ax18  - Switch robot type
show             - Show current pose
send             - Send current pose to visualizer
help             - Show help
quit             - Exit program
```

### Eksempel

```
> rx 10
Set rx = 10.00°
> ry 5
Set ry = 5.00°
> ty 180
Set ty = 180.00mm
> send
Sent pose to visualizer
> show
Current pose:
  Rotation:    rx=10.00° ry=5.00° rz=0.00°
  Translation: tx=0.00mm ty=180.00mm tz=0.00mm
  Robot type:  MX64 (home_height=205.0mm)

> robot ax18
Switched to AX18
> home
Reset to home position
> send
Sent pose to visualizer
> quit
Goodbye!
```

### Integrasjon

Kan brukes med:
- `plot_stw_polygon` - Full kinematikk visualizer
- `plot_stw_polygon_no_knee` - Enkel polygon visualizer
- `viz-stewart-compare` - Side-by-side sammenligning

Alle lytter på samme UDP port (5005).
