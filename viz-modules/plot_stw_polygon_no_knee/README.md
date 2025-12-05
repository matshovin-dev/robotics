# plot_stw_polygon_no_knee

Enkel Stewart platform visualizer som tegner platform som to sekskanter (base og platform) koblet med rette linjer.

## Begrensninger

- **Ingen kinematikk**: Kun geometrisk transformasjon av platform polygon
- **Rette ben**: Bena tegnes som rette linjer (ikke realistisk)
- **Ingen kneledd**: Kneledd visualiseres ikke

For realistisk visualisering med kinematikk, se `viz-stewart-kinematics`.

## Støtter flere roboter

Bytter automatisk geometri basert på `robot_type` i pose packet:
- `ROBOT_TYPE_MX64` (default)
- `ROBOT_TYPE_AX18`

## Bygging

```bash
make                    # Bygg visualizer
make test_animation     # Bygg animasjonstest
make clean              # Rens opp
```

## Bruk

### 1. Start visualizeren

```bash
./plot_stw_polygon_no_knee
```

Programmet lytter på UDP port 5005 og venter på pose packets.

### 2. Send poses til visualizeren

I et annet terminal:

```bash
./test_animation
```

Dette sender sinusformet animasjon i alle 6 frihetsgrader (rx, ry, rz, tx, ty, tz) til visualizeren.

Eller bruk `viz_sender` biblioteket i din egen kode:

```c
#include "viz_sender.h"
#include "stewart/pose.h"

struct stewart_pose pose = {
    .rx = 10.0f, .ry = 0.0f, .rz = 5.0f,
    .tx = 0.0f, .ty = 150.0f, .tz = 0.0f
};

viz_send_pose(&pose, ROBOT_TYPE_MX64, VIZ_PORT);
```

## Kamerakontroll

- **Arrow keys**: Roter kamera rundt platform
  - ← → : Roter azimuth (horisontal)
  - ↑ ↓ : Roter elevation (vertikal)
- **Q / W**: Zoom inn / ut
- **A / S**: Senk / hev platform relativt til kamera
- **R**: Reset kamera til standardvisning
- **ESC**: Avslutt programmet

## Koordinatsystem

- **X-akse**: Rød (høyre)
- **Y-akse**: Grønn (opp)
- **Z-akse**: Blå (fremover)

## Pose format

- `rx, ry, rz`: Rotasjon i grader (Euler XYZ)
- `tx, ty, tz`: Translasjon i millimeter (absolutt posisjon)
