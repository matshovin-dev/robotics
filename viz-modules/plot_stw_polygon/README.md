# plot_stw_polygon

Stewart platform visualizer med full kinematikk - tegner platform med realistiske kneledd.

## Funksjonalitet

- **Full kinematikk**: Beregner motor-vinkler og kneledd-posisjoner via inverse kinematics
- **Realistisk visualisering**: Viser hvordan bena faktisk bøyer seg
- **Dynamisk geometri**: Bytter automatisk mellom robot-typer basert på pose packet

## Sammenligning med plot_stw_polygon_no_knee

| Feature | plot_stw_polygon | plot_stw_polygon_no_knee |
|---------|------------------|--------------------------|
| Kinematikk | ✅ Full IK | ❌ Kun transformasjon |
| Kneledd | ✅ Viser kneledd | ❌ Rette linjer |
| Ytelse | Middels | Rask |
| Bruk | Realistisk vis. | Enkel geometri-test |

## Visualisering

- **Base sekskant**: Blå
- **Platform sekskant**: Rød
- **Ben (motor arm + pushrod)**: Grå linjer med kne
- **Koordinatsystem**: X (rød), Y (grønn), Z (blå)

## Bygging

```bash
make                # Bygg visualizer
make test_animation # Bygg animasjonstest
make clean          # Rens opp
```

## Bruk

### 1. Start visualizeren

```bash
./plot_stw_polygon
```

Programmet lytter på UDP port 5005.

### 2. Send poses til visualizeren

I et annet terminal:

```bash
./test_animation
```

Dette sender sinusformet animasjon i alle 6 frihetsgrader til plotteren.

Eller bruk viz_sender biblioteket i din egen kode:

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

## Støttede roboter

- **ROBOT_TYPE_MX64** (default)
- **ROBOT_TYPE_AX18**

Bytter automatisk basert på `robot_type` felt i pose packet.

## Pose format

- `rx, ry, rz`: Rotasjon i grader (Euler XYZ)
- `tx, ty, tz`: Translasjon i millimeter (absolutt posisjon)
- `robot_type`: ROBOT_TYPE_MX64 eller ROBOT_TYPE_AX18
