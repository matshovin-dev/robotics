# Stewart Platform Kinematics Visualizer

Avansert 3D-visualisering av Stewart platform med faktisk inverse kinematics beregning.

## Hva er nytt vs viz-stewart?

**viz-stewart** (enkel):
- Bare geometrisk transformasjon (ingen kinematikk)
- Viser: base, platform, enkle linjer mellom dem

**viz-stewart-kinematics** (avansert):
- Kjører **faktisk inverse kinematics** på hver pose
- Viser: base, platform, motor arms, pushrods, og **knee points**
- Printer motorvinkler til terminal
- Detekterer uoppnåelige poser (rød blinking)

## Features

### Visuell rendering:
- **Base sekskant** (blå) - faste monteringspunkter
- **Platform sekskant** (rød) - transformerte punkter
- **Motor arms** (gul) - korte armer fra base til knee
- **Pushrods** (oransje) - lange stenger fra knee til platform
- **Knee points** (grønne kuler) - der motor arm møter pushrod
- **Koordinatsystem** (RGB = XYZ)
- **Error detection** - platform blinker rød hvis pose er uoppnåelig

### Terminal output:
```
Motors: [0]=245.2° [1]=123.5° [2]=256.8° [3]=110.4° [4]=268.1° [5]=98.7°
Motors: [0]=247.3° [1]=121.2° [2]=258.9° [3]=108.1° [4]=270.5° [5]=96.3° ⚠️  ERROR: Pose unreachable!
```

## Bygging

```bash
make clean
make all
```

## Kjøring

### Start visualisatoren:
```bash
./viz-stewart-kinematics
```

Eller via Makefile:
```bash
make run
```

### Send poses fra experiments:
```bash
cd ../../experiments/stewart-lab
make run
```

Visualisatoren lytter på **UDP port 9001** for pose packets.

## Arkitektur

### Dependencies:
- `libs/math` - Matematikk bibliotek (vec3, matrix, utils, geometry)
- `platforms/stewart` - Stewart kinematikk (geometry, pose, inverse, forward)
- `viz-modules/common` - UDP protokoll
- GLFW + OpenGL - 3D rendering

### Kinematikk-pipeline:
1. Motta UDP pose packet (rx, ry, rz, tx, ty, tz)
2. Konverter til `stewart_pose`
3. Kjør `stewart_kinematics_inverse()` → motor vinkler + knee posisjoner
4. Render alt i 3D med faktiske geometri

## Robot-typer

Støtter både:
- **ROBOT_MX64** - Dynamixel MX64 motorer (default)
- **ROBOT_AX18** - Dynamixel AX18 servos

Robot-type sendes i UDP packet og byttes automatisk.

## Tekniske detaljer

- **Window size**: 1024x768
- **Camera**: Eye=(500, 300, 500), Center=(0, 100, 0)
- **FOV**: 45°
- **UDP Port**: 9001
- **Frame rate**: ~60 FPS (VSync)

## Sammenligning

| Feature                  | viz-stewart | viz-stewart-kinematics |
|--------------------------|-------------|------------------------|
| Geometri transformasjon  | ✅          | ✅                     |
| Inverse kinematics       | ❌          | ✅                     |
| Knee positions           | ❌          | ✅                     |
| Motor angles             | ❌          | ✅                     |
| Error detection          | ❌          | ✅                     |
| Motor arm rendering      | ❌          | ✅                     |
| Pushrod rendering        | ❌          | ✅                     |

## Brukstilfeller

- **Kinematikk validering** - Sjekk at IK-algoritmen gir riktige vinkler
- **Workspace visualisering** - Se hvilke poser som er oppnåelige
- **Robot debugging** - Forstå hvordan motorer beveger seg
- **Utdanning** - Lær hvordan Stewart platforms fungerer

## Se også

- `viz-stewart/` - Enklere visualisator uten kinematikk
- `experiments/stewart-lab/` - Pose generator for testing
- `platforms/stewart/` - Kinematikk bibliotek
