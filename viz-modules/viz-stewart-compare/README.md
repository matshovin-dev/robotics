# Stewart Platform Comparison Visualizer

Sammenlign to Stewart platform poser side-om-side i sanntid med full inverse kinematics beregning.

## Hva er dette?

**viz-stewart-compare** visualiserer **to uavhengige poser samtidig** med forskjellige farger, slik at du kan:

- Sammenligne ønsket vs faktisk posisjon
- Visualisere target vs current state
- Debugge kinematics-forskjeller
- Se lag og feil i sanntid
- Sammenligne to forskjellige robotkonfigurasjoner

## Fargevalg

### Pose 1 (Port 9001) - CYAN/TURKIS 🔵
- **Reference/Target/Desired** pose
- Platform: Cyan
- Motor arms: Gul-cyan
- Pushrods: Cyan-rød
- Knees: Cyan kuler

### Pose 2 (Port 9002) - MAGENTA/LILLA 🟣
- **Actual/Current/Measured** pose
- Platform: Magenta
- Motor arms: Gul-magenta
- Pushrods: Magenta-rød
- Knees: Magenta kuler

### Base - GRÅ ⚪
- Deles av begge poser
- Faste monteringspunkter

## UDP-porter

```
Port 9001: Pose 1 (Cyan - reference)
Port 9002: Pose 2 (Magenta - actual)
```

Begge porter lytter kontinuerlig og oppdaterer hver pose uavhengig.

## Bygging

```bash
make clean
make all
```

## Kjøring

### 1. Start visualisatoren:
```bash
./viz-stewart-compare
```

Eller via Makefile:
```bash
make run
```

### 2. Send poses fra test-program:
```bash
cd ../../experiments/stewart-lab
./build/compare_demo
```

Dette sender to forskjellige poser:
- **Cyan (9001)**: Reference bevegelse
- **Magenta (9002)**: Samme bevegelse men med 0.3s lag og mindre amplitude

## Kontroller

- **Arrow keys** (←/→/↑/↓) - Roter kamera
- **Q/W** - Zoom inn/ut
- **R** - Reset kamera til default
- **ESC** - Lukk vindu

## Brukstilfeller

### 1. Target vs Current visualisering
```bash
# Terminal 1: Visualizer
./viz-stewart-compare

# Terminal 2: Send target til port 9001
./my_controller --target-port 9001

# Terminal 3: Send current til port 9002
./my_robot_feedback --actual-port 9002
```

### 2. Sammenligne to forskjellige trajektorier
```bash
# Send trajektorie A til 9001 (cyan)
# Send trajektorie B til 9002 (magenta)
```

### 3. Debugging av kinematics
```bash
# Send samme pose til begge porter fra forskjellige IK-algoritmer
# Se om de gir samme resultat
```

### 4. Lag-visualisering
```bash
# Send target pose til 9001
# Send delayed/lagged pose til 9002
# Se systemets lag visuelt
```

## Features

✅ **Dual inverse kinematics** - Beregner IK for begge poser
✅ **Ortogonal projeksjon** - Ingen perspektiv-forvrengning
✅ **Interaktiv kamera** - Roter fritt rundt platformen
✅ **Tydelig fargedifferensiering** - Cyan vs Magenta
✅ **Knee positions** - Grønne/magenta kuler
✅ **Motor angles** - Printet i terminal for begge poser
✅ **Error detection** - Varsler hvis pose er uoppnåelig

## Sammenligning med andre visualisatorer

| Feature                  | viz-stewart | viz-stewart-kinematics | viz-stewart-compare |
|--------------------------|-------------|------------------------|---------------------|
| Geometri transformasjon  | ✅          | ✅                     | ✅                  |
| Inverse kinematics       | ❌          | ✅                     | ✅ (x2)             |
| Knee positions           | ❌          | ✅                     | ✅ (x2)             |
| Motor angles             | ❌          | ✅                     | ✅ (x2)             |
| Error detection          | ❌          | ✅                     | ✅ (x2)             |
| Camera controls          | ❌          | ✅                     | ✅                  |
| **Dual-pose comparison** | ❌          | ❌                     | ✅                  |
| **Two UDP ports**        | ❌          | ❌                     | ✅                  |

## Terminal output

```
Pose 1 Motors: [0]=245.2° [1]=123.5° [2]=256.8° [3]=110.4° [4]=268.1° [5]=98.7°
Pose 2 Motors: [0]=242.1° [1]=125.8° [2]=254.3° [3]=112.1° [4]=265.9° [5]=100.2°
```

## Tekniske detaljer

- **Window size**: 1024x768
- **Camera**: Ortogonal projeksjon med justerbar zoom
- **UDP Ports**: 9001 (Pose 1), 9002 (Pose 2)
- **Frame rate**: ~60 FPS (VSync)
- **Robot type**: MX64 (default), støtter også AX18

## Se også

- `viz-stewart/` - Enkel geometri-visualisator
- `viz-stewart-kinematics/` - Single-pose med kinematikk
- `experiments/stewart-lab/compare_demo.c` - Test-program som sender til begge porter
