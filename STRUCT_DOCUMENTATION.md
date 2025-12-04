# Struct Dokumentasjonsstandard (Kompakt format)

## Prinsipper

- **Kompakt** - Enkel å lese, ingen alignment
- **Konsistent** - Samme format overalt
- **Enheeter** - Alltid spesifiser når relevant
- **Selvforklarende** - Kort men tydelig

---

## Standard Template

```c
/**
 * struct struct_name - Kort beskrivelse
 *
 * @field1 Type (unit) - Beskrivelse
 * @field2[N] Type (unit) - Beskrivelse
 * @field3 Type - Beskrivelse
 *
 * [Lengre beskrivelse av strukturens formål og bruk]
 *
 * @note [Viktig merknad]
 * @invariant [Viktig constraint]
 */
struct struct_name {
    Type field1;
    Type field2[N];
    Type field3;
};
```

---

## Eksempler

### Eksempel 1: Enkel vektor

```c
/**
 * struct vec3 - 3D vektor
 *
 * @x float - X-komponent
 * @y float - Y-komponent
 * @z float - Z-komponent
 *
 * Representerer punkt eller retning i 3D rom.
 */
struct vec3 {
    float x;
    float y;
    float z;
};
```

### Eksempel 2: Pose med enheeter

```c
/**
 * struct stewart_pose - Platform pose (6 graders frihet)
 *
 * @rx float (deg) - Roll (rotasjon rundt X-akse)
 * @ry float (deg) - Pitch (rotasjon rundt Y-akse)
 * @rz float (deg) - Yaw (rotasjon rundt Z-akse)
 * @tx float (mm) - X translasjon fra origo
 * @ty float (mm) - Y translasjon (høyde fra origo)
 * @tz float (mm) - Z translasjon fra origo
 *
 * Representerer topp-platformens posisjon og orientering i 3D rom.
 * Origo er midtpunkt på base-platform ved motoraksel-høyde.
 *
 * Koordinatsystem: X+=høyre, Y+=opp, Z+=fremover
 *
 * @note En pose er uavhengig av platformens home posisjon
 */
struct stewart_pose {
    float rx;
    float ry;
    float rz;
    float tx;
    float ty;
    float tz;
};
```

### Eksempel 3: Resultat-struct med arrays

```c
/**
 * struct stewart_inverse_result - Inverse kinematics resultat
 *
 * @motor_angles_deg[6] float (deg) - Motor vinkler
 * @knee_points[6] vec3 (mm) - Kne posisjoner
 * @platform_points_transformed[6] vec3 (mm) - Transformerte punkter
 * @error int - 0=success, 1=NaN detected
 *
 * Inneholder alle beregnede verdier fra inverse kinematics.
 * Beregnes fra ønsket platform pose.
 *
 * @note Alle arrays indeksert 0-5 for motor nummer
 */
struct stewart_inverse_result {
    float motor_angles_deg[6];
    struct vec3 knee_points[6];
    struct vec3 platform_points_transformed[6];
    int error;
};
```

### Eksempel 4: Konfigurasjon med mange felt

```c
/**
 * struct stewart_geometry - Stewart platform fysisk geometri
 *
 * @base_points[6] vec3 (mm) - Base attachment points (y=0)
 * @platform_home_points[6] vec3 (mm) - Platform points at home
 * @home_height float (mm) - Avstand base til platform
 * @short_foot_length float (mm) - Motor arm lengde
 * @long_foot_length float (mm) - Pushrod lengde
 * @motor_arm_outward int - 1=utover (MX64), 0=innover (AX18)
 * @max_motor_angle_024_deg float (deg) - Max vinkel motor 0,2,4
 * @min_motor_angle_024_deg float (deg) - Min vinkel motor 0,2,4
 * @max_motor_angle_135_deg float (deg) - Max vinkel motor 1,3,5
 * @min_motor_angle_135_deg float (deg) - Min vinkel motor 1,3,5
 * @motor_clamp_limit_angle_deg float (deg) - Soft dampening grense
 * @max_pose_rotation_amplitude float - Max rotasjon amplitude
 * @max_pose_rotation_bias float - Max rotasjon bias/offset
 * @max_pose_translation_amplitude float (mm) - Max translasjon amplitude
 * @max_pose_translation_bias float (mm) - Max translasjon bias
 *
 * Inneholder alle fysiske dimensjoner og kinematiske grenser for en
 * spesifikk Stewart platform konfigurasjon.
 *
 * Origo ligger i platform base ved 6 motoraksler.
 *
 * Motor layout (sett ovenfra, CCW nummerering):
 *    3     2
 *   4       1
 *      5 0
 *
 * @invariant max_motor_angle > min_motor_angle for begge grupper
 * @note platform_home_points kopieres ved bruk (const i struct)
 */
struct stewart_geometry {
    struct vec3 base_points[6];
    struct vec3 platform_home_points[6];
    float home_height;
    float short_foot_length;
    float long_foot_length;
    int motor_arm_outward;
    float max_motor_angle_024_deg;
    float min_motor_angle_024_deg;
    float max_motor_angle_135_deg;
    float min_motor_angle_135_deg;
    float motor_clamp_limit_angle_deg;
    float max_pose_rotation_amplitude;
    float max_pose_rotation_bias;
    float max_pose_translation_amplitude;
    float max_pose_translation_bias;
};
```

### Eksempel 5: Matrise med spesiell layout

```c
/**
 * struct mat3 - 3x3 rotasjonsmatrise
 *
 * @m[9] float - Matrise-elementer i column-major order
 *
 * Representerer 3x3 rotasjonsmatrise for 3D transformasjoner.
 *
 * Layout (column-major):
 *   m[0]  m[3]  m[6]
 *   m[1]  m[4]  m[7]
 *   m[2]  m[5]  m[8]
 *
 * @note Column-major: kolonner sammenhengende i minnet
 */
struct mat3 {
    float m[9];
};
```

---

## Retningslinjer

### 1. Feltformat

**Standard:**
```c
 * @field_name Type (unit) - Beskrivelse
```

**Array:**
```c
 * @field_name[N] Type (unit) - Beskrivelse
```

**Uten enhet:**
```c
 * @field_name Type - Beskrivelse
```

### 2. Enheeter - bruk når relevant

**Fysiske:**
- `(mm)` - millimeter
- `(m)` - meter
- `(cm)` - centimeter
- `(kg)` - kilogram
- `(N)` - Newton

**Vinkler:**
- `(deg)` - grader
- `(rad)` - radianer

**Tid:**
- `(s)` - sekunder
- `(ms)` - millisekunder
- `(us)` - mikrosekunder
- `(Hz)` - Hertz

**Elektrisk:**
- `(V)` - Volt
- `(A)` - Ampere
- `(W)` - Watt

### 3. Array-størrelser - alltid spesifiser

```c
 * @points[6] vec3 (mm) - 6 attachment points
 * @matrix[9] float - 3x3 matrix (column-major)
 * @buffer[256] char - String buffer
```

### 4. Beskrivelser - kort og konsist

**Godt:**
```c
 * @motor_angle float (deg) - Motor rotation angle
```

**For langt (unngå):**
```c
 * @motor_angle float (deg) - The angle of rotation for the motor shaft measured in degrees from the zero position
```

Hvis beskrivelsen er lang, bruk main description:
```c
 * @motor_angle float (deg) - Motor rotation angle
 *
 * Motor angle is measured from zero position (straight down).
 * Positive angles rotate counter-clockwise when viewed from motor axis.
```

### 5. Invarianter og merknader

**@invariant** - viktige constraints:
```c
 * @invariant max > min
 * @invariant All arrays size [6]
 * @invariant home_height > 0
```

**@note** - viktige merknader:
```c
 * @note Thread-safe read-only after init
 * @note Arrays indexed 0-5 for motor number
 * @note Column-major layout
```

### 6. ASCII-art for komplekse layouts

For motor-layout, coordinate systems, etc:
```c
 * Motor layout (top view, CCW):
 *    3     2
 *   4       1
 *      5 0
 *
 * Coordinate system:
 *   X+ = Right (red axis)
 *   Y+ = Up (green axis)
 *   Z+ = Forward (blue axis)
```

---

## VS Code Snippet

Legg til i `.vscode/c.code-snippets`:

```json
{
  "Struct Comment": {
    "prefix": "ssc",
    "body": [
      "/**",
      " * struct ${1:name} - ${2:Description}",
      " *",
      " * @${3:field} ${4:Type} (${5:unit}) - ${6:Description}",
      " *",
      " * ${7:Longer description}",
      " *",
      " * @note ${8:Note}",
      " */"
    ],
    "description": "Struct comment template"
  }
}
```

**Bruk:** Skriv `ssc` + Tab

---

## Checklist før commit

- [ ] Struct har `struct name` og kort beskrivelse
- [ ] Alle felt dokumentert med `@field`
- [ ] Typer spesifisert for alle felt
- [ ] Enheeter spesifisert for numeriske felt
- [ ] Array-størrelser dokumentert
- [ ] Lengre beskrivelse av formål
- [ ] Invarianter dokumentert hvis relevante
- [ ] Viktige merknader med `@note`
- [ ] Kompakt format (ingen unødvendig alignment)

---

## Sammenligning: Før vs Etter

### Før:
```c
struct stewart_pose {
    float rx; // roll
    float ry; // pitch
    float rz; // yaw
    float tx, ty, tz; // position
};
```

### Etter:
```c
/**
 * struct stewart_pose - Platform pose (6DOF)
 *
 * @rx float (deg) - Roll (rotasjon rundt X)
 * @ry float (deg) - Pitch (rotasjon rundt Y)
 * @rz float (deg) - Yaw (rotasjon rundt Z)
 * @tx float (mm) - X translasjon
 * @ty float (mm) - Y translasjon (høyde)
 * @tz float (mm) - Z translasjon
 *
 * Platformens posisjon og orientering i 3D rom.
 * Origo er base-midtpunkt ved motoraksel-høyde.
 *
 * @note Koordinater: X+=høyre, Y+=opp, Z+=fremover
 */
struct stewart_pose {
    float rx;
    float ry;
    float rz;
    float tx;
    float ty;
    float tz;
};
```

**Fordel:** Umiddelbart tydelig typer, enheeter, og formål! ✨

---

Dette er den kompakte standarden - enkel og konsistent! 🎯
