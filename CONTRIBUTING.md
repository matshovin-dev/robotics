# Contributing to Robotics Platform Project

## Dokumentasjonsstandard

Dette prosjektet bruker en **streng kommentarstruktur** som fokuserer på **input/output avhengigheter**. Målet er å gjøre koden umiddelbart forståelig ved å vise nøyaktig hvilke felter som leses og skrives.

---

## Kommentar-mal

### Plassering

- **PUBLIC funksjoner** (deklarert i `.h`): Kommentar i `.h` filen (vises i IntelliSense)
- **STATIC funksjoner** (kun i `.c`): Kommentar i `.c` filen (over implementasjonen)

---

### Template 1: Kompleks funksjon (med algoritme)

```c
/**
 * @function function_name
 * @api PUBLIC | STATIC
 *
 * @input  param1->field1[6]     Type (unit)
 * @input  param1->field2        Type (unit)
 * @input  param2->{x,y,z}       Type (unit)
 *
 * @output result->field1[6]     Type (unit)
 * @output result->field2        Type (unit)
 * @output return                Type (unit)
 *
 * [Kort beskrivelse av hva funksjonen gjør - 1-2 setninger]
 *
 * Algoritme:
 * 1. [Steg 1 beskrivelse]
 * 2. [Steg 2 beskrivelse]
 *    - [Sub-steg a]
 *    - [Sub-steg b]
 * 3. [Steg 3 beskrivelse]
 *
 * @note [Viktige merknader, edge cases, etc.]
 */
```

**Eksempel:**

```c
/**
 * @function stewart_kinematics_inverse
 * @api PUBLIC
 *
 * @input  geom->base_points[6]                      struct vec3 (mm)
 * @input  geom->platform_home_points[6]             struct vec3 (mm)
 * @input  geom->short_foot_length                   float (mm)
 * @input  geom->long_foot_length                    float (mm)
 * @input  pose_in->{rx,ry,rz}                       float (degrees)
 * @input  pose_in->{tx,ty,tz}                       float (mm)
 *
 * @output result_out->motor_angles_deg[6]           float (degrees)
 * @output result_out->knee_points[6]                struct vec3 (mm)
 * @output result_out->platform_points_transformed[6] struct vec3 (mm)
 * @output result_out->error                         int (0=success, 1=NaN)
 *
 * Beregner motor-vinkler, knepunkt og nye topp platform punkt
 * fra ønsket platform pose.
 *
 * Algoritme:
 * 1. Roterer og translaterer geom.platform_home_points med pose_in, til
 *    result_out->platform_points_transformed
 * 2. For hver motor:
 *    - Projiser platform_points_transformed på motor plan
 *    - Beregn trekant geometri (pythagoras + cosinus-setningen)
 *    - Finn motor vinkel
 *    - Beregn kne posisjon
 * 3. Motor vinkler blir hard eller soft clamped til geometri-grenser.
 */
void stewart_kinematics_inverse(const struct stewart_geometry *geom,
                                const struct stewart_pose *pose_in,
                                struct stewart_inverse_result *result_out,
                                int debug);
```

---

### Template 2: Middels kompleks funksjon

```c
/**
 * @function function_name
 * @api PUBLIC | STATIC
 *
 * @input  param1->field  Type (unit)
 * @input  param2         Type (unit)
 *
 * @output result->field  Type (unit)
 * @output return         Type (unit)
 *
 * [Beskrivelse av hva funksjonen gjør]
 */
```

**Eksempel:**

```c
/**
 * @function vec3_add
 * @api PUBLIC
 *
 * @input  a->{x,y,z}    float
 * @input  b->{x,y,z}    float
 * @output out->{x,y,z}  float (a + b)
 *
 * Legger sammen to vektorer: out = a + b
 */
void vec3_add(const struct vec3 *a, const struct vec3 *b, struct vec3 *out);
```

---

### Template 3: Enkel funksjon (kompakt)

```c
/**
 * @function function_name
 * @api PUBLIC | STATIC
 *
 * @input  param  Type (unit)
 * @output return Type (unit)
 *
 * [Kort beskrivelse]
 */
```

**Eksempel:**

```c
/**
 * @function vec3_length
 * @api PUBLIC
 *
 * @input  v->{x,y,z}  float
 * @output return      float
 *
 * Beregner lengden av vektor (sqrt(x² + y² + z²)).
 */
float vec3_length(const struct vec3 *v);
```

---

## Retningslinjer

### 1. **I/O først** - Viktigste informasjon øverst
Alltid vis `@input` og `@output` først, før beskrivelsen. Dette gjør at IntelliSense umiddelbart viser avhengigheter.

### 2. **Typer alltid**
Vis datatype for alle parametre. Eksempler:
- `float`
- `int`
- `struct vec3`
- `const char*`

### 3. **Enheeter**
Spesifiser enheeter når relevant:
- `(mm)` - millimeter
- `(degrees)` - grader
- `(radians)` - radianer
- `(meters)` - meter
- `(rad)` - radianer (kort form)

### 4. **Felt-notasjon**
Vis nøyaktig hvilke felter som brukes:
- `param->field` - enkelt felt
- `param->{x,y,z}` - flere felt
- `param->array[6]` - array
- `param->nested.field` - nested struktur

### 5. **In-place modifikasjon**
Hvis en funksjon endrer input in-place, vis det både som input OG output:
```c
/**
 * @input  v->{x,y,z}  float
 * @output v->{x,y,z}  float (normalized to length 1)
 */
void vec3_normalize(struct vec3 *v);
```

### 6. **Algoritme-seksjoner**
Kun for komplekse funksjoner med flere distinkte steg. Hold det konsist:
```c
 * Algoritme:
 * 1. [Første steg]
 * 2. [Andre steg]
 *    - [Sub-steg]
 * 3. [Tredje steg]
```

### 7. **Konsistens**
Bruk samme mal i hele prosjektet. Alle PUBLIC funksjoner skal dokumenteres.

---

## VS Code Snippets

For rask innsetting, legg til i `.vscode/c.code-snippets`:

```json
{
  "Stewart Function Comment": {
    "prefix": "sfc",
    "body": [
      "/**",
      " * @function ${1:function_name}",
      " * @api ${2|PUBLIC,STATIC|}",
      " * ",
      " * @input  ${3:param}  ${4:Type}",
      " * ",
      " * @output ${5:return}  ${6:Type}",
      " *",
      " * ${7:Beskrivelse}",
      " */"
    ],
    "description": "Stewart Platform function comment template"
  },
  "Stewart Complex Function Comment": {
    "prefix": "sfcc",
    "body": [
      "/**",
      " * @function ${1:function_name}",
      " * @api ${2|PUBLIC,STATIC|}",
      " * ",
      " * @input  ${3:param1}  ${4:Type}",
      " * @input  ${5:param2}  ${6:Type}",
      " * ",
      " * @output ${7:result.field}  ${8:Type}",
      " * @output ${9:return}         ${10:Type}",
      " *",
      " * ${11:Beskrivelse}",
      " *",
      " * Algoritme:",
      " * 1. ${12:Steg 1}",
      " * 2. ${13:Steg 2}",
      " *    - ${14:Sub-steg}",
      " */"
    ],
    "description": "Complex function with algorithm steps"
  }
}
```

Bruk:
- Skriv `sfc` + Tab for enkel funksjon
- Skriv `sfcc` + Tab for kompleks funksjon

---

## Hvorfor denne standarden?

### Problem før:
- 10 minutter for å forstå hva en funksjon gjør
- Glemt detaljer underveis
- Vanskelig debugging - uklar på hvilke felt som endres

### Løsning nå:
- **15 sekunder** til full forståelse
- **Null gjetning** om hvilke felt som brukes
- **IntelliSense** viser alt når du trenger det
- **Enklere debugging** - ser avhengigheter umiddelbart

---

## Eksempel på IntelliSense

Når du hoverer over en funksjon, ser du umiddelbart:

```
stewart_kinematics_inverse()

INPUT:
  geom->base_points[6]           struct vec3 (mm)
  geom->platform_home_points[6]  struct vec3 (mm)
  pose_in->{rx,ry,rz}            float (degrees)

OUTPUT:
  result_out->motor_angles_deg[6]  float (degrees)
  result_out->knee_points[6]       struct vec3 (mm)
  result_out->error                int (0=success, 1=NaN)
```

**Dette er grunnen til at vi dokumenterer slik!**

---

## Struct Dokumentasjon (Kompakt format)

Structs dokumenteres med samme prinsipper som funksjoner - fokus på klarhet og enheeter.

### Template:

```c
/**
 * struct struct_name - Kort beskrivelse
 *
 * @field1 Type (unit) - Beskrivelse
 * @field2[N] Type (unit) - Beskrivelse
 * @field3 Type - Beskrivelse
 *
 * [Lengre beskrivelse av strukturens formål]
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

### Eksempel:

```c
/**
 * struct stewart_pose - Platform pose (6DOF)
 *
 * @rx float (deg) - Roll (rotasjon rundt X-akse)
 * @ry float (deg) - Pitch (rotasjon rundt Y-akse)
 * @rz float (deg) - Yaw (rotasjon rundt Z-akse)
 * @tx float (mm) - X translasjon fra origo
 * @ty float (mm) - Y translasjon (høyde)
 * @tz float (mm) - Z translasjon fra origo
 *
 * Representerer topp-platformens posisjon og orientering.
 * Origo er base-midtpunkt ved motoraksel-høyde.
 *
 * @note En pose er uavhengig av home posisjon
 */
struct stewart_pose {
    float rx, ry, rz;
    float tx, ty, tz;
};
```

### VS Code Snippet for Structs:

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
    ]
  }
}
```

**Bruk:** Skriv `ssc` + Tab

Se `STRUCT_DOCUMENTATION.md` for detaljer og flere eksempler.

---

## Checklist før commit

### Funksjoner:
- [ ] Alle PUBLIC funksjoner har `@function` og `@api PUBLIC`
- [ ] Alle STATIC funksjoner har `@function` og `@api STATIC`
- [ ] `@input` og `@output` viser nøyaktige felt med typer
- [ ] Enheeter er spesifisert der det er relevant
- [ ] Komplekse funksjoner har algoritme-seksjoner

### Structs:
- [ ] Alle structs har `struct name` og kort beskrivelse
- [ ] Alle felt dokumentert med `@field Type (unit)`
- [ ] Array-størrelser spesifisert `[N]`
- [ ] Enheeter spesifisert for numeriske felt
- [ ] Lengre beskrivelse av formål
- [ ] Invarianter og merknader hvis relevante

### Generelt:
- [ ] Konsistent formatering med eksisterende kode
- [ ] Kompakt format (ingen unødvendig alignment)

---

## Spørsmål?

Se eksempler i:
- `platforms/stewart/include/stewart/kinematics.h` - Funksjoner
- `platforms/stewart/include/stewart/pose.h` - Structs
- `libs/math/include/robotics/math/vec3.h` - Math bibliotek
- `STRUCT_DOCUMENTATION.md` - Detaljert struct guide

Eller spør maintainer!
