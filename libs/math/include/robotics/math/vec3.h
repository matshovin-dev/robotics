#ifndef ROBOTICS_MATH_VEC3_H
#define ROBOTICS_MATH_VEC3_H

/**
 * struct vec3 - 3D vektor
 * @x: X-komponent
 * @y: Y-komponent
 * @z: Z-komponent
 */
struct vec3 {
	float x;
	float y;
	float z;
};

/**
 * vec3_length - Beregn lengden av vektor
 * @v: input vektor
 *
 * Retur: lengden av vektoren (sqrt(x² + y² + z²))
 */
float vec3_length(const struct vec3 *v);

/**
 * vec3_length_squared - Beregn lengde i kvadrat (uten sqrt)
 * @v: input vektor
 *
 * Raskere enn vec3_length() når man kun trenger å sammenligne lengder.
 *
 * Retur: lengde kvadrert (x² + y² + z²)
 */
float vec3_length_squared(const struct vec3 *v);

/**
 * vec3_normalize - Normaliser vektor til lengde 1
 * @v: vektor som skal normaliseres (modifiseres in-place)
 *
 * Hvis vektoren har lengde 0, endres den ikke.
 */
void vec3_normalize(struct vec3 *v);

/**
 * vec3_scale - Skaler vektor med faktor
 * @v: vektor som skal skaleres (modifiseres in-place)
 * @s: skaleringsfaktor
 */
void vec3_scale(struct vec3 *v, float s);

/**
 * vec3_negate - Inverter vektor (multipliser med -1)
 * @v: vektor som skal inverteres (modifiseres in-place)
 */
void vec3_negate(struct vec3 *v);

/*
 * 2 vektorer mot hverandre
 */

/**
 * vec3_add - Legg sammen to vektorer
 * @a: første vektor
 * @b: andre vektor
 * @out: output vektor (a + b)
 */
void vec3_add(const struct vec3 *a, const struct vec3 *b, struct vec3 *out);

/**
 * vec3_sub - Trekk fra to vektorer
 * @a: første vektor
 * @b: andre vektor
 * @out: output vektor (a - b)
 */
void vec3_sub(const struct vec3 *a, const struct vec3 *b, struct vec3 *out);

/**
 * vec3_dot - Beregn dot product (scalar product)
 * @a: første vektor
 * @b: andre vektor
 *
 * Retur: dot product (a·b = ax*bx + ay*by + az*bz)
 */
float vec3_dot(const struct vec3 *a, const struct vec3 *b);

/**
 * vec3_cross - Beregn cross product
 * @a: første vektor
 * @b: andre vektor
 * @out: output vektor (a × b)
 *
 * Cross product står vinkelrett på både a og b.
 */
void vec3_cross(const struct vec3 *a, const struct vec3 *b, struct vec3 *out);

/**
 * vec3_distance - Beregn avstand mellom to punkter
 * @a: første punkt
 * @b: andre punkt
 *
 * Retur: euklidsk avstand mellom a og b
 */
float vec3_distance(const struct vec3 *a, const struct vec3 *b);

/**
 * vec3_distance_squared - Beregn avstand i kvadrat (uten sqrt)
 * @a: første punkt
 * @b: andre punkt
 *
 * Raskere enn vec3_distance() når man kun trenger å sammenligne avstander.
 *
 * Retur: avstand i kvadrat
 */
float vec3_distance_squared(const struct vec3 *a, const struct vec3 *b);

#endif /* ROBOTICS_MATH_VEC3_H */