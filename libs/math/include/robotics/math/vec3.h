#ifndef ROBOTICS_MATH_VEC3_H
#define ROBOTICS_MATH_VEC3_H

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

/**
 * @function vec3_length_squared
 * @api PUBLIC
 *
 * @input  v->{x,y,z}  float
 * @output return      float
 *
 * Beregner lengde i kvadrat (x² + y² + z²), uten sqrt.
 * Raskere enn vec3_length() når man kun trenger å sammenligne lengder.
 */
float vec3_length_squared(const struct vec3 *v);

/**
 * @function vec3_normalize
 * @api PUBLIC
 *
 * @input  v->{x,y,z}  float
 * @output v->{x,y,z}  float (normalized to length 1)
 *
 * Normaliserer vektor til lengde 1 (in-place).
 * Hvis vektoren har lengde 0, endres den ikke.
 */
void vec3_normalize(struct vec3 *v);

/**
 * @function vec3_scale
 * @api PUBLIC
 *
 * @input  v->{x,y,z}  float
 * @input  s           float
 * @output v->{x,y,z}  float (scaled by s)
 *
 * Skalerer vektor med faktor s (in-place).
 */
void vec3_scale(struct vec3 *v, float s);

/**
 * @function vec3_negate
 * @api PUBLIC
 *
 * @input  v->{x,y,z}  float
 * @output v->{x,y,z}  float (negated)
 *
 * Inverterer vektor - multipliserer med -1 (in-place).
 */
void vec3_negate(struct vec3 *v);

/*
 * 2 vektorer mot hverandre
 */

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

/**
 * @function vec3_sub
 * @api PUBLIC
 *
 * @input  a->{x,y,z}    float
 * @input  b->{x,y,z}    float
 * @output out->{x,y,z}  float (a - b)
 *
 * Trekker fra to vektorer: out = a - b
 */
void vec3_sub(const struct vec3 *a, const struct vec3 *b, struct vec3 *out);

/**
 * @function vec3_dot
 * @api PUBLIC
 *
 * @input  a->{x,y,z}  float
 * @input  b->{x,y,z}  float
 * @output return      float (a·b)
 *
 * Beregner dot product (scalar product): a·b = ax*bx + ay*by + az*bz
 */
float vec3_dot(const struct vec3 *a, const struct vec3 *b);

/**
 * @function vec3_cross
 * @api PUBLIC
 *
 * @input  a->{x,y,z}    float
 * @input  b->{x,y,z}    float
 * @output out->{x,y,z}  float (a × b)
 *
 * Beregner cross product: out = a × b
 * Cross product står vinkelrett på både a og b.
 */
void vec3_cross(const struct vec3 *a, const struct vec3 *b, struct vec3 *out);

/**
 * @function vec3_distance
 * @api PUBLIC
 *
 * @input  a->{x,y,z}  float
 * @input  b->{x,y,z}  float
 * @output return      float
 *
 * Beregner euklidsk avstand mellom to punkter.
 */
float vec3_distance(const struct vec3 *a, const struct vec3 *b);

/**
 * @function vec3_distance_squared
 * @api PUBLIC
 *
 * @input  a->{x,y,z}  float
 * @input  b->{x,y,z}  float
 * @output return      float
 *
 * Beregner avstand i kvadrat, uten sqrt.
 * Raskere enn vec3_distance() når man kun trenger å sammenligne avstander.
 */
float vec3_distance_squared(const struct vec3 *a, const struct vec3 *b);

#endif ROBOTICS_MATH_VEC3_H