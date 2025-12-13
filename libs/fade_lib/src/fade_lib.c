#include "fade_lib.h"
#include "stewart/geometry.h"
#include <math.h>

// ============ Kurve-transformasjoner ============

float curve_linear(float t)
{
	return t;
}

float curve_smoothstep(float t)
{
	// S-kurve: 3t² - 2t³
	return t * t * (3.0f - 2.0f * t);
}

float curve_ease_in(float t)
{
	// Kvadratisk ease-in
	return t * t;
}

float curve_ease_out(float t)
{
	// Kvadratisk ease-out
	return 1.0f - (1.0f - t) * (1.0f - t);
}

float curve_ease_in_out(float t)
{
	// Kombinert ease-in-out
	if (t < 0.5f)
		return 2.0f * t * t;
	else
		return 1.0f - 2.0f * (1.0f - t) * (1.0f - t);
}

// ============ Hjelpefunksjon for pose-blending ============

static void blend_poses(struct stewart_pose *a, struct stewart_pose *b, float t,
			struct stewart_pose *out)
{
	out->rx = a->rx + t * (b->rx - a->rx);
	out->ry = a->ry + t * (b->ry - a->ry);
	out->rz = a->rz + t * (b->rz - a->rz);
	out->tx = a->tx + t * (b->tx - a->tx);
	out->ty = a->ty + t * (b->ty - a->ty);
	out->tz = a->tz + t * (b->tz - a->tz);
}

// ============ Fade-funksjoner ============

void fade_linear(struct move *from, struct move *to, float crossfader,
		 const struct stewart_geometry *geom, struct move_playback *pb,
		 struct stewart_pose *out)
{
	struct stewart_pose pose_from, pose_to;
	move_evaluate(from, pb, geom, &pose_from);
	move_evaluate(to, pb, geom, &pose_to);

	float t = curve_linear(crossfader);
	blend_poses(&pose_from, &pose_to, t, out);
}

void fade_smoothstep(struct move *from, struct move *to, float crossfader,
		     const struct stewart_geometry *geom,
		     struct move_playback *pb, struct stewart_pose *out)
{
	struct stewart_pose pose_from, pose_to;
	move_evaluate(from, pb, geom, &pose_from);
	move_evaluate(to, pb, geom, &pose_to);

	float t = curve_smoothstep(crossfader);
	blend_poses(&pose_from, &pose_to, t, out);
}

void fade_ease_in(struct move *from, struct move *to, float crossfader,
		  const struct stewart_geometry *geom, struct move_playback *pb,
		  struct stewart_pose *out)
{
	struct stewart_pose pose_from, pose_to;
	move_evaluate(from, pb, geom, &pose_from);
	move_evaluate(to, pb, geom, &pose_to);

	float t = curve_ease_in(crossfader);
	blend_poses(&pose_from, &pose_to, t, out);
}

void fade_ease_out(struct move *from, struct move *to, float crossfader,
		   const struct stewart_geometry *geom,
		   struct move_playback *pb, struct stewart_pose *out)
{
	struct stewart_pose pose_from, pose_to;
	move_evaluate(from, pb, geom, &pose_from);
	move_evaluate(to, pb, geom, &pose_to);

	float t = curve_ease_out(crossfader);
	blend_poses(&pose_from, &pose_to, t, out);
}

// ============ Hjelpefunksjon for fase-interpolering ============

// Interpolerer fase med korteste vei mod 2pi
// Faser er lagret som 0.0-1.0 i move_lib, så vi jobber i den skalaen
static float lerp_phase(float a, float b, float t)
{
	// Finn differansen
	float diff = b - a;

	// Wrap til [-0.5, 0.5] for korteste vei
	if (diff > 0.5f)
		diff -= 1.0f;
	else if (diff < -0.5f)
		diff += 1.0f;

	// Interpoler og wrap til [0, 1]
	float result = a + t * diff;
	if (result < 0.0f)
		result += 1.0f;
	else if (result > 1.0f)
		result -= 1.0f;

	return result;
}

// Blander en harmonic (amplitude lineært, phase korteste vei)
static void blend_harmonic(struct move_harmonic *a, struct move_harmonic *b,
			   float t, struct move_harmonic *out)
{
	out->amplitude = a->amplitude + t * (b->amplitude - a->amplitude);
	out->phase = lerp_phase(a->phase, b->phase, t);
}

// Blander en DOF (3 harmonics + bias)
static void blend_dof(struct move_dof *a, struct move_dof *b, float t,
		      struct move_dof *out)
{
	for (int i = 0; i < MOVE_NUM_HARMONICS; i++) {
		blend_harmonic(&a->h[i], &b->h[i], t, &out->h[i]);
	}
	out->bias = a->bias + t * (b->bias - a->bias);
}

// Blander en hel move (alle 42 parametre)
static void blend_move(struct move *a, struct move *b, float t,
		       struct move *out)
{
	for (int i = 0; i < MOVE_NUM_DOFS; i++) {
		blend_dof(&a->dof[i], &b->dof[i], t, &out->dof[i]);
	}
}

void fade_params(struct move *from, struct move *to, float crossfader,
		 const struct stewart_geometry *geom, struct move_playback *pb,
		 struct stewart_pose *out)
{
	// Lag en midlertidig move med blendede parametre
	struct move blended;
	blend_move(from, to, crossfader, &blended);

	// Evaluer den blendede move-en
	move_evaluate(&blended, pb, geom, out);
}

void fade_dip_home(struct move *from, struct move *to, float crossfader,
		   const struct stewart_geometry *geom,
		   struct move_playback *pb, struct stewart_pose *out)
{
	// Smoothstep crossfade mellom posene
	struct stewart_pose pose_from, pose_to;
	move_evaluate(from, pb, geom, &pose_from);
	move_evaluate(to, pb, geom, &pose_to);

	float t = curve_smoothstep(crossfader);
	blend_poses(&pose_from, &pose_to, t, out);

	// "Master volume" kurve: 1 -> 0 -> 1, myk dip ved cf=0.5
	// Bruker cos for myk kurve: cos(cf * pi) går fra 1 til -1
	// Transformerer til 0->1->0 med (1 - cos(cf * 2pi)) / 2 invertert
	// Enklere: sin(cf * pi) gir 0 -> 1 -> 0, så master = 1 - sin(cf * pi)
	float dip = sinf(crossfader * (float)M_PI);  // 0 -> 1 -> 0
	float master = 1.0f - dip;  // 1 -> 0 -> 1

	// Skaler alle posisjoner mot 0 (home)
	out->rx *= master;
	out->ry *= master;
	out->rz *= master;
	out->tx *= master;
	out->ty *= master;
	out->tz *= master;
}

// Midtpose som kan settes manuelt
struct stewart_pose fade_mid_pose = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

// Hold-tid i midten (0.0-1.0, f.eks. 0.2 = 20% av fade-tiden)
float fade_mid_hold = 0.2f;

// Hevet ty for fade_hold_rot
float fade_mid_ty = 15.0f;

void fade_via_pose(struct move *from, struct move *to, float crossfader,
		   const struct stewart_geometry *geom,
		   struct move_playback *pb, struct stewart_pose *out)
{
	struct stewart_pose pose_from, pose_to;
	move_evaluate(from, pb, geom, &pose_from);
	move_evaluate(to, pb, geom, &pose_to);

	// Beregn soner: in_end = når vi når midtpose, out_start = når vi forlater
	float in_end = (1.0f - fade_mid_hold) / 2.0f;   // f.eks. 0.4
	float out_start = 1.0f - in_end;                // f.eks. 0.6

	if (crossfader < in_end) {
		// Fase 1: from -> mid_pose
		float t = curve_smoothstep(crossfader / in_end);
		blend_poses(&pose_from, &fade_mid_pose, t, out);
	} else if (crossfader > out_start) {
		// Fase 3: mid_pose -> to
		float t = curve_smoothstep((crossfader - out_start) / in_end);
		blend_poses(&fade_mid_pose, &pose_to, t, out);
	} else {
		// Fase 2: hold i midtpose
		*out = fade_mid_pose;
	}
}

void fade_hold_rot(struct move *from, struct move *to, float crossfader,
		   const struct stewart_geometry *geom,
		   struct move_playback *pb, struct stewart_pose *out)
{
	struct stewart_pose pose_from, pose_to;
	move_evaluate(from, pb, geom, &pose_from);
	move_evaluate(to, pb, geom, &pose_to);

	// Midtpose: rotasjoner fra "from" ved slutten, hevet ty, tx/tz = 0
	struct stewart_pose mid;
	mid.rx = pose_from.rx;
	mid.ry = pose_from.ry;
	mid.rz = pose_from.rz;
	mid.tx = 0.0f;
	mid.ty = fade_mid_ty;
	mid.tz = 0.0f;

	// Beregn soner
	float in_end = (1.0f - fade_mid_hold) / 2.0f;
	float out_start = 1.0f - in_end;

	if (crossfader < in_end) {
		// Fase 1: from -> mid (hevet med from-rotasjon)
		float t = curve_smoothstep(crossfader / in_end);
		blend_poses(&pose_from, &mid, t, out);
	} else if (crossfader > out_start) {
		// Fase 3: mid -> to
		float t = curve_smoothstep((crossfader - out_start) / in_end);
		blend_poses(&mid, &pose_to, t, out);
	} else {
		// Fase 2: hold i midtpose med from-rotasjon
		*out = mid;
	}
}
