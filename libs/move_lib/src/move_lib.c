/**
 * @file move_lib.c
 * @brief Move library implementation
 */

#include "move_lib.h"
#include "stewart/geometry.h"
#include "stewart/pose.h"
#include "../../song_lib/vendor/cJSON.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TWO_PI (2.0f * M_PI)

/*
 * Global state
 */
struct move move_lib[MOVE_LIB_SIZE];

struct move_mixer move_mixer = {
	.deck_a = 0,
	.deck_b = 1,
	.crossfader = 0.0f,
	.volume_a = 1.0f,
	.volume_b = 1.0f,
};

struct move_playback move_playback = {
	.t = 0.0f,
	.bpm = 120.0f,
	.master_phase = 0.0f,
};

/*
 * Phase functions
 */
float move_phase_1(const struct move_playback *pb)
{
	float beats_per_sec = pb->bpm / 60.0f;
	return fmodf(TWO_PI * pb->t * beats_per_sec + pb->master_phase, TWO_PI);
}

float move_phase_05(const struct move_playback *pb)
{
	float beats_per_sec = pb->bpm / 60.0f;
	return fmodf(TWO_PI * pb->t * beats_per_sec * 0.5f + pb->master_phase,
		     TWO_PI);
}

float move_phase_025(const struct move_playback *pb)
{
	float beats_per_sec = pb->bpm / 60.0f;
	return fmodf(TWO_PI * pb->t * beats_per_sec * 0.25f + pb->master_phase,
		     TWO_PI);
}

/*
 * Evaluate a single DOF
 */
static float eval_dof(const struct move_dof *dof, float phase1, float phase05,
		      float phase025, float max_amp, float max_bias)
{
	float result = 0.0f;

	/* Harmonic 0: 1 beat */
	result += max_amp * dof->h[0].amplitude *
		  sinf(phase1 + TWO_PI * dof->h[0].phase);

	/* Harmonic 1: 1/2 beat */
	result += max_amp * dof->h[1].amplitude *
		  sinf(phase05 + TWO_PI * dof->h[1].phase);

	/* Harmonic 2: 1/4 beat */
	result += max_amp * dof->h[2].amplitude *
		  sinf(phase025 + TWO_PI * dof->h[2].phase);

	/* Bias: -1.0 to +1.0, directly scaled by max_bias */
	result += max_bias * dof->bias;

	return result;
}

/*
 * Evaluate DOF with derivatives
 * For A*sin(ωt + φ):
 *   pos = A*sin(ωt + φ)
 *   vel = A*ω*cos(ωt + φ)
 *   acc = -A*ω²*sin(ωt + φ)
 *   jerk = -A*ω³*cos(ωt + φ)
 */
static void eval_dof_derivatives(const struct move_dof *dof,
				 float phase1, float phase05, float phase025,
				 float omega1, float omega05, float omega025,
				 float max_amp, float max_bias,
				 float *pos, float *vel, float *acc, float *jerk)
{
	*pos = 0.0f;
	*vel = 0.0f;
	*acc = 0.0f;
	*jerk = 0.0f;

	/* Harmonic 0: 1 beat (ω = omega1) */
	float ph0 = phase1 + TWO_PI * dof->h[0].phase;
	float A0 = max_amp * dof->h[0].amplitude;
	float s0 = sinf(ph0);
	float c0 = cosf(ph0);
	*pos += A0 * s0;
	*vel += A0 * omega1 * c0;
	*acc += -A0 * omega1 * omega1 * s0;
	*jerk += -A0 * omega1 * omega1 * omega1 * c0;

	/* Harmonic 1: 1/2 beat (ω = omega05) */
	float ph1 = phase05 + TWO_PI * dof->h[1].phase;
	float A1 = max_amp * dof->h[1].amplitude;
	float s1 = sinf(ph1);
	float c1 = cosf(ph1);
	*pos += A1 * s1;
	*vel += A1 * omega05 * c1;
	*acc += -A1 * omega05 * omega05 * s1;
	*jerk += -A1 * omega05 * omega05 * omega05 * c1;

	/* Harmonic 2: 1/4 beat (ω = omega025) */
	float ph2 = phase025 + TWO_PI * dof->h[2].phase;
	float A2 = max_amp * dof->h[2].amplitude;
	float s2 = sinf(ph2);
	float c2 = cosf(ph2);
	*pos += A2 * s2;
	*vel += A2 * omega025 * c2;
	*acc += -A2 * omega025 * omega025 * s2;
	*jerk += -A2 * omega025 * omega025 * omega025 * c2;

	/* Bias only affects position (constant) */
	*pos += max_bias * dof->bias;
}

/*
 * Core evaluation
 */
void move_evaluate(const struct move *m, const struct move_playback *pb,
		   const struct stewart_geometry *geom,
		   struct stewart_pose *out)
{
	float p1 = move_phase_1(pb);
	float p05 = move_phase_05(pb);
	float p025 = move_phase_025(pb);

	out->rx = eval_dof(&m->dof[DOF_RX], p1, p05, p025,
			   geom->max_pose_rotation_amplitude,
			   geom->max_pose_rotation_bias);
	out->ry = eval_dof(&m->dof[DOF_RY], p1, p05, p025,
			   geom->max_pose_rotation_amplitude,
			   geom->max_pose_rotation_bias);
	out->rz = eval_dof(&m->dof[DOF_RZ], p1, p05, p025,
			   geom->max_pose_rotation_amplitude,
			   geom->max_pose_rotation_bias);

	out->tx = eval_dof(&m->dof[DOF_TX], p1, p05, p025,
			   geom->max_pose_translation_amplitude,
			   geom->max_pose_translation_bias);
	out->ty = eval_dof(&m->dof[DOF_TY], p1, p05, p025,
			   geom->max_pose_translation_amplitude,
			   geom->max_pose_translation_bias);
	out->tz = eval_dof(&m->dof[DOF_TZ], p1, p05, p025,
			   geom->max_pose_translation_amplitude,
			   geom->max_pose_translation_bias);
}

void move_evaluate_derivatives(const struct move *m,
			       const struct move_playback *pb,
			       const struct stewart_geometry *geom,
			       struct stewart_pose *pos,
			       struct stewart_pose *vel,
			       struct stewart_pose *acc,
			       struct stewart_pose *jerk)
{
	float p1 = move_phase_1(pb);
	float p05 = move_phase_05(pb);
	float p025 = move_phase_025(pb);

	/* Angular frequencies: ω = 2π * f */
	float beats_per_sec = pb->bpm / 60.0f;
	float omega1 = TWO_PI * beats_per_sec;        /* 1 beat */
	float omega05 = TWO_PI * beats_per_sec * 0.5f;  /* 1/2 beat */
	float omega025 = TWO_PI * beats_per_sec * 0.25f; /* 1/4 beat */

	/* Rotations */
	eval_dof_derivatives(&m->dof[DOF_RX], p1, p05, p025,
			     omega1, omega05, omega025,
			     geom->max_pose_rotation_amplitude,
			     geom->max_pose_rotation_bias,
			     &pos->rx, &vel->rx, &acc->rx, &jerk->rx);
	eval_dof_derivatives(&m->dof[DOF_RY], p1, p05, p025,
			     omega1, omega05, omega025,
			     geom->max_pose_rotation_amplitude,
			     geom->max_pose_rotation_bias,
			     &pos->ry, &vel->ry, &acc->ry, &jerk->ry);
	eval_dof_derivatives(&m->dof[DOF_RZ], p1, p05, p025,
			     omega1, omega05, omega025,
			     geom->max_pose_rotation_amplitude,
			     geom->max_pose_rotation_bias,
			     &pos->rz, &vel->rz, &acc->rz, &jerk->rz);

	/* Translations */
	eval_dof_derivatives(&m->dof[DOF_TX], p1, p05, p025,
			     omega1, omega05, omega025,
			     geom->max_pose_translation_amplitude,
			     geom->max_pose_translation_bias,
			     &pos->tx, &vel->tx, &acc->tx, &jerk->tx);
	eval_dof_derivatives(&m->dof[DOF_TY], p1, p05, p025,
			     omega1, omega05, omega025,
			     geom->max_pose_translation_amplitude,
			     geom->max_pose_translation_bias,
			     &pos->ty, &vel->ty, &acc->ty, &jerk->ty);
	eval_dof_derivatives(&m->dof[DOF_TZ], p1, p05, p025,
			     omega1, omega05, omega025,
			     geom->max_pose_translation_amplitude,
			     geom->max_pose_translation_bias,
			     &pos->tz, &vel->tz, &acc->tz, &jerk->tz);
}

void move_evaluate_mixed(const struct move_mixer *mix,
			 const struct move_playback *pb,
			 const struct stewart_geometry *geom,
			 struct stewart_pose *out)
{
	struct stewart_pose a, b;

	/* Evaluate both decks with same phase */
	move_evaluate(&move_lib[mix->deck_a], pb, geom, &a);
	move_evaluate(&move_lib[mix->deck_b], pb, geom, &b);

	/* Crossfade with individual volumes */
	float fa = (1.0f - mix->crossfader) * mix->volume_a;
	float fb = mix->crossfader * mix->volume_b;

	out->rx = a.rx * fa + b.rx * fb;
	out->ry = a.ry * fa + b.ry * fb;
	out->rz = a.rz * fa + b.rz * fb;
	out->tx = a.tx * fa + b.tx * fb;
	out->ty = a.ty * fa + b.ty * fb;
	out->tz = a.tz * fa + b.tz * fb;
}

/*
 * Playback control
 */
void move_playback_tick(struct move_playback *pb, float dt)
{
	pb->t += dt;
}

void move_playback_reset(struct move_playback *pb)
{
	pb->t = 0.0f;
}

void move_playback_set_bpm(struct move_playback *pb, float bpm)
{
	pb->bpm = bpm;
}

/*
 * Mixer control
 */
void move_mixer_set_crossfade(struct move_mixer *mix, float value)
{
	if (value < 0.0f)
		value = 0.0f;
	if (value > 1.0f)
		value = 1.0f;
	mix->crossfader = value;
}

void move_mixer_set_deck_a(struct move_mixer *mix, int move_index)
{
	if (move_index >= 0 && move_index < MOVE_LIB_SIZE)
		mix->deck_a = move_index;
}

void move_mixer_set_deck_b(struct move_mixer *mix, int move_index)
{
	if (move_index >= 0 && move_index < MOVE_LIB_SIZE)
		mix->deck_b = move_index;
}

void move_mixer_swap_decks(struct move_mixer *mix)
{
	int tmp = mix->deck_a;
	mix->deck_a = mix->deck_b;
	mix->deck_b = tmp;

	float vtmp = mix->volume_a;
	mix->volume_a = mix->volume_b;
	mix->volume_b = vtmp;

	mix->crossfader = 1.0f - mix->crossfader;
}

/*
 * Move manipulation
 */
void move_clear(struct move *m)
{
	memset(m, 0, sizeof(*m));
	/* bias = 0.0 is neutral, no loop needed after memset */
}

void move_copy(struct move *dst, const struct move *src)
{
	memcpy(dst, src, sizeof(*dst));
}

void move_randomize(struct move *m, float intensity)
{
	for (int d = 0; d < MOVE_NUM_DOFS; d++) {
		for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
			m->dof[d].h[h].amplitude =
				intensity * (float)rand() / (float)RAND_MAX;
			m->dof[d].h[h].phase = (float)rand() / (float)RAND_MAX;
		}
		/* bias stays 0.0 (neutral) from randomize */
	}
}

void move_interpolate(struct move *dst, const struct move *a,
		      const struct move *b, float t)
{
	float inv_t = 1.0f - t;

	for (int d = 0; d < MOVE_NUM_DOFS; d++) {
		for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
			dst->dof[d].h[h].amplitude =
				inv_t * a->dof[d].h[h].amplitude +
				t * b->dof[d].h[h].amplitude;
			dst->dof[d].h[h].phase = inv_t * a->dof[d].h[h].phase +
						 t * b->dof[d].h[h].phase;
		}
		dst->dof[d].bias = inv_t * a->dof[d].bias + t * b->dof[d].bias;
	}
}

void move_mirror(struct move *m, int dof_mask)
{
	for (int d = 0; d < MOVE_NUM_DOFS; d++) {
		if (dof_mask & (1 << d)) {
			for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
				m->dof[d].h[h].amplitude =
					-m->dof[d].h[h].amplitude;
			}
			m->dof[d].bias = -m->dof[d].bias;
		}
	}
}

void move_phase_shift(struct move *m, int dof_mask, float shift)
{
	for (int d = 0; d < MOVE_NUM_DOFS; d++) {
		if (dof_mask & (1 << d)) {
			for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
				m->dof[d].h[h].phase += shift;
				/* Wrap til [0, 1] */
				while (m->dof[d].h[h].phase > 1.0f)
					m->dof[d].h[h].phase -= 1.0f;
				while (m->dof[d].h[h].phase < 0.0f)
					m->dof[d].h[h].phase += 1.0f;
			}
		}
	}
}

void move_scale_amplitude(struct move *m, int dof_mask, float scale)
{
	for (int d = 0; d < MOVE_NUM_DOFS; d++) {
		if (dof_mask & (1 << d)) {
			for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
				m->dof[d].h[h].amplitude *= scale;
			}
			m->dof[d].bias *= scale;
		}
	}
}

void move_swap_dofs(struct move *m, int dof_a, int dof_b)
{
	if (dof_a < 0 || dof_a >= MOVE_NUM_DOFS)
		return;
	if (dof_b < 0 || dof_b >= MOVE_NUM_DOFS)
		return;

	struct move_dof tmp = m->dof[dof_a];
	m->dof[dof_a] = m->dof[dof_b];
	m->dof[dof_b] = tmp;
}

/*
 * Spline transitions
 *
 * C0: Linear interpolation p(u) = (1-u)*p0 + u*p1
 *
 * C1: Cubic Hermite spline
 *     p(u) = h00*p0 + h10*v0*T + h01*p1 + h11*v1*T
 *     where T = duration, and basis functions are:
 *     h00 = 2u³ - 3u² + 1
 *     h10 = u³ - 2u² + u
 *     h01 = -2u³ + 3u²
 *     h11 = u³ - u²
 *
 * C2: Quintic Hermite spline (matches pos, vel, acc)
 *     p(u) = a0 + a1*u + a2*u² + a3*u³ + a4*u⁴ + a5*u⁵
 *     Coefficients derived from boundary conditions.
 */

/* Helper: capture target pose at future time */
static void capture_target_derivatives(const struct move *m,
				       const struct move_playback *pb,
				       const struct stewart_geometry *geom,
				       float t_offset,
				       struct stewart_pose *pos,
				       struct stewart_pose *vel,
				       struct stewart_pose *acc)
{
	/* Create temporary playback at future time */
	struct move_playback future_pb = *pb;
	future_pb.t += t_offset;

	struct stewart_pose jerk;  /* unused but required */
	move_evaluate_derivatives(m, &future_pb, geom, pos, vel, acc, &jerk);
}

void move_spline_init_c0(struct move_spline *spline,
			 const struct move *from,
			 const struct move *to,
			 const struct move_playback *pb,
			 const struct stewart_geometry *geom,
			 float duration)
{
	spline->t_start = pb->t;
	spline->duration = duration;
	spline->continuity = 0;

	/* Capture current "from" position */
	move_evaluate(from, pb, geom, &spline->p0);

	/* Capture "to" position at end of transition */
	struct move_playback end_pb = *pb;
	end_pb.t += duration;
	move_evaluate(to, &end_pb, geom, &spline->p1);

	/* C0 doesn't use velocities/accelerations, but zero them for safety */
	memset(&spline->v0, 0, sizeof(spline->v0));
	memset(&spline->v1, 0, sizeof(spline->v1));
	memset(&spline->a0, 0, sizeof(spline->a0));
	memset(&spline->a1, 0, sizeof(spline->a1));
}

void move_spline_init_c1(struct move_spline *spline,
			 const struct move *from,
			 const struct move *to,
			 const struct move_playback *pb,
			 const struct stewart_geometry *geom,
			 float duration)
{
	spline->t_start = pb->t;
	spline->duration = duration;
	spline->continuity = 1;

	/* Capture current "from" position and velocity */
	struct stewart_pose jerk;
	move_evaluate_derivatives(from, pb, geom,
				  &spline->p0, &spline->v0, &spline->a0, &jerk);

	/* Capture "to" position and velocity at end of transition */
	capture_target_derivatives(to, pb, geom, duration,
				   &spline->p1, &spline->v1, &spline->a1);
}

void move_spline_init_c2(struct move_spline *spline,
			 const struct move *from,
			 const struct move *to,
			 const struct move_playback *pb,
			 const struct stewart_geometry *geom,
			 float duration)
{
	spline->t_start = pb->t;
	spline->duration = duration;
	spline->continuity = 2;

	/* Capture current "from" position, velocity, and acceleration */
	struct stewart_pose jerk;
	move_evaluate_derivatives(from, pb, geom,
				  &spline->p0, &spline->v0, &spline->a0, &jerk);

	/* Capture "to" at end of transition */
	capture_target_derivatives(to, pb, geom, duration,
				   &spline->p1, &spline->v1, &spline->a1);
}

void move_spline_init_c0_ease(struct move_spline *spline,
			      const struct move *from,
			      const struct move *to,
			      const struct move_playback *pb,
			      const struct stewart_geometry *geom,
			      float duration,
			      float ease_in,
			      float ease_out)
{
	spline->t_start = pb->t;
	spline->duration = duration;
	spline->continuity = 3;  /* Special: c0 with ease */
	spline->ease_in = (ease_in > 0.5f) ? 0.5f : ease_in;
	spline->ease_out = (ease_out > 0.5f) ? 0.5f : ease_out;

	/* Capture current "from" position */
	move_evaluate(from, pb, geom, &spline->p0);

	/* Capture "to" position at end of transition */
	struct move_playback end_pb = *pb;
	end_pb.t += duration;
	move_evaluate(to, &end_pb, geom, &spline->p1);

	/* Zero velocities/accelerations (not used) */
	memset(&spline->v0, 0, sizeof(spline->v0));
	memset(&spline->v1, 0, sizeof(spline->v1));
	memset(&spline->a0, 0, sizeof(spline->a0));
	memset(&spline->a1, 0, sizeof(spline->a1));
}

/* Evaluate single DOF with C0 (linear) */
static float spline_eval_c0(float p0, float p1, float u)
{
	return (1.0f - u) * p0 + u * p1;
}

/*
 * Apply ease-in/out to normalized time u
 *
 * Kubisk ease med C1 kontinuitet: starter/slutter med slope=0,
 * matcher lineær slope=1 ved grensene.
 *
 * ease_in:  andel av starten med ease (0.0-0.5)
 * ease_out: andel av slutten med ease (0.0-0.5)
 */
static float apply_ease(float u, float ease_in, float ease_out)
{
	if (ease_in <= 0.0f && ease_out <= 0.0f)
		return u;  /* Ingen ease, ren lineær */

	float lin_end = 1.0f - ease_out;
	float t;

	if (u <= 0.0f) {
		t = 0.0f;
	} else if (u >= 1.0f) {
		t = 1.0f;
	} else if (u < ease_in && ease_in > 0.0f) {
		/*
		 * Ease-in: kubisk kurve som starter med slope=0, ender med slope=1
		 * Betingelser: t(0)=0, t'(0)=0, t(e)=e, t'(e)=1
		 * Løsning: t = u²(2e - u) / e²
		 */
		float e = ease_in;
		t = (u * u * (2.0f * e - u)) / (e * e);
	} else if (u > lin_end && ease_out > 0.0f) {
		/*
		 * Ease-out: kubisk kurve som starter med slope=1, ender med slope=0
		 * Speilet versjon av ease-in
		 */
		float one_minus_u = 1.0f - u;
		float e = ease_out;
		t = 1.0f - (one_minus_u * one_minus_u * (2.0f * e - one_minus_u)) / (e * e);
	} else {
		/* Lineær sone */
		t = u;
	}

	return t;
}

/* Evaluate single DOF with C0 + ease */
static float spline_eval_c0_ease(float p0, float p1, float u,
				 float ease_in, float ease_out)
{
	float t = apply_ease(u, ease_in, ease_out);
	return (1.0f - t) * p0 + t * p1;
}

/* Evaluate single DOF with C1 (cubic Hermite) */
static float spline_eval_c1(float p0, float p1, float v0, float v1,
			    float u, float T)
{
	float u2 = u * u;
	float u3 = u2 * u;

	/* Hermite basis functions */
	float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
	float h10 = u3 - 2.0f * u2 + u;
	float h01 = -2.0f * u3 + 3.0f * u2;
	float h11 = u3 - u2;

	return h00 * p0 + h10 * (v0 * T) + h01 * p1 + h11 * (v1 * T);
}

/* Evaluate single DOF with C2 (quintic) */
static float spline_eval_c2(float p0, float p1,
			    float v0, float v1,
			    float a0, float a1,
			    float u, float T)
{
	/*
	 * Quintic polynomial: p(u) = sum(ai * u^i) for i=0..5
	 *
	 * Boundary conditions (in normalized time u = t/T):
	 *   p(0) = p0,  p(1) = p1
	 *   p'(0) = v0*T,  p'(1) = v1*T
	 *   p''(0) = a0*T²,  p''(1) = a1*T²
	 *
	 * Solving gives:
	 *   a0 = p0
	 *   a1 = v0*T
	 *   a2 = a0*T²/2
	 *   a3 = 10*(p1-p0) - 6*v0*T - 4*v1*T - 1.5*a0*T² + 0.5*a1*T²
	 *   a4 = -15*(p1-p0) + 8*v0*T + 7*v1*T + 1.5*a0*T² - a1*T²
	 *   a5 = 6*(p1-p0) - 3*v0*T - 3*v1*T - 0.5*a0*T² + 0.5*a1*T²
	 */
	float T2 = T * T;
	float dp = p1 - p0;
	float v0T = v0 * T;
	float v1T = v1 * T;
	float a0T2 = a0 * T2;
	float a1T2 = a1 * T2;

	float c0 = p0;
	float c1 = v0T;
	float c2 = 0.5f * a0T2;
	float c3 = 10.0f * dp - 6.0f * v0T - 4.0f * v1T - 1.5f * a0T2 + 0.5f * a1T2;
	float c4 = -15.0f * dp + 8.0f * v0T + 7.0f * v1T + 1.5f * a0T2 - a1T2;
	float c5 = 6.0f * dp - 3.0f * v0T - 3.0f * v1T - 0.5f * a0T2 + 0.5f * a1T2;

	float u2 = u * u;
	float u3 = u2 * u;
	float u4 = u3 * u;
	float u5 = u4 * u;

	return c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 + c5 * u5;
}

int move_spline_evaluate(const struct move_spline *spline,
			 const struct move_playback *pb,
			 struct stewart_pose *out)
{
	float elapsed = pb->t - spline->t_start;

	/* Before start: return start pose */
	if (elapsed <= 0.0f) {
		*out = spline->p0;
		return 1;
	}

	/* After end: return end pose */
	if (elapsed >= spline->duration) {
		*out = spline->p1;
		return 0;  /* Transition complete */
	}

	/* Normalized time u in [0, 1] */
	float u = elapsed / spline->duration;
	float T = spline->duration;

	/* Evaluate based on continuity level */
	switch (spline->continuity) {
	case 0:  /* C0 - linear */
		out->rx = spline_eval_c0(spline->p0.rx, spline->p1.rx, u);
		out->ry = spline_eval_c0(spline->p0.ry, spline->p1.ry, u);
		out->rz = spline_eval_c0(spline->p0.rz, spline->p1.rz, u);
		out->tx = spline_eval_c0(spline->p0.tx, spline->p1.tx, u);
		out->ty = spline_eval_c0(spline->p0.ty, spline->p1.ty, u);
		out->tz = spline_eval_c0(spline->p0.tz, spline->p1.tz, u);
		break;

	case 1:  /* C1 - cubic Hermite */
		out->rx = spline_eval_c1(spline->p0.rx, spline->p1.rx,
					 spline->v0.rx, spline->v1.rx, u, T);
		out->ry = spline_eval_c1(spline->p0.ry, spline->p1.ry,
					 spline->v0.ry, spline->v1.ry, u, T);
		out->rz = spline_eval_c1(spline->p0.rz, spline->p1.rz,
					 spline->v0.rz, spline->v1.rz, u, T);
		out->tx = spline_eval_c1(spline->p0.tx, spline->p1.tx,
					 spline->v0.tx, spline->v1.tx, u, T);
		out->ty = spline_eval_c1(spline->p0.ty, spline->p1.ty,
					 spline->v0.ty, spline->v1.ty, u, T);
		out->tz = spline_eval_c1(spline->p0.tz, spline->p1.tz,
					 spline->v0.tz, spline->v1.tz, u, T);
		break;

	case 2:  /* C2 - quintic */
		out->rx = spline_eval_c2(spline->p0.rx, spline->p1.rx,
					 spline->v0.rx, spline->v1.rx,
					 spline->a0.rx, spline->a1.rx, u, T);
		out->ry = spline_eval_c2(spline->p0.ry, spline->p1.ry,
					 spline->v0.ry, spline->v1.ry,
					 spline->a0.ry, spline->a1.ry, u, T);
		out->rz = spline_eval_c2(spline->p0.rz, spline->p1.rz,
					 spline->v0.rz, spline->v1.rz,
					 spline->a0.rz, spline->a1.rz, u, T);
		out->tx = spline_eval_c2(spline->p0.tx, spline->p1.tx,
					 spline->v0.tx, spline->v1.tx,
					 spline->a0.tx, spline->a1.tx, u, T);
		out->ty = spline_eval_c2(spline->p0.ty, spline->p1.ty,
					 spline->v0.ty, spline->v1.ty,
					 spline->a0.ty, spline->a1.ty, u, T);
		out->tz = spline_eval_c2(spline->p0.tz, spline->p1.tz,
					 spline->v0.tz, spline->v1.tz,
					 spline->a0.tz, spline->a1.tz, u, T);
		break;

	case 3:  /* C0 with ease-in/out */
		out->rx = spline_eval_c0_ease(spline->p0.rx, spline->p1.rx, u,
					      spline->ease_in, spline->ease_out);
		out->ry = spline_eval_c0_ease(spline->p0.ry, spline->p1.ry, u,
					      spline->ease_in, spline->ease_out);
		out->rz = spline_eval_c0_ease(spline->p0.rz, spline->p1.rz, u,
					      spline->ease_in, spline->ease_out);
		out->tx = spline_eval_c0_ease(spline->p0.tx, spline->p1.tx, u,
					      spline->ease_in, spline->ease_out);
		out->ty = spline_eval_c0_ease(spline->p0.ty, spline->p1.ty, u,
					      spline->ease_in, spline->ease_out);
		out->tz = spline_eval_c0_ease(spline->p0.tz, spline->p1.tz, u,
					      spline->ease_in, spline->ease_out);
		break;

	default:
		*out = spline->p0;  /* Fallback */
		break;
	}

	return 1;  /* Still in transition */
}

/*
 * Serialization
 */
int move_to_floats(const struct move *m, float *out, int max_floats)
{
	if (max_floats < MOVE_TOTAL_PARAMS)
		return -1;

	int idx = 0;
	for (int d = 0; d < MOVE_NUM_DOFS; d++) {
		for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
			out[idx++] = m->dof[d].h[h].amplitude;
			out[idx++] = m->dof[d].h[h].phase;
		}
		out[idx++] = m->dof[d].bias;
	}

	return idx;
}

int move_from_floats(struct move *m, const float *in, int num_floats)
{
	if (num_floats < MOVE_TOTAL_PARAMS)
		return -1;

	int idx = 0;
	for (int d = 0; d < MOVE_NUM_DOFS; d++) {
		for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
			m->dof[d].h[h].amplitude = in[idx++];
			m->dof[d].h[h].phase = in[idx++];
		}
		m->dof[d].bias = in[idx++];
	}

	return idx;
}

/*
 * Library management
 */
void move_lib_clear_all(void)
{
	for (int i = 0; i < MOVE_LIB_SIZE; i++)
		move_clear(&move_lib[i]);
}

void move_lib_clear(int index)
{
	if (index >= 0 && index < MOVE_LIB_SIZE)
		move_clear(&move_lib[index]);
}

void move_lib_randomize_range(int start, int end, float intensity)
{
	if (start < 0)
		start = 0;
	if (end > MOVE_LIB_SIZE)
		end = MOVE_LIB_SIZE;

	for (int i = start; i < end; i++) {
		move_randomize(&move_lib[i], intensity);
		snprintf(move_lib[i].name, MOVE_NAME_LEN, "rnd%d", i);
	}
}

/*
 * Default presets
 */
void move_lib_init(void)
{
	move_lib_clear_all();

	/* Move 0: Still (home position) */
	strncpy(move_lib[0].name, "still", MOVE_NAME_LEN - 1);
	move_lib[0].flags = MOVE_FLAG_PRESET;

	/* Move 1: Simple nod (rx at 1 beat) */
	strncpy(move_lib[1].name, "nod", MOVE_NAME_LEN - 1);
	move_lib[1].dof[DOF_RX].h[0].amplitude = 0.6f;
	move_lib[1].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 2: Side tilt (ry at 1 beat) */
	strncpy(move_lib[2].name, "tilt", MOVE_NAME_LEN - 1);
	move_lib[2].dof[DOF_RY].h[0].amplitude = 0.5f;
	move_lib[2].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 3: Twist (rz at 1/2 beat) */
	strncpy(move_lib[3].name, "twist", MOVE_NAME_LEN - 1);
	move_lib[3].dof[DOF_RZ].h[1].amplitude = 0.4f;
	move_lib[3].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 4: Bounce (ty at 1 beat) */
	strncpy(move_lib[4].name, "bounce", MOVE_NAME_LEN - 1);
	move_lib[4].dof[DOF_TY].h[0].amplitude = 0.7f;
	move_lib[4].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 5: Sway (tx at 1 beat) */
	strncpy(move_lib[5].name, "sway", MOVE_NAME_LEN - 1);
	move_lib[5].dof[DOF_TX].h[0].amplitude = 0.5f;
	move_lib[5].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 6: Circle (tx + tz, 90 deg phase diff) */
	strncpy(move_lib[6].name, "circle", MOVE_NAME_LEN - 1);
	move_lib[6].dof[DOF_TX].h[0].amplitude = 0.5f;
	move_lib[6].dof[DOF_TZ].h[0].amplitude = 0.5f;
	move_lib[6].dof[DOF_TZ].h[0].phase = 0.25f;
	move_lib[6].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 7: Complex (multi-harmonic) */
	strncpy(move_lib[7].name, "complex", MOVE_NAME_LEN - 1);
	move_lib[7].dof[DOF_RX].h[0].amplitude = 0.4f;
	move_lib[7].dof[DOF_RY].h[1].amplitude = 0.3f;
	move_lib[7].dof[DOF_RY].h[1].phase = 0.25f;
	move_lib[7].dof[DOF_TY].h[0].amplitude = 0.5f;
	move_lib[7].dof[DOF_TY].h[2].amplitude = 0.2f;
	move_lib[7].dof[DOF_TY].h[2].phase = 0.5f;
	move_lib[7].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 8: Wave (all rotations, staggered phase) */
	strncpy(move_lib[8].name, "wave", MOVE_NAME_LEN - 1);
	move_lib[8].dof[DOF_RX].h[0].amplitude = 0.4f;
	move_lib[8].dof[DOF_RY].h[0].amplitude = 0.4f;
	move_lib[8].dof[DOF_RY].h[0].phase = 0.33f;
	move_lib[8].dof[DOF_RZ].h[0].amplitude = 0.3f;
	move_lib[8].dof[DOF_RZ].h[0].phase = 0.66f;
	move_lib[8].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 9: Pulse (ty with all harmonics) */
	strncpy(move_lib[9].name, "pulse", MOVE_NAME_LEN - 1);
	move_lib[9].dof[DOF_TY].h[0].amplitude = 0.5f;
	move_lib[9].dof[DOF_TY].h[1].amplitude = 0.25f;
	move_lib[9].dof[DOF_TY].h[2].amplitude = 0.125f;
	move_lib[9].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;
}

/*
 * JSON save/load
 */
static const char *dof_names[MOVE_NUM_DOFS] = {
	"rx", "ry", "rz", "tx", "ty", "tz"
};

int move_lib_save(const char *path)
{
	cJSON *root = cJSON_CreateObject();
	cJSON *moves = cJSON_CreateArray();

	for (int i = 0; i < MOVE_LIB_SIZE; i++) {
		/* Skip empty moves (no name and all zeros) */
		if (move_lib[i].name[0] == '\0' && move_lib[i].flags == 0)
			continue;

		cJSON *move = cJSON_CreateObject();
		cJSON_AddNumberToObject(move, "index", i);
		cJSON_AddStringToObject(move, "name", move_lib[i].name);
		cJSON_AddNumberToObject(move, "flags", move_lib[i].flags);
		cJSON_AddNumberToObject(move, "category", move_lib[i].category);

		/* Laban placeholder (empty for now) */
		cJSON *laban = cJSON_CreateObject();
		cJSON_AddStringToObject(laban, "weight", "");
		cJSON_AddStringToObject(laban, "time", "");
		cJSON_AddStringToObject(laban, "space", "");
		cJSON_AddStringToObject(laban, "flow", "");
		cJSON_AddItemToObject(move, "laban", laban);

		/* DOF params */
		cJSON *params = cJSON_CreateObject();
		for (int d = 0; d < MOVE_NUM_DOFS; d++) {
			cJSON *dof = cJSON_CreateObject();
			cJSON *harmonics = cJSON_CreateArray();

			for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
				cJSON *harm = cJSON_CreateObject();
				cJSON_AddNumberToObject(harm, "amp",
					move_lib[i].dof[d].h[h].amplitude);
				cJSON_AddNumberToObject(harm, "phase",
					move_lib[i].dof[d].h[h].phase);
				cJSON_AddItemToArray(harmonics, harm);
			}

			cJSON_AddItemToObject(dof, "h", harmonics);
			cJSON_AddNumberToObject(dof, "bias", move_lib[i].dof[d].bias);
			cJSON_AddItemToObject(params, dof_names[d], dof);
		}
		cJSON_AddItemToObject(move, "params", params);

		cJSON_AddItemToArray(moves, move);
	}

	cJSON_AddItemToObject(root, "moves", moves);

	char *json_str = cJSON_Print(root);
	cJSON_Delete(root);

	if (!json_str)
		return -1;

	FILE *f = fopen(path, "w");
	if (!f) {
		free(json_str);
		return -1;
	}

	fprintf(f, "%s\n", json_str);
	fclose(f);
	free(json_str);

	return 0;
}

int move_lib_load(const char *path)
{
	FILE *f = fopen(path, "r");
	if (!f)
		return -1;

	fseek(f, 0, SEEK_END);
	long size = ftell(f);
	fseek(f, 0, SEEK_SET);

	char *json_str = malloc(size + 1);
	if (!json_str) {
		fclose(f);
		return -1;
	}

	fread(json_str, 1, size, f);
	json_str[size] = '\0';
	fclose(f);

	cJSON *root = cJSON_Parse(json_str);
	free(json_str);

	if (!root)
		return -1;

	cJSON *moves = cJSON_GetObjectItem(root, "moves");
	if (!cJSON_IsArray(moves)) {
		cJSON_Delete(root);
		return -1;
	}

	int count = 0;
	cJSON *move;
	cJSON_ArrayForEach(move, moves) {
		cJSON *idx_item = cJSON_GetObjectItem(move, "index");
		if (!cJSON_IsNumber(idx_item))
			continue;

		int idx = (int)idx_item->valuedouble;
		if (idx < 0 || idx >= MOVE_LIB_SIZE)
			continue;

		/* Name */
		cJSON *name = cJSON_GetObjectItem(move, "name");
		if (cJSON_IsString(name)) {
			strncpy(move_lib[idx].name, name->valuestring,
				MOVE_NAME_LEN - 1);
			move_lib[idx].name[MOVE_NAME_LEN - 1] = '\0';
		}

		/* Flags and category */
		cJSON *flags = cJSON_GetObjectItem(move, "flags");
		if (cJSON_IsNumber(flags))
			move_lib[idx].flags = (int)flags->valuedouble;

		cJSON *category = cJSON_GetObjectItem(move, "category");
		if (cJSON_IsNumber(category))
			move_lib[idx].category = (int)category->valuedouble;

		/* Params */
		cJSON *params = cJSON_GetObjectItem(move, "params");
		if (cJSON_IsObject(params)) {
			for (int d = 0; d < MOVE_NUM_DOFS; d++) {
				cJSON *dof = cJSON_GetObjectItem(params, dof_names[d]);
				if (!cJSON_IsObject(dof))
					continue;

				cJSON *harmonics = cJSON_GetObjectItem(dof, "h");
				if (cJSON_IsArray(harmonics)) {
					int h = 0;
					cJSON *harm;
					cJSON_ArrayForEach(harm, harmonics) {
						if (h >= MOVE_NUM_HARMONICS)
							break;
						cJSON *amp = cJSON_GetObjectItem(harm, "amp");
						cJSON *phase = cJSON_GetObjectItem(harm, "phase");
						if (cJSON_IsNumber(amp))
							move_lib[idx].dof[d].h[h].amplitude =
								(float)amp->valuedouble;
						if (cJSON_IsNumber(phase))
							move_lib[idx].dof[d].h[h].phase =
								(float)phase->valuedouble;
						h++;
					}
				}

				cJSON *bias = cJSON_GetObjectItem(dof, "bias");
				if (cJSON_IsNumber(bias))
					move_lib[idx].dof[d].bias = (float)bias->valuedouble;
			}
		}

		count++;
	}

	cJSON_Delete(root);
	return count;
}
