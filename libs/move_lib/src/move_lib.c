/**
 * @file move_lib.c
 * @brief Move library implementation
 */

#include "move_lib.h"
#include <math.h>
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
	.phase_offset_b = 0.0f,
};

struct move_playback move_playback = {
	.t = 0.0f,
	.bpm = 120.0f,
	.master_phase = 0.0f,
	.master_volume = 1.0f,
};

struct move_limits move_limits = {
	.max_rot_amp = 15.0f,
	.max_rot_bias = 10.0f,
	.max_trans_amp = 25.0f,
	.max_trans_bias = 15.0f,
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
	return fmodf(TWO_PI * pb->t * beats_per_sec * 0.5f + pb->master_phase, TWO_PI);
}

float move_phase_025(const struct move_playback *pb)
{
	float beats_per_sec = pb->bpm / 60.0f;
	return fmodf(TWO_PI * pb->t * beats_per_sec * 0.25f + pb->master_phase, TWO_PI);
}

/*
 * Evaluate a single DOF
 */
static float eval_dof(const struct move_dof *dof,
		      float phase1, float phase05, float phase025,
		      float max_amp, float max_bias)
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

	/* Bias: centered around 0.5, so (bias - 0.5) gives -0.5 to +0.5 */
	result += max_bias * (dof->bias - 0.5f);

	return result;
}

/*
 * Core evaluation
 */
void move_evaluate(const struct move *m,
		   const struct move_playback *pb,
		   const struct move_limits *lim,
		   struct move_pose *out)
{
	float p1 = move_phase_1(pb);
	float p05 = move_phase_05(pb);
	float p025 = move_phase_025(pb);
	float vol = pb->master_volume;

	out->rx = vol * eval_dof(&m->rx, p1, p05, p025,
				 lim->max_rot_amp, lim->max_rot_bias);
	out->ry = vol * eval_dof(&m->ry, p1, p05, p025,
				 lim->max_rot_amp, lim->max_rot_bias);
	out->rz = vol * eval_dof(&m->rz, p1, p05, p025,
				 lim->max_rot_amp, lim->max_rot_bias);

	out->tx = vol * eval_dof(&m->tx, p1, p05, p025,
				 lim->max_trans_amp, lim->max_trans_bias);
	out->ty = vol * eval_dof(&m->ty, p1, p05, p025,
				 lim->max_trans_amp, lim->max_trans_bias);
	out->tz = vol * eval_dof(&m->tz, p1, p05, p025,
				 lim->max_trans_amp, lim->max_trans_bias);
}

void move_evaluate_mixed(const struct move_mixer *mix,
			 const struct move_playback *pb,
			 const struct move_limits *lim,
			 struct move_pose *out)
{
	struct move_pose a, b;

	/* Evaluate deck A */
	move_evaluate(&move_lib[mix->deck_a], pb, lim, &a);

	/* Evaluate deck B with phase offset */
	struct move_playback pb_b = *pb;
	pb_b.master_phase += TWO_PI * mix->phase_offset_b;
	move_evaluate(&move_lib[mix->deck_b], &pb_b, lim, &b);

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

void move_evaluate_single(int index, struct move_pose *out)
{
	if (index < 0 || index >= MOVE_LIB_SIZE) {
		memset(out, 0, sizeof(*out));
		return;
	}
	move_evaluate(&move_lib[index], &move_playback, &move_limits, out);
}

void move_evaluate_mixer(struct move_pose *out)
{
	move_evaluate_mixed(&move_mixer, &move_playback, &move_limits, out);
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

void move_mixer_set_phase_offset(struct move_mixer *mix, float offset)
{
	mix->phase_offset_b = fmodf(offset, 1.0f);
	if (mix->phase_offset_b < 0.0f)
		mix->phase_offset_b += 1.0f;
}

/*
 * Move manipulation
 */
void move_clear(struct move *m)
{
	memset(m, 0, sizeof(*m));
	/* Set bias to 0.5 (neutral) for all DOFs */
	for (int i = 0; i < MOVE_NUM_DOFS; i++)
		m->dof[i].bias = 0.5f;
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
			m->dof[d].h[h].phase =
				(float)rand() / (float)RAND_MAX;
		}
		m->dof[d].bias = 0.5f; /* Keep bias neutral */
	}
}

void move_interpolate(struct move *dst,
		      const struct move *a,
		      const struct move *b,
		      float t)
{
	float inv_t = 1.0f - t;

	for (int d = 0; d < MOVE_NUM_DOFS; d++) {
		for (int h = 0; h < MOVE_NUM_HARMONICS; h++) {
			dst->dof[d].h[h].amplitude =
				inv_t * a->dof[d].h[h].amplitude +
				t * b->dof[d].h[h].amplitude;
			dst->dof[d].h[h].phase =
				inv_t * a->dof[d].h[h].phase +
				t * b->dof[d].h[h].phase;
		}
		dst->dof[d].bias =
			inv_t * a->dof[d].bias + t * b->dof[d].bias;
	}
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

	for (int i = start; i < end; i++)
		move_randomize(&move_lib[i], intensity);
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
	move_lib[1].rx.h[0].amplitude = 0.6f;
	move_lib[1].rx.h[0].phase = 0.0f;
	move_lib[1].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 2: Side tilt (ry at 1 beat) */
	strncpy(move_lib[2].name, "tilt", MOVE_NAME_LEN - 1);
	move_lib[2].ry.h[0].amplitude = 0.5f;
	move_lib[2].ry.h[0].phase = 0.0f;
	move_lib[2].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 3: Twist (rz at 1/2 beat) */
	strncpy(move_lib[3].name, "twist", MOVE_NAME_LEN - 1);
	move_lib[3].rz.h[1].amplitude = 0.4f;
	move_lib[3].rz.h[1].phase = 0.0f;
	move_lib[3].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 4: Bounce (ty at 1 beat) */
	strncpy(move_lib[4].name, "bounce", MOVE_NAME_LEN - 1);
	move_lib[4].ty.h[0].amplitude = 0.7f;
	move_lib[4].ty.h[0].phase = 0.0f;
	move_lib[4].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 5: Sway (tx at 1 beat) */
	strncpy(move_lib[5].name, "sway", MOVE_NAME_LEN - 1);
	move_lib[5].tx.h[0].amplitude = 0.5f;
	move_lib[5].tx.h[0].phase = 0.0f;
	move_lib[5].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 6: Circle (tx + tz, 90° phase diff) */
	strncpy(move_lib[6].name, "circle", MOVE_NAME_LEN - 1);
	move_lib[6].tx.h[0].amplitude = 0.5f;
	move_lib[6].tx.h[0].phase = 0.0f;
	move_lib[6].tz.h[0].amplitude = 0.5f;
	move_lib[6].tz.h[0].phase = 0.25f;  /* 90° offset */
	move_lib[6].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 7: Complex (multi-harmonic) */
	strncpy(move_lib[7].name, "complex", MOVE_NAME_LEN - 1);
	move_lib[7].rx.h[0].amplitude = 0.4f;
	move_lib[7].rx.h[0].phase = 0.0f;
	move_lib[7].ry.h[1].amplitude = 0.3f;
	move_lib[7].ry.h[1].phase = 0.25f;
	move_lib[7].ty.h[0].amplitude = 0.5f;
	move_lib[7].ty.h[0].phase = 0.0f;
	move_lib[7].ty.h[2].amplitude = 0.2f;
	move_lib[7].ty.h[2].phase = 0.5f;
	move_lib[7].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 8: Wave (all rotations, staggered phase) */
	strncpy(move_lib[8].name, "wave", MOVE_NAME_LEN - 1);
	move_lib[8].rx.h[0].amplitude = 0.4f;
	move_lib[8].rx.h[0].phase = 0.0f;
	move_lib[8].ry.h[0].amplitude = 0.4f;
	move_lib[8].ry.h[0].phase = 0.33f;
	move_lib[8].rz.h[0].amplitude = 0.3f;
	move_lib[8].rz.h[0].phase = 0.66f;
	move_lib[8].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;

	/* Move 9: Pulse (ty with all harmonics) */
	strncpy(move_lib[9].name, "pulse", MOVE_NAME_LEN - 1);
	move_lib[9].ty.h[0].amplitude = 0.5f;
	move_lib[9].ty.h[0].phase = 0.0f;
	move_lib[9].ty.h[1].amplitude = 0.25f;
	move_lib[9].ty.h[1].phase = 0.0f;
	move_lib[9].ty.h[2].amplitude = 0.125f;
	move_lib[9].ty.h[2].phase = 0.0f;
	move_lib[9].flags = MOVE_FLAG_PRESET | MOVE_FLAG_LOOPABLE;
}
