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
