/**
 * @file move_lib.h
 * @brief Move library for BPM-synchronized Stewart platform motion
 *
 * Hierarchical structure for defining moves:
 *   Move → 6 DOFs (rx, ry, rz, tx, ty, tz)
 *   DOF → 3 harmonics + bias
 *   Harmonic → amplitude, phase, frequency multiplier
 *
 * Features:
 *   - DJ-style mixer with crossfader between two moves
 *   - Per-deck volume and phase offset
 *   - BPM-synchronized playback
 *   - Same code runs on desktop and Teensy
 */

#ifndef MOVE_LIB_H
#define MOVE_LIB_H

/*
 * Configuration
 */
#define MOVE_LIB_SIZE 100
#define MOVE_NAME_LEN 16
#define MOVE_NUM_DOFS 6
#define MOVE_NUM_HARMONICS 3
#define MOVE_PARAMS_PER_DOF 7  /* 3x(amp,phase) + bias */
#define MOVE_TOTAL_PARAMS (MOVE_NUM_DOFS * MOVE_PARAMS_PER_DOF)  /* 42 */

/*
 * Harmonic component - a single sine oscillator
 */
struct move_harmonic {
	float amplitude;  /* 0.0 - 1.0, scaled by max_amp */
	float phase;      /* 0.0 - 1.0, scaled to 0 - 2π */
};

/*
 * Degree of freedom motion - 3 harmonics at different beat divisions + bias
 *   h[0] = 1 beat (full BPM)
 *   h[1] = 1/2 beat (half BPM)
 *   h[2] = 1/4 beat (quarter BPM)
 */
struct move_dof {
	struct move_harmonic h[MOVE_NUM_HARMONICS];
	float bias;  /* -1.0 to +1.0, scaled by max_bias */
};

/*
 * Move flags for metadata
 */
#define MOVE_FLAG_SYMMETRIC   (1 << 0)  /* Symmetric motion pattern */
#define MOVE_FLAG_LOOPABLE    (1 << 1)  /* Good for looping */
#define MOVE_FLAG_TRANSITION  (1 << 2)  /* Intended as transition move */
#define MOVE_FLAG_PRESET      (1 << 3)  /* Factory preset, don't overwrite */

/*
 * DOF indices
 */
#define DOF_RX 0
#define DOF_RY 1
#define DOF_RZ 2
#define DOF_TX 3
#define DOF_TY 4
#define DOF_TZ 5

/*
 * Complete move definition
 */
struct move {
	char name[MOVE_NAME_LEN];
	struct move_dof dof[MOVE_NUM_DOFS];  /* rx, ry, rz, tx, ty, tz */
	int flags;
	int category;
};

/*
 * Mixer - DJ-deck style crossfade between two moves
 */
struct move_mixer {
	int deck_a;           /* Move index for deck A */
	int deck_b;           /* Move index for deck B */
	float crossfader;     /* 0.0 = only A, 1.0 = only B */
	float volume_a;       /* Volume for deck A (0.0 - 1.0) */
	float volume_b;       /* Volume for deck B (0.0 - 1.0) */
};

/*
 * Playback state - runtime values separate from move definitions
 */
struct move_playback {
	float t;              /* Accumulated time (seconds) */
	float bpm;            /* Beats per minute */
	float master_phase;   /* Global phase offset (radians) */
};

/*
 * Global state
 */
extern struct move move_lib[MOVE_LIB_SIZE];
extern struct move_mixer move_mixer;
extern struct move_playback move_playback;

/* Forward declarations for stewart types */
struct stewart_geometry;
struct stewart_pose;

/*
 * Core evaluation functions
 */

/**
 * move_evaluate - Evaluate a single move at current playback state
 * @m: Pointer to move definition
 * @pb: Pointer to playback state
 * @geom: Pointer to robot geometry (for scaling limits)
 * @out: Output pose
 */
void move_evaluate(const struct move *m,
		   const struct move_playback *pb,
		   const struct stewart_geometry *geom,
		   struct stewart_pose *out);

/**
 * move_evaluate_mixed - Evaluate mixer output (crossfade between two moves)
 * @mix: Pointer to mixer state
 * @pb: Pointer to playback state
 * @geom: Pointer to robot geometry (for scaling limits)
 * @out: Output pose
 */
void move_evaluate_mixed(const struct move_mixer *mix,
			 const struct move_playback *pb,
			 const struct stewart_geometry *geom,
			 struct stewart_pose *out);

/*
 * Phase functions
 */

/** Get phase for 1 beat cycle (full BPM) */
float move_phase_1(const struct move_playback *pb);

/** Get phase for 1/2 beat cycle */
float move_phase_05(const struct move_playback *pb);

/** Get phase for 1/4 beat cycle */
float move_phase_025(const struct move_playback *pb);

/*
 * Playback control
 */

/** Advance playback time by dt seconds */
void move_playback_tick(struct move_playback *pb, float dt);

/** Reset playback to t=0 */
void move_playback_reset(struct move_playback *pb);

/** Set BPM */
void move_playback_set_bpm(struct move_playback *pb, float bpm);

/*
 * Mixer control
 */

/** Set crossfader position (0.0 - 1.0) */
void move_mixer_set_crossfade(struct move_mixer *mix, float value);

/** Load move into deck A */
void move_mixer_set_deck_a(struct move_mixer *mix, int move_index);

/** Load move into deck B */
void move_mixer_set_deck_b(struct move_mixer *mix, int move_index);

/** Swap decks A and B */
void move_mixer_swap_decks(struct move_mixer *mix);

/*
 * Move manipulation
 */

/** Clear move to zero */
void move_clear(struct move *m);

/** Copy move */
void move_copy(struct move *dst, const struct move *src);

/** Randomize move parameters */
void move_randomize(struct move *m, float intensity);

/** Interpolate between two moves (t: 0.0 = a, 1.0 = b) */
void move_interpolate(struct move *dst,
		      const struct move *a,
		      const struct move *b,
		      float t);

/*
 * Serialization (for save/load)
 */

/** Export move to flat float array (returns number of floats written) */
int move_to_floats(const struct move *m, float *out, int max_floats);

/** Import move from flat float array (returns number of floats read) */
int move_from_floats(struct move *m, const float *in, int num_floats);

/*
 * Library management
 */

/** Initialize library with default presets */
void move_lib_init(void);

/** Clear all moves in library */
void move_lib_clear_all(void);

/** Clear specific move in library */
void move_lib_clear(int index);

/** Initialize a range of moves with random values */
void move_lib_randomize_range(int start, int end, float intensity);

#endif /* MOVE_LIB_H */
