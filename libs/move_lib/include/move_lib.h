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

#include "stewart/pose.h"  /* Required for move_spline struct */

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

/* Forward declaration for stewart geometry (full definition not needed) */
struct stewart_geometry;

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
 * move_evaluate_derivatives - Evaluate move with all derivatives
 * @m: Pointer to move definition
 * @pb: Pointer to playback state
 * @geom: Pointer to robot geometry (for scaling limits)
 * @pos: Output position (same as move_evaluate)
 * @vel: Output velocity (d/dt)
 * @acc: Output acceleration (d²/dt²)
 * @jerk: Output jerk (d³/dt³)
 *
 * Analytically computes derivatives for spline matching (C0-C3 continuity).
 * Units: pos in deg/mm, vel in deg/s or mm/s, acc in deg/s² or mm/s², etc.
 */
void move_evaluate_derivatives(const struct move *m,
			       const struct move_playback *pb,
			       const struct stewart_geometry *geom,
			       struct stewart_pose *pos,
			       struct stewart_pose *vel,
			       struct stewart_pose *acc,
			       struct stewart_pose *jerk);

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

/**
 * move_mirror - Speile utvalgte DOF-er (inverterer amplitude)
 * @m: Move å modifisere (in-place)
 * @dof_mask: Bitmask av DOF-er å speile (bruk DOF_RX, DOF_RY, etc.)
 *
 * Eksempel: move_mirror(m, (1<<DOF_RY) | (1<<DOF_RZ) | (1<<DOF_TX));
 */
void move_mirror(struct move *m, int dof_mask);

/**
 * move_phase_shift - Forskyv fase på utvalgte DOF-er
 * @m: Move å modifisere (in-place)
 * @dof_mask: Bitmask av DOF-er å endre
 * @shift: Faseforskyvning (0.0-1.0, wraps)
 */
void move_phase_shift(struct move *m, int dof_mask, float shift);

/**
 * move_scale_amplitude - Skaler amplitude på utvalgte DOF-er
 * @m: Move å modifisere (in-place)
 * @dof_mask: Bitmask av DOF-er å endre
 * @scale: Skaleringsfaktor (1.0 = uendret)
 */
void move_scale_amplitude(struct move *m, int dof_mask, float scale);

/**
 * move_swap_dofs - Bytt to DOF-er med hverandre
 * @m: Move å modifisere (in-place)
 * @dof_a: Første DOF
 * @dof_b: Andre DOF
 */
void move_swap_dofs(struct move *m, int dof_a, int dof_b);

/*
 * Spline transitions - smooth interpolation between moves
 *
 * Continuity levels:
 *   C0: Position matches at boundaries (linear blend, may have velocity jumps)
 *   C1: Position + velocity match (cubic Hermite, smooth but acceleration jumps)
 *   C2: Position + velocity + acceleration match (quintic, very smooth)
 */

/**
 * Spline transition state - holds precomputed coefficients
 */
struct move_spline {
	/* Start/end poses and derivatives (captured at transition start/end) */
	struct stewart_pose p0, p1;   /* Positions */
	struct stewart_pose v0, v1;   /* Velocities */
	struct stewart_pose a0, a1;   /* Accelerations */

	/* Transition timing */
	float t_start;    /* When transition starts (seconds) */
	float duration;   /* Transition duration (seconds) */

	/* Which continuity level (0, 1, 2, or 3 for c0_ease) */
	int continuity;

	/* Ease parameters (for continuity=3, c0_ease) */
	float ease_in;    /* 0.0-0.5: portion of transition with ease-in */
	float ease_out;   /* 0.0-0.5: portion of transition with ease-out */
};

/**
 * move_spline_init_c0 - Initialize C0 spline (position only)
 * @spline: Output spline state
 * @from: Source move
 * @to: Target move
 * @pb: Current playback state (captures t_start)
 * @geom: Robot geometry
 * @duration: Transition duration in seconds
 */
void move_spline_init_c0(struct move_spline *spline,
			 const struct move *from,
			 const struct move *to,
			 const struct move_playback *pb,
			 const struct stewart_geometry *geom,
			 float duration);

/**
 * move_spline_init_c1 - Initialize C1 spline (position + velocity)
 * @spline: Output spline state
 * @from: Source move
 * @to: Target move
 * @pb: Current playback state
 * @geom: Robot geometry
 * @duration: Transition duration in seconds
 */
void move_spline_init_c1(struct move_spline *spline,
			 const struct move *from,
			 const struct move *to,
			 const struct move_playback *pb,
			 const struct stewart_geometry *geom,
			 float duration);

/**
 * move_spline_init_c2 - Initialize C2 spline (position + velocity + acceleration)
 * @spline: Output spline state
 * @from: Source move
 * @to: Target move
 * @pb: Current playback state
 * @geom: Robot geometry
 * @duration: Transition duration in seconds
 */
void move_spline_init_c2(struct move_spline *spline,
			 const struct move *from,
			 const struct move *to,
			 const struct move_playback *pb,
			 const struct stewart_geometry *geom,
			 float duration);

/**
 * move_spline_init_c0_ease - Initialize C0 spline with optional ease-in/out
 * @spline: Output spline state
 * @from: Source move
 * @to: Target move
 * @pb: Current playback state
 * @geom: Robot geometry
 * @duration: Transition duration in seconds
 * @ease_in: Ease-in amount (0.0-0.5, e.g. 0.2 = 20% ease-in)
 * @ease_out: Ease-out amount (0.0-0.5, e.g. 0.2 = 20% ease-out)
 *
 * Kombinerer lineær C0 med myk ease ved start/slutt.
 * ease_in=0.2, ease_out=0.0 gir myk start, hard slutt.
 */
void move_spline_init_c0_ease(struct move_spline *spline,
			      const struct move *from,
			      const struct move *to,
			      const struct move_playback *pb,
			      const struct stewart_geometry *geom,
			      float duration,
			      float ease_in,
			      float ease_out);

/**
 * move_spline_evaluate - Evaluate spline at current time
 * @spline: Spline state (must be initialized)
 * @pb: Current playback state
 * @out: Output pose
 *
 * Returns 1 if still in transition, 0 if transition complete.
 * When complete, out contains the target pose.
 */
int move_spline_evaluate(const struct move_spline *spline,
			 const struct move_playback *pb,
			 struct stewart_pose *out);

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

/**
 * move_lib_save - Save move library to JSON file
 * @path: Path to JSON file
 *
 * Returns 0 on success, -1 on error.
 */
int move_lib_save(const char *path);

/**
 * move_lib_load - Load move library from JSON file
 * @path: Path to JSON file
 *
 * Returns number of moves loaded, or -1 on error.
 */
int move_lib_load(const char *path);

#endif /* MOVE_LIB_H */
