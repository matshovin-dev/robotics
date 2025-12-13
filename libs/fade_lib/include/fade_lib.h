#ifndef FADE_LIB_H
#define FADE_LIB_H

#include "stewart/pose.h"
#include "move_lib.h"

// Fade-funksjon signatur
typedef void (*fade_func_t)(struct move *from, struct move *to, float crossfader,
			    const struct stewart_geometry *geom,
			    struct move_playback *pb, struct stewart_pose *out);

// Tilgjengelige fade-funksjoner
void fade_linear(struct move *from, struct move *to, float crossfader,
		 const struct stewart_geometry *geom, struct move_playback *pb,
		 struct stewart_pose *out);

void fade_smoothstep(struct move *from, struct move *to, float crossfader,
		     const struct stewart_geometry *geom,
		     struct move_playback *pb, struct stewart_pose *out);

void fade_ease_in(struct move *from, struct move *to, float crossfader,
		  const struct stewart_geometry *geom, struct move_playback *pb,
		  struct stewart_pose *out);

void fade_ease_out(struct move *from, struct move *to, float crossfader,
		   const struct stewart_geometry *geom, struct move_playback *pb,
		   struct stewart_pose *out);

// Blander de 42 move-parameterne direkte (fase går korteste vei mod 2pi)
void fade_params(struct move *from, struct move *to, float crossfader,
		 const struct stewart_geometry *geom, struct move_playback *pb,
		 struct stewart_pose *out);

// Smoothstep med "dip to home" - fader via home-posisjon midt i overgangen
void fade_dip_home(struct move *from, struct move *to, float crossfader,
		   const struct stewart_geometry *geom, struct move_playback *pb,
		   struct stewart_pose *out);

// Smoothstep som går via en manuelt satt midtpose
extern struct stewart_pose fade_mid_pose;  // Sett denne i koden
extern float fade_mid_hold;                // Hold-tid i midten (0.0-1.0)
extern float fade_mid_ty;                  // Hevet ty for fade_hold_rot
void fade_via_pose(struct move *from, struct move *to, float crossfader,
		   const struct stewart_geometry *geom, struct move_playback *pb,
		   struct stewart_pose *out);

// Holder rotasjon fra forrige move i hevet posisjon
void fade_hold_rot(struct move *from, struct move *to, float crossfader,
		   const struct stewart_geometry *geom, struct move_playback *pb,
		   struct stewart_pose *out);

// Hjelpefunksjoner for kurve-transformasjoner
float curve_linear(float t);
float curve_smoothstep(float t);
float curve_ease_in(float t);
float curve_ease_out(float t);
float curve_ease_in_out(float t);

#endif // FADE_LIB_H
