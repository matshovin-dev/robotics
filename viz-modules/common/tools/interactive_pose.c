/**
 * @file interactive_pose.c
 * @brief Interaktiv pose-setter for Stewart platform visualizere
 *
 * Lar deg sette rx, ry, rz, tx, ty, tz manuelt fra kommandolinjen
 * og sende til visualizere via UDP.
 *
 * Støtter også animasjonsmodus med sinusbevegelse på alle DOF.
 *
 * Kan brukes med både plot_stw_polygon og plot_stw_polygon_no_knee.
 */

#include "stewart/geometry.h"
#include "stewart/pose.h"
#include "viz_sender.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Animasjonsparametere */
static struct {
	int active;
	float t;		/* Tid (sekunder) */
	float speed;		/* Hastighetsfaktor */
	float amp_rx, amp_ry, amp_rz;	/* Rotasjonsamplituder (grader) */
	float amp_tx, amp_ty, amp_tz;	/* Translasjonsamplituder (mm) */
	float freq_rx, freq_ry, freq_rz;	/* Rotasjonsfrekvenser */
	float freq_tx, freq_ty, freq_tz;	/* Translasjonsfrekvenser */
} anim = {
	.active = 0,
	.t = 0.0f,
	.speed = 1.0f,
	.amp_rx = 8.0f, .amp_ry = 8.0f, .amp_rz = 5.0f,
	.amp_tx = 15.0f, .amp_ty = 20.0f, .amp_tz = 15.0f,
	.freq_rx = 0.7f, .freq_ry = 0.5f, .freq_rz = 0.3f,
	.freq_tx = 0.4f, .freq_ty = 0.6f, .freq_tz = 0.35f,
};

/**
 * has_input - Sjekk om det er input tilgjengelig på stdin
 * Returnerer 1 hvis input er tilgjengelig, 0 ellers.
 * Blokkerer ikke.
 */
static int has_input(void)
{
	struct timeval tv = { 0, 0 };
	fd_set fds;

	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

static void print_usage(void)
{
	printf("\nInteractive Stewart Platform Pose Sender\n");
	printf("=========================================\n\n");
	printf("Commands:\n");
	printf("  rx <deg>  - Set rotation X (degrees)\n");
	printf("  ry <deg>  - Set rotation Y (degrees)\n");
	printf("  rz <deg>  - Set rotation Z (degrees)\n");
	printf("  tx <mm>   - Set translation X (millimeters)\n");
	printf("  ty <mm>   - Set translation Y (millimeters)\n");
	printf("  tz <mm>   - Set translation Z (millimeters)\n");
	printf("  home      - Reset to home position\n");
	printf("  robot mx64|ax18 - Switch robot type\n");
	printf("  show      - Show current pose\n");
	printf("  send      - Send current pose to visualizer\n");
	printf("\nAnimation:\n");
	printf("  anim      - Toggle animation on/off\n");
	printf("  anim on   - Start animation\n");
	printf("  anim off  - Stop animation\n");
	printf("  speed <f> - Set animation speed (default 1.0)\n");
	printf("  amp <dof> <val> - Set amplitude (e.g. 'amp rx 10')\n");
	printf("  freq <dof> <val> - Set frequency (e.g. 'freq ty 0.5')\n");
	printf("  anim show - Show animation parameters\n");
	printf("\n");
	printf("  help      - Show this help\n");
	printf("  quit      - Exit program\n\n");
}

static void print_pose(const struct stewart_pose *pose,
		       enum stewart_robot_type robot_type,
		       const struct stewart_geometry *geom)
{
	printf("\nCurrent pose:\n");
	printf("  Rotation:    rx=%.2f° ry=%.2f° rz=%.2f°\n", pose->rx,
	       pose->ry, pose->rz);
	printf("  Translation: tx=%.2fmm ty=%.2fmm tz=%.2fmm\n", pose->tx,
	       pose->ty, pose->tz);
	printf("  Robot type:  %s (home_height=%.1fmm)\n",
	       robot_type == ROBOT_TYPE_MX64 ? "MX64" : "AX18",
	       geom->home_height);
	printf("  Animation:   %s\n", anim.active ? "ON" : "OFF");
	printf("\n");
}

static void print_anim_params(void)
{
	printf("\nAnimation parameters:\n");
	printf("  Speed: %.2f\n", anim.speed);
	printf("  Amplitudes:  rx=%.1f° ry=%.1f° rz=%.1f°\n",
	       anim.amp_rx, anim.amp_ry, anim.amp_rz);
	printf("               tx=%.1fmm ty=%.1fmm tz=%.1fmm\n",
	       anim.amp_tx, anim.amp_ty, anim.amp_tz);
	printf("  Frequencies: rx=%.2f ry=%.2f rz=%.2f\n",
	       anim.freq_rx, anim.freq_ry, anim.freq_rz);
	printf("               tx=%.2f ty=%.2f tz=%.2f\n",
	       anim.freq_tx, anim.freq_ty, anim.freq_tz);
	printf("\n");
}

/**
 * update_animation - Oppdater pose basert på sinusanimasjon
 */
static void update_animation(struct stewart_pose *pose,
			     const struct stewart_geometry *geom, float dt)
{
	anim.t += dt * anim.speed;

	pose->rx = anim.amp_rx * sinf(2.0f * M_PI * anim.freq_rx * anim.t);
	pose->ry = anim.amp_ry * sinf(2.0f * M_PI * anim.freq_ry * anim.t);
	pose->rz = anim.amp_rz * sinf(2.0f * M_PI * anim.freq_rz * anim.t);
	pose->tx = anim.amp_tx * sinf(2.0f * M_PI * anim.freq_tx * anim.t);
	pose->ty = geom->home_height +
		   anim.amp_ty * sinf(2.0f * M_PI * anim.freq_ty * anim.t);
	pose->tz = anim.amp_tz * sinf(2.0f * M_PI * anim.freq_tz * anim.t);
}

/**
 * parse_amp_freq - Parse 'amp <dof> <val>' eller 'freq <dof> <val>'
 */
static int parse_amp_freq(const char *line, int is_amp)
{
	char dof[8];
	float val;

	if (sscanf(line, "%*s %s %f", dof, &val) != 2) {
		printf("Usage: %s <dof> <value>\n", is_amp ? "amp" : "freq");
		printf("  dof: rx, ry, rz, tx, ty, tz\n");
		return 0;
	}

	float *target = NULL;
	if (strcmp(dof, "rx") == 0)
		target = is_amp ? &anim.amp_rx : &anim.freq_rx;
	else if (strcmp(dof, "ry") == 0)
		target = is_amp ? &anim.amp_ry : &anim.freq_ry;
	else if (strcmp(dof, "rz") == 0)
		target = is_amp ? &anim.amp_rz : &anim.freq_rz;
	else if (strcmp(dof, "tx") == 0)
		target = is_amp ? &anim.amp_tx : &anim.freq_tx;
	else if (strcmp(dof, "ty") == 0)
		target = is_amp ? &anim.amp_ty : &anim.freq_ty;
	else if (strcmp(dof, "tz") == 0)
		target = is_amp ? &anim.amp_tz : &anim.freq_tz;
	else {
		printf("Unknown DOF: %s\n", dof);
		return 0;
	}

	*target = val;
	printf("Set %s %s = %.2f\n", is_amp ? "amplitude" : "frequency",
	       dof, val);
	return 1;
}

int main(void)
{
	struct stewart_pose pose = { 0 };
	enum stewart_robot_type robot_type = ROBOT_TYPE_MX64;
	const struct stewart_geometry *geom = &ROBOT_MX64;
	int sock;
	char line[256];
	char cmd[64];
	float value;
	struct timeval last_time, now;
	int need_prompt = 1;

	printf("Interactive Stewart Platform Pose Sender\n");
	printf("=========================================\n\n");

	/* Initialiser pose til home */
	pose.tx = 0.0f;
	pose.ty = geom->home_height;
	pose.tz = 0.0f;
	pose.rx = 0.0f;
	pose.ry = 0.0f;
	pose.rz = 0.0f;

	/* Opprett UDP sender */
	sock = viz_sender_create();
	if (sock < 0) {
		fprintf(stderr, "Failed to create UDP sender\n");
		return 1;
	}

	printf("Ready! Type 'help' for commands.\n");
	print_pose(&pose, robot_type, geom);

	gettimeofday(&last_time, NULL);

	/* Interaktiv løkke */
	while (1) {
		/* Vis prompt hvis nødvendig */
		if (need_prompt && !anim.active) {
			printf("> ");
			fflush(stdout);
			need_prompt = 0;
		}

		/* Sjekk for input */
		if (has_input()) {
			if (!fgets(line, sizeof(line), stdin))
				break;

			need_prompt = 1;

			/* Parse kommando */
			int should_send = 0;
			if (sscanf(line, "%s", cmd) >= 1) {
				if (strcmp(cmd, "rx") == 0 &&
				    sscanf(line, "%*s %f", &value) == 1) {
					pose.rx = value;
					printf("Set rx = %.2f°\n", pose.rx);
					should_send = 1;
					if (anim.active) {
						anim.active = 0;
						printf("Animation stopped\n");
					}
				} else if (strcmp(cmd, "ry") == 0 &&
					   sscanf(line, "%*s %f", &value) == 1) {
					pose.ry = value;
					printf("Set ry = %.2f°\n", pose.ry);
					should_send = 1;
					if (anim.active) {
						anim.active = 0;
						printf("Animation stopped\n");
					}
				} else if (strcmp(cmd, "rz") == 0 &&
					   sscanf(line, "%*s %f", &value) == 1) {
					pose.rz = value;
					printf("Set rz = %.2f°\n", pose.rz);
					should_send = 1;
					if (anim.active) {
						anim.active = 0;
						printf("Animation stopped\n");
					}
				} else if (strcmp(cmd, "tx") == 0 &&
					   sscanf(line, "%*s %f", &value) == 1) {
					pose.tx = value;
					printf("Set tx = %.2fmm\n", pose.tx);
					should_send = 1;
					if (anim.active) {
						anim.active = 0;
						printf("Animation stopped\n");
					}
				} else if (strcmp(cmd, "ty") == 0 &&
					   sscanf(line, "%*s %f", &value) == 1) {
					pose.ty = value;
					printf("Set ty = %.2fmm\n", pose.ty);
					should_send = 1;
					if (anim.active) {
						anim.active = 0;
						printf("Animation stopped\n");
					}
				} else if (strcmp(cmd, "tz") == 0 &&
					   sscanf(line, "%*s %f", &value) == 1) {
					pose.tz = value;
					printf("Set tz = %.2fmm\n", pose.tz);
					should_send = 1;
					if (anim.active) {
						anim.active = 0;
						printf("Animation stopped\n");
					}
				} else if (strcmp(cmd, "home") == 0) {
					pose.rx = pose.ry = pose.rz = 0.0f;
					pose.tx = pose.tz = 0.0f;
					pose.ty = geom->home_height;
					printf("Reset to home position\n");
					should_send = 1;
					if (anim.active) {
						anim.active = 0;
						printf("Animation stopped\n");
					}
				} else if (strcmp(cmd, "robot") == 0) {
					char robot[16];
					if (sscanf(line, "%*s %s", robot) == 1) {
						if (strcmp(robot, "mx64") == 0) {
							robot_type = ROBOT_TYPE_MX64;
							geom = &ROBOT_MX64;
							printf("Switched to MX64\n");
							should_send = 1;
						} else if (strcmp(robot, "ax18") == 0) {
							robot_type = ROBOT_TYPE_AX18;
							geom = &ROBOT_AX18;
							printf("Switched to AX18\n");
							should_send = 1;
						} else {
							printf("Unknown robot type. Use 'mx64' or 'ax18'\n");
						}
					}
				} else if (strcmp(cmd, "anim") == 0) {
					char arg[16];
					if (sscanf(line, "%*s %s", arg) == 1) {
						if (strcmp(arg, "on") == 0) {
							anim.active = 1;
							anim.t = 0.0f;
							printf("Animation started\n");
						} else if (strcmp(arg, "off") == 0) {
							anim.active = 0;
							printf("Animation stopped\n");
						} else if (strcmp(arg, "show") == 0) {
							print_anim_params();
						} else {
							printf("Usage: anim [on|off|show]\n");
						}
					} else {
						/* Toggle */
						anim.active = !anim.active;
						anim.t = 0.0f;
						printf("Animation %s\n",
						       anim.active ? "started" : "stopped");
					}
				} else if (strcmp(cmd, "speed") == 0) {
					if (sscanf(line, "%*s %f", &value) == 1) {
						anim.speed = value;
						printf("Animation speed = %.2f\n", anim.speed);
					} else {
						printf("Usage: speed <factor>\n");
					}
				} else if (strcmp(cmd, "amp") == 0) {
					parse_amp_freq(line, 1);
				} else if (strcmp(cmd, "freq") == 0) {
					parse_amp_freq(line, 0);
				} else if (strcmp(cmd, "show") == 0) {
					print_pose(&pose, robot_type, geom);
				} else if (strcmp(cmd, "send") == 0) {
					int ret = viz_sender_send_pose(sock, &pose,
								       robot_type, VIZ_PORT);
					if (ret < 0)
						printf("Failed to send pose (error %d)\n", ret);
					else
						printf("Sent pose to visualizer\n");
				} else if (strcmp(cmd, "help") == 0) {
					print_usage();
				} else if (strcmp(cmd, "quit") == 0 ||
					   strcmp(cmd, "exit") == 0) {
					break;
				} else {
					printf("Unknown command. Type 'help' for usage.\n");
				}
			}

			/* Auto-send hvis pose ble endret manuelt */
			if (should_send && !anim.active) {
				viz_sender_send_pose(sock, &pose, robot_type, VIZ_PORT);
			}
		}

		/* Oppdater animasjon */
		if (anim.active) {
			gettimeofday(&now, NULL);
			float dt = (now.tv_sec - last_time.tv_sec) +
				   (now.tv_usec - last_time.tv_usec) / 1000000.0f;
			last_time = now;

			update_animation(&pose, geom, dt);
			viz_sender_send_pose(sock, &pose, robot_type, VIZ_PORT);
		}

		/* Sleep for å ikke bruke 100% CPU */
		usleep(16000); /* ~60 Hz */
	}

	close(sock);
	printf("\nGoodbye!\n");
	return 0;
}
