/**
 * @file viz_debug.h
 * @brief Debug/quiet mode for visualizers
 *
 * Compile with -DVIZ_QUIET to suppress all terminal output.
 * Default is verbose (debug) mode.
 */

#ifndef VIZ_DEBUG_H
#define VIZ_DEBUG_H

#include <stdio.h>

#ifdef VIZ_QUIET
#define viz_printf(...) ((void)0)
#else
#define viz_printf(...) printf(__VA_ARGS__)
#endif

#endif /* VIZ_DEBUG_H */
