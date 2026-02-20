#ifndef MTI_VOID_H
#define MTI_VOID_H

#include <stdint.h>
#include <stdbool.h>
#include "mti_radar.h"

/* Configuration defaults */
#define MTI_VOID_DEFAULT_HYSTERESIS_PCT 5   /* Exit hysteresis percent */
#define MTI_VOID_DEFAULT_THRESHOLD_MM   500 /* Void threshold radius from hole center in mm */
#define MTI_VOID_DEFAULT_HOLE_DIA_MM    300 /* Nominal borehole diameter in mm */
#define MTI_VOID_PROBE_DIA_MM           60  /* Probe body diameter in mm (hard-coded) */

/**
 * Initialize the Void application logic. Call once after device init.
 */
void mti_void_init(void);

/**
 * Periodic processing. Call after mti_radar_process() in the main loop.
 */
void mti_void_process(void);

/**
 * Set the void threshold in millimeters (radius from hole center).
 * When worst-case calculated radius exceeds this threshold, void is detected.
 * Uses fail-safe formula: calc_radius = hole_radius + measured_distance
 */
void mti_void_set_threshold_mm(uint32_t threshold_mm);

/**
 * Set exit hysteresis percent (0–100).
 */
void mti_void_set_hysteresis_pct(uint8_t pct);

/**
 * Set debounce frame counts (enter, exit).
 */
void mti_void_set_debounce_frames(uint8_t enter_frames, uint8_t exit_frames);

/**
 * Get current global Void state.
 */
bool mti_void_is_detected(void);

/* Optional getters (useful for @vd,? responses) */
uint32_t mti_void_get_threshold_mm(void);
uint8_t  mti_void_get_hysteresis_pct(void);
uint8_t  mti_void_get_debounce_enter_frames(void);
uint8_t  mti_void_get_debounce_exit_frames(void);

/**
 * Get void state for specific satellite (1-6).
 * Returns false if satellite ID invalid.
 */
bool mti_void_get_satellite_state(uint8_t sat_id);

/**
 * Set/get the nominal borehole diameter in mm.
 * Used to calculate worst-case void radius (fail-safe approach).
 */
void     mti_void_set_hole_diameter_mm(uint32_t diameter_mm);
uint32_t mti_void_get_hole_diameter_mm(void);

#endif /* MTI_VOID_H */
