/**
 * @file    subsystem3.h
 *  Subsystem 3 -- Line Following + Barcode Decoder (Buddy 3, Lau Ning Xuan).
 *
 *  This header is subsystem 3's PUBLIC interface: only what other
 *  subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *  files.  Other subsystems include it as "../subsystem3/subsystem3.h".
 *
 *  Stage 2:
 *    Line (20 ms): two sensors STRADDLE the tape, L (GP1) left of it and
 *    R (GP0) right of it, both white when centred (1 = black).  Steering
 *    goes to Motion through IF-03 (line_steer_mm_s, latest value only).
 *    Crossing = both black for about one tape width.
 *    Barcode: GP2 edges are stamped with the encoder odometer in the
 *    interrupt, so bar widths are distances, not times (TEAM.md 7.1 B3).
 *    The decoder reads the full Code 39 table in both directions (B1, B2)
 *    and keeps the 4-entry mapping that arrives over MQTT (B7, B8).
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

/* Line sensors, straddling the tape (7 Nov; before that GP1 rode on the
   tape and GP0 sat beside its right edge) */
#define PIN_IR_LINE_L  1 /* left of the tape */
#define PIN_IR_LINE_R  0 /* right of the tape */
#define PIN_IR_BARCODE 2
#define IR_BLACK       1 /* D0 level over black (bring-up) */

/* Barcode sensor input.  0: its digital D0 on GP2 (the module's pot sets
   the threshold).  1: its analogue AO on GP28 (ADC 2); the threshold and
   hysteresis are set in software, from the white and black levels it
   has seen.  For 1: power the module from 3V3, NOT 5 V (AO would go
   above the ADC's 3.3 V limit), and wire AO to GP28.  Analogue since
   7 Nov (B wired to GP28 then). */
#define S3_BARCODE_ANALOG 1
#define PIN_IR_BARCODE_AO 28
#define S3_BAR_ADC_CH     2 /* GP28 = ADC input 2 */
/* AO is higher over black (TCRT5000 modules); the live view shows it */
#define S3_BAR_BLACK_HIGH 1
/* Fixed thresholds (raw 0..4095): 0 = learn them from the white and black
   levels seen (the live view prints them to copy here).  From the 7 Nov
   live view: white 170-570, tape about 4040.  But printed bars read much
   greyer (S3-B00, 7 Nov: wide bars peaked at only 2400-3300, narrow ones
   did not reach 2100 at all), so 2500 / 1700 lost bars: lowered to sit
   just above the whitest white. */
#define S3_BAR_ADC_ON  1300 /* black from here (above, if BLACK_HIGH) */
#define S3_BAR_ADC_OFF 800  /* white again from here (white up to 570) */
/* Relative edges (used instead of ON/OFF when 1): black once the reading
   rises S3_BAR_DELTA above the lowest since the last bar (and is at least
   S3_BAR_BLACK_MIN), white again once it falls S3_BAR_DELTA below the
   highest since the bar began.  "A small" through B's blurred spot (7 Nov,
   S3-B00): narrow bars peaked at 1700-3100, yet the narrow space between
   two wide bars stayed above 1050, so no fixed pair of thresholds sees
   both.  Simulated (spot sigma 1.2-1.6 mm, ink 65-100 %): fixed 2500/1700
   decoded 24/54, 1300/800 6/54, relative 54/54. */
#define S3_BAR_RELATIVE 1
/* 15 Nov: 700 -> 300.  The course barcode in the live view: white 160-220,
   wide bars 2000-3600, but thin bars only 929-1535, and the gap between
   two thin bars does not fall back to white, so a 700 step (and the 1500
   floor below) missed thin bars: the 15 Nov course reads lost bars, broke
   into pieces or merged.  Noise is +-20, so 300 keeps a wide margin.
   BACK to 700 the same day: driving over the code with 700 / 300 gave
   only 7-element pieces of even 6-7 mm blobs (neighbouring thin bars and
   spaces run together), against 23-25 of 29 elements with 1500 / 700. */
#define S3_BAR_DELTA    700 /* white noise is +-20 */
/* never black below this (white up to 570). Raised from 500 / 900 on 7 Nov
 * at the team's request (fewer faint marks seen as bars); simulated "A
 * small" still 54/54, 800 / 1800 began to lose reads */
/* 15 Nov: 1500 -> 700: thin bars peaked at 929-1535 (most below 1500);
 * 700 is above every white seen (220 here, 570 on 7 Nov's worst surface).
 * A faint mark (grout) seen as a bar makes a short run the decoder drops,
 * or a stray mark beside a code, which it skips. */
/* BACK to 1500 the same day (see S3_BAR_DELTA). */
#define S3_BAR_BLACK_MIN 1500
/* learned: black-white gap needed before anything counts as black */
#define S3_BAR_MIN_SPAN 400

/* Geometry, measured 6 Nov */
/* line sensors in front of the wheel axle (was a guess of 60) */
#define S3_SENSOR_TO_AXLE_MM 115
#define S3_TAPE_MM           20 /* line tape width */
/* L to R, sensor head centre to centre, sideways.  Not used by the code:
   it is how the robot must be built.  A little wider than the tape, so
   both read white when centred, with a few mm to spare each side: about
   30 mm for 20 mm tape (7 Nov).  Much wider = it wanders before it
   corrects; narrower than the tape = both black all the time. */
#define S3_LR_GAP_MM 30
/* barcode sensor B in front of the line sensors; it is BEHIND (6 Nov), so
 * negative */
#define S3_BAR_AHEAD_MM (-40)
#define S3_BAR_LEFT_MM  70 /* B left of the left line sensor */

/* Junctions (details in line.c).
   Crossing (X, crossroads, T end): L and R both black for about one tape
   width (the tape is narrower than the L-R gap, so only a cross line
   does that).  A branch to the LEFT only: B (which also reads the
   barcodes) sees a black patch about one tape width long while the robot
   was driving straight there.  A branch to the RIGHT only cannot be told
   from a correction: the track has none (7 Nov).  Two within
   S3_JUNC_MERGE_MM = one junction. */
#define S3_JUNC_MIN_MM   (S3_TAPE_MM * 6 / 10) /* tape width -40 % .. +40 % */
#define S3_JUNC_MAX_MM   (S3_TAPE_MM * 14 / 10)
#define S3_JUNC_MERGE_MM 40
#define JUNC_SIDE_S      1 /* a crossing: both line sensors black */
#define JUNC_SIDE_B      2 /* a left arm, seen by B */

/* Private to subsystem 3, shared between its own .c files */
IMPORT ID      gh_tid_line;
IMPORT ID      gh_cyc_line;
IMPORT int32_t g_tmg_line;
IMPORT ID      gh_tid_barcode;
IMPORT int32_t g_tmg_barcode;

#include "line.h"
#include "barcode.h"

#endif /* SUBSYSTEM3_H */

/*** end of file ***/

