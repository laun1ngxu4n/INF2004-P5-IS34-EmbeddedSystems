/**
 * @file    barcode.h
 *  Barcode Decoder (barcode.c): Code 39 results, the barcode sensor,
 *  the letter-to-move mapping.  Part of subsystem 3's interface
 *  (subsystem3.h includes it).
 */

#ifndef BARCODE_H
#define BARCODE_H

#include "../app.h"

/* Barcode results */
/* Each field in the bits its range needs (Change Requirement 1) */
typedef struct
{
    uint32_t seq APP_FIELD(8);         /* +1 per decode attempt; wraps: it
                                          only tells a new result */
    bool b_ok       APP_FIELD(1);      /* false = READ_FAIL */
    uint32_t letter APP_FIELD(7);      /* decoded character, 7-bit ASCII
                                          ('A'..'Z', '0'..'9', ...) */
    bool b_reversed      APP_FIELD(1); /* read backwards */
    uint32_t elements    APP_FIELD(7); /* bars + spaces seen, <= 99 */
    uint32_t narrow_mm10 APP_FIELD(9); /* mean narrow element, 0.1 mm,
                                          saturates at 51.1 mm */
    uint32_t wide_mm10 APP_FIELD(9);   /* mean wide element, the same */
    uint32_t fixed     APP_FIELD(3);   /* 0 clean read; BAR_FIX_STAR one '*'
                                          damaged; BAR_FIX_MERGED a lost
                                          narrow element put back;
                                          BAR_FIX_CUT a missing end bar
                                          put back; BAR_FIX_THIN bars read
                                          thin, widened; BAR_FIX_HALF half
                                          a code, never acted on */
} barcode_result_t;
#define BAR_FIX_STAR      1
#define BAR_FIX_MERGED    2
#define BAR_FIX_CUT       3 /* a weak narrow bar at an end not seen */
#define BAR_FIX_THIN      4 /* every bar read thin: widened (20 Nov) */
#define BAR_FIX_HALF      5 /* a letter and one '*' only: reported, not used */
#define BAR_FIX_HALF_USED 6 /* the same, mapped and clean: used (20 Nov) */
IMPORT void     barcode_last (barcode_result_t * r);
IMPORT bool     barcode_busy (void);  /* inside a barcode now (Motion slows) */
IMPORT uint32_t barcode_edges (void); /* barcode sensor edges since boot */
/** Elements of the run B is in now (0 = none): a lone patch is 1. */
IMPORT int32_t barcode_run_elements (void);
IMPORT bool    barcode_black (void); /* sensor sees black now (either input) */
/** Analogue input only (S3_BARCODE_ANALOG 1): last raw value and the learned
   white / black levels, 0..4095.  false in the digital build. */
IMPORT bool barcode_adc (int32_t * raw, int32_t * white, int32_t * black);
/** B is over bars now: hold the steering straight. */
IMPORT bool barcode_in_bars (void);
/** +1 for every result published. */
IMPORT uint32_t barcode_seq (void);
/** Drop a half-read code (start of a run); blocks 30 ms. */
IMPORT void barcode_reset (void);
/** Ignore the barcode sensor for the next mm of driving: no read, no
   slowing, no straight-steering hold (after a bypass, 20 Nov). */
IMPORT void barcode_ignore_mm (int32_t mm);
#define BARCODE_DBG_MAX 40 /* barcode_debug(): widths kept */
/** The last code that ended, any length; returns a seq. */
IMPORT uint32_t barcode_debug (int32_t * n, int32_t * w_mm10, int32_t max);
/** Last first-bar under B (published as soon as it ends): a left branch
   if about one tape wide; else a grout line or a barcode's start bar.
   Its centre (odometer counts) and width.  Returns a seq, +1 for each. */
IMPORT uint32_t barcode_mark (int32_t * centre_odo, int32_t * width_mm10);

/* Mapping (TEAM.md 7.1 B7/B8): exactly 4 pairs "K=L,Q=R,X=F,M=U".
   L left, R right, F forward, U U-turn.  Returns true if accepted.  On
   false the old mapping stays and why[] says why. */
#define MOVE_NONE    0
#define MOVE_LEFT    'L'
#define MOVE_RIGHT   'R'
#define MOVE_FORWARD 'F'
#define MOVE_UTURN   'U'
IMPORT bool    barcode_set_mapping (const uint8_t * text, int32_t len,
                                    uint8_t * why, int32_t why_max);
IMPORT int32_t barcode_move_for (uint8_t ch); /* MOVE_... or MOVE_NONE */
/** The letter mapped to each move, in the order L, R, F, U; 0 for every
   move while there is no mapping. */
IMPORT void    barcode_mapping_get (uint8_t * p_letters);
IMPORT int32_t barcode_mapping_text (uint8_t * out,
                                     int32_t   max); /* "K=L,Q=R,..." or "" */

IMPORT void barcode_task (int32_t stacd, void * exinf);
IMPORT ER   barcode_hw_init (void);

#endif /* BARCODE_H */

/*** end of file ***/
