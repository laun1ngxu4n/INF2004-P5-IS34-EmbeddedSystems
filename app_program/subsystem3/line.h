/**
 * @file    line.h
 *  Line Following (line.c): steering for Motion, the line state,
 *  junctions.  Part of subsystem 3's interface (subsystem3.h
 *  includes it).
 */

#ifndef LINE_H
#define LINE_H

#include "../app.h"

/** IF-03: steering for Motion, mm/s added to the right wheel and taken
   from the left (+ = turn left).  Latest value only. */
IMPORT int32_t line_steer_mm_s (void);

/* Line state, for Mission and telemetry */
#define LINE_ON     0 /* following */
#define LINE_SEARCH 1 /* overshot a tight curve: turning back */
#define LINE_LOST   2 /* no line for too long */
/* a line sensor looks stuck (line_fault() says which); Mission stops as for
 * LINE_LOST */
#define LINE_FAULT 3
IMPORT int32_t line_state (void);
#define LINE_FAULT_BOTH 1              /* both black for S3_STUCK_BOTH_MM */
#define LINE_FAULT_ONE  2              /* one black for S3_STUCK_ONE_MM */
IMPORT int32_t  line_fault (void);     /* LINE_FAULT_... or 0 */
IMPORT uint32_t line_junctions (void); /* junctions seen since boot */
IMPORT void line_wobble (uint32_t * corrections, uint32_t * sweeps); /* since
                            the start: corrections begun, and sweeps right
                            across the tape (zigzag) */
IMPORT int32_t line_junction_odo (void);   /* last junction's centre, where the
                        line sensors crossed it (odometer
                        counts) */
IMPORT int32_t line_junction_sides (void); /* its branches: JUNC_SIDE_S | _B */
IMPORT void    line_set_enabled (bool on); /* false: no junction / lost events
                              (during turns and bypasses) */
IMPORT bool    line_any_black (void); /* raw: either line sensor on black */
IMPORT int32_t line_speed_pct (void); /* % of the follow speed Motion may
                     use: 100 on track, less while
                     correcting or searching */

IMPORT void line_task (int32_t stacd, void * exinf);
IMPORT ER   line_hw_init (void);

#endif /* LINE_H */

/*** end of file ***/

