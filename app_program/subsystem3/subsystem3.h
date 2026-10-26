/*
 *	subsystem3.h
 *	Subsystem 3 -- Line Following + Barcode Decoder (Buddy 3, Lau Ning Xuan).
 *
 *	This header is subsystem 3's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem3/subsystem3.h".
 *
 *	Stage 0: Line Following runs every 20 ms; the Barcode Decoder waits
 *	with a 10 ms timeout (later it is also woken by each IR_BARCODE edge).
 *	Neither reads the IR sensors yet (GP0 left, GP1 centre, GP2 barcode;
 *	0 = white, 1 = black at the bring-up pot settings).
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

/* Private to subsystem 3, shared between its own .c files */
IMPORT ID	tid_line;
IMPORT ID	cyc_line;
IMPORT INT	tmg_line;
IMPORT ID	tid_barcode;
IMPORT INT	tmg_barcode;
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	barcode_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */
