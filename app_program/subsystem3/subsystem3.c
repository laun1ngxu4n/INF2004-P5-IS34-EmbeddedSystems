/*
 *  subsystem3.c
 *  Subsystem 3 -- Line Following + Barcode Decoder: creates its objects
 *  and starts its tasks.
 */

#include "subsystem3.h"

EXPORT ID  tid_line;
EXPORT ID  cyc_line;
EXPORT INT tmg_line;
EXPORT ID  tid_barcode;
EXPORT INT tmg_barcode;

EXPORT ER subsystem3_init (void)
{
    ER er;

    er = line_hw_init();
    if (er < E_OK)
    {
        return er;
    }
    er = barcode_hw_init();
    if (er < E_OK)
    {
        return er;
    }
    tmg_line    = app_timing_add("line", PRI_LINE, PERIOD_LINE_MS);
    tmg_barcode = app_timing_add("barcode", PRI_BARCODE, TIMEOUT_BARCODE_MS);
    cyc_line    = app_create_wake_cyclic(&tid_line, PERIOD_LINE_MS);
    if (tmg_line < E_OK)
    {
        return tmg_line;
    }
    if (tmg_barcode < E_OK)
    {
        return tmg_barcode;
    }
    return (cyc_line < E_OK) ? cyc_line : E_OK;
}

EXPORT ER subsystem3_start (void)
{
    tid_line = app_start_task(line_task, PRI_LINE, 0, "line_task");
    if (tid_line < E_OK)
    {
        return tid_line;
    }
    tid_barcode = app_start_task(barcode_task, PRI_BARCODE, 0, "barcode_task");
    if (tid_barcode < E_OK)
    {
        return tid_barcode;
    }
    return tk_sta_cyc(cyc_line);
}

/*** end of file ***/

