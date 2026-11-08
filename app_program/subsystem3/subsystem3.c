/**
 * @file    subsystem3.c
 *  Subsystem 3 -- Line Following + Barcode Decoder: creates its objects
 *  and starts its tasks.
 */

#include "subsystem3.h"

EXPORT ID      gh_tid_line;
EXPORT ID      gh_cyc_line;
EXPORT int32_t g_tmg_line;
EXPORT ID      gh_tid_barcode;
EXPORT int32_t g_tmg_barcode;

/** Creates subsystem 3's objects (line and barcode tasks). */
EXPORT ER subsystem3_init (void)
{
    ER err;

    err = line_hw_init();

    if (err < E_OK)
    {
        return err;
    }

    err = barcode_hw_init();

    if (err < E_OK)
    {
        return err;
    }

    g_tmg_line    = app_timing_add("line", PRI_LINE, PERIOD_LINE_MS);
    g_tmg_barcode = app_timing_add("barcode", PRI_BARCODE, TIMEOUT_BARCODE_MS);
    gh_cyc_line   = app_main_create_wake_cyclic(&gh_tid_line, PERIOD_LINE_MS);

    if (g_tmg_line < E_OK)
    {
        return g_tmg_line;
    }

    if (g_tmg_barcode < E_OK)
    {
        return g_tmg_barcode;
    }

    return (gh_cyc_line < E_OK) ? gh_cyc_line : E_OK;
}

/** Starts subsystem 3's tasks (line following, barcode decoder). */
EXPORT ER subsystem3_start (void)
{
    gh_tid_line = app_main_start_task(line_task, PRI_LINE, 0, "line_task");

    if (gh_tid_line < E_OK)
    {
        return gh_tid_line;
    }

    gh_tid_barcode =
        app_main_start_task(barcode_task, PRI_BARCODE, 0, "barcode_task");

    if (gh_tid_barcode < E_OK)
    {
        return gh_tid_barcode;
    }

    return tk_sta_cyc(gh_cyc_line);
}

/*** end of file ***/

