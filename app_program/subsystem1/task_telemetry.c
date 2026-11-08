/*
 *  task_telemetry.c
 *  Subsystem 1 -- the Telemetry task (Notification merged in), the event
 *  outbox and the MQTT command inbox.
 *
 *  Every 100 ms: IF-01 ping once a second, the status line once a second,
 *  the live sensor line twice a second, the timing table every 10 s.
 *  Events from every subsystem go into a small outbox; the WiFi task
 *  pulls them with app_mqtt_tx() and hands
 *  commands to app_mqtt_rx().  Both hooks only copy under APP_LOCK and
 *  never wait, because they run in the kernel's WiFi task.
 *
 *  MQTT images also send a copy of the whole console on is34/log, so a
 *  test can run on the battery with no USB cable: the panel shows it and
 *  saves it (tools/panel/is34_panel.py, "Save log").  Each message is
 *  "L<seq> <dropped>|<text>": seq counts messages (a gap = lost on the
 *  WiFi), dropped counts bytes the robot could not keep (buffer full).
 */

#include "subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#define OUTBOX_N 8

typedef struct
{
    UB topic; /* 0 event, 1 telemetry, 2 ack/map, 3 sensors */
    UB text[TELE_TEXT_MAX];
} tele_msg_t;

LOCAL tele_msg_t   g_outbox[OUTBOX_N];
LOCAL UW           g_ob_w;
LOCAL UW           g_ob_r; /* under APP_LOCK */
LOCAL UW           g_out_dropped;
LOCAL const char * gp_state_text = "IDLE";
LOCAL volatile UW  g_start_count; /* MQTT START commands since boot */

EXPORT UW tele_start_count (void)
{
    return g_start_count;
}

#if defined(TM_MQTT) && TM_MQTT
/* Copy of the console for is34/log.  8 KB holds the start-up and Stage 0
   output until the WiFi is up (~4 KB), and a few seconds of the busiest
   test output; the link takes about 2.5 KB/s. */
#define LOG_RING 8192 /* a power of 2 */
LOCAL UB   g_log_ring[LOG_RING];
LOCAL UW   g_log_w;
LOCAL UW   g_log_r;       /* under APP_LOCK */
LOCAL UW   g_log_dropped; /* bytes lost: ring full */
LOCAL UW   g_log_seq;
LOCAL BOOL gb_log_turn; /* last message sent was a log chunk */

/* Called by the console driver for every byte it sends: any context */
EXPORT void tm_console_mirror (const UB * p_buf, INT size)
{
    UINT irq_state;
    INT  idx;

    APP_LOCK(irq_state);
    for (idx = 0; idx < size; idx++)
    {
        if ('\r' == p_buf[idx])
        {
            continue; /* the panel wants LF line ends only */
        }
        if ((g_log_w - g_log_r) >= LOG_RING)
        {
            g_log_dropped++;
        }
        else
        {
            g_log_ring[g_log_w & (LOG_RING - 1)] = p_buf[idx];
            g_log_w++;
        }
    }
    APP_UNLOCK(irq_state);
}

/* The next log chunk into buf; 0 = nothing waiting */
LOCAL int log_take (char * p_topic, unsigned tmax, unsigned char * p_buf,
                    unsigned bmax)
{
    UB       head[24];
    UINT     irq_state;
    unsigned idx;
    unsigned count;
    unsigned head_len;
    UW       avail;

    APP_LOCK(irq_state);
    avail = g_log_w - g_log_r;
    APP_UNLOCK(irq_state);
    if ((0u == avail) || (bmax < 64u))
    {
        return 0;
    }

    head_len = (unsigned)tm_sprintf(head, (const UB *)"L%d %d|",
                                    (INT)(g_log_seq + 1u), (INT)g_log_dropped);
    for (idx = 0; idx < head_len; idx++)
    {
        p_buf[idx] = head[idx];
    }
    count = bmax - 1u - head_len;
    if (count > avail)
    {
        count = (unsigned)avail;
    }
    APP_LOCK(irq_state);
    for (idx = 0; idx < count; idx++)
    {
        p_buf[head_len + idx] = g_log_ring[(g_log_r + idx) & (LOG_RING - 1)];
    }
    g_log_r += count;
    APP_UNLOCK(irq_state);
    g_log_seq++;

    for (idx = 0; ((idx + 1) < tmax) && (TOPIC_LOG[idx] != '\0'); idx++)
    {
        p_topic[idx] = TOPIC_LOG[idx];
    }
    p_topic[idx] = '\0';
    return (int)(head_len + count);
}
#endif

LOCAL void outbox_put (UB topic, const UB * p_text)
{
    UINT irq_state;
    INT  idx;

    APP_LOCK(irq_state);
    if ((g_ob_w - g_ob_r) >= OUTBOX_N)
    {
        g_ob_r++; /* full: drop the oldest */
        g_out_dropped++;
    }
    g_outbox[g_ob_w % OUTBOX_N].topic = topic;
    for (idx = 0; (idx < (TELE_TEXT_MAX - 1)) && (p_text[idx] != '\0'); idx++)
    {
        g_outbox[g_ob_w % OUTBOX_N].text[idx] = p_text[idx];
    }
    g_outbox[g_ob_w % OUTBOX_N].text[idx] = '\0';
    g_ob_w++;
    APP_UNLOCK(irq_state);
}

EXPORT void tele_event (const UB * p_text)
{
    tm_printf((UB *)"[EVT] %s\n", p_text);
    outbox_put(0, p_text);
}

EXPORT void tele_set_state (const char * p_state)
{
    gp_state_text = p_state;
}

/* ---- MQTT hooks, called from the WiFi task (mqtt_utk.c) ------------- */

/* TRUE if s starts with prefix; *rest = what follows */
LOCAL BOOL starts_with (const char * p_text, const char * p_prefix,
                        const char ** pp_rest)
{
    while (*p_prefix != '\0')
    {
        if (*p_text++ != *p_prefix++)
        {
            return FALSE;
        }
    }
    *pp_rest = p_text;
    return TRUE;
}

LOCAL BOOL same (const char * p_first, const char * p_second)
{
    while ((*p_first != '\0') && (*p_first == *p_second))
    {
        p_first++;
        p_second++;
    }
    return *p_first == *p_second;
}

EXPORT int app_mqtt_tx (char * p_topic, unsigned tmax, unsigned char * p_buf,
                        unsigned bmax)
{
    static const char * names[4] = {TOPIC_EVENT, TOPIC_TELEMETRY, TOPIC_ACK_MAP,
                                    TOPIC_SENSORS};
    tele_msg_t          msg;
    UINT                irq_state;
    unsigned            idx;
    BOOL                b_have;

    APP_LOCK(irq_state);
    b_have = (g_ob_r != g_ob_w);
    APP_UNLOCK(irq_state);
#if defined(TM_MQTT) && TM_MQTT
    /* Outbox and log take turns, so a long log never holds up an event */
    if ((!b_have) || (!gb_log_turn))
    {
        int len = log_take(p_topic, tmax, p_buf, bmax);
        if (len > 0)
        {
            gb_log_turn = TRUE;
            return len;
        }
    }
    gb_log_turn = FALSE;
#endif
    APP_LOCK(irq_state);
    if (g_ob_r == g_ob_w)
    {
        APP_UNLOCK(irq_state);
        return 0;
    }
    msg = g_outbox[g_ob_r % OUTBOX_N];
    g_ob_r++;
    APP_UNLOCK(irq_state);

    for (idx = 0; ((idx + 1) < tmax) && (names[msg.topic][idx] != '\0'); idx++)
    {
        p_topic[idx] = names[msg.topic][idx];
    }
    p_topic[idx] = '\0';
    for (idx = 0; ((idx + 1) < bmax) && (msg.text[idx] != '\0'); idx++)
    {
        p_buf[idx] = msg.text[idx];
    }
    return (int)idx;
}

EXPORT void app_mqtt_rx (const char * p_topic, const unsigned char * p_data,
                         unsigned len)
{
    const char * p_cmd;
    UB           why[64];
    UB           msg[TELE_TEXT_MAX];

    if (!starts_with(p_topic, TOPIC_CMD_PREFIX, &p_cmd))
    {
        return;
    }
    if (same(p_cmd, "map"))
    {
        if (barcode_set_mapping(p_data, (INT)len, why, sizeof(why)))
        {
            UB map[24];
            barcode_mapping_text(map, sizeof(map));
            tm_sprintf(msg, (const UB *)"OK %s", map);
        }
        else
        {
            tm_sprintf(msg, (const UB *)"REJECTED %s (old mapping kept)", why);
        }
        tm_printf((UB *)"[MQTT] map: %s\n", msg);
        outbox_put(2, msg);
    }
    else if (same(p_cmd, "start"))
    {
        tm_printf((UB *)"[MQTT] start\n");
        g_start_count++;
        tk_set_flg(gh_flg_mission, MSN_FLG_START);
    }
    else if (same(p_cmd, "stop"))
    {
        app_stop_request(STOP_MQTT);
        tk_set_flg(gh_flg_mission, MSN_FLG_STOP);
        tm_printf((UB *)"[MQTT] stop\n");
    }
}

/* ---- the task -------------------------------------------------------- */

LOCAL void status_line (void)
{
    motion_status_t  status;
    barcode_result_t last_code;
    UB               text[TELE_TEXT_MAX];
    UB               map[24];

    motion_get_status(&status);
    barcode_last(&last_code);
    barcode_mapping_text(map, sizeof(map));
    tm_sprintf(text,
               (const UB *)"state=%s speed=%d,%d dist=%d line=%d junctions=%d "
                           "barcode=%c map=%s ramp_peak=%d.%d stop=%d",
               gp_state_text, status.speed_l, status.speed_r,
               (INT)S2_COUNTS_TO_MM(motion_odometer()), line_state(),
               (INT)line_junctions(), (last_code.b_ok ? last_code.letter : '-'),
               (map[0] != '\0') ? (char *)map : "none",
               imu_ramp_peak_mm10() / 10, imu_ramp_peak_mm10() % 10,
               (INT)app_stop_reasons());
    outbox_put(1, text);
}

/* Live sensor readings for the laptop panel (not graded, just to see what
   the robot sees): ir = side, centre, barcode (1 = black), steer mm/s,
   hold = steering held straight (B over bars), inbar = inside a barcode, front
   = last forward sonar mm (-1 open, -2 no ECHO), tilt = sin(pitch) x1000, h10 =
   climbed height in 0.1 mm, imu = IMU answering */
LOCAL void sensor_line (void)
{
    UB text[TELE_TEXT_MAX];

    tm_sprintf(text,
               (const UB *)"ir=%d%d%d steer=%d hold=%d inbar=%d front=%d "
                           "tilt=%d h10=%d imu=%d",
               (INT)(IR_BLACK == app_gpio_get(PIN_IR_LINE_L)),
               (INT)(IR_BLACK == app_gpio_get(PIN_IR_LINE_R)),
               (INT)barcode_black(), line_steer_mm_s(), (INT)barcode_in_bars(),
               (INT)barcode_busy(), obstacle_front_mm(), imu_pitch_mdeg_sin(),
               imu_height_mm10(), (INT)imu_ok());
    outbox_put(3, text);
}

EXPORT void telemetry_task (INT stacd, void * p_exinf)
{
    UINT report_tick = 0;
    UINT ping_tick   = 0;
    UINT status_tick = 0;
    UINT sensor_tick = 0;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR); /* woken every 100 ms */
        app_timing_wake(g_tmg_telemetry, tk_can_wup(TSK_SELF));

        if (++ping_tick >= S1_PING_TICKS)
        {
            ping_tick = 0;
            CHECK(tk_set_flg(gh_flg_mission, MSN_FLG_PING), "IF-01 ping");
        }
        if (++status_tick >= S1_STATUS_TICKS)
        {
            status_tick = 0;
            status_line();
        }
        if (++sensor_tick >= S1_SENSOR_TICKS)
        {
            sensor_tick = 0;
            sensor_line();
        }
        if (++report_tick >= S1_REPORT_TICKS)
        {
            report_tick = 0;
            /* The report's printing counts in this task's run time on
               purpose: it is real Telemetry load at the lowest priority. */
            app_timing_report();
        }

        app_timing_done(g_tmg_telemetry);
    }
}

/*** end of file ***/

