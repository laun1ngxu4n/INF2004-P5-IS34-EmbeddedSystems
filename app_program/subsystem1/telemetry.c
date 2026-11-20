/**
 * @file    telemetry.c
 *  Subsystem 1 -- the Telemetry task (Notification merged in), the event
 *  outbox and the MQTT command inbox.
 *
 *  Every 100 ms: IF-01 ping once a second, the status line once a second,
 *  the live sensor line twice a second, the timing table every 10 s (1 min
 *  in the Mission and normal images), and one short beep for each event.
 *  Events from every subsystem go into a small outbox; the WiFi task
 *  pulls them with app_mqtt_tx() and hands
 *  commands to app_mqtt_rx().  Both hooks only copy under APP_LOCK and
 *  never wait, because they run in the kernel's WiFi task.
 *
 *  Change Requirement 1: the status line, the sensor readings and the
 *  mapping acknowledgement are binary (wire.c, docs/team/MQTT_WIRE_FORMAT.md);
 *  the mapping command arrives binary.  Events stay text: they are
 *  sentences for the operator, not values.  The MQTT link's state is
 *  FAULT_WIFI_LOSS in the fault word (a warning, never a stop).
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
#include "telemetry.h"
#include "wire.h"

#define OUTBOX_N 8

#define TOPIC_IDX_EVENT     0u
#define TOPIC_IDX_TELEMETRY 1u
#define TOPIC_IDX_ACK       2u
#define TOPIC_IDX_SENSORS   3u
#define LINK_LOST_US        1000000u /* the glue asks every 100 ms */

/* Buzzer (20 Nov, team): one short beep for every event, on the Robo
   Pico's piezo (GP22, PWM slice 3 channel A; the board's mute switch
   silences it).  telemetry_event() asks for it from any task; this task
   plays it for one 100 ms period, so nothing waits for it.  Events in
   the same 100 ms give one beep. */
#define PIN_BUZZER   22
#define BUZZER_SLICE 3
#define BUZZER_WRAP  499u /* 1 MHz / 500 = 2 kHz */

LOCAL volatile bool gb_beep_req; /* any task sets it, under APP_LOCK */

/* One outgoing message: binary (wire.c) or event text, and its length */
typedef struct
{
    uint8_t topic APP_FIELD(2); /* TOPIC_IDX_... */
    uint8_t       len;          /* 0..TELE_TEXT_MAX - 1 bytes */
    uint8_t       data[TELE_TEXT_MAX];
} tele_msg_t;

LOCAL tele_msg_t        g_outbox[OUTBOX_N];
LOCAL uint32_t          g_ob_w;
LOCAL uint32_t          g_ob_r; /* under APP_LOCK */
LOCAL uint32_t          g_out_dropped;
LOCAL volatile uint32_t g_start_count; /* MQTT START commands since boot */
LOCAL volatile uint32_t g_link_us;     /* when the glue last asked for a
                                          message: a time, all 32 bits */

EXPORT uint32_t telemetry_start_count (void)
{
    return g_start_count;
}

#if defined(TM_MQTT) && TM_MQTT
/* Copy of the console for is34/log.  8 KB holds the start-up and Stage 0
   output until the WiFi is up (~4 KB), and a few seconds of the busiest
   test output; the link takes about 2.5 KB/s. */
#define LOG_RING 8192 /* a power of 2 */
LOCAL uint8_t  g_log_ring[LOG_RING];
LOCAL uint32_t g_log_w;
LOCAL uint32_t g_log_r;       /* under APP_LOCK */
LOCAL uint32_t g_log_dropped; /* bytes lost: ring full */
LOCAL uint32_t g_log_seq;
LOCAL bool     gb_log_turn; /* last message sent was a log chunk */

/** Called by the console driver for every byte it sends: any context.
   NOTE: INT size, as the caller declares it (lib/libtm/.../tm_com.c;
   Barr C deviation D2). */
EXPORT void tm_console_mirror (const uint8_t * p_buf, INT size)
{
    uint32_t irq_state;
    int32_t  idx;

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

/** The next log chunk into buf; 0 = nothing waiting */
LOCAL int32_t log_take (char * p_topic, uint32_t tmax, uint8_t * p_buf,
                        uint32_t bmax)
{
    uint8_t  head[24];
    uint32_t irq_state;
    uint32_t idx;
    uint32_t count;
    uint32_t head_len;
    uint32_t avail;

    APP_LOCK(irq_state);
    avail = g_log_w - g_log_r;
    APP_UNLOCK(irq_state);

    if ((0u == avail) || (bmax < 64u))
    {
        return 0;
    }

    /* tm_sprintf returns the length written, 0..23 (head[24]); the counters
       show as negative only after 2^31 messages */
    head_len =
        (uint32_t)tm_sprintf(head, app_str("L%d %d|"),
                             (int32_t)(g_log_seq + 1u), (int32_t)g_log_dropped);

    for (idx = 0; idx < head_len; idx++)
    {
        p_buf[idx] = head[idx];
    }

    count = bmax - 1u - head_len;

    if (count > avail)
    {
        count = avail;
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
    /* at most bmax - 1, the caller's buffer size: fits int32_t */
    return (int32_t)(head_len + count);
}
#endif

/** Queues a message for MQTT (len bytes, at most TELE_TEXT_MAX - 1); drops
   the oldest if the queue is full. */
LOCAL void outbox_put (uint32_t topic, const uint8_t * p_data, int32_t len)
{
    uint32_t     irq_state;
    int32_t      idx;
    tele_msg_t * p_msg;

    APP_LOCK(irq_state);

    if ((g_ob_w - g_ob_r) >= OUTBOX_N)
    {
        g_ob_r++; /* full: drop the oldest */
        g_out_dropped++;
    }

    p_msg        = &g_outbox[g_ob_w % OUTBOX_N];
    p_msg->topic = topic;

    for (idx = 0; (idx < len) && (idx < (TELE_TEXT_MAX - 1)); idx++)
    {
        p_msg->data[idx] = p_data[idx];
    }

    /* idx <= TELE_TEXT_MAX - 1 = 159: fits uint8_t */
    p_msg->len = (uint8_t)idx;
    g_ob_w++;
    APP_UNLOCK(irq_state);
}

/** Queues a text (an event) for MQTT. */
LOCAL void outbox_text (uint32_t topic, const uint8_t * p_text)
{
    int32_t len = 0;

    while ((len < (TELE_TEXT_MAX - 1)) && (p_text[len] != '\0'))
    {
        len++;
    }

    outbox_put(topic, p_text, len);
}

EXPORT void telemetry_event (const uint8_t * p_text)
{
    uint32_t irq_state;

    tm_printf(app_str("[EVT] %s\n"), p_text);
    outbox_text(TOPIC_IDX_EVENT, p_text);
    APP_LOCK(irq_state);
    gb_beep_req = true;
    APP_UNLOCK(irq_state);
}

/** The buzzer's PWM: 2 kHz, silent until a beep. */
LOCAL void buzzer_init (void)
{
    app_gpio_hw_release(RESETS_RESET_PWM);
    /* 125 MHz / 125 = 1 MHz (slice 3's B pin, GP7, is the sonar's ECHO
       as a GPIO: the slice's divider does not touch it) */
    out_w(PWM_BASE + (BUZZER_SLICE * 0x14) + PWM_CHx_DIV, 125u << 4);
    pwm_set_wrap(PIN_BUZZER, BUZZER_WRAP);
    pwm_set_cc(PIN_BUZZER, 0u);
    pwm_set_pin(PIN_BUZZER);
    pwm_set_enabled(PIN_BUZZER, true);
}

/** One 100 ms period of the buzzer: off after a beep, else on for a new
   request.  Returns whether it is on now. */
LOCAL bool buzzer_tick (bool b_on)
{
    uint32_t irq_state;
    bool     b_req;

    if (b_on)
    {
        pwm_set_cc(PIN_BUZZER, 0u);
        return false;
    }

    APP_LOCK(irq_state);
    b_req       = gb_beep_req;
    gb_beep_req = false;
    APP_UNLOCK(irq_state);

    if (b_req)
    {
        pwm_set_cc(PIN_BUZZER, (BUZZER_WRAP + 1u) / 2u); /* half duty */
    }

    return b_req;
}

/* ---- MQTT hooks, called from the WiFi task (mqtt_utk.c) ------------- */

/** true if s starts with prefix; *rest = what follows */
LOCAL bool starts_with (const char * p_text, const char * p_prefix,
                        const char ** pp_rest)
{
    while (*p_prefix != '\0')
    {
        if (*p_text != *p_prefix)
        {
            return false;
        }

        p_text++;
        p_prefix++;
    }

    *pp_rest = p_text;
    return true;
}

/** true if two strings are equal. */
LOCAL bool same (const char * p_first, const char * p_second)
{
    while ((*p_first != '\0') && (*p_first == *p_second))
    {
        p_first++;
        p_second++;
    }

    return *p_first == *p_second;
}

/** Called by the MQTT glue to fetch the next message to send; its length,
 * or 0. */
EXPORT int app_mqtt_tx (char * p_topic, unsigned tmax, unsigned char * p_buf,
                        unsigned bmax)
{
    static const char * names[4] = {TOPIC_EVENT, TOPIC_TELEMETRY, TOPIC_ACK_MAP,
                                    TOPIC_SENSORS};
    tele_msg_t          msg;
    uint32_t            irq_state;
    uint32_t            idx;
#if defined(TM_MQTT) && TM_MQTT
    bool b_have;
#endif

    g_link_us = app_timing_now_us(); /* asked only while MQTT is connected */
#if defined(TM_MQTT) && TM_MQTT
    APP_LOCK(irq_state);
    b_have = (g_ob_r != g_ob_w);
    APP_UNLOCK(irq_state);

    /* Outbox and log take turns, so a long log never holds up an event */
    if ((!b_have) || (!gb_log_turn))
    {
        int32_t len = log_take(p_topic, tmax, p_buf, bmax);

        if (len > 0)
        {
            gb_log_turn = true;
            return len;
        }
    }

    gb_log_turn = false;
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

    for (idx = 0; (idx < bmax) && (idx < msg.len); idx++)
    {
        p_buf[idx] = msg.data[idx];
    }

    /* idx < bmax, the glue's buffer size: fits int */
    return (int)idx;
}

/** A mapping command: binary (WIRE_MAP_LEN bytes, the panel) or the text
   "K=L,Q=R,X=F,M=U" (typed by hand).  Answered with the binary
   acknowledgement on is34/ack/map; why one was refused goes out as an
   event (a sentence). */
LOCAL void on_map_command (const uint8_t * p_data, int32_t len)
{
    uint8_t letters[4];
    uint8_t text[24];
    uint8_t why[64];
    uint8_t msg[TELE_TEXT_MAX];
    uint8_t ack[WIRE_ACK_LEN];
    bool    b_ok;

    if ((len > 0) && (WIRE_VERSION == p_data[0]))
    {
        b_ok = wire_map_unpack(p_data, len, letters);

        if (b_ok)
        {
            /* letters for L, R, F, U into the text barcode_set_mapping()
               checks (each letter is A-Z / 0-9) */
            (void)tm_sprintf(text, app_str("%c=L,%c=R,%c=F,%c=U"), letters[0],
                             letters[1], letters[2], letters[3]);
            b_ok = barcode_set_mapping(text, 15, why, sizeof(why));
        }
        else
        {
            (void)tm_sprintf(why,
                             app_str("not a version %d mapping of %d "
                                     "bytes with 4 letters"),
                             WIRE_VERSION, WIRE_MAP_LEN);
        }
    }
    else
    {
        b_ok = barcode_set_mapping(p_data, len, why, sizeof(why));
    }

    barcode_mapping_get(letters);
    outbox_put(TOPIC_IDX_ACK, ack, wire_ack_pack(b_ok, letters, ack));

    if (b_ok)
    {
        (void)barcode_mapping_text(text, sizeof(text));
        tm_printf(app_str("[MQTT] map: OK %s\n"), text);
    }
    else
    {
        (void)tm_sprintf(
            msg, app_str("mapping REJECTED: %s (old mapping kept)"), why);
        telemetry_event(msg);
    }
}

/** Called by the MQTT glue for each message received: START, STOP, mapping. */
EXPORT void app_mqtt_rx (const char * p_topic, const unsigned char * p_data,
                         unsigned len)
{
    const char * p_cmd;

    if (!starts_with(p_topic, TOPIC_CMD_PREFIX, &p_cmd))
    {
        return;
    }

    if (same(p_cmd, "map"))
    {
        on_map_command(p_data, (int32_t)len); /* len < 128 (the glue's
                                                 buffer): fits int32_t */
    }
    else if (same(p_cmd, "start"))
    {
        tm_printf(app_str("[MQTT] start\n"));
        g_start_count++;
        tk_set_flg(gh_flg_mission, MSN_FLG_START);
    }
    else if (same(p_cmd, "stop"))
    {
        app_stop_request(STOP_MQTT);
        tk_set_flg(gh_flg_mission, MSN_FLG_STOP);
        tm_printf(app_str("[MQTT] stop\n"));
    }
    else
    {
        /* nothing to do for any other case */
    }
}

/* ---- the task -------------------------------------------------------- */

/** Builds the 1 s status message for the panel (binary, wire.c). */
LOCAL void status_line (void)
{
    motion_status_t  status;
    barcode_result_t last_code;
    wire_status_t    values;
    uint8_t          msg[WIRE_STATUS_LEN];

    motion_get_status(&status);
    barcode_last(&last_code);
    barcode_mapping_get(values.map_letter);
    values.mission_state  = mission_state();
    values.line_state     = line_state(); /* 0..3: not negative */
    values.stop_reasons   = app_stop_reasons();
    values.faults         = app_stop_faults();
    values.speed_l        = status.speed_l;
    values.speed_r        = status.speed_r;
    values.dist_mm        = motion_counts_to_mm(motor_odometer());
    values.junctions      = line_junctions();
    values.last_letter    = last_code.b_ok ? last_code.letter : 0u;
    values.ramp_peak_mm10 = imu_ramp_peak_mm10();
    outbox_put(TOPIC_IDX_TELEMETRY, msg, wire_status_pack(&values, msg));
}

/** Live sensor readings for the laptop panel (binary, wire.c): the line
   and barcode sensors, steering, B over bars, inside a barcode, the last
   forward sonar reading, sin(pitch), climbed height, IMU answering. */
LOCAL void sensor_line (void)
{
    wire_sensors_t values;
    uint8_t        msg[WIRE_SENSORS_LEN];

    values.b_ir_left   = IR_BLACK == app_gpio_get(PIN_IR_LINE_L);
    values.b_ir_right  = IR_BLACK == app_gpio_get(PIN_IR_LINE_R);
    values.b_ir_bar    = barcode_black();
    values.b_hold      = barcode_in_bars();
    values.b_inbar     = barcode_busy();
    values.b_imu_ok    = imu_ok();
    values.steer       = line_steer_mm_s();
    values.front_mm    = obstacle_front_mm();
    values.tilt_x1000  = imu_pitch_mdeg_sin();
    values.height_mm10 = imu_height_mm10();
    outbox_put(TOPIC_IDX_SENSORS, msg, wire_sensors_pack(&values, msg));
}

/** The MQTT link: FAULT_WIFI_LOSS while the glue has not asked for a
   message for LINK_LOST_US (it asks every 100 ms while connected).  A
   warning: it never stops the robot. */
LOCAL void check_link (void)
{
#if defined(TM_MQTT) && TM_MQTT
    if ((app_timing_now_us() - g_link_us) > LINK_LOST_US)
    {
        app_stop_fault_set(FAULT_WIFI_LOSS);
    }
    else
    {
        app_stop_fault_clear(FAULT_WIFI_LOSS);
    }
#endif
}

/** The telemetry task: status line every 1 s, sensors every 0.5 s, timing
 * table. */
EXPORT void telemetry_task (int32_t stacd, void * p_exinf)
{
    uint32_t report_tick = 0;
    uint32_t ping_tick   = 0;
    uint32_t status_tick = 0;
    uint32_t sensor_tick = 0;
    bool     b_beeping   = false;

    (void)stacd;
    (void)p_exinf;
    buzzer_init();

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR); /* woken every 100 ms */
        app_timing_wake(g_tmg_telemetry, tk_can_wup(TSK_SELF));
        b_beeping = buzzer_tick(b_beeping);

        ping_tick++;

        if (ping_tick >= S1_PING_TICKS)
        {
            ping_tick = 0;
            app_check(tk_set_flg(gh_flg_mission, MSN_FLG_PING), "IF-01 ping");
        }

        status_tick++;

        if (status_tick >= S1_STATUS_TICKS)
        {
            status_tick = 0;
            check_link();
            status_line();
        }

        sensor_tick++;

        if (sensor_tick >= S1_SENSOR_TICKS)
        {
            sensor_tick = 0;
            sensor_line();
        }

        report_tick++;

        if (report_tick >= S1_REPORT_TICKS)
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
