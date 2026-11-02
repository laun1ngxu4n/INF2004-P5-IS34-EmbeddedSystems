/*
 * P5-IS34 bring-up test: WiFi + MQTT (Buddy 1)
 *
 * What it measures (all printed over USB serial):
 *   - CYW43 init time, WiFi join + DHCP time, MQTT connect time
 *     (design budget: join 5 s, MQTT 3 s, 8 s per attempt)
 *   - cyw43_arch_poll() call time, average and worst case
 *   - Whole activation time: poll + state machine + command handling +
 *     heartbeat/telemetry publish (excludes the 1 s USB summary line)
 *   - Gap between the starts of consecutive polls (the "poll gap")
 *   - Heartbeat interval jitter on the robot side (NFR5b: 1 s +/- 100 ms)
 *   - WiFi RSSI in every heartbeat, for the course walk (A10)
 *   - Largest MQTT payload the lwIP buffer accepts (a capacity probe;
 *     "pub ok" means the publish call succeeded, not broker receipt:
 *     use mqtt_watch.py on the laptop to confirm delivery)
 *   - Command handling: strict JSON, boot_id + seq duplicate rule,
 *     unknown command, oversize, and a 4-slot receive queue whose
 *     overflow is counted and reported (never silent)
 *   - Outage and recovery time after the AP or broker is switched off
 *     and back on (NFR5: <= 20 s after both are available)
 *
 * Buttons (Robo Pico onboard, active low):
 *   GP20  cycle the poll period 10 -> 45 -> 95 -> 245 -> 495 -> 1995 ms,
 *         to see what happens if the priority-9 Telemetry task is starved
 *   GP21  mark an event: press when you switch the AP/broker OFF
 *         (measures detection time) and again when you switch it back ON
 *         (measures recovery time; includes your reaction time)
 *
 * Plain Pico SDK, poll mode, no RTOS. lwIP callbacks run inside
 * cyw43_arch_poll() on the same thread as the main loop, so the command
 * queue below needs no locking. The real Telemetry task will call the
 * same cyw43_arch_poll() from a 10 ms periodic task.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/rand.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

#include "cmd_json.h"

#ifdef WIFI_RUNTIME_CONFIG
/*
 * Prebuilt image: no credentials compiled in. SSID, password and broker
 * IP are typed into the serial monitor at start-up and kept in RAM only.
 */
static char g_cfg_ssid[33];
static char g_cfg_pass[64];
static char g_cfg_broker[16];
#define WIFI_SSID        g_cfg_ssid
#define WIFI_PASSWORD    g_cfg_pass
#define MQTT_BROKER_IP   g_cfg_broker
#define MQTT_BROKER_PORT 1883
#else
#include "wifi_config.h"
#endif

/* ---- Pins ---------------------------------------------------------- */
#define BTN_PERIOD_PIN          20u
#define BTN_MARK_PIN            21u
#define BTN_DEBOUNCE_MS         50u

/* ---- Design values under test -------------------------------------- */
#define WIFI_JOIN_TIMEOUT_MS    5000u
#define MQTT_CONNECT_TIMEOUT_MS 3000u
#define HEARTBEAT_PERIOD_MS     1000u
#define TELEMETRY_PERIOD_MS     1000u
#define STATS_PERIOD_MS         1000u
#define RECONNECT_LIMIT_MS      20000u
#define CMD_MAX_BYTES           512u
#define CMD_QUEUE_LEN           4u
#define FAILS_TO_WIFI_FAULT     5u

/*
 * lwIP's MQTT cyclic timer ticks every 5 s, so a silent broker loss is
 * only detected after roughly 1.5 x keep-alive rounded up to 5 s steps.
 * With 5 s keep-alive that is about 10 s.
 */
#define MQTT_KEEPALIVE_S        5u

#define TOPIC_HEARTBEAT         "robot/heartbeat"
#define TOPIC_STATE             "robot/telemetry/state"
#define TOPIC_EVENTS            "robot/telemetry/events"
#define TOPIC_SIZE_TEST         "robot/test/size"
#define TOPIC_CMD_CONTROL       "robot/cmd/control"

static const uint32_t g_backoff_ms[] = {500u, 1000u, 2000u, 2000u};
#define BACKOFF_STEPS           (sizeof(g_backoff_ms) / sizeof(g_backoff_ms[0]))

/*
 * Deliberately not divisors of 1000 ms, so a late poll shows up as
 * heartbeat jitter instead of lining up with the 1 s deadline.
 */
static const uint32_t g_poll_period_ms[] = {10u, 45u, 95u, 245u, 495u, 1995u};
#define POLL_PERIOD_STEPS       (sizeof(g_poll_period_ms) / sizeof(g_poll_period_ms[0]))

static const uint16_t g_size_test_bytes[] = {128u, 240u, 256u, 400u, 512u, 600u};
#define SIZE_TEST_STEPS         (sizeof(g_size_test_bytes) / sizeof(g_size_test_bytes[0]))

typedef enum
{
    NET_WIFI_JOINING = 0,
    NET_MQTT_CONNECTING,
    NET_CONNECTED,
    NET_BACKOFF
} net_state_t;

static const char *const g_state_name[] =
{
    "WIFI_JOINING", "MQTT_CONNECTING", "CONNECTED", "BACKOFF"
};

typedef struct
{
    uint32_t count;
    uint64_t sum_us;
    uint32_t max_us;
} timing_stat_t;

typedef struct
{
    char     topic[64];
    char     buf[CMD_MAX_BYTES + 1u];
    uint32_t len;
    bool     oversize;
    uint64_t rx_us;
} cmd_slot_t;

/* ---- MQTT client state (written from lwIP callbacks) ---------------- */
static mqtt_client_t *gp_client;
static volatile bool  g_mqtt_accepted;
static volatile bool  g_mqtt_failed;
static volatile int   g_mqtt_fail_status;

/* Receive queue: callbacks fill slot [head], main loop handles [tail]. */
static cmd_slot_t g_cmdq[CMD_QUEUE_LEN];
static uint32_t   g_cmdq_head;          /* committed messages */
static uint32_t   g_cmdq_tail;          /* handled messages */
static bool       g_rx_filling;         /* a message is being copied into [head] */
static uint32_t   g_cmd_dropped;        /* arrived while the queue was full */
static uint32_t   g_cmdq_peak;

/* ---- Poll statistics (every poll goes through do_poll) -------------- */
static timing_stat_t g_poll_win;
static timing_stat_t g_poll_all;
static timing_stat_t g_gap_win;
static uint64_t      g_last_poll_start_us;

/* ---- Test state ---------------------------------------------------- */
static uint32_t g_boot_id;
static uint32_t g_last_seq;
static uint32_t g_hb_seq;
static uint32_t g_pub_ok;
static uint32_t g_pub_err;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void stat_add(timing_stat_t *p_stat, uint32_t value_us)
{
    p_stat->count++;
    p_stat->sum_us += value_us;
    if (value_us > p_stat->max_us)
    {
        p_stat->max_us = value_us;
    }
}

static uint32_t stat_avg(const timing_stat_t *p_stat)
{
    return (p_stat->count == 0u) ? 0u : (uint32_t)(p_stat->sum_us / p_stat->count);
}

/* The only place cyw43_arch_poll() is called, so every poll is measured. */
static void do_poll(void)
{
    uint64_t p_start = time_us_64();
    uint32_t p_us;

    if (g_last_poll_start_us != 0u)
    {
        stat_add(&g_gap_win, (uint32_t)(p_start - g_last_poll_start_us));
    }
    g_last_poll_start_us = p_start;
    cyw43_arch_poll();
    p_us = (uint32_t)(time_us_64() - p_start);
    stat_add(&g_poll_win, p_us);
    stat_add(&g_poll_all, p_us);
}

/* ---- lwIP callbacks ------------------------------------------------ */
static void mqtt_connection_cb(mqtt_client_t *p_client, void *p_arg,
                               mqtt_connection_status_t status)
{
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        g_mqtt_accepted = true;
    }
    else
    {
        g_mqtt_failed = true;
        g_mqtt_fail_status = (int)status;
    }
}

static void mqtt_incoming_publish_cb(void *p_arg, const char *p_topic, u32_t tot_len)
{
    uint32_t used = g_cmdq_head - g_cmdq_tail;
    cmd_slot_t *p_slot;

    if (used >= CMD_QUEUE_LEN)
    {
        /* Full: drop this message and count it. The main loop reports
         * the new total on robot/telemetry/events. */
        g_cmd_dropped++;
        g_rx_filling = false;
        return;
    }
    p_slot = &g_cmdq[g_cmdq_head % CMD_QUEUE_LEN];
    strncpy(p_slot->topic, p_topic, sizeof(p_slot->topic) - 1u);
    p_slot->topic[sizeof(p_slot->topic) - 1u] = '\0';
    p_slot->len = 0u;
    p_slot->oversize = (tot_len > CMD_MAX_BYTES);
    p_slot->rx_us = time_us_64();
    g_rx_filling = true;
}

static void mqtt_incoming_data_cb(void *p_arg, const u8_t *p_data, u16_t len, u8_t flags)
{
    cmd_slot_t *p_slot;

    if (!g_rx_filling)
    {
        return;     /* message being dropped */
    }
    p_slot = &g_cmdq[g_cmdq_head % CMD_QUEUE_LEN];
    if (!p_slot->oversize)
    {
        uint32_t room = CMD_MAX_BYTES - p_slot->len;
        uint32_t copy = (len < room) ? len : room;
        memcpy(&p_slot->buf[p_slot->len], p_data, copy);
        p_slot->len += copy;
    }
    if ((flags & MQTT_DATA_FLAG_LAST) != 0u)
    {
        p_slot->buf[p_slot->len] = '\0';
        g_cmdq_head++;
        g_rx_filling = false;
        if ((g_cmdq_head - g_cmdq_tail) > g_cmdq_peak)
        {
            g_cmdq_peak = g_cmdq_head - g_cmdq_tail;
        }
    }
}

static void mqtt_sub_cb(void *p_arg, err_t result)
{
    printf("[mqtt] subscribe %s result=%d\n", TOPIC_CMD_CONTROL, (int)result);
}

/* ---- Helpers ------------------------------------------------------- */
static err_t publish_text(const char *p_topic, const char *p_text, uint8_t qos)
{
    err_t err = mqtt_publish(gp_client, p_topic, p_text, (u16_t)strlen(p_text),
                             qos, 0u, NULL, NULL);
    if (err == ERR_OK)
    {
        g_pub_ok++;
    }
    else
    {
        g_pub_err++;
    }
    return err;
}

/*
 * Checks one command and acks it on robot/telemetry/events.
 * Order: oversize, malformed, old boot, duplicate/old seq, unknown cmd.
 * A rejected command never advances the accepted sequence number.
 */
static void command_handle(const cmd_slot_t *p_slot)
{
    const char *p_verdict;
    json_obj_t obj;
    uint32_t boot_id = 0u;
    uint32_t seq = 0u;
    bool have_seq = false;
    char reply[192];

    if (p_slot->oversize)
    {
        p_verdict = "rejected_oversize";
    }
    else if (!cmd_json_parse(p_slot->buf, p_slot->len, &obj))
    {
        p_verdict = "rejected_malformed";
    }
    else if (!cmd_json_get_u32(&obj, "boot_id", &boot_id) ||
             !cmd_json_get_u32(&obj, "seq", &seq) ||
             !cmd_json_has(&obj, "cmd"))
    {
        have_seq = cmd_json_get_u32(&obj, "seq", &seq);
        p_verdict = "rejected_malformed";
    }
    else
    {
        have_seq = true;
        if (boot_id != g_boot_id)
        {
            p_verdict = "rejected_old_boot";
        }
        else if (seq <= g_last_seq)
        {
            p_verdict = "rejected_duplicate";
        }
        else if (!cmd_json_str_equals(&obj, "cmd", "ping"))
        {
            p_verdict = "rejected_unknown_cmd";
        }
        else
        {
            g_last_seq = seq;
            p_verdict = "accepted";
        }
    }

    (void)snprintf(reply, sizeof(reply),
                   "{\"ack\":\"%s\",\"seq\":%lu,\"seq_known\":%s,\"handle_us\":%lu,"
                   "\"t_ms\":%lu}",
                   p_verdict, (unsigned long)seq, have_seq ? "true" : "false",
                   (unsigned long)(time_us_64() - p_slot->rx_us),
                   (unsigned long)now_ms());
    (void)publish_text(TOPIC_EVENTS, reply, 1u);
    printf("[cmd] %s on %s: %s -> %s\n", p_slot->oversize ? "(oversize)" : "",
           p_slot->topic, p_slot->oversize ? "" : p_slot->buf, p_verdict);
}

/*
 * Capacity probe: publishes increasing sizes once, in its own phase.
 * Polls go through do_poll(), and every steady-state window is restarted
 * afterwards, so the probe cannot show up as a false poll gap.
 */
static void payload_size_test(void)
{
    static char payload[700];
    uint32_t i;

    printf("[size] CAPACITY PROBE (separate phase, excluded from steady-state stats; "
           "tests buffer capacity, not the 512 B limit rule). MQTT_OUTPUT_RINGBUF_SIZE=%d\n",
           MQTT_OUTPUT_RINGBUF_SIZE);
    g_poll_win = (timing_stat_t){0};
    g_gap_win = (timing_stat_t){0};
    for (i = 0u; i < SIZE_TEST_STEPS; i++)
    {
        uint16_t n = g_size_test_bytes[i];
        uint32_t t_end = now_ms() + 200u;
        err_t err;

        memset(payload, 'x', n);
        payload[n] = '\0';
        err = mqtt_publish(gp_client, TOPIC_SIZE_TEST, payload, n, 0u, 0u, NULL, NULL);
        printf("[size] %3u bytes -> %s (err=%d)\n", (unsigned)n,
               (err == ERR_OK) ? "OK" : "FAILED", (int)err);

        /* Let lwIP drain the buffer before the next size. */
        while (now_ms() < t_end)
        {
            do_poll();
            sleep_ms(5u);
        }
    }
    printf("[size] probe phase: poll max %lu us, gap max %lu ms. Steady-state windows restart now.\n",
           (unsigned long)g_poll_win.max_us, (unsigned long)(g_gap_win.max_us / 1000u));
    g_poll_win = (timing_stat_t){0};
    g_gap_win = (timing_stat_t){0};
    g_last_poll_start_us = 0u;
}

static bool button_pressed(uint32_t pin, bool *p_was_down, uint32_t *p_last_ms)
{
    bool down = !gpio_get(pin);
    bool pressed = false;
    uint32_t t = now_ms();

    if (down && !*p_was_down && ((t - *p_last_ms) > BTN_DEBOUNCE_MS))
    {
        pressed = true;
        *p_last_ms = t;
    }
    *p_was_down = down;
    return pressed;
}

static void button_init(uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

#ifdef WIFI_RUNTIME_CONFIG
/* Reads one line from USB serial with echo and backspace. */
static void read_line(const char *p_prompt, char *p_buf, uint32_t size, bool hide)
{
    uint32_t n = 0u;
    uint32_t idle_ms = 0u;

    printf("%s", p_prompt);
    for (;;)
    {
        int c = getchar_timeout_us(100000u);
        if (c == PICO_ERROR_TIMEOUT)
        {
            idle_ms += 100u;
            if ((n == 0u) && (idle_ms >= 5000u))
            {
                printf("\n%s", p_prompt);   /* in case the monitor opened late */
                idle_ms = 0u;
            }
            continue;
        }
        idle_ms = 0u;
        if ((c == '\r') || (c == '\n'))
        {
            if (n == 0u)
            {
                continue;   /* ignore empty lines and the LF after a CR */
            }
            break;
        }
        if (((c == 0x08) || (c == 0x7F)) && (n > 0u))
        {
            n--;
            printf("\b \b");
            continue;
        }
        if ((c >= 0x20) && (c < 0x7F) && (n < (size - 1u)))
        {
            p_buf[n] = (char)c;
            n++;
            putchar(hide ? '*' : c);
        }
    }
    p_buf[n] = '\0';
    printf("\n");
}

static void ask_config(void)
{
    ip_addr_t check;

    printf("\nType the settings and press Enter after each (kept in RAM only).\n");
    printf("Hotspot must be 2.4 GHz and WPA2. Broker IP = your laptop's IP on the hotspot.\n");
    read_line("WiFi SSID: ", g_cfg_ssid, sizeof(g_cfg_ssid), false);
    read_line("WiFi password: ", g_cfg_pass, sizeof(g_cfg_pass), true);
    for (;;)
    {
        read_line("Broker IP (e.g. 172.20.10.3): ", g_cfg_broker, sizeof(g_cfg_broker), false);
        if (ipaddr_aton(g_cfg_broker, &check))
        {
            break;
        }
        printf("not an IPv4 address, try again\n");
    }
}
#endif

/* ---- Main ---------------------------------------------------------- */
int main(void)
{
    net_state_t state = NET_WIFI_JOINING;
    net_state_t prev_state = NET_BACKOFF;
    ip_addr_t broker_ip;
    struct mqtt_connect_client_info_t client_info;
    timing_stat_t hb_jitter = {0};
    timing_stat_t act_win = {0};
    timing_stat_t act_all = {0};
    timing_stat_t rssi_call = {0};
    uint32_t poll_idx = 0u;
    uint32_t t_state = 0u;
    uint32_t backoff_until = 0u;
    uint32_t fail_count = 0u;
    uint32_t t_lost = 0u;
    uint32_t t_off_mark = 0u;      /* press 1: AP/broker switched off */
    uint32_t t_on_mark = 0u;       /* press 2: AP/broker back on */
    uint32_t t_connected = 0u;     /* last time MQTT became connected */
    bool next_mark_is_on = false;
    bool off_armed = false;        /* waiting for LOST after the OFF mark */
    bool on_armed = false;         /* waiting for reconnect after the ON mark */
    bool lost_since_off = false;
    uint32_t next_hb = 0u;
    uint32_t next_tel = 0u;
    uint32_t next_stats = 0u;
    uint32_t last_hb_ms = 0u;
    uint32_t hb_out_of_spec = 0u;
    uint32_t hb_skipped = 0u;
    bool period_req = false;
    bool mark_req = false;
    uint32_t mark_req_ms = 0u;
    uint32_t outages = 0u;
    uint32_t dropped_reported = 0u;
    int32_t  rssi = 0;
    bool ever_connected = false;
    bool size_test_done = false;
    bool period_down = false;
    bool mark_down = false;
    uint32_t period_last = 0u;
    uint32_t mark_last = 0u;
    uint32_t t0;
    absolute_time_t next_poll;

    stdio_init_all();
    button_init(BTN_PERIOD_PIN);
    button_init(BTN_MARK_PIN);
    sleep_ms(3000u); /* time to open the serial monitor */

    printf("\n=== P5-IS34 WiFi/MQTT bring-up test (v3) ===\n");
#ifdef WIFI_RUNTIME_CONFIG
    ask_config();
#endif

    t0 = now_ms();
    if (cyw43_arch_init_with_country(CYW43_COUNTRY('S', 'G', 0)) != 0)
    {
        printf("[init] cyw43_arch_init FAILED\n");
        return 1;
    }
    printf("[init] cyw43 init %lu ms\n", (unsigned long)(now_ms() - t0));
    cyw43_arch_enable_sta_mode();

    g_boot_id = get_rand_32() & 0x7FFFFFFFu;
    printf("[init] boot_id=%lu  broker=%s:%d  keepalive=%u s  cmd queue %u slots\n",
           (unsigned long)g_boot_id, MQTT_BROKER_IP, MQTT_BROKER_PORT,
           MQTT_KEEPALIVE_S, CMD_QUEUE_LEN);

    if (!ipaddr_aton(MQTT_BROKER_IP, &broker_ip))
    {
        printf("[init] bad MQTT_BROKER_IP\n");
        return 1;
    }

    gp_client = mqtt_client_new();
    memset(&client_info, 0, sizeof(client_info));
    client_info.client_id = "p5is34-robot";
    client_info.keep_alive = MQTT_KEEPALIVE_S;

    next_poll = get_absolute_time();

    for (;;)
    {
        uint32_t t;
        uint64_t act_start = time_us_64();

        /* ---- One "task activation": poll, then the state machine ---- */
        do_poll();
        t = now_ms();

        if (state != prev_state)
        {
            printf("[%7lu] state -> %s\n", (unsigned long)t, g_state_name[state]);
            prev_state = state;
        }

        switch (state)
        {
            case NET_WIFI_JOINING:
                if (t_state == 0u)
                {
                    t_state = t;
                    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
                    (void)cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
                                                        CYW43_AUTH_WPA2_AES_PSK);
                }
                if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP)
                {
                    printf("[wifi] joined + DHCP in %lu ms, ip=%s\n",
                           (unsigned long)(t - t_state),
                           ip4addr_ntoa(netif_ip4_addr(netif_default)));
                    state = NET_MQTT_CONNECTING;
                    t_state = 0u;
                }
                else if ((t - t_state) > WIFI_JOIN_TIMEOUT_MS)
                {
                    printf("[wifi] join timeout (%u ms), link status=%d\n",
                           WIFI_JOIN_TIMEOUT_MS,
                           cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA));
                    state = NET_BACKOFF;
                    t_state = 0u;
                }
                break;

            case NET_MQTT_CONNECTING:
                if (t_state == 0u)
                {
                    err_t err;
                    t_state = t;
                    g_mqtt_accepted = false;
                    g_mqtt_failed = false;
                    err = mqtt_client_connect(gp_client, &broker_ip, MQTT_BROKER_PORT,
                                              mqtt_connection_cb, NULL, &client_info);
                    /* connect wipes the client, so set the callbacks after it */
                    mqtt_set_inpub_callback(gp_client, mqtt_incoming_publish_cb,
                                            mqtt_incoming_data_cb, NULL);
                    if (err != ERR_OK)
                    {
                        printf("[mqtt] connect call failed err=%d\n", (int)err);
                        g_mqtt_failed = true;
                    }
                }
                if (g_mqtt_accepted)
                {
                    printf("[mqtt] connected in %lu ms\n", (unsigned long)(t - t_state));
                    (void)mqtt_subscribe(gp_client, TOPIC_CMD_CONTROL, 1u, mqtt_sub_cb, NULL);
                    if (t_lost != 0u)
                    {
                        printf("[outage] #%lu total outage %lu ms\n",
                               (unsigned long)outages, (unsigned long)(t - t_lost));
                        t_lost = 0u;
                    }
                    t_connected = t;
                    if (on_armed)
                    {
                        uint32_t rec = t - t_on_mark;
                        printf("[outage] RECOVERY %lu ms from 'back on' mark -> %s (limit %u ms)\n",
                               (unsigned long)rec,
                               (rec <= RECONNECT_LIMIT_MS) ? "PASS" : "FAIL",
                               RECONNECT_LIMIT_MS);
                        on_armed = false;
                    }
                    state = NET_CONNECTED;
                    t_state = 0u;
                    fail_count = 0u;
                    ever_connected = true;
                    next_hb = t;
                    next_tel = t + 500u;
                    last_hb_ms = 0u;
                }
                else if (g_mqtt_failed || ((t - t_state) > MQTT_CONNECT_TIMEOUT_MS))
                {
                    printf("[mqtt] connect failed (%s, status=%d)\n",
                           g_mqtt_failed ? "refused/error" : "timeout",
                           g_mqtt_fail_status);
                    mqtt_disconnect(gp_client);
                    state = NET_BACKOFF;
                    t_state = 0u;
                }
                break;

            case NET_CONNECTED:
                if ((cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) ||
                    g_mqtt_failed || !mqtt_client_is_connected(gp_client))
                {
                    outages++;
                    t_lost = t;
                    printf("[outage] #%lu LOST at %lu ms (%s)\n", (unsigned long)outages,
                           (unsigned long)t,
                           (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP)
                               ? "wifi link down" : "mqtt dropped");
                    if (off_armed)
                    {
                        printf("[outage] detection took %lu ms after the 'off' mark\n",
                               (unsigned long)(t - t_off_mark));
                        off_armed = false;
                        lost_since_off = true;
                    }
                    mqtt_disconnect(gp_client);
                    g_rx_filling = false;
                    state = NET_BACKOFF;
                    fail_count = 0u;
                    break;
                }

                if (!size_test_done)
                {
                    payload_size_test();
                    size_test_done = true;
                    /* restart the 1 s schedule and the jitter stats */
                    t = now_ms();
                    next_hb = t;
                    next_tel = t + 500u;
                    last_hb_ms = 0u;
                    hb_jitter = (timing_stat_t){0};
                    hb_out_of_spec = 0u;
                    act_win = (timing_stat_t){0};
                    act_start = time_us_64();
                }

                /* Handle every queued command (bounded by the queue size). */
                {
                    uint32_t handled = 0u;
                    while ((g_cmdq_tail != g_cmdq_head) && (handled < CMD_QUEUE_LEN))
                    {
                        command_handle(&g_cmdq[g_cmdq_tail % CMD_QUEUE_LEN]);
                        g_cmdq_tail++;
                        handled++;
                    }
                }
                if (g_cmd_dropped != dropped_reported)
                {
                    char ev[96];
                    dropped_reported = g_cmd_dropped;
                    (void)snprintf(ev, sizeof(ev),
                                   "{\"event\":\"cmd_dropped\",\"total\":%lu,\"t_ms\":%lu}",
                                   (unsigned long)g_cmd_dropped, (unsigned long)t);
                    (void)publish_text(TOPIC_EVENTS, ev, 1u);
                    printf("[cmd] QUEUE FULL - %lu command(s) dropped in total\n",
                           (unsigned long)g_cmd_dropped);
                }

                if ((int32_t)(t - next_hb) >= 0)
                {
                    char hb[192];
                    uint64_t r0 = time_us_64();
                    if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) != 0)
                    {
                        rssi = 0;
                    }
                    stat_add(&rssi_call, (uint32_t)(time_us_64() - r0));
                    if (last_hb_ms != 0u)
                    {
                        uint32_t interval = t - last_hb_ms;
                        uint32_t dev = (interval > HEARTBEAT_PERIOD_MS)
                                       ? (interval - HEARTBEAT_PERIOD_MS)
                                       : (HEARTBEAT_PERIOD_MS - interval);
                        stat_add(&hb_jitter, dev * 1000u);
                        if (dev > 100u)
                        {
                            hb_out_of_spec++;
                        }
                    }
                    last_hb_ms = t;
                    g_hb_seq++;
                    (void)snprintf(hb, sizeof(hb),
                                   "{\"boot_id\":%lu,\"seq\":%lu,\"uptime_ms\":%lu,"
                                   "\"poll_period_ms\":%lu,\"poll_max_us\":%lu,"
                                   "\"act_max_us\":%lu,\"rssi\":%ld}",
                                   (unsigned long)g_boot_id, (unsigned long)g_hb_seq,
                                   (unsigned long)t,
                                   (unsigned long)g_poll_period_ms[poll_idx],
                                   (unsigned long)g_poll_win.max_us,
                                   (unsigned long)act_win.max_us, (long)rssi);
                    (void)publish_text(TOPIC_HEARTBEAT, hb, 0u);
                    next_hb += HEARTBEAT_PERIOD_MS; /* fixed rate, no drift */
                    if ((int32_t)(t - next_hb) >= 0)
                    {
                        /* Missed one or more slots (starved): skip them
                         * instead of sending a catch-up burst. */
                        hb_skipped += 1u + ((t - next_hb) / HEARTBEAT_PERIOD_MS);
                        next_hb = t + HEARTBEAT_PERIOD_MS;
                    }
                }

                if ((int32_t)(t - next_tel) >= 0)
                {
                    /* Realistic-size placeholder for robot/telemetry/state. */
                    char tel[256];
                    (void)snprintf(tel, sizeof(tel),
                                   "{\"t_ms\":%lu,\"state\":\"READY\",\"spd_l_mms\":0,"
                                   "\"spd_r_mms\":0,\"enc_l\":0,\"enc_r\":0,\"dist_mm\":0,"
                                   "\"ir\":[0,0,0],\"us_mm\":0,\"pitch_deg\":0.0,"
                                   "\"heading_deg\":0.0,\"stop_reasons\":0}",
                                   (unsigned long)t);
                    (void)publish_text(TOPIC_STATE, tel, 0u);
                    next_tel += TELEMETRY_PERIOD_MS;
                    if ((int32_t)(t - next_tel) >= 0)
                    {
                        next_tel = t + TELEMETRY_PERIOD_MS;   /* skip, no burst */
                    }
                }
                break;

            case NET_BACKOFF:
            default:
                if (backoff_until == 0u)
                {
                    uint32_t gap = g_backoff_ms[(fail_count < BACKOFF_STEPS)
                                                ? fail_count : (BACKOFF_STEPS - 1u)];
                    fail_count++;
                    backoff_until = t + gap;
                    printf("[retry] attempt %lu failed/lost, waiting %lu ms\n",
                           (unsigned long)fail_count, (unsigned long)gap);
                    if (fail_count == (ever_connected ? (2u * FAILS_TO_WIFI_FAULT)
                                                      : FAILS_TO_WIFI_FAULT))
                    {
                        printf("[retry] design would enter WIFI_FAULT here "
                               "(test keeps retrying every <=2 s)\n");
                    }
                }
                if ((int32_t)(t - backoff_until) >= 0)
                {
                    backoff_until = 0u;
                    state = (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP)
                            ? NET_MQTT_CONNECTING : NET_WIFI_JOINING;
                }
                break;
        }

        /* ---- Buttons ---------------------------------------------- */
        if (period_req)
        {
            period_req = false;
            poll_idx = (poll_idx + 1u) % POLL_PERIOD_STEPS;
            printf("[%7lu] poll period now %lu ms\n", (unsigned long)t,
                   (unsigned long)g_poll_period_ms[poll_idx]);
            hb_jitter = (timing_stat_t){0};
            hb_out_of_spec = 0u;
            hb_skipped = 0u;
        }
        if (mark_req)
        {
            mark_req = false;
            /* Presses alternate OFF, ON, OFF, ON ... whatever the state. */
            if (!next_mark_is_on)
            {
                t_off_mark = mark_req_ms;
                off_armed = (state == NET_CONNECTED);
                lost_since_off = !off_armed;
                on_armed = false;
                next_mark_is_on = true;
                printf("[%7lu] MARK 1/2: AP/broker switched OFF (press GP21 again when it is back ON)%s\n",
                       (unsigned long)mark_req_ms,
                       off_armed ? "" : " - note: robot was already disconnected");
            }
            else
            {
                t_on_mark = mark_req_ms;
                next_mark_is_on = false;
                off_armed = false;
                printf("[%7lu] MARK 2/2: AP/broker back ON\n", (unsigned long)mark_req_ms);
                if (!lost_since_off)
                {
                    printf("[outage] robot NEVER detected a loss between the marks "
                           "(outage too short to notice, or nothing was switched off) - trial not valid\n");
                }
                else if ((state == NET_CONNECTED) && (t_connected >= t_off_mark))
                {
                    printf("[outage] already reconnected %lu ms BEFORE the 'back on' mark "
                           "(recovery <= 0 ms; you pressed late) -> PASS\n",
                           (unsigned long)(t_on_mark - t_connected));
                }
                else
                {
                    on_armed = true;   /* RECOVERY prints when MQTT reconnects */
                }
            }
        }

        {
            uint32_t a_us = (uint32_t)(time_us_64() - act_start);
            stat_add(&act_win, a_us);
            stat_add(&act_all, a_us);
        }

        /* ---- Once-a-second summary (not counted in activation time) - */
        if ((int32_t)(t - next_stats) >= 0)
        {
            printf("[%7lu] %-15s poll(%lums) call avg=%luus max=%luus all_max=%luus | "
                   "activation max=%luus all_max=%luus | gap max=%lums | "
                   "hb_dev avg=%lums max=%lums >100ms:%lu skipped:%lu | rssi=%ld dBm (call max %luus) | "
                   "pub calls ok=%lu err=%lu | cmd q peak=%lu dropped=%lu\n",
                   (unsigned long)t, g_state_name[state],
                   (unsigned long)g_poll_period_ms[poll_idx],
                   (unsigned long)stat_avg(&g_poll_win), (unsigned long)g_poll_win.max_us,
                   (unsigned long)g_poll_all.max_us,
                   (unsigned long)act_win.max_us, (unsigned long)act_all.max_us,
                   (unsigned long)(g_gap_win.max_us / 1000u),
                   (unsigned long)(stat_avg(&hb_jitter) / 1000u),
                   (unsigned long)(hb_jitter.max_us / 1000u),
                   (unsigned long)hb_out_of_spec, (unsigned long)hb_skipped,
                   (long)rssi, (unsigned long)rssi_call.max_us,
                   (unsigned long)g_pub_ok, (unsigned long)g_pub_err,
                   (unsigned long)g_cmdq_peak, (unsigned long)g_cmd_dropped);
            g_poll_win = (timing_stat_t){0};
            g_gap_win = (timing_stat_t){0};
            act_win = (timing_stat_t){0};
            next_stats = t + STATS_PERIOD_MS;
        }

        /* ---- Wait for the next activation (emulates the task period) */
        next_poll = delayed_by_ms(next_poll, g_poll_period_ms[poll_idx]);
        if (absolute_time_diff_us(get_absolute_time(), next_poll) < 0)
        {
            next_poll = get_absolute_time(); /* overran, don't try to catch up */
        }
        /* Wait in 1 ms steps so buttons are seen at any poll period. */
        while (absolute_time_diff_us(get_absolute_time(), next_poll) > 0)
        {
            if (button_pressed(BTN_PERIOD_PIN, &period_down, &period_last))
            {
                period_req = true;
                next_poll = get_absolute_time();   /* apply the new period now */
                break;
            }
            if (button_pressed(BTN_MARK_PIN, &mark_down, &mark_last))
            {
                mark_req = true;
                mark_req_ms = now_ms();
            }
            sleep_ms(1u);
        }
    }
}
