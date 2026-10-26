/* Host test: WiFi test command queue + verdicts, using the real source. */
#include <string.h>
#include "sdk_stub.h"
#include "lwip/apps/mqtt.h"
stub_timer_t stub_timer;
uint64_t stub_now_us = 1000000u;
bool stub_gpio_level[32];
void tight_loop_contents(void) {}

static char g_last_pub[256];
static int g_pubs;
err_t mqtt_publish(mqtt_client_t *c, const char *topic, const void *payload, u16_t len,
                   u8_t qos, u8_t retain, mqtt_request_cb_t cb, void *arg)
{
    (void)c; (void)qos; (void)retain; (void)cb; (void)arg;
    if (strcmp(topic, "robot/telemetry/events") == 0)
    {
        size_t n = (len < sizeof(g_last_pub) - 1u) ? len : sizeof(g_last_pub) - 1u;
        memcpy(g_last_pub, payload, n);
        g_last_pub[n] = '\0';
        g_pubs++;
    }
    return ERR_OK;
}

#define main wifi_main
#include "../tests/wifi_mqtt/wifi_mqtt_test.c"
#undef main

static int g_fail;

static void deliver(const char *p_msg, uint32_t tot_len)
{
    size_t n = strlen(p_msg);
    mqtt_incoming_publish_cb(NULL, "robot/cmd/control", tot_len ? tot_len : (u32_t)n);
    if (tot_len > CMD_MAX_BYTES)
    {
        /* oversize arrives in two chunks */
        mqtt_incoming_data_cb(NULL, (const u8_t *)p_msg, (u16_t)n, 0u);
        mqtt_incoming_data_cb(NULL, (const u8_t *)p_msg, (u16_t)n, MQTT_DATA_FLAG_LAST);
    }
    else
    {
        mqtt_incoming_data_cb(NULL, (const u8_t *)p_msg, (u16_t)n, MQTT_DATA_FLAG_LAST);
    }
}

static void handle_all_expect(const char *const *pp_want, int n_want)
{
    int i = 0;
    while (g_cmdq_tail != g_cmdq_head)
    {
        command_handle(&g_cmdq[g_cmdq_tail % CMD_QUEUE_LEN]);
        g_cmdq_tail++;
        if ((i >= n_want) || (strstr(g_last_pub, pp_want[i]) == NULL))
        {
            printf("FAIL ack %d: got %s want %s\n", i, g_last_pub, (i < n_want) ? pp_want[i] : "(none)");
            g_fail++;
        }
        else
        {
            printf("PASS ack %d: %s\n", i, g_last_pub);
        }
        i++;
    }
    if (i != n_want)
    {
        printf("FAIL handled %d, expected %d\n", i, n_want);
        g_fail++;
    }
}

int main(void)
{
    char big[700];
    g_boot_id = 77u;

    printf("-- verdict order --\n");
    deliver("{\"boot_id\":77,\"seq\":1,\"cmd\":\"ping\"}", 0u);
    deliver("{\"boot_id\":77,\"seq\":1,\"cmd\":\"ping\"}", 0u);
    deliver("{\"boot_id\":78,\"seq\":2,\"cmd\":\"ping\"}", 0u);
    deliver("{\"boot_id\":77,\"seq\":2,\"cmd\":\"dance\"}", 0u);
    {
        const char *want[] = {"\"accepted\"", "rejected_duplicate", "rejected_old_boot",
                              "rejected_unknown_cmd"};
        handle_all_expect(want, 4);
    }
    deliver("{\"boot_id\":77,\"seq\":2,\"cmd\":\"ping\",}", 0u);
    deliver("{\"boot_id\":77,\"cmd\":\"ping\"}", 0u);
    memset(big, 'x', 600u); big[600] = '\0';
    deliver(big, 600u);
    deliver("{\"boot_id\":77,\"seq\":2,\"cmd\":\"ping\"}", 0u);
    {
        const char *want[] = {"rejected_malformed", "rejected_malformed", "rejected_oversize",
                              "\"accepted\""};
        handle_all_expect(want, 4);
    }

    printf("-- burst of 6 before the main loop runs (queue = %u) --\n", CMD_QUEUE_LEN);
    {
        int k;
        char msg[96];
        for (k = 0; k < 6; k++)
        {
            snprintf(msg, sizeof(msg), "{\"boot_id\":77,\"seq\":%d,\"cmd\":\"ping\"}", 10 + k);
            deliver(msg, 0u);
        }
    }
    printf("%s queued %lu, dropped %lu (want 4 queued, 2 dropped and counted)\n",
           ((g_cmdq_head - g_cmdq_tail) == 4u && g_cmd_dropped == 2u) ? "PASS" : "FAIL",
           (unsigned long)(g_cmdq_head - g_cmdq_tail), (unsigned long)g_cmd_dropped);
    if (!((g_cmdq_head - g_cmdq_tail) == 4u && g_cmd_dropped == 2u)) { g_fail++; }
    {
        const char *want[] = {"\"seq\":10", "\"seq\":11", "\"seq\":12", "\"seq\":13"};
        handle_all_expect(want, 4);
    }
    printf("-- queue usable again after a drop --\n");
    deliver("{\"boot_id\":77,\"seq\":20,\"cmd\":\"ping\"}", 0u);
    {
        const char *want[] = {"\"accepted\""};
        handle_all_expect(want, 1);
    }
    printf("wifi cmd queue: %s (%d failures)\n", g_fail ? "FAILED" : "all passed", g_fail);
    return g_fail ? 1 : 0;
}
