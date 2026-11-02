/* Host test: strict command JSON parser (tests/wifi_mqtt/cmd_json.c). */
#include <stdio.h>
#include <string.h>
#include "../tests/wifi_mqtt/cmd_json.h"

static int g_fail;

static void expect(const char *p_text, int want_ok, unsigned want_seq)
{
    json_obj_t obj;
    uint32_t seq = 0u;
    int ok = cmd_json_parse(p_text, strlen(p_text), &obj);
    int got_seq = ok && cmd_json_get_u32(&obj, "seq", &seq);
    int pass = (ok == want_ok) && (!want_ok || (got_seq && seq == want_seq));
    printf("%-4s %-48s parse=%s%s\n", pass ? "PASS" : "FAIL", p_text, ok ? "ok" : "rejected",
           (ok && got_seq) ? "" : "");
    if (!pass) { g_fail++; }
}

int main(void)
{
    json_obj_t obj;
    const char *cmd = "{\"boot_id\":7,\"seq\":3,\"cmd\":\"ping\"}";

    printf("-- valid inputs (must be accepted) --\n");
    expect("{\"seq\":1}", 1, 1);
    expect("{\"seq\" :1}", 1, 1);
    expect(" {\"seq\":1}", 1, 1);
    expect("{\"seq\"\n:1}", 1, 1);
    expect("{ \"boot_id\" : 5 , \"seq\" : 0 , \"cmd\" : \"ping\" }\r\n", 1, 0);
    expect("{\"seq\":4294967295}", 1, 4294967295u);
    printf("-- malformed inputs (must be rejected) --\n");
    expect("{\"seq\":-1}", 0, 0);
    expect("{\"seq\":1.5}", 0, 0);
    expect("{\"seq\":1e3}", 0, 0);
    expect("{\"seq\":4294967296}", 0, 0);
    expect("{\"seq\":7garbage}", 0, 0);
    expect("{\"seq\":1", 0, 0);
    expect("{\"seq\":01}", 0, 0);
    expect("{\"seq\":1,}", 0, 0);
    expect("{\"nested\":{\"seq\":5}}", 0, 0);
    expect("{\"a\":[1]}", 0, 0);
    expect("{\"a\":true}", 0, 0);
    expect("{\"seq\":1,\"seq\":2}", 0, 0);
    expect("{\"seq\":1} x", 0, 0);
    expect("{\"cmd\":\"pi\\\"ng\"}", 0, 0);
    expect("{seq:1}", 0, 0);
    expect("", 0, 0);
    printf("-- field lookups --\n");
    if (cmd_json_parse(cmd, strlen(cmd), &obj) && cmd_json_str_equals(&obj, "cmd", "ping") &&
        !cmd_json_str_equals(&obj, "cmd", "pin") && !cmd_json_str_equals(&obj, "seq", "3") &&
        !cmd_json_has(&obj, "boot"))
    {
        printf("PASS cmd string compare / missing key / wrong type\n");
    }
    else
    {
        printf("FAIL cmd string compare / missing key / wrong type\n");
        g_fail++;
    }
    printf("cmd_json: %s (%d failures)\n", g_fail ? "FAILED" : "all passed", g_fail);
    return g_fail ? 1 : 0;
}
