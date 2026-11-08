/*
 * Golden vectors for the binary MQTT messages (Change Requirement 1).
 * Builds the robot's own app_program/subsystem1/wire.c on the laptop;
 * tools/panel/test_wire.py compiles and runs this and checks every message
 * against the panel's decoder (is34_wire.py) and the format document.
 *
 *   wire_golden            print the test messages, one per line
 *   wire_golden <hex>      unpack a map command: "1 KQXM" or "0"
 *
 * Laptop code, not robot firmware.
 */
#include <stdio.h>
#include <string.h>

#include "subsystem1/wire.h"

static void hex (const uint8_t * p_buf, int32_t len)
{
    int32_t idx;

    for (idx = 0; idx < len; idx++)
    {
        printf("%02x", p_buf[idx]);
    }

    printf("\n");
}

static void status (uint32_t mission, uint32_t line, uint32_t stop,
                    uint32_t faults, int32_t speed_l, int32_t speed_r,
                    int32_t dist, uint32_t junctions, uint8_t last,
                    const char * p_map, int32_t ramp)
{
    wire_status_t in;
    uint8_t       out[WIRE_STATUS_LEN];
    int32_t       len;

    in.mission_state  = mission;
    in.line_state     = line;
    in.stop_reasons   = stop;
    in.faults         = faults;
    in.speed_l        = speed_l;
    in.speed_r        = speed_r;
    in.dist_mm        = dist;
    in.junctions      = junctions;
    in.last_letter    = last;
    memcpy(in.map_letter, p_map, 4);
    in.ramp_peak_mm10 = ramp;
    len               = wire_status_pack(&in, out);
    printf("status mission=%u line=%u stop=%u faults=%u speed_l=%ld "
           "speed_r=%ld dist_mm=%ld junctions=%u last=%u map_L=%u map_R=%u "
           "map_F=%u map_U=%u ramp_peak_mm10=%ld len=%ld hex=",
           mission, line, stop, faults, (long)speed_l, (long)speed_r,
           (long)dist, junctions, last, (uint8_t)p_map[0], (uint8_t)p_map[1],
           (uint8_t)p_map[2], (uint8_t)p_map[3], (long)ramp, (long)len);
    hex(out, len);
}

static void sensors (bool l, bool r, bool b, bool hold, bool inbar, bool imu,
                     int32_t steer, int32_t front, int32_t tilt, int32_t h10)
{
    wire_sensors_t in = {l, r, b, hold, inbar, imu, steer, front, tilt, h10};
    uint8_t        out[WIRE_SENSORS_LEN];
    int32_t        len = wire_sensors_pack(&in, out);

    printf("sensors ir_l=%d ir_r=%d ir_b=%d hold=%d inbar=%d imu_ok=%d "
           "steer=%ld front=%ld tilt=%ld h10=%ld len=%ld hex=",
           l, r, b, hold, inbar, imu, (long)steer, (long)front, (long)tilt,
           (long)h10, (long)len);
    hex(out, len);
}

static void ack (bool ok, const char * p_map)
{
    uint8_t out[WIRE_ACK_LEN];
    int32_t len = wire_ack_pack(ok, (const uint8_t *)p_map, out);

    printf("ack ok=%d map_L=%u map_R=%u map_F=%u map_U=%u len=%ld hex=", ok,
           (uint8_t)p_map[0], (uint8_t)p_map[1], (uint8_t)p_map[2],
           (uint8_t)p_map[3], (long)len);
    hex(out, len);
}

int main (int argc, char ** argv)
{
    if (argc == 2)
    {
        uint8_t  buf[16];
        uint8_t  letters[4] = {0, 0, 0, 0};
        int32_t  len        = 0;
        unsigned byte;

        while ((len < 16) && (sscanf(argv[1] + (2 * len), "%2x", &byte) == 1))
        {
            buf[len++] = (uint8_t)byte;
        }

        if (wire_map_unpack(buf, len, letters))
        {
            printf("1 %c%c%c%c\n", letters[0], letters[1], letters[2],
                   letters[3]);
        }
        else
        {
            printf("0\n");
        }

        return 0;
    }

    status(2, 0, 0, 0x40, 120, -118, 12345, 3, 'F', "KQXM", 123);
    status(0, 1, 2, 0x01, 0, 0, 0, 0, 0, "\0\0\0\0", 0);
    status(3, 3, 7, 0xFF, 3000, -3000, 600000, 300, '-', "A1Z9", 5000);
    status(1, 2, 1, 0x30, -2048, 2047, -1000000, 255, '0', "BCDE", -5);
    sensors(true, false, true, false, true, true, -25, 1234, -87, 156);
    sensors(false, true, false, true, false, false, 200, -1, 2000, -9000);
    sensors(false, false, false, false, false, true, -128, -2, -1024, 4095);
    sensors(true, true, true, true, true, true, 127, 9000, 1023, -4096);
    ack(true, "KQXM");
    ack(false, "\0\0\0\0");
    return 0;
}
