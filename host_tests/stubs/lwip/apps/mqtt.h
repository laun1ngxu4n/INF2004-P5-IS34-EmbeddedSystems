#pragma once
#include "sdk_stub.h"
typedef uint32_t u32_t; typedef uint16_t u16_t; typedef uint8_t u8_t; typedef int8_t err_t;
#define ERR_OK 0
#define MQTT_DATA_FLAG_LAST 1
#define MQTT_OUTPUT_RINGBUF_SIZE 1024
typedef struct { int dummy; } mqtt_client_t;
typedef int mqtt_connection_status_t;
#define MQTT_CONNECT_ACCEPTED 0
struct mqtt_connect_client_info_t { const char *client_id; uint16_t keep_alive; };
typedef void (*mqtt_request_cb_t)(void *arg, err_t err);
err_t mqtt_publish(mqtt_client_t *c, const char *topic, const void *payload, u16_t len,
                   u8_t qos, u8_t retain, mqtt_request_cb_t cb, void *arg);
static inline void cyw43_arch_poll(void) {}
#include "lwip/ip_addr.h"
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *c, void *arg, mqtt_connection_status_t s);
typedef void (*mqtt_incoming_publish_cb_t)(void *arg, const char *topic, u32_t tot_len);
typedef void (*mqtt_incoming_data_cb_t)(void *arg, const u8_t *data, u16_t len, u8_t flags);
static mqtt_client_t stub_client;
static inline mqtt_client_t *mqtt_client_new(void) { return &stub_client; }
static inline err_t mqtt_client_connect(mqtt_client_t *c, const ip_addr_t *ip, uint16_t port,
    mqtt_connection_cb_t cb, void *arg, const struct mqtt_connect_client_info_t *ci)
{ (void)c; (void)ip; (void)port; (void)cb; (void)arg; (void)ci; return 0; }
static inline void mqtt_set_inpub_callback(mqtt_client_t *c, mqtt_incoming_publish_cb_t p,
    mqtt_incoming_data_cb_t d, void *arg) { (void)c; (void)p; (void)d; (void)arg; }
static inline err_t mqtt_subscribe(mqtt_client_t *c, const char *t, u8_t q, mqtt_request_cb_t cb, void *a)
{ (void)c; (void)t; (void)q; (void)cb; (void)a; return 0; }
static inline void mqtt_disconnect(mqtt_client_t *c) { (void)c; }
static inline u8_t mqtt_client_is_connected(mqtt_client_t *c) { (void)c; return 1; }
