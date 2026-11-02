#pragma once
#include "sdk_stub.h"
/* Only so main() compiles; host tests never call main(). */
typedef struct { int d; } cyw43_t;
static cyw43_t cyw43_state;
#define CYW43_COUNTRY(a, b, c) 0
#define CYW43_ITF_STA 0
#define CYW43_LINK_UP 3
#define CYW43_AUTH_WPA2_AES_PSK 0
static inline int cyw43_arch_init_with_country(int c) { (void)c; return 0; }
static inline void cyw43_arch_enable_sta_mode(void) {}
static inline int cyw43_wifi_leave(cyw43_t *s, int i) { (void)s; (void)i; return 0; }
static inline int cyw43_arch_wifi_connect_async(const char *a, const char *b, int c)
{ (void)a; (void)b; (void)c; return 0; }
static inline int cyw43_tcpip_link_status(cyw43_t *s, int i) { (void)s; (void)i; return 0; }
static inline int cyw43_wifi_link_status(cyw43_t *s, int i) { (void)s; (void)i; return 0; }
static inline int cyw43_wifi_get_rssi(cyw43_t *s, int32_t *r) { (void)s; *r = -50; return 0; }
