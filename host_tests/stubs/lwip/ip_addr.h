#pragma once
#include "sdk_stub.h"
typedef struct { uint32_t a; } ip_addr_t;
static inline int ipaddr_aton(const char *s, ip_addr_t *a) { (void)s; a->a = 0; return 1; }
static inline const char *ip4addr_ntoa(const void *a) { (void)a; return "0.0.0.0"; }
