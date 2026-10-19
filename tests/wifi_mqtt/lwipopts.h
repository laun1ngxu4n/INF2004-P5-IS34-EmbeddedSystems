/*
 * lwIP options for the WiFi/MQTT bring-up test (NO_SYS, poll mode).
 * Based on the pico-examples common settings, trimmed to what MQTT needs.
 */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define MEM_LIBC_MALLOC             0

#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24

#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_DHCP                   1
#define LWIP_TCP_KEEPALIVE          1

#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))

#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0
#define LWIP_CHKSUM_ALGORITHM       3

#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0
#define LWIP_STATS                  0

/* MQTT needs one extra system timeout for its cyclic timer. */
#define MEMP_NUM_SYS_TIMEOUT        (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 1)

/*
 * MQTT output ring buffer. lwIP's default is 256 bytes, which means a
 * publish of more than ~240 bytes of payload fails with ERR_MEM.
 * The design allows payloads up to 512 bytes (NFR9), so this must be
 * raised. Set it back to 256 to see the payload-size test fail.
 */
#define MQTT_OUTPUT_RINGBUF_SIZE    1024
#define MQTT_REQ_MAX_IN_FLIGHT      8

#endif /* LWIPOPTS_H */
