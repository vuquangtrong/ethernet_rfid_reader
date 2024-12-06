#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

// refer to LwIP/src/include/lwip/opt.h

#define NO_SYS                          1 // default = 0
#define SYS_LIGHTWEIGHT_PROT            0 // default = 1
#define MEM_LIBC_MALLOC                 1 // default = 0
#define MEM_ALIGNMENT                   4 // default = 1
// #define MEM_SIZE                        1600 // default = 1600
// #define PBUF_POOL_SIZE                  16 // default = 16
#define LWIP_RAW                        0 // default = 1
#define LWIP_SINGLE_NETIF               1 // default = 0
#define LWIP_NETCONN                    0 // default = 1
#define LWIP_SOCKET                     0 // default = 1
#define LWIP_STATS                      0 // default = 1
#define LWIP_ARP                        1 // default = 1
#define LWIP_IPV4                       1 // default = 1
#define LWIP_IPV6                       0 // default = 0
#define LWIP_ICMP                       0 // default = 1
// #define LWIP_IGMP                       0 // default = 0
// #define LWIP_DNS                        0 // default = 0
// #define LWIP_DHCP                       0 // default = 0
// #define LWIP_UDP                        1 // default = 1
// #define LWIP_TCP                        1 // default = 1
// #define LWIP_NETIF_HOSTNAME             0 // default = 0
// #define LWIP_NETIF_API                  0 // default = 0
// #define LWIP_NETIF_STATUS_CALLBACK      0 // default = 0
// #define LWIP_NETIF_LINK_CALLBACK        0 // default = 0
#define LWIP_DBG_MIN_LEVEL              LWIP_DBG_LEVEL_SERIOUS // default = LWIP_DBG_LEVEL_ALL
#define LWIP_DBG_TYPES_ON               LWIP_DBG_OFF // default = LWIP_DBG_ON
// #define LWIP_ALL_COMPONENTS             LWIP_DBG_OFF // default = LWIP_DBG_OFF
// #define LWIP_PERF                       0 // default = 0

#endif /* __LWIPOPTS_H__ */
