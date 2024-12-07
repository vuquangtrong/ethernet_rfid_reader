#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

// refer to LwIP/src/include/lwip/opt.h

/* No OS used */
#define NO_SYS                      1 // no api, no socket, netconn
#define SYS_LIGHTWEIGHT_PROT        0 // no inter-task protection

/* No high level APIs */
#define LWIP_SOCKET                 0 // no os so no socket
#define LWIP_NETCONN                0 // no os so no netconn

/* IP Layer */
// #define LWIP_IPV4                   1 // (default = 1)
// #define LWIP_IPV6                   0 // (default = 0)
// #define LWIP_RAW                    0 // (default = 0) no hook into the IP layer itself
// #define LWIP_ICMP                   1 // (default = 1)
// #define LWIP_IGMP                   0 // (default = 0)
// #define LWIP_UDP                    1 // (default = 1)
// #define LWIP_TCP                    1 // (default = 1)
// #define LWIP_DHCP                   0 // (default = 0)
// #define LWIP_DNS                    0 // (default = 0)

/* Memory */
#define MEM_ALIGNMENT               4 // should be set to the alignment of the CPU
#define MEM_LIBC_MALLOC             1 // Use malloc/free/realloc provided by C-library
#define MEMP_NUM_PBUF               8 // (default = 16) the number of memp struct pbufs (used for PBUF_ROM and PBUF_REF)
#define MEMP_NUM_RAW_PCB            4 // (default = 4) the number of RAW connection PCBs
#define MEMP_NUM_UDP_PCB            4 // (default = 4) the number of UDP connection PCBs
#define MEMP_NUM_TCP_PCB            4 // (default = 5) the number of TCP connection PCBs
#define MEMP_NUM_TCP_PCB_LISTEN     4 // (default = 8) the number of listening TCP connections
#define PBUF_POOL_SIZE              16 // (default = 16) the number of buffers in the pbuf pool

/* Network Interface */
#define LWIP_SINGLE_NETIF               1 // (default = 0) use a single netif only, no routing
// #define LWIP_NETIF_HOSTNAME             0 // (default = 0)
// #define LWIP_NETIF_API                  0 // (default = 0)
// #define LWIP_NETIF_STATUS_CALLBACK      0 // (default = 0)
// #define LWIP_NETIF_EXT_STATUS_CALLBACK  0 // (default = 0)
// #define LWIP_NETIF_LINK_CALLBACK        0 // (default = 0)
// #define LWIP_NETIF_REMOVE_CALLBACK      0 // (default = 0)
// #define LWIP_NETIF_LOOPBACK             0 // (default = 0)

/* Statistics */
#define LWIP_STATS                      0 // (default = 1) no statistics collection

/* Debugging */
// #define LWIP_DEBUG                      1 // (default = 0) enable printing messages to stdout
// #define LWIP_DBG_MIN_LEVEL              LWIP_DBG_LEVEL_ALL // (default = LWIP_DBG_LEVEL_ALL)
// #define LWIP_DBG_TYPES_ON               LWIP_DBG_ON // (default = LWIP_DBG_ON)

// all components are disabled by default, turn on only the ones you need
// #define TIMERS_DEBUG                    LWIP_DBG_ON
// #define ETHARP_DEBUG                    LWIP_DBG_ON
// #define PBUF_DEBUG                      LWIP_DBG_ON
// #define MEM_DEBUG                       LWIP_DBG_ON
// #define MEMP_DEBUG                      LWIP_DBG_ON
// #define ICMP_DEBUG                      LWIP_DBG_ON
// #define IP_DEBUG                        LWIP_DBG_ON
// #define TCP_DEBUG                       LWIP_DBG_ON
// #define UDP_DEBUG                       LWIP_DBG_ON
// #define DHCP_DEBUG                      LWIP_DBG_ON
// #define DNS_DEBUG                       LWIP_DBG_ON

#endif /* __LWIPOPTS_H__ */
