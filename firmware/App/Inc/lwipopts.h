/*
 * Refer contrib/examples/example_app/lwipopts.h for more options
 */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define LWIP_IPV4                   1
#define LWIP_IPV6                   0

#define NO_SYS                      1
#define LWIP_SOCKET                 (NO_SYS==0)
#define LWIP_NETCONN                (NO_SYS==0)
#define LWIP_NETIF_API              (NO_SYS==0)

#define LWIP_IGMP                   LWIP_IPV4
#define LWIP_ICMP                   LWIP_IPV4

#define LWIP_DEBUG                  1
#define SYS_LIGHTWEIGHT_PROT        (NO_SYS==0)

#endif /* __LWIPOPTS_H__ */
