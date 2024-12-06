#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

// refer to LwIP/src/include/lwip/opt.h

/* No OS used */
#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        0

/* No high level APIs */
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

/* Memory */
#define MEM_ALIGNMENT               4

/* Debugging */
// #define LWIP_DEBUG                  1
// #define PBUF_DEBUG                  LWIP_DBG_ON
// #define MEM_DEBUG                   LWIP_DBG_ON
// #define MEMP_DEBUG                  LWIP_DBG_ON

#endif /* __LWIPOPTS_H__ */
