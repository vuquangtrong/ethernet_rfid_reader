#ifndef __ETH_H__
#define __ETH_H__

#define DEBUG_ENC_RX_TX 0

/* Network Interface */
#define IFNAME0 'e'
#define IFNAME1 '0'

#define ETH_MAC_ADDR_0 0x00
#define ETH_MAC_ADDR_1 0x80
#define ETH_MAC_ADDR_2 0xE1
#define ETH_MAC_ADDR_3 0xFF
#define ETH_MAC_ADDR_4 0x00
#define ETH_MAC_ADDR_5 0x01

#define IP_ADDR_0 192
#define IP_ADDR_1 168
#define IP_ADDR_2 1
#define IP_ADDR_3 10

#define NETMASK_ADDR_0 255
#define NETMASK_ADDR_1 255
#define NETMASK_ADDR_2 255
#define NETMASK_ADDR_3 0

#define GATEWAY_ADDR_0 192
#define GATEWAY_ADDR_1 168
#define GATEWAY_ADDR_2 1
#define GATEWAY_ADDR_3 1

/* Methods of Ethernet Interface */
void eth_init();
void eth_input();

#endif // __ETH_H__
