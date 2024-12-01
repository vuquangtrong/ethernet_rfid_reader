#ifndef __APP_H
#define __APP_H

#include "enc28j60.h"

/* Maximum number of ethernet packets in use */
#define MAX_ETH_PACKETS 8

/* MAC address for the ENC28J60 interface, byte 0 */
#define MAC_ADDR_BYTE_0 0xAA
/* MAC address for the ENC28J60 interface, byte 1 */
#define MAC_ADDR_BYTE_1 0xBB
/* MAC address for the ENC28J60 interface, byte 2 */
#define MAC_ADDR_BYTE_2 0xCC
/* MAC address for the ENC28J60 interface, byte 3 */
#define MAC_ADDR_BYTE_3 0xDD
/* MAC address for the ENC28J60 interface, byte 4 */
#define MAC_ADDR_BYTE_4 0xEE
/* MAC address for the ENC28J60 interface, byte 5 */
#define MAC_ADDR_BYTE_5 0xFF

/* Static IP address for the ENC28J60 interface [129.168.0.22] */
#define ENC28_IP_ADDR ((192 << 0) | (168 << 8) | (0 << 16) | (22 << 24))

/* Methods of ENC28_SPI_Context */
void nss_pin_op(uint8_t val);
void spi_out_op(const uint8_t *buff, size_t len);
void spi_in_op(uint8_t *buff, size_t len);
void spi_in_out_op(const uint8_t *tx, uint8_t *rx, size_t len);
void wait_nano(uint32_t val);

/* Main Application */
__attribute__((noreturn)) void app_main(void);

#endif // __APP_H
