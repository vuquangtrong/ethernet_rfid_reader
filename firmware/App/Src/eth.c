#include "eth.h"
#include "enc28j60.h"
#include "main.h"
#include <lwip/etharp.h>
#include <lwip/netif.h>
#include <string.h>

#if DEBUG_ENC_RX_TX
#include <stdio.h>
#endif

extern SPI_HandleTypeDef hspi2;

/* Methods of ENC28_SPI_Context */
static void nss_pin_op(uint8_t val) {
    HAL_GPIO_WritePin(ETH_NSS_GPIO_Port, ETH_NSS_Pin, val);
}

static void spi_out_op(const uint8_t *buff, size_t len) {
    HAL_SPI_Transmit(&hspi2, buff, len, 1);
}

static void spi_in_op(uint8_t *buff, size_t len) {
    HAL_SPI_Receive(&hspi2, buff, len, 1);
}

static void spi_in_out_op(const uint8_t *tx, uint8_t *rx, size_t len) {
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len, 1);
}

static void wait_nano(uint32_t val) {
    __NOP(); // 1 instruction spends 14ns at 72MHz
    val = val >> 4;
    while (val--) {
        __NOP();
    }
}

/* Global Ethernet Context */
ENC28_SPI_Context enc28ctx = {
    .nss_pin_op = &nss_pin_op,
    .spi_out_op = &spi_out_op,
    .spi_in_op = &spi_in_op,
    .spi_in_out_op = &spi_in_out_op,
    .wait_nano = &wait_nano,
};

static void enc28_init() {
    ENC28_CommandStatus status = ENC28_OK;

    /* Send a dummy byte to initialize SPI */
    enc28ctx.nss_pin_op(1); // CS high
    enc28ctx.spi_out_op((uint8_t[]){0x00}, 1);

    /* */
    status = enc28_do_soft_reset(&enc28ctx);
    if (status != ENC28_OK) {
        printf("enc28_do_soft_reset failed\n");
        return;
    }
    HAL_Delay(100);

    status = enc28_do_init(
        (ENC28_MAC_Address){
            .addr = {
                ETH_MAC_ADDR_0,
                ETH_MAC_ADDR_1,
                ETH_MAC_ADDR_2,
                ETH_MAC_ADDR_3,
                ETH_MAC_ADDR_4,
                ETH_MAC_ADDR_5,
            }},
        &enc28ctx);
    if (status != ENC28_OK) {
        printf("enc28_do_init failed\n");
        return;
    }
    HAL_Delay(100);

    /* */
    ENC28_MAC_Address mac_addr = {0};
    status = enc28_do_read_mac(&enc28ctx, &mac_addr);
    if (status != ENC28_OK) {
        printf("enc28_do_read_mac failed\n");
        return;
    }

    printf("PHMAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac_addr.addr[0],
           mac_addr.addr[1],
           mac_addr.addr[2],
           mac_addr.addr[3],
           mac_addr.addr[4],
           mac_addr.addr[5]);
    if (mac_addr.addr[0] != ETH_MAC_ADDR_0 ||
        mac_addr.addr[1] != ETH_MAC_ADDR_1 ||
        mac_addr.addr[2] != ETH_MAC_ADDR_2 ||
        mac_addr.addr[3] != ETH_MAC_ADDR_3 ||
        mac_addr.addr[4] != ETH_MAC_ADDR_4 ||
        mac_addr.addr[5] != ETH_MAC_ADDR_5) {
        printf("Invalid MAC address\n");
        return;
    }

    /* */
    ENC28_HW_Rev hw_rev = {0};
    status = enc28_do_read_hw_rev(&enc28ctx, &hw_rev);
    if (status != ENC28_OK) {
        printf("enc28_do_read_hw_rev failed\n");
        return;
    }

    printf("PHID1: 0x%04x\n", hw_rev.phid1);
    printf("PHID2: 0x%04x\n", hw_rev.phid2);
    printf("REVID: 0x%02x\n", hw_rev.revid);
    if (hw_rev.phid1 != 0x0083 ||
        hw_rev.phid2 != 0x1400 ||
        hw_rev.revid != 0x06) {
        printf("Invalid HW revision\n");
        return;
    }

    /* */
    status = enc28_begin_packet_transfer(&enc28ctx);
    if (status != ENC28_OK) {
        printf("enc28_begin_packet_transfer failed\n");
        return;
    }

    printf("ENC28: initialized\n");
}

/* Global Ethernet Interface Instance */
struct netif eth0;

static void low_level_init(struct netif *netif) {
    /* set MAC hardware address */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    netif->hwaddr[0] = ETH_MAC_ADDR_0;
    netif->hwaddr[1] = ETH_MAC_ADDR_1;
    netif->hwaddr[2] = ETH_MAC_ADDR_2;
    netif->hwaddr[3] = ETH_MAC_ADDR_3;
    netif->hwaddr[4] = ETH_MAC_ADDR_4;
    netif->hwaddr[5] = ETH_MAC_ADDR_5;

    /* maximum transfer unit */
    netif->mtu = 1518;

    /* device capabilities */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;

    /* actual hardware initialization */
    enc28_init();

    /* netif is now up*/
    netif->flags |= NETIF_FLAG_LINK_UP;

    printf("low_level_init done\n");
}

static err_t low_level_output(struct netif *netif, struct pbuf *p) {
    (void)netif;
    struct pbuf *q;
    /* Send the data from the pbuf to the interface, one pbuf at a
       time. The size of the data in each pbuf is kept in the ->len
       variable. */
    for (q = p; q != NULL; q = q->next) {
        if (ENC28_OK != enc28_write_packet(&enc28ctx, q->payload, q->len)) {
            return ERR_IF;
        }
        #if DEBUG_ENC_RX_TX
        printf("--> %d\n", q->len);
        #endif
    }

    return ERR_OK;
}

static uint8_t pkt_buf[1600];
static struct pbuf *low_level_input(struct netif *netif) {
    (void)netif;
    struct pbuf *p = NULL, *q;

    /* Read packet */
    u16_t len;
    ENC28_Receive_Status_Vector status_vec = {0};
    ENC28_CommandStatus rcv_stat = enc28_read_packet(&enc28ctx, pkt_buf, 1600, &status_vec);
    if (rcv_stat != ENC28_OK) {
        return NULL;
    }

    /* Packet is valid, process it */
    len = (status_vec.packet_len_hi << 8) | status_vec.packet_len_lo;
    #if DEBUG_ENC_RX_TX
    printf("<-- %d\n", len);
    #endif

    /* We allocate a pbuf chain of pbufs from the pool. */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (p != NULL) {
        uint32_t offset = 0;
        for (q = p; q != NULL; q = q->next) {
            memcpy(q->payload, pkt_buf + offset, q->len);
            offset += q->len;
        }
    }

    return p;
}

static void ethernetif_input(struct netif *netif) {
    struct pbuf *p;

    /* move received packet into a new pbuf */
    p = low_level_input(netif);
    /* if no packet could be read, silently ignore this */
    if (p != NULL) {
        /* pass all packets to ethernet_input, which decides what packets it supports */
        if (netif->input(p, netif) != ERR_OK) {
            pbuf_free(p);
            p = NULL;
        }
    }
}

static err_t ethernetif_init(struct netif *netif) {
    /* Interface name */
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    /* Output methods */
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    /* Hardware init */
    low_level_init(netif);

    printf("ethernetif_init done\n");
    return ERR_OK;
}

void eth_init() {
    struct ip4_addr ipaddr;
    IP4_ADDR(&ipaddr,
             IP_ADDR_0,
             IP_ADDR_1,
             IP_ADDR_2,
             IP_ADDR_3);

    /* Set IP Address and processing methods */
    netif_add(&eth0, &ipaddr, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, &ethernetif_init, &netif_input);

    /* Set the default interface */
    netif_set_default(&eth0);

    /* When Link (HW) is up, process to set interface up */
    if (netif_is_link_up(&eth0)) {
        netif_set_up(&eth0);
    }
    printf("eth_init done\n");
}

void eth_input() {
    /* read data and process it */
    ethernetif_input(&eth0);
}
