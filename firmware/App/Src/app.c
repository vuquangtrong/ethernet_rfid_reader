#include "app.h"
#include "main.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

/* Methods of ENC28_SPI_Context */
void nss_pin_op(uint8_t val)
{
    HAL_GPIO_WritePin(ETH_NSS_GPIO_Port, ETH_NSS_Pin, val);
}

void spi_out_op(const uint8_t *buff, size_t len)
{
    HAL_SPI_Transmit(&hspi2, buff, len, 1);
}

void spi_in_op(uint8_t *buff, size_t len)
{
    HAL_SPI_Receive(&hspi2, buff, len, 1);
}

void spi_in_out_op(const uint8_t *tx, uint8_t *rx, size_t len)
{
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len, 1);
}

void wait_nano(uint32_t val)
{
    __NOP(); // 1 instruction spends 14ns at 72MHz
    val = val >> 4;
    while (val--)
    {
        __NOP();
    }
}

/* Global Ethernet Context */
ENC28_SPI_Context enc28ctx = {
    .nss_pin_op = nss_pin_op,
    .spi_out_op = spi_out_op,
    .spi_in_op = spi_in_op,
    .spi_in_out_op = spi_in_out_op,
    .wait_nano = wait_nano,
};

__attribute__((noreturn)) void app_main(void)
{
    setbuf(stdout, NULL);
    printf("\n\r");

    /* Send a dummy byte to initialize SPI */
    enc28ctx.nss_pin_op(1); // CS high
    enc28ctx.spi_out_op((uint8_t[]){0x00}, 1);

    /* Ethernet Initialization */
    printf("ENC28J60 Driver Initializing...\n\r");

    assert(ENC28_OK == enc28_do_soft_reset(&enc28ctx));
    HAL_Delay(100);

    assert(ENC28_OK == enc28_do_init(
                           (ENC28_MAC_Address){
                               .addr = {
                                   MAC_ADDR_BYTE_0,
                                   MAC_ADDR_BYTE_1,
                                   MAC_ADDR_BYTE_2,
                                   MAC_ADDR_BYTE_3,
                                   MAC_ADDR_BYTE_4,
                                   MAC_ADDR_BYTE_5,
                               }},
                           &enc28ctx));
    HAL_Delay(100);

    ENC28_MAC_Address mac_addr;
    assert(ENC28_OK == enc28_do_read_mac(&enc28ctx, &mac_addr));
    printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n\r",
           mac_addr.addr[0],
           mac_addr.addr[1],
           mac_addr.addr[2],
           mac_addr.addr[3],
           mac_addr.addr[4],
           mac_addr.addr[5]);
    assert(mac_addr.addr[0] == MAC_ADDR_BYTE_0);
    assert(mac_addr.addr[1] == MAC_ADDR_BYTE_1);
    assert(mac_addr.addr[2] == MAC_ADDR_BYTE_2);
    assert(mac_addr.addr[3] == MAC_ADDR_BYTE_3);
    assert(mac_addr.addr[4] == MAC_ADDR_BYTE_4);
    assert(mac_addr.addr[5] == MAC_ADDR_BYTE_5);

    ENC28_HW_Rev hw_rev;
    assert(ENC28_OK == enc28_do_read_hw_rev(&enc28ctx, &hw_rev));
    printf("PHID1: 0x%04x\n\r", hw_rev.phid1);
    printf("PHID2: 0x%04x\n\r", hw_rev.phid2);
    printf("REVID: 0x%02x\n\r", hw_rev.revid);
    assert(hw_rev.phid1 == 0x0083);
    assert(hw_rev.phid2 == 0x1400);
    assert(hw_rev.revid == 0x06);

    uint8_t i = 0;
    while (1)
    {
        printf("Hello, World! %d\n\r", i++);
        HAL_Delay(1000);
    }
}
