#include "app.h"
#include "eth.h"
#include "lwip/init.h"
#include "main.h"

uint32_t sys_now(void) {
    return HAL_GetTick();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ETH_IRQ_Pin) {
        eth_irq();
    }
}
__attribute__((noreturn)) void app_main(void) {
    lwip_init();
    eth_init();

    while (1) {
        eth_poll();
    }
}
