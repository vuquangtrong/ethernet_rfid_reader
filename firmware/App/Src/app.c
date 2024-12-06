#include "app.h"
#include "eth.h"
#include "main.h"
#include <lwip/init.h>
#include <lwip/timeouts.h>

uint8_t eth_irq = 0;

uint32_t sys_now(void) {
    return HAL_GetTick();
}

__attribute__((noreturn)) void app_main(void) {
    setbuf(stdout, NULL);
    printf("\n");

    lwip_init();
    eth_init();

    while (1) {
        eth_input();
        sys_check_timeouts();
        HAL_Delay(1);
    }
}
