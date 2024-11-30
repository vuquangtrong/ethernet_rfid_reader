#include "app.h"
#include "main.h"
#include <stdio.h>

__attribute__((noreturn)) void app_main(void)
{
    setbuf(stdout, NULL);
    uint8_t i = 0;
    while (1)
    {
        printf("Hello, World! %d\n\r", i++);
        HAL_Delay(1000);
    }
}
