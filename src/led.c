#include "led.h"
#include "main.h"
#include "defines.h"

struct gpio_pin led1;
struct gpio_pin led2;


void led_set(uint8_t type, uint8_t value)
{
    switch (type) {
    case LED_BOOTING:
        GPIO_Write(led1, 0);
        GPIO_Write(led2, 0);
        break;
    case LED_READY:
        GPIO_Write(led1, 1);
        break;
    case LED_ERROR:
        GPIO_Write(led1, 0);
        GPIO_Write(led2, 1);
        break;
    default:
        break;
    }
}


void led_init(void)
{
    led1 = GPIO_Setup(GPIO('B', 7), GPIO_OUTPUT, 0);
    led2 = GPIO_Setup(GPIO('B', 8), GPIO_OUTPUT, 0);
}
