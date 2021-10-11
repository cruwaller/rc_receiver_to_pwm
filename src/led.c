#include "led.h"
#include "main.h"
#include "defines.h"

#ifdef PIN_LED1
struct gpio_pin led1;
#define LED1_SET(_X) (GPIO_Write(led1, (_X)))
#else
#define LED1_SET(_X)
#endif
#ifdef PIN_LED2
struct gpio_pin led2;
#define LED2_SET(_X) (GPIO_Write(led2, (_X)))
#else
#define LED2_SET(_X)
#endif


void led_set(uint8_t const type)
{
    switch (type) {
    case LED_BOOTING:
        LED1_SET(0);
        LED2_SET(0);
        break;
    case LED_READY:
        LED1_SET(1);
        break;
    case LED_ERROR:
        LED1_SET(1);
        LED2_SET(1);
        break;
    default:
        break;
    }
}


void led_init(void)
{
#ifdef PIN_LED1
    led1 = GPIO_Setup(IO_CREATE(PIN_LED1), GPIO_OUTPUT, 0);
#endif
#ifdef PIN_LED2
    led2 = GPIO_Setup(IO_CREATE(PIN_LED2), GPIO_OUTPUT, 0);
#endif
    led_set(LED_BOOTING);
}
