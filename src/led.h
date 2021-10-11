#ifndef LED_H_
#define LED_H_

#include <stdint.h>

enum {
    LED_BOOTING,
    LED_READY,
    LED_ERROR,
};

void led_set(uint8_t type, uint8_t value);
void led_init(void);

#endif
