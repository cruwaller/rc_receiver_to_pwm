#ifndef LED_H_
#define LED_H_

#include <stdint.h>

enum {
    LED_BOOTING = 0b00,
    LED_READY = 0b01,
    LED_ERROR = 0b10,
    LED_UPDATE = 0b11,
};

void led_set(uint8_t type);
void led_init(void);

#endif
