#ifndef __HELPERS_H_
#define __HELPERS_H_

#include <stdint.h>
#include <stddef.h>


#define PACKED __attribute__((packed))
#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Macros for big-endian (assume little endian host for now) etc
#define BYTE_SWAP_U16(x) ((uint16_t)__builtin_bswap16(x))
#define BYTE_SWAP_U32(x) ((uint32_t)__builtin_bswap32(x))

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))


static inline float MAP_F(float x, float in_min, float in_max,
                          float out_min, float out_max)
{
    return CONSTRAIN((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
}

static inline uint16_t MAP_U16(uint16_t x, uint16_t in_min, uint16_t in_max,
                               uint16_t out_min, uint16_t out_max)
{
    return CONSTRAIN((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
};

static inline uint16_t MAP_I16(int16_t x, int16_t in_min, int16_t in_max,
                               int16_t out_min, int16_t out_max)
{
    return CONSTRAIN((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
};

#endif /* HELPERS_H_ */
