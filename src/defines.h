#pragma once

#ifndef NUM_CHANNELS
#define NUM_CHANNELS 0x8U
#endif

#define US_OUT_MIN 1000U
#define US_OUT_MAX 2000U
#define US_OUT_MID ((US_OUT_MIN + US_OUT_MAX) / 2)

#define ANALOG_MIN  US_OUT_MIN
#define ANALOG_MAX  US_OUT_MAX


#ifndef __section
#define __section(S)    __attribute__((section(S)))
#endif
#ifndef __aligned
#define	__aligned(x)	__attribute__((__aligned__(x)))
#endif

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

#if !RAM_CODE_SIZE
#define FAST_CODE_1
#define FAST_CODE_2
#else // !RAM_CODE_SIZE
/* FAST_CODE_1 is always linked into RAM */
#define FAST_CODE_1  __section(".ram_code")
/* FAST_CODE_2 is linked into RAM only if enough space */
#if RAM_CODE_SIZE < 8192
#define FAST_CODE_2
#else
#define FAST_CODE_2 __section(".ram_code")
#endif
#endif // RAM_CODE_SIZE

#define DRAM_FORCED     __section(".data")
#define WORD_ALIGNED    __aligned(32)
