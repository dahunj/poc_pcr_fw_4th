#pragma once

#include "core_cmFunc.h"
#include "core_cmInstr.h"

extern int lock_cnt;

#define lock() \
    do{ \
        __disable_irq(); \
        lock_cnt++; \
    } while(0)

#define unlock() \
    do{ \
        lock_cnt--; \
        if(lock_cnt<= 0) \
        { \
            lock_cnt=0; \
            __enable_irq(); \
        } \
    } while(0)


char* sub_three(float input);
char* system_elapsedTime(void);
uint32_t times10(uint32_t input);
void btoah(const uint8_t *src, uint8_t *dest, uint16_t srclen);
int ahtob(const uint8_t *str);
uint32_t convertTo_uint32(uint8_t *input);
uint8_t atoh (uint8_t data);
