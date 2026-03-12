#ifndef __COMMON_UTILS_H__
#define __COMMON_UTILS_H__

#include "at32f403a_407.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void memset(void *dest, uint8_t val, uint32_t len);
void memcpy(void *dest, const void *src, uint32_t len);
uint16_t crc16(const uint8_t *data, uint16_t len);

#endif /* __COMMON_UTILS_H__ */