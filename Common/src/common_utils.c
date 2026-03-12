#include "common_utils.h"

void delay_ms(uint32_t ms)
{
  uint32_t i, j;
  for (i = 0; i < ms; i++)
  {
    for (j = 0; j < 1000; j++)
    {
      __NOP();
    }
  }
}

void delay_us(uint32_t us)
{
  uint32_t i;
  for (i = 0; i < us; i++)
  {
    __NOP();
  }
}

void memset(void *dest, uint8_t val, uint32_t len)
{
  uint8_t *p = (uint8_t *)dest;
  while (len--)
  {
    *p++ = val;
  }
}

void memcpy(void *dest, const void *src, uint32_t len)
{
  uint8_t *d = (uint8_t *)dest;
  const uint8_t *s = (const uint8_t *)src;
  while (len--)
  {
    *d++ = *s++;
  }
}

uint16_t crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  uint16_t i;
  
  while (len--)
  {
    crc ^= *data++;
    for (i = 0; i < 8; i++)
    {
      if (crc & 0x0001)
      {
        crc = (crc >> 1) ^ 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}