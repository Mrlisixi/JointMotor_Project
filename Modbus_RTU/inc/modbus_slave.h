#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include <stdint.h>

void Modbus_Init(uint8_t slaveAddr);
void Modbus_FeedData(uint8_t *data, uint16_t len);
void Modbus_ParseFrame(void);

#endif