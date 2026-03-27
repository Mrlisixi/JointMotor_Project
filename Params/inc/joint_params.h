#ifndef __JOINT_PARAMS_H
#define __JOINT_PARAMS_H

#include "stdint.h"

#define PARAM_TOTAL_COUNT     328

#define ADDR_P0   0x2000
#define ADDR_P1   0x2100
#define ADDR_P2   0x2200
#define ADDR_P3   0x2300
#define ADDR_P4   0x2400
#define ADDR_P5   0x2500
#define ADDR_P6   0x2600
#define ADDR_P7   0x2700
#define ADDR_P8   0x2800
#define ADDR_P9   0x2900
#define ADDR_PA   0x3000
#define ADDR_PB   0x3100
#define ADDR_PC   0x3200
#define ADDR_PD   0x3300

typedef enum {
    PARAM_READ_ONLY  = 0,
    PARAM_READ_WRITE = 1
} ParamAccessTypeDef;

typedef enum {
    PARAM_TYPE_U16,
    PARAM_TYPE_I16,
    PARAM_TYPE_U32,
    PARAM_TYPE_I32
} ParamDataTypeDef;

typedef struct {
    uint16_t            addr;
    const char          name[32];
    ParamAccessTypeDef  access;
    ParamDataTypeDef    type;
    int32_t             min;
    int32_t             max;
    const char          unit[8];
} ParamItemTypeDef;

void            JointParam_Init(void);
const ParamItemTypeDef* JointParam_FindByAddr(uint16_t addr);
int32_t         JointParam_Read(uint16_t addr);
uint8_t         JointParam_Write(uint16_t addr, int32_t value);
uint16_t        JointParam_GetCount(void);
const ParamItemTypeDef* JointParam_GetByIndex(uint16_t index);
void            JointParam_LoadDefault(void);

#endif