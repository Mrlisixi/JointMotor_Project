#ifndef __FLASH_PARAM_H
#define __FLASH_PARAM_H

#include "stdint.h"

// AT32F407 1024KB 最后扇区，绝对安全
#define PARAM_FLASH_BASE    0x080E0000UL

void Flash_LoadParamsFromFlash(void);
void Flash_SaveParamsToFlash(void);
void Flash_RestoreFactory(void);

#endif