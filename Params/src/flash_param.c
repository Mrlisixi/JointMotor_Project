#include "flash_param.h"
#include "joint_params.h"
#include "at32f403a_407_flash.h"

void Flash_LoadParamsFromFlash(void)
{
    uint32_t *pFlash = (uint32_t *)PARAM_FLASH_BASE;

    for (uint16_t i = 0; i < PARAM_TOTAL_COUNT; i++)
    {
        int32_t val = (int32_t)pFlash[i];
        const ParamItemTypeDef *item = JointParam_GetByIndex(i);

        if (item && val >= item->min && val <= item->max)
        {
            JointParam_Write(item->addr, val);
        }
    }
}

void Flash_SaveParamsToFlash(void)
{
    flash_unlock();
    flash_sector_erase(PARAM_FLASH_BASE);

    for (uint16_t i = 0; i < PARAM_TOTAL_COUNT; i++)
    {
        uint32_t addr = PARAM_FLASH_BASE + i * 4;
        int32_t val = JointParam_Read(JointParam_GetByIndex(i)->addr);
        flash_word_program(addr, (uint32_t)val);
    }
    flash_lock();
}

void Flash_RestoreFactory(void)
{
    JointParam_LoadDefault();
    Flash_SaveParamsToFlash();
}