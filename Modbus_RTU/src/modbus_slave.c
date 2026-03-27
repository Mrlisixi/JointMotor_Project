#include "modbus_slave.h"
#include "joint_params.h"
#include "flash_param.h"
#include "cdc_class.h"
#include "string.h"

// 外部声明
uint32_t wk_timebase_get(void);
extern usbd_core_type usb_core_dev;   // 你工程的USB句柄
extern uint8_t usbd_app_buffer_fs1[]; // 你官方的发送缓冲区

#define MODBUS_RX_BUF_SIZE  128
#define MODBUS_TIMEOUT      5

static uint8_t  g_slaveAddr      = 1;
static uint8_t  g_rxBuffer[MODBUS_RX_BUF_SIZE];
static uint16_t g_rxIndex        = 0;
static uint32_t g_lastRxTime     = 0;

static uint16_t Modbus_CRC16(uint8_t* buf, uint16_t len);
static void Modbus_SendReply(uint8_t *buf, uint16_t len);

// ===================== 初始化 =====================
void Modbus_Init(uint8_t slaveAddr) {
    g_slaveAddr = slaveAddr;
    g_rxIndex = 0;
    memset(g_rxBuffer, 0, MODBUS_RX_BUF_SIZE);
}

// ===================== 外部喂数据（USB收到后调用）=====================
void Modbus_FeedData(uint8_t *data, uint16_t len)
{
    for(uint16_t i=0; i<len; i++)
    {
        if(g_rxIndex >= MODBUS_RX_BUF_SIZE)
        {
            g_rxIndex = 0;
        }
        g_rxBuffer[g_rxIndex++] = data[i];
    }
    g_lastRxTime = wk_timebase_get();
}

// ===================== 解析帧（只在USB任务里调用）=====================
// ===================== 解析帧（只在USB任务里调用）=====================
void Modbus_ParseFrame(void)
{
    // 不够一帧，直接返回
    if (g_rxIndex < 8) return;
    
    // 帧间隔超时
    // if (wk_timebase_get() - g_lastRxTime < MODBUS_TIMEOUT) return;

    // CRC 校验
    uint16_t crc = Modbus_CRC16(g_rxBuffer, g_rxIndex - 2);
    uint8_t crcLo = crc & 0xFF;
    uint8_t crcHi = (crc >> 8) & 0xFF;

    if (g_rxBuffer[g_rxIndex - 2] != crcLo || g_rxBuffer[g_rxIndex - 1] != crcHi)
    {
        g_rxIndex = 0;
        return;
    }

    // 从站地址不匹配
    if (g_rxBuffer[0] != g_slaveAddr)
    {
        g_rxIndex = 0;
        return;
    }

    uint8_t func     = g_rxBuffer[1];
    uint16_t regAddr = (g_rxBuffer[2] << 8) | g_rxBuffer[3];
    uint16_t regNum  = (g_rxBuffer[4] << 8) | g_rxBuffer[5];

    // ===================== 0x01 读线圈 =====================
    if (func == 0x01)
    {
        uint8_t resp[64];
        uint8_t byteCnt = (regNum + 7) / 8;
        uint8_t respLen = 3 + byteCnt;

        resp[0] = g_slaveAddr;
        resp[1] = 0x01;
        resp[2] = byteCnt;

        // 这里填充你自己的线圈数据（示例：全0）
        for (uint8_t i = 0; i < byteCnt; i++)
            resp[3 + i] = 0x00;

        uint16_t respCrc = Modbus_CRC16(resp, respLen - 2);
        resp[respLen - 2] = respCrc & 0xFF;
        resp[respLen - 1] = (respCrc >> 8) & 0xFF;

        Modbus_SendReply(resp, respLen);
    }

    // ===================== 0x02 读离散输入 =====================
    else if (func == 0x02)
    {
        uint8_t resp[64];
        uint8_t byteCnt = (regNum + 7) / 8;
        uint8_t respLen = 3 + byteCnt;

        resp[0] = g_slaveAddr;
        resp[1] = 0x02;
        resp[2] = byteCnt;

        // 这里填充你自己的离散输入（示例：全0）
        for (uint8_t i = 0; i < byteCnt; i++)
            resp[3 + i] = 0x00;

        uint16_t respCrc = Modbus_CRC16(resp, respLen - 2);
        resp[respLen - 2] = respCrc & 0xFF;
        resp[respLen - 1] = (respCrc >> 8) & 0xFF;

        Modbus_SendReply(resp, respLen);
    }

    // ===================== 0x03 读保持寄存器 =====================
    else if (func == 0x03)
    {
        uint8_t resp[MODBUS_RX_BUF_SIZE];
        uint8_t respLen = 5 + 2 * regNum;

        resp[0] = g_slaveAddr;
        resp[1] = 0x03;
        resp[2] = regNum * 2;

        for (uint16_t i = 0; i < regNum; i++)
        {
            int32_t val = JointParam_Read(regAddr + i);
            resp[3 + 2 * i] = (val >> 8) & 0xFF;
            resp[4 + 2 * i] = val & 0xFF;
        }

        uint16_t respCrc = Modbus_CRC16(resp, respLen - 2);
        resp[respLen - 2] = respCrc & 0xFF;
        resp[respLen - 1] = (respCrc >> 8) & 0xFF;

        Modbus_SendReply(resp, respLen);
    }

    // ===================== 0x05 写单个线圈 =====================
    else if (func == 0x05)
    {
        // 原帧返回
        Modbus_SendReply(g_rxBuffer, 8);
    }

    // ===================== 0x06 写单个寄存器 =====================
    else if (func == 0x06)
    {
        uint16_t writeVal = (g_rxBuffer[4] << 8) | g_rxBuffer[5];
        JointParam_Write(regAddr, writeVal);
        Flash_SaveParamsToFlash();

        // 原帧返回
        Modbus_SendReply(g_rxBuffer, 8);
    }

    // ===================== 0x10 写多个寄存器 =====================
    else if (func == 0x10 && g_rxIndex >= 11)
    {
        for (uint16_t i = 0; i < regNum; i++)
        {
            int32_t val = (g_rxBuffer[7 + 2 * i] << 8) | g_rxBuffer[8 + 2 * i];
            JointParam_Write(regAddr + i, val);
        }
        Flash_SaveParamsToFlash();

        uint8_t resp[8];
        resp[0] = g_slaveAddr;
        resp[1] = 0x10;
        resp[2] = g_rxBuffer[2]; resp[3] = g_rxBuffer[3];
        resp[4] = g_rxBuffer[4]; resp[5] = g_rxBuffer[5];
        
        uint16_t respCrc = Modbus_CRC16(resp, 6);
        resp[6] = respCrc & 0xFF;
        resp[7] = (respCrc >> 8) & 0xFF;

        Modbus_SendReply(resp, 8);
    }

    // 处理完成，清空缓冲区
    g_rxIndex = 0;
}

// ===================== USB 发送（使用你官方发送函数）=====================
static void Modbus_SendReply(uint8_t *buf, uint16_t len)
{
    if(len == 0 || len > 64) return;

    // 复制到官方发送缓冲区
    memcpy(usbd_app_buffer_fs1, buf, len);

    // 你官方的带超时发送
    uint32_t timeout = 500000;
    do{
        if(usb_vcp_send_data(&usb_core_dev, usbd_app_buffer_fs1, len) == SUCCESS){
            break;
        }
    }while(timeout--);
}

// ===================== CRC16 =====================
static uint16_t Modbus_CRC16(uint8_t* buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}