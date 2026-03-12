#ifndef __MODBUS_PROTOCOL_H__
#define __MODBUS_PROTOCOL_H__

#include "usb_cdc.h"
#include "motor_params.h"

/* Modbus从站地址 */
#define MODBUS_SLAVE_ADDRESS 0x01

/* Modbus功能码 */
#define MODBUS_FC_READ_HOLDING_REGISTERS 0x03  /* 读取保持寄存器 */
#define MODBUS_FC_WRITE_SINGLE_REGISTER 0x06  /* 写入单个保持寄存器 */
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS 0x10  /* 写入多个保持寄存器 */
#define MODBUS_FC_READ_INPUT_REGISTERS 0x04  /* 读取输入寄存器 */

/* Modbus错误码 */
#define MODBUS_ERR_ILLEGAL_FUNCTION 0x01  /* 非法功能 */
#define MODBUS_ERR_ILLEGAL_DATA_ADDRESS 0x02  /* 非法数据地址 */
#define MODBUS_ERR_ILLEGAL_DATA_VALUE 0x03  /* 非法数据值 */
#define MODBUS_ERR_SERVER_DEVICE_FAILURE 0x04  /* 服务器设备故障 */
#define MODBUS_ERR_ACKNOWLEDGE 0x05  /* 确认 */
#define MODBUS_ERR_SERVER_DEVICE_BUSY 0x06  /* 服务器设备忙 */
#define MODBUS_ERR_NEGATIVE_ACKNOWLEDGE 0x07  /* 否定确认 */
#define MODBUS_ERR_MEMORY_PARITY_ERROR 0x08  /* 内存奇偶校验错误 */

/* 寄存器地址映射 */
/* 保持寄存器（可读写） */
#define REG_TARGET_VELOCITY 0x0000  /* 目标速度 */
#define REG_KP_GAIN 0x0002  /* 比例增益 */
#define REG_KI_GAIN 0x0004  /* 积分增益 */
#define REG_MAX_VELOCITY 0x0006  /* 最大速度 */
#define REG_MAX_CURRENT 0x0008  /* 最大电流 */
#define REG_ACCELERATION 0x000A  /* 加速度 */
#define REG_DECELERATION 0x000C  /* 减速度 */

/* 输入寄存器（只读） */
#define REG_ACTUAL_VELOCITY 0x1000  /* 实际速度 */
#define REG_ACTUAL_CURRENT 0x1002  /* 实际电流 */
#define REG_BUS_VOLTAGE 0x1004  /* 母线电压 */
#define REG_TEMPERATURE 0x1006  /* 温度 */
#define REG_ERROR_CODE 0x1008  /* 错误码 */

/* 通信帧结构体 */
typedef struct
{
  uint8_t slave_address;  /* 从站地址 */
  uint8_t function_code;  /* 功能码 */
  uint8_t data[252];      /* 数据 */
  uint16_t crc;           /* CRC校验 */
} modbus_frame_t;

/* 读取保持寄存器请求 */
typedef struct
{
  uint16_t start_address;  /* 起始地址 */
  uint16_t quantity;       /* 数量 */
} modbus_read_holding_registers_request_t;

/* 读取保持寄存器响应 */
typedef struct
{
  uint8_t byte_count;      /* 字节数 */
  uint16_t registers[64];  /* 寄存器值 */
} modbus_read_holding_registers_response_t;

/* 写入单个寄存器请求 */
typedef struct
{
  uint16_t address;        /* 地址 */
  uint16_t value;          /* 值 */
} modbus_write_single_register_request_t;

/* 写入单个寄存器响应 */
typedef struct
{
  uint16_t address;        /* 地址 */
  uint16_t value;          /* 值 */
} modbus_write_single_register_response_t;

/* 写入多个寄存器请求 */
typedef struct
{
  uint16_t start_address;  /* 起始地址 */
  uint16_t quantity;       /* 数量 */
  uint8_t byte_count;      /* 字节数 */
  uint16_t registers[64];  /* 寄存器值 */
} modbus_write_multiple_registers_request_t;

/* 写入多个寄存器响应 */
typedef struct
{
  uint16_t start_address;  /* 起始地址 */
  uint16_t quantity;       /* 数量 */
} modbus_write_multiple_registers_response_t;

/* 异常响应 */
typedef struct
{
  uint8_t exception_code;  /* 异常码 */
} modbus_exception_response_t;

/* 函数声明 */
void modbus_protocol_init(void);
void modbus_protocol_process(void);
void modbus_send_response(uint8_t function_code, void *data, uint16_t data_len);
void modbus_send_exception_response(uint8_t function_code, uint8_t exception_code);

#endif /* __MODBUS_PROTOCOL_H__ */