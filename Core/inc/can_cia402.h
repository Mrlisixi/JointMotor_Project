/* CAN总线通信Cia402协议模块 */
#ifndef CAN_CIA402_H
#define CAN_CIA402_H

#include "at32f403a_407.h"

/* 前向声明 */
typedef struct motor_ds402_params_t motor_ds402_params_t;

/* 控制模式枚举 */
typedef enum {
    CIA402_MODE_NO_MODE = 0,
    CIA402_MODE_PROFILE_POSITION,
    CIA402_MODE_PROFILE_VELOCITY,
    CIA402_MODE_PROFILE_TORQUE,
    CIA402_MODE_HOMING,
    CIA402_MODE_INTERPOLATED_POSITION,
    CIA402_MODE_CYCLIC_SYNC_POSITION,
    CIA402_MODE_CYCLIC_SYNC_VELOCITY,
    CIA402_MODE_CYCLIC_SYNC_TORQUE
} cia402_mode_t;

/**
  * @brief  CIA402协议状态机状态
  */
typedef enum {
    CIA402_STATE_START = 0,                   /* 初始状态 */
    CIA402_STATE_NOT_READY_TO_SWITCH_ON,      /* 未准备好切换到ON状态 */
    CIA402_STATE_SWITCH_ON_DISABLED,           /* 切换到ON状态被禁用 */
    CIA402_STATE_READY_TO_SWITCH_ON,           /* 准备好切换到ON状态 */
    CIA402_STATE_SWITCHED_ON,                 /* 已切换到ON状态 */
    CIA402_STATE_OPERATION_ENABLED,           /* 操作已启用 */
    CIA402_STATE_QUICK_STOP_ACTIVE,            /* 快速停止激活 */
    CIA402_STATE_FAULT_REACTION_ACTIVE,        /* 故障反应激活 */
    CIA402_STATE_FAULT,                       /* 故障状态 */
    CIA402_STATE_MAX                          /* 状态最大值 */
} cia402_state_t;

/**
  * @brief  CIA402控制字位定义
  */
#define CIA402_CTRLWORD_SWITCH_ON            (0x0001)    /* 切换到ON状态 */
#define CIA402_CTRLWORD_ENABLE_OPERATION     (0x0002)    /* 启用操作 */
#define CIA402_CTRLWORD_QUICK_STOP           (0x0004)    /* 快速停止 */
#define CIA402_CTRLWORD_FAULT_RESET          (0x0080)    /* 故障复位 */
#define CIA402_CTRLWORD_HALT                 (0x0100)    /* 停止 */

/**
  * @brief  CIA402状态字位定义
  */
#define CIA402_STATUSWORD_READY_TO_SWITCH_ON     (0x0001)    /* 准备好切换到ON状态 */
#define CIA402_STATUSWORD_SWITCHED_ON            (0x0002)    /* 已切换到ON状态 */
#define CIA402_STATUSWORD_OPERATION_ENABLED      (0x0004)    /* 操作已启用 */
#define CIA402_STATUSWORD_FAULT                  (0x0008)    /* 故障 */
#define CIA402_STATUSWORD_VOLTAGE_ENABLED        (0x0010)    /* 电压已启用 */
#define CIA402_STATUSWORD_QUICK_STOP             (0x0020)    /* 快速停止 */
#define CIA402_STATUSWORD_SWITCH_ON_DISABLED     (0x0040)    /* 切换到ON状态被禁用 */
#define CIA402_STATUSWORD_WARNING                (0x0080)    /* 警告 */
#define CIA402_STATUSWORD_REMOTE                 (0x0200)    /* 远程控制 */
#define CIA402_STATUSWORD_TARGET_REACHED         (0x0400)    /* 已到达目标 */
#define CIA402_STATUSWORD_INTERNAL_LIMIT         (0x0800)    /* 内部限制 */
#define CIA402_STATUSWORD_OPTIONAL_ERROR         (0x1000)    /* 可选错误 */
#define CIA402_STATUSWORD_MANUFACTURER_SPECIFIC  (0x2000)    /* 制造商特定 */

/**
  * @brief  初始化CIA402设备
  * @param  device: DS402参数结构体指针
  * @retval 无
  */
void cia402_init(motor_ds402_params_t *device);

/**
  * @brief  处理CIA402协议
  * @param  device: DS402参数结构体指针
  * @retval 无
  */
void cia402_process(motor_ds402_params_t *device);

/**
  * @brief  处理CIA402状态机
  * @param  device: DS402参数结构体指针
  * @retval 无
  */
void cia402_process_state_machine(motor_ds402_params_t *device);

/**
  * @brief  设置控制字
  * @param  device: DS402参数结构体指针
  * @param  control_word: 控制字
  * @retval 无
  */
void cia402_set_control_word(motor_ds402_params_t *device, uint16_t control_word);

/**
  * @brief  获取状态字
  * @param  device: DS402参数结构体指针
  * @retval 状态字
  */
uint16_t cia402_get_status_word(motor_ds402_params_t *device);

/**
  * @brief  设置目标位置
  * @param  device: DS402参数结构体指针
  * @param  position: 目标位置
  * @retval 无
  */
void cia402_set_target_position(motor_ds402_params_t *device, int32_t position);

/**
  * @brief  设置目标速度
  * @param  device: DS402参数结构体指针
  * @param  velocity: 目标速度
  * @retval 无
  */
void cia402_set_target_velocity(motor_ds402_params_t *device, int32_t velocity);

/**
  * @brief  设置目标转矩
  * @param  device: DS402参数结构体指针
  * @param  torque: 目标转矩
  * @retval 无
  */
void cia402_set_target_torque(motor_ds402_params_t *device, int16_t torque);

/**
  * @brief  设置控制模式
  * @param  device: DS402参数结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void cia402_set_control_mode(motor_ds402_params_t *device, cia402_mode_t mode);

/**
  * @brief  获取当前状态
  * @param  device: DS402参数结构体指针
  * @retval 状态
  */
cia402_state_t cia402_get_state(motor_ds402_params_t *device);

/**
  * @brief  获取当前控制模式
  * @param  device: DS402参数结构体指针
  * @retval 控制模式
  */
cia402_mode_t cia402_get_control_mode(motor_ds402_params_t *device);

#endif /* CAN_CIA402_H */