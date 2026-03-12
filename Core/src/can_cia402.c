/* CAN总线通信Cia402协议模块 */
#include "can_cia402.h"
#include "motor_params.h"
#include <stddef.h>

/**
  * @brief  初始化CIA402设备
  * @param  device: CIA402设备结构体指针
  * @retval none
  */
void cia402_init(motor_ds402_params_t *device)
{
    if (device == NULL) return;
    
    /* 初始化DS402参数 */
    device->control_word = 0;
    device->status_word = 0;
    device->target_position = 0;
    device->target_velocity = 0;
    device->target_torque_percent = 0;
    device->actual_position_inc = 0;
    device->actual_velocity = 0;
    device->actual_torque = 0;
    device->error_code = 0;
    
    /* 初始化控制模式 */
    device->operation_mode = CIA402_MODE_NO_MODE;
    
    /* 初始化状态机状态 */
    device->current_state = CIA402_STATE_START;
}

/**
  * @brief  处理CIA402状态机
  * @param  device: CIA402设备结构体指针
  * @retval none
  */
void cia402_process_state_machine(motor_ds402_params_t *device)
{
    if (device == NULL) return;
    
    /* 根据当前状态和控制字更新状态 */
    switch ((cia402_state_t)device->current_state)
    {
        case CIA402_STATE_START:
            device->current_state = (uint8_t)CIA402_STATE_NOT_READY_TO_SWITCH_ON;
            break;
            
        case CIA402_STATE_NOT_READY_TO_SWITCH_ON:
            if ((device->control_word & CIA402_CTRLWORD_SWITCH_ON) == 0 &&
                (device->control_word & CIA402_CTRLWORD_ENABLE_OPERATION) == 0 &&
                (device->control_word & CIA402_CTRLWORD_QUICK_STOP) != 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_SWITCH_ON_DISABLED;
            }
            break;
            
        case CIA402_STATE_SWITCH_ON_DISABLED:
            if ((device->control_word & CIA402_CTRLWORD_SWITCH_ON) != 0 &&
                (device->control_word & CIA402_CTRLWORD_ENABLE_OPERATION) == 0 &&
                (device->control_word & CIA402_CTRLWORD_QUICK_STOP) != 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_READY_TO_SWITCH_ON;
            }
            break;
            
        case CIA402_STATE_READY_TO_SWITCH_ON:
            if ((device->control_word & CIA402_CTRLWORD_SWITCH_ON) != 0 &&
                (device->control_word & CIA402_CTRLWORD_ENABLE_OPERATION) != 0 &&
                (device->control_word & CIA402_CTRLWORD_QUICK_STOP) != 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_OPERATION_ENABLED;
            }
            else if ((device->control_word & CIA402_CTRLWORD_SWITCH_ON) == 0 &&
                     (device->control_word & CIA402_CTRLWORD_ENABLE_OPERATION) == 0 &&
                     (device->control_word & CIA402_CTRLWORD_QUICK_STOP) != 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_SWITCH_ON_DISABLED;
            }
            break;
            
        case CIA402_STATE_OPERATION_ENABLED:
            if ((device->control_word & CIA402_CTRLWORD_QUICK_STOP) == 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_QUICK_STOP_ACTIVE;
            }
            else if ((device->control_word & CIA402_CTRLWORD_SWITCH_ON) == 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_SWITCHED_ON;
            }
            break;
            
        case CIA402_STATE_SWITCHED_ON:
            if ((device->control_word & CIA402_CTRLWORD_ENABLE_OPERATION) != 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_OPERATION_ENABLED;
            }
            else if ((device->control_word & CIA402_CTRLWORD_SWITCH_ON) == 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_READY_TO_SWITCH_ON;
            }
            break;
            
        case CIA402_STATE_QUICK_STOP_ACTIVE:
            if ((device->control_word & CIA402_CTRLWORD_QUICK_STOP) != 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_READY_TO_SWITCH_ON;
            }
            break;
            
        case CIA402_STATE_FAULT:
            if ((device->control_word & CIA402_CTRLWORD_FAULT_RESET) != 0)
            {
                device->current_state = (uint8_t)CIA402_STATE_SWITCH_ON_DISABLED;
                device->error_code = 0;
            }
            break;
            
        default:
            break;
    }
    
    /* 更新状态字 */
    device->status_word = 0;
    switch ((cia402_state_t)device->current_state)
    {
        case CIA402_STATE_NOT_READY_TO_SWITCH_ON:
            break;
            
        case CIA402_STATE_SWITCH_ON_DISABLED:
            device->status_word |= CIA402_STATUSWORD_SWITCH_ON_DISABLED;
            break;
            
        case CIA402_STATE_READY_TO_SWITCH_ON:
            device->status_word |= CIA402_STATUSWORD_READY_TO_SWITCH_ON;
            device->status_word |= CIA402_STATUSWORD_VOLTAGE_ENABLED;
            break;
            
        case CIA402_STATE_SWITCHED_ON:
            device->status_word |= CIA402_STATUSWORD_SWITCHED_ON;
            device->status_word |= CIA402_STATUSWORD_VOLTAGE_ENABLED;
            break;
            
        case CIA402_STATE_OPERATION_ENABLED:
            device->status_word |= CIA402_STATUSWORD_READY_TO_SWITCH_ON;
            device->status_word |= CIA402_STATUSWORD_SWITCHED_ON;
            device->status_word |= CIA402_STATUSWORD_OPERATION_ENABLED;
            device->status_word |= CIA402_STATUSWORD_VOLTAGE_ENABLED;
            break;
            
        case CIA402_STATE_QUICK_STOP_ACTIVE:
            device->status_word |= CIA402_STATUSWORD_QUICK_STOP;
            device->status_word |= CIA402_STATUSWORD_VOLTAGE_ENABLED;
            break;
            
        case CIA402_STATE_FAULT:
            device->status_word |= CIA402_STATUSWORD_FAULT;
            break;
            
        default:
            break;
    }
}

/**
  * @brief  设置控制字
  * @param  device: CIA402设备结构体指针
  * @param  control_word: 控制字
  * @retval none
  */
void cia402_set_control_word(motor_ds402_params_t *device, uint16_t control_word)
{
    if (device == NULL) return;
    device->control_word = control_word;
}

/**
  * @brief  获取状态字
  * @param  device: CIA402设备结构体指针
  * @retval 状态字
  */
uint16_t cia402_get_status_word(motor_ds402_params_t *device)
{
    if (device == NULL) return 0;
    return device->status_word;
}

/**
  * @brief  设置目标位置
  * @param  device: CIA402设备结构体指针
  * @param  position: 目标位置
  * @retval none
  */
void cia402_set_target_position(motor_ds402_params_t *device, int32_t position)
{
    if (device == NULL) return;
    device->target_position = position;
}

/**
  * @brief  设置目标速度
  * @param  device: CIA402设备结构体指针
  * @param  velocity: 目标速度
  * @retval none
  */
void cia402_set_target_velocity(motor_ds402_params_t *device, int32_t velocity)
{
    if (device == NULL) return;
    device->target_velocity = velocity;
}

/**
  * @brief  设置目标转矩
  * @param  device: CIA402设备结构体指针
  * @param  torque: 目标转矩
  * @retval none
  */
void cia402_set_target_torque(motor_ds402_params_t *device, int16_t torque)
{
    if (device == NULL) return;
    device->target_torque_percent = torque;
}

/**
  * @brief  设置控制模式
  * @param  device: CIA402设备结构体指针
  * @param  mode: 控制模式
  * @retval none
  */
void cia402_set_control_mode(motor_ds402_params_t *device, cia402_mode_t mode)
{
    if (device == NULL) return;
    device->operation_mode = mode;
}

/**
  * @brief  获取控制模式
  * @param  device: CIA402设备结构体指针
  * @retval 控制模式
  */
cia402_mode_t cia402_get_control_mode(motor_ds402_params_t *device)
{
    if (device == NULL) return CIA402_MODE_NO_MODE;
    return (cia402_mode_t)device->operation_mode;
}

/**
  * @brief  处理CIA402协议
  * @param  device: CIA402设备结构体指针
  * @retval none
  */
void cia402_process(motor_ds402_params_t *device)
{
    if (device == NULL) return;
    cia402_process_state_machine(device);
}

/**
  * @brief  获取当前状态
  * @param  device: CIA402设备结构体指针
  * @retval 当前状态
  */
cia402_state_t cia402_get_state(motor_ds402_params_t *device)
{
    if (device == NULL) return CIA402_STATE_START;
    return (cia402_state_t)device->current_state;
}