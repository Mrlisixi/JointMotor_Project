#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "at32f403a_407.h"
#include "can_cia402.h"
#include "motor_params.h"
#include "motor_foc.h"
#include "motor_monitor.h"



/**
  * @brief  初始化电机控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_init(motor_params_t *motor);

/**
  * @brief  处理电机控制逻辑
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_process(motor_params_t *motor);

/**
  * @brief  设置目标位置
  * @param  motor: 电机参数结构体指针
  * @param  position: 目标位置
  * @retval 无
  */
void motor_control_set_target_position(motor_params_t *motor, float position);

/**
  * @brief  设置目标速度
  * @param  motor: 电机参数结构体指针
  * @param  velocity: 目标速度
  * @retval 无
  */
void motor_control_set_target_velocity(motor_params_t *motor, float velocity);

/**
  * @brief  设置目标转矩
  * @param  motor: 电机参数结构体指针
  * @param  torque: 目标转矩
  * @retval 无
  */
void motor_control_set_target_torque(motor_params_t *motor, float torque);

/**
  * @brief  设置控制模式
  * @param  motor: 电机参数结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void motor_control_set_control_mode(motor_params_t *motor, foc_control_mode_t mode);

/**
  * @brief  使能电机控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_enable(motor_params_t *motor);

/**
  * @brief  禁用电机控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_disable(motor_params_t *motor);

/**
  * @brief  获取电机状态
  * @param  motor: 电机参数结构体指针
  * @retval 电机状态
  */
motor_state_t motor_control_get_state(motor_params_t *motor);

/**
  * @brief  处理故障
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_handle_fault(motor_params_t *motor);

/**
  * @brief  更新传感器数据
  * @param  motor: 电机参数结构体指针
  * @param  current_u: U相电流
  * @param  current_v: V相电流
  * @param  current_w: W相电流
  * @param  voltage: 母线电压
  * @param  temperature: 温度
  * @param  position: 位置
  * @param  velocity: 速度
  * @retval 无
  */
void motor_control_update_sensor_data(motor_params_t *motor, float current_u, float current_v, float current_w, float voltage, float temperature, float position, float velocity);

/**
  * @brief  初始化无感控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_sensorless_init(motor_params_t *motor);

/**
  * @brief  使能无感控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_sensorless_enable(motor_params_t *motor);

/**
  * @brief  禁用无感控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_sensorless_disable(motor_params_t *motor);

/**
  * @brief  设置无感控制参数
  * @param  motor: 电机参数结构体指针
  * @param  k1: 参数K1
  * @param  k2: 参数K2
  * @param  k3: 参数K3
  * @retval 无
  */
void motor_control_sensorless_set_params(motor_params_t *motor, float k1, float k2, float k3);

#endif