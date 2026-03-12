#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "at32f403a_407.h"
#include "motor_params.h"



/**
  * @brief  初始化FOC控制参数
  * @param  foc: FOC控制参数结构体指针
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void foc_init(motor_foc_params_t *foc, motor_params_t *params);

/**
  * @brief  设置FOC控制模式
  * @param  foc: FOC控制参数结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void foc_set_control_mode(motor_foc_params_t *foc, foc_control_mode_t mode);

/**
  * @brief  设置目标位置
  * @param  foc: FOC控制参数结构体指针
  * @param  position: 目标位置
  * @retval 无
  */
void foc_set_target_position(motor_foc_params_t *foc, float position);

/**
  * @brief  设置目标速度
  * @param  foc: FOC控制参数结构体指针
  * @param  velocity: 目标速度
  * @retval 无
  */
void foc_set_target_velocity(motor_foc_params_t *foc, float velocity);

/**
  * @brief  设置目标转矩
  * @param  foc: FOC控制参数结构体指针
  * @param  torque: 目标转矩
  * @retval 无
  */
void foc_set_target_torque(motor_foc_params_t *foc, float torque);

/**
  * @brief  设置PID参数
  * @param  foc: FOC控制参数结构体指针
  * @param  kp_pos: 位置环比例系数
  * @param  ki_pos: 位置环积分系数
  * @param  kp_vel: 速度环比例系数
  * @param  ki_vel: 速度环积分系数
  * @param  kp_current: 电流环比例系数
  * @param  ki_current: 电流环积分系数
  * @retval 无
  */
void foc_set_pid_gains(motor_foc_params_t *foc, float kp_pos, float ki_pos, float kp_vel, float ki_vel, float kp_current, float ki_current);

/**
  * @brief  更新电机角度
  * @param  foc: FOC控制参数结构体指针
  * @param  mechanical_angle: 机械角度
  * @param  pole_pairs: 极对数
  * @retval 无
  */
void foc_update_angle(motor_foc_params_t *foc, float mechanical_angle, int pole_pairs);


/**
  * @brief  处理FOC控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_process(motor_foc_params_t *foc);

/**
  * @brief  使能FOC控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_enable(motor_foc_params_t *foc);

/**
  * @brief  禁用FOC控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_disable(motor_foc_params_t *foc);

/**
  * @brief  初始化无感控制
  * @param  foc: FOC控制参数结构体指针
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void foc_sensorless_init(motor_foc_params_t *foc, motor_params_t *params);

/**
  * @brief  使能无感控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_sensorless_enable(motor_foc_params_t *foc);

/**
  * @brief  禁用无感控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_sensorless_disable(motor_foc_params_t *foc);

/**
  * @brief  更新无感控制参数
  * @param  foc: FOC控制参数结构体指针
  * @param  k1: 参数K1
  * @param  k2: 参数K2
  * @param  k3: 参数K3
  * @retval 无
  */
void foc_sensorless_set_params(motor_foc_params_t *foc, float k1, float k2, float k3);

/**
  * @brief  估算电机角度和速度
  * @param  foc: FOC控制参数结构体指针
  * @param  bus_voltage: 母线电压
  * @retval 无
  */
void foc_sensorless_estimate(motor_foc_params_t *foc, float bus_voltage);

/**
  * @brief  处理FOC控制（带无感控制）
  * @param  foc: FOC控制参数结构体指针
  * @param  bus_voltage: 母线电压
  * @retval 无
  */
void foc_process_with_sensorless(motor_foc_params_t *foc, float bus_voltage);

#endif