#ifndef MOTOR_MONITOR_H
#define MOTOR_MONITOR_H

#include "at32f403a_407.h"
#include "motor_params.h"



/**
  * @brief  故障类型枚举
  */
typedef enum {
    FAULT_NONE = 0,            /* 无故障 */
    FAULT_OVERCURRENT = 1,     /* 过流故障 */
    FAULT_UNDERVOLTAGE = 2,    /* 欠压故障 */
    FAULT_OVERVOLTAGE = 3,     /* 过压故障 */
    FAULT_OVERTEMPERATURE = 4, /* 过热故障 */
    FAULT_OVERLOAD = 5         /* 过载故障 */
} fault_type_t;

/**
  * @brief  初始化电机监控
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_monitor_init(motor_params_t *params);

/**
  * @brief  设置监控限制值
  * @param  params: 电机参数结构体指针
  * @param  max_current: 最大电流
  * @param  min_voltage: 最小电压
  * @param  max_voltage: 最大电压
  * @param  max_temperature: 最大温度
  * @retval 无
  */
void motor_monitor_set_limits(motor_params_t *params, float max_current, float min_voltage, float max_voltage, float max_temperature);

/**
  * @brief  使能保护功能
  * @param  params: 电机参数结构体指针
  * @param  overcurrent: 过流保护使能
  * @param  undervolt: 欠压保护使能
  * @param  overvolt: 过压保护使能
  * @param  overtemperature: 过热保护使能
  * @retval 无
  */
void motor_monitor_enable_protection(motor_params_t *params, uint8_t overcurrent, uint8_t undervolt, uint8_t overvolt, uint8_t overtemperature);

/**
  * @brief  更新电机电流
  * @param  params: 电机参数结构体指针
  * @param  current_u: U相电流
  * @param  current_v: V相电流
  * @param  current_w: W相电流
  * @retval 无
  */
void motor_monitor_update_current(motor_params_t *params, float current_u, float current_v, float current_w);

/**
  * @brief  更新母线电压
  * @param  params: 电机参数结构体指针
  * @param  bus_voltage: 母线电压
  * @retval 无
  */
void motor_monitor_update_voltage(motor_params_t *params, float bus_voltage);

/**
  * @brief  更新温度
  * @param  params: 电机参数结构体指针
  * @param  temperature: 温度
  * @retval 无
  */
void motor_monitor_update_temperature(motor_params_t *params, float temperature);

/**
  * @brief  处理监控逻辑
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_monitor_process(motor_params_t *params);

/**
  * @brief  获取故障类型
  * @param  params: 电机参数结构体指针
  * @retval 故障类型
  */
fault_type_t motor_monitor_get_fault(motor_params_t *params);

/**
  * @brief  检查是否有故障激活
  * @param  params: 电机参数结构体指针
  * @retval 1表示有故障激活，0表示无故障
  */
uint8_t motor_monitor_is_fault_active(motor_params_t *params);

/**
  * @brief  清除故障
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_monitor_clear_fault(motor_params_t *params);

/**
  * @brief  获取A相电流
  * @param  params: 电机参数结构体指针
  * @retval A相电流值
  */
float motor_monitor_get_current_a(motor_params_t *params);

/**
  * @brief  获取B相电流
  * @param  params: 电机参数结构体指针
  * @retval B相电流值
  */
float motor_monitor_get_current_b(motor_params_t *params);

/**
  * @brief  获取C相电流
  * @param  params: 电机参数结构体指针
  * @retval C相电流值
  */
float motor_monitor_get_current_c(motor_params_t *params);

/**
  * @brief  获取母线电压
  * @param  params: 电机参数结构体指针
  * @retval 母线电压值
  */
float motor_monitor_get_bus_voltage(motor_params_t *params);

/**
  * @brief  获取温度
  * @param  params: 电机参数结构体指针
  * @retval 温度值
  */
float motor_monitor_get_temperature(motor_params_t *params);

/* ADC转换函数 */

/**
  * @brief  ADC值转换为电流
  * @param  params: 电机参数结构体指针
  * @param  adc_value: ADC值
  * @retval 电流值
  */
float motor_monitor_convert_adc_to_current(motor_params_t *params, uint16_t adc_value);

/**
  * @brief  ADC值转换为电压
  * @param  params: 电机参数结构体指针
  * @param  adc_value: ADC值
  * @retval 电压值
  */
float motor_monitor_convert_adc_to_voltage(motor_params_t *params, uint16_t adc_value);

/**
  * @brief  ADC值转换为温度
  * @param  params: 电机参数结构体指针
  * @param  adc_value: ADC值
  * @retval 温度值
  */
float motor_monitor_convert_adc_to_temperature(motor_params_t *params, uint16_t adc_value);

#endif