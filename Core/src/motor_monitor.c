#include "motor_monitor.h"
#include <stddef.h>
#include <math.h>
#include <stdio.h>

/**
  * @brief  初始化电机监控
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_monitor_init(motor_params_t *params) {
    if (params == NULL) return;
    
    // 初始化监控参数
    params->monitor.u_phase_current = 0.0;
    params->monitor.v_phase_current = 0.0;
    params->monitor.w_phase_current = 0.0;
    params->monitor.bus_voltage = 0.0;
    params->monitor.temperature = 0.0;
    params->monitor.sample_freq = (uint32_t)motor_params_get(params, PARAM_ID_MONITOR_SAMPLE_FREQ);
    
    // 从参数字典获取保护参数
    params->protection.max_current = motor_params_get(params, PARAM_ID_MAX_CURRENT_PROT);
    params->protection.min_voltage = motor_params_get(params, PARAM_ID_MIN_VOLTAGE_PROT);
    params->protection.max_voltage = motor_params_get(params, PARAM_ID_MAX_VOLTAGE_PROT);
    params->protection.max_temperature = motor_params_get(params, PARAM_ID_MAX_TEMPERATURE_PROT);
    params->protection.overcurrent_prot = (uint8_t)motor_params_get(params, PARAM_ID_OVERCURRENT_PROT);
    params->protection.undervolt_prot = (uint8_t)motor_params_get(params, PARAM_ID_UNDERVOLT_PROT);
    params->protection.overvolt_prot = (uint8_t)motor_params_get(params, PARAM_ID_OVERVOLT_PROT);
    params->protection.overtemperature_prot = (uint8_t)motor_params_get(params, PARAM_ID_OVERTEMPERATURE_PROT);
    params->protection.fault_status = 0;
    params->protection.fault_active = 0;
}

/**
  * @brief  设置监控限制值
  * @param  params: 电机参数结构体指针
  * @param  max_current: 最大电流
  * @param  min_voltage: 最小电压
  * @param  max_voltage: 最大电压
  * @param  max_temperature: 最大温度
  * @retval 无
  */
void motor_monitor_set_limits(motor_params_t *params, float max_current, float min_voltage, float max_voltage, float max_temperature) {
    if (params == NULL) return;
    
    params->protection.max_current = max_current;
    params->protection.min_voltage = min_voltage;
    params->protection.max_voltage = max_voltage;
    params->protection.max_temperature = max_temperature;
}

/**
  * @brief  使能保护功能
  * @param  params: 电机参数结构体指针
  * @param  overcurrent: 过流保护使能
  * @param  undervolt: 欠压保护使能
  * @param  overvolt: 过压保护使能
  * @param  overtemperature: 过热保护使能
  * @retval 无
  */
void motor_monitor_enable_protection(motor_params_t *params, uint8_t overcurrent, uint8_t undervolt, uint8_t overvolt, uint8_t overtemperature) {
    if (params == NULL) return;
    
    params->protection.overcurrent_prot = overcurrent;
    params->protection.undervolt_prot = undervolt;
    params->protection.overvolt_prot = overvolt;
    params->protection.overtemperature_prot = overtemperature;
}

/**
  * @brief  更新电机电流
  * @param  params: 电机参数结构体指针
  * @param  current_a: A相电流
  * @param  current_b: B相电流
  * @param  current_c: C相电流
  * @retval 无
  */
void motor_monitor_update_current(motor_params_t *params, float current_u, float current_v, float current_w) {
    if (params == NULL) return;
    
    params->monitor.u_phase_current = current_u;
    params->monitor.v_phase_current = current_v;
    params->monitor.w_phase_current = current_w;
}

/**
  * @brief  更新母线电压
  * @param  params: 电机参数结构体指针
  * @param  bus_voltage: 母线电压
  * @retval 无
  */
void motor_monitor_update_voltage(motor_params_t *params, float bus_voltage) {
    if (params == NULL) return;
    
    params->monitor.bus_voltage = bus_voltage;
}

/**
  * @brief  更新温度
  * @param  params: 电机参数结构体指针
  * @param  temperature: 温度
  * @retval 无
  */
void motor_monitor_update_temperature(motor_params_t *params, float temperature) {
    if (params == NULL) return;
    
    params->monitor.temperature = temperature;
}

/**
  * @brief  处理监控逻辑
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_monitor_process(motor_params_t *params) {
    if (params == NULL) return;
    
    uint8_t fault_detected = 0;
    
    if (params->protection.overcurrent_prot) {
        float max_current = (params->monitor.u_phase_current > params->monitor.v_phase_current) ? params->monitor.u_phase_current : params->monitor.v_phase_current;
        max_current = (max_current > params->monitor.w_phase_current) ? max_current : params->monitor.w_phase_current;
        if (max_current > params->protection.max_current) {
            params->protection.fault_status = FAULT_OVERCURRENT;
            params->protection.fault_active = 1;
            fault_detected = 1;
        }
    }
    
    if (!fault_detected && params->protection.undervolt_prot) {
        if (params->monitor.bus_voltage < params->protection.min_voltage) {
            params->protection.fault_status = FAULT_UNDERVOLTAGE;
            params->protection.fault_active = 1;
            fault_detected = 1;
        }
    }
    
    if (!fault_detected && params->protection.overvolt_prot) {
        if (params->monitor.bus_voltage > params->protection.max_voltage) {
            params->protection.fault_status = FAULT_OVERVOLTAGE;
            params->protection.fault_active = 1;
            fault_detected = 1;
        }
    }
    
    if (!fault_detected && params->protection.overtemperature_prot) {
        if (params->monitor.temperature > params->protection.max_temperature) {
            params->protection.fault_status = FAULT_OVERTEMPERATURE;
            params->protection.fault_active = 1;
            fault_detected = 1;
        }
    }
    
    if (!fault_detected) {
        params->protection.fault_status = FAULT_NONE;
        params->protection.fault_active = 0;
    }
}

/**
  * @brief  获取故障类型
  * @param  params: 电机参数结构体指针
  * @retval 故障类型
  */
fault_type_t motor_monitor_get_fault(motor_params_t *params) {
    if (params == NULL) return FAULT_NONE;
    return (fault_type_t)params->protection.fault_status;
}

/**
  * @brief  检查是否有故障激活
  * @param  params: 电机参数结构体指针
  * @retval 1表示有故障激活，0表示无故障
  */
uint8_t motor_monitor_is_fault_active(motor_params_t *params) {
    if (params == NULL) return 0;
    return params->protection.fault_active;
}

/**
  * @brief  清除故障
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_monitor_clear_fault(motor_params_t *params) {
    if (params == NULL) return;
    params->protection.fault_status = FAULT_NONE;
    params->protection.fault_active = 0;
}

/**
  * @brief  获取A相电流
  * @param  params: 电机参数结构体指针
  * @retval A相电流值
  */
float motor_monitor_get_current_a(motor_params_t *params) {
    if (params == NULL) return 0.0f;
    return params->monitor.u_phase_current;
}

/**
  * @brief  获取B相电流
  * @param  params: 电机参数结构体指针
  * @retval B相电流值
  */
float motor_monitor_get_current_b(motor_params_t *params) {
    if (params == NULL) return 0.0f;
    return params->monitor.v_phase_current;
}

/**
  * @brief  获取C相电流
  * @param  params: 电机参数结构体指针
  * @retval C相电流值
  */
float motor_monitor_get_current_c(motor_params_t *params) {
    if (params == NULL) return 0.0f;
    return params->monitor.w_phase_current;
}

/**
  * @brief  获取母线电压
  * @param  params: 电机参数结构体指针
  * @retval 母线电压值
  */
float motor_monitor_get_bus_voltage(motor_params_t *params) {
    if (params == NULL) return 0.0f;
    return params->monitor.bus_voltage;
}

/**
  * @brief  获取温度
  * @param  params: 电机参数结构体指针
  * @retval 温度值
  */
float motor_monitor_get_temperature(motor_params_t *params) {
    if (params == NULL) return 0.0f;
    return params->monitor.temperature;
}

/**
  * @brief  将ADC值转换为电流值
  * @param  params: 电机参数结构体指针
  * @param  adc_value: ADC原始值
  * @retval 电流值（安培）
  */
float motor_monitor_convert_adc_to_current(motor_params_t *params, uint16_t adc_value) {
    if (params == NULL) return 0.0f;
    
    /* ADC参考电压为3.3V，12位ADC */
    /* 电流转换：使用1mΩ分流电阻，RS724XQ放大增益为20 */
    float adc_ref_voltage = 3.3f;
    float adc_max_value = 4095.0f;
    float shunt_resistor = 0.001f; // 分流电阻 (1mΩ)
    float amp_gain = 20.0f; // RS724XQ放大增益
    float zero_current_voltage = 1.65f; // 零电流时的电压 (Vcc/2)
    
    float voltage = adc_value * adc_ref_voltage / adc_max_value;
    float current = (voltage - zero_current_voltage) / (shunt_resistor * amp_gain);
    
    // 添加调试信息
    static uint32_t counter = 0;
    if (counter % 100 == 0) {
        printf("ADC value: %d, Voltage: %.3fV, Current: %.3fA\r\n", adc_value, voltage, current);
    }
    counter++;
    
    return current;
}

/**
  * @brief  将ADC值转换为电压值
  * @param  params: 电机参数结构体指针
  * @param  adc_value: ADC原始值
  * @retval 电压值（伏特）
  */
float motor_monitor_convert_adc_to_voltage(motor_params_t *params, uint16_t adc_value) {
    if (params == NULL) return 0.0f;
    
    /* ADC参考电压为3.3V，12位ADC */
    /* 电压转换：使用18K:1K电阻分压 */
    float adc_ref_voltage = 3.3f;
    float adc_max_value = 4095.0f;
    float r1 = 18000.0f; // 分压电阻1
    float r2 = 1000.0f; // 分压电阻2
    float vbus_ratio = (r1 + r2) / r2; // 分压比
    
    float voltage = adc_value * adc_ref_voltage / adc_max_value * vbus_ratio;
    return voltage;
}

/**
  * @brief  将ADC值转换为温度值
  * @param  params: 电机参数结构体指针
  * @param  adc_value: ADC原始值
  * @retval 温度值（摄氏度）
  */
float motor_monitor_convert_adc_to_temperature(motor_params_t *params, uint16_t adc_value) {
    if (params == NULL) return 0.0f;
    
    /* ADC参考电压为3.3V，12位ADC */
    /* 温度转换：使用KNTC0603/10KF3950热敏电阻与3.3K电阻分压 */
    float adc_ref_voltage = 3.3f;
    float adc_max_value = 4095.0f;
    float r_fixed = 3300.0f; // 固定电阻
    
    float adc_voltage_temp = adc_value * adc_ref_voltage / adc_max_value;
    float r_ntc = r_fixed * (adc_ref_voltage - adc_voltage_temp) / adc_voltage_temp; // 计算NTC电阻值
    
    /* 使用B值公式计算温度 */
    float b_value = 3950.0f; // KNT3950的B值
    float r_ref = 10000.0f; // 25℃时的参考电阻
    float t_ref = 298.15f; // 25℃的绝对温度(K)
    
    float inv_t = 1.0f / t_ref + (1.0f / b_value) * log(r_ntc / r_ref);
    float temperature = (1.0f / inv_t) - 273.15f; // 转换为摄氏度
    
    return temperature;
}