#include "motor_control.h"
#include "can_cia402.h"
#include "motor_params.h"
#include "motor_monitor.h"
#include <stddef.h>

/**
  * @brief  初始化电机控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_init(motor_params_t *motor) {
    float max_current, min_voltage, max_voltage, max_temperature;
    uint8_t overcurrent_prot, undervolt_prot, overvolt_prot, overtemperature_prot;
    float kp_pos, ki_pos, kp_vel, ki_vel, kp_current, ki_current;
    
    if (motor == NULL) return;
    
    motor->state = MOTOR_STATE_IDLE;
    motor->initialized = 0;
    
    cia402_init(&motor->ds402);
    motor_params_load_default(motor);
    foc_init(&motor->foc, motor);
    motor_monitor_init(motor);
    
    /* 从参数字典获取保护参数 */
    max_current = motor_params_get(motor, PARAM_ID_MAX_CURRENT);
    min_voltage = motor_params_get(motor, PARAM_ID_MIN_VOLTAGE_PROT);
    max_voltage = motor_params_get(motor, PARAM_ID_MAX_VOLTAGE_PROT);
    max_temperature = motor_params_get(motor, PARAM_ID_TEMP_THRESHOLD);
    
    motor_monitor_set_limits(motor, max_current, min_voltage, max_voltage, max_temperature);
    
    /* 从参数字典获取保护使能参数 */
    overcurrent_prot = (uint8_t)motor_params_get(motor, PARAM_ID_OVERCURRENT_PROT);
    undervolt_prot = (uint8_t)motor_params_get(motor, PARAM_ID_UNDERVOLT_PROT);
    overvolt_prot = (uint8_t)motor_params_get(motor, PARAM_ID_OVERVOLT_PROT);
    overtemperature_prot = (uint8_t)motor_params_get(motor, PARAM_ID_OVERTEMPERATURE_PROT);
    
    motor_monitor_enable_protection(motor, overcurrent_prot, undervolt_prot, overvolt_prot, overtemperature_prot);
    
    /* 从参数字典获取PID参数 */
    kp_pos = motor_params_get(motor, PARAM_ID_POSITION_KP);
    ki_pos = motor_params_get(motor, PARAM_ID_POSITION_KI);
    kp_vel = motor_params_get(motor, PARAM_ID_VELOCITY_KP);
    ki_vel = motor_params_get(motor, PARAM_ID_VELOCITY_KI);
    kp_current = motor_params_get(motor, PARAM_ID_CURRENT_KP);
    ki_current = motor_params_get(motor, PARAM_ID_CURRENT_KI);
    
    foc_set_pid_gains(&motor->foc, kp_pos, ki_pos, kp_vel, ki_vel, kp_current, ki_current);
    
    motor->initialized = 1;
    motor->state = MOTOR_STATE_INIT;
}

/**
  * @brief  处理电机控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_process(motor_params_t *motor) {
    float bus_voltage;
    
    if (motor == NULL || !motor->initialized) {
        return;
    }
    
    cia402_process(&motor->ds402);
    motor_monitor_process(motor);
    
    if (motor_monitor_is_fault_active(motor)) {
        motor->state = MOTOR_STATE_FAULT;
        motor_control_handle_fault(motor);
        return;
    }
    
    switch (motor->state) {
        case MOTOR_STATE_INIT:
            if (cia402_get_state(&motor->ds402) == CIA402_STATE_OPERATION_ENABLED) {
                motor->state = MOTOR_STATE_RUNNING;
                foc_enable(&motor->foc);
            }
            break;
        case MOTOR_STATE_RUNNING:
            if (motor->foc.sensorless_enabled) {
                // 从参数字典获取母线电压
                bus_voltage = motor_params_get(motor, PARAM_ID_BUS_VOLTAGE_MON);
                foc_process_with_sensorless(&motor->foc, bus_voltage);
            } else {
                foc_process(&motor->foc);
            }
            break;
        case MOTOR_STATE_FAULT:
            foc_disable(&motor->foc);
            break;
        case MOTOR_STATE_STOPPING:
            foc_disable(&motor->foc);
            if (motor->foc.current_velocity < 0.1f) {
                motor->state = MOTOR_STATE_IDLE;
            }
            break;
        case MOTOR_STATE_IDLE:
        default:
            break;
    }
}

/**
  * @brief  设置目标位置
  * @param  motor: 电机参数结构体指针
  * @param  position: 目标位置
  * @retval 无
  */
void motor_control_set_target_position(motor_params_t *motor, float position) {
    if (motor == NULL) return;
    foc_set_target_position(&motor->foc, position);
    foc_set_control_mode(&motor->foc, FOC_CONTROL_MODE_POSITION);
}

/**
  * @brief  设置目标速度
  * @param  motor: 电机参数结构体指针
  * @param  velocity: 目标速度
  * @retval 无
  */
void motor_control_set_target_velocity(motor_params_t *motor, float velocity) {
    if (motor == NULL) return;
    foc_set_target_velocity(&motor->foc, velocity);
    foc_set_control_mode(&motor->foc, FOC_CONTROL_MODE_VELOCITY);
}

/**
  * @brief  设置目标转矩
  * @param  motor: 电机参数结构体指针
  * @param  torque: 目标转矩
  * @retval 无
  */
void motor_control_set_target_torque(motor_params_t *motor, float torque) {
    if (motor == NULL) return;
    foc_set_target_torque(&motor->foc, torque);
    foc_set_control_mode(&motor->foc, FOC_CONTROL_MODE_TORQUE);
}

/**
  * @brief  设置控制模式
  * @param  motor: 电机参数结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void motor_control_set_control_mode(motor_params_t *motor, foc_control_mode_t mode) {
    if (motor == NULL) return;
    foc_set_control_mode(&motor->foc, mode);
}

/**
  * @brief  启用电机
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_enable(motor_params_t *motor) {
    if (motor == NULL) return;
    cia402_set_control_word(&motor->ds402, 0x000F); /* 启用电压、开关、操作 */
    foc_enable(&motor->foc); /* 直接启用FOC */
    motor->state = MOTOR_STATE_RUNNING; /* 直接设置为运行状态 */
}

/**
  * @brief  禁用电机
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_disable(motor_params_t *motor) {
    if (motor == NULL) return;
    cia402_set_control_word(&motor->ds402, 0x0000);
    motor->state = MOTOR_STATE_STOPPING;
}

/**
  * @brief  获取电机状态
  * @param  motor: 电机参数结构体指针
  * @retval 电机状态
  */
motor_state_t motor_control_get_state(motor_params_t *motor) {
    if (motor == NULL) return MOTOR_STATE_IDLE;
    return motor->state;
}

/**
  * @brief  处理故障
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_handle_fault(motor_params_t *motor) {
    if (motor == NULL) return;
    foc_disable(&motor->foc);
    cia402_set_control_word(&motor->ds402, 0x0080); /* 故障复位 */
    motor_monitor_clear_fault(motor); /* 清除监控系统故障 */
    motor_params_clear_errors(motor); /* 清除所有错误状态 */
    motor->state = MOTOR_STATE_INIT; /* 重置电机状态到初始化状态 */
}

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
void motor_control_update_sensor_data(motor_params_t *motor, float current_u, float current_v, float current_w, float voltage, float temperature, float position, float velocity) {
    int pole_pairs;
    
    if (motor == NULL) return;

    /* 更新参数字典中的监控参数 */
    motor_params_update(motor, PARAM_ID_U_PHASE_CURRENT, current_u);
    motor_params_update(motor, PARAM_ID_V_PHASE_CURRENT, current_v);
    motor_params_update(motor, PARAM_ID_W_PHASE_CURRENT, current_w);
    motor_params_update(motor, PARAM_ID_BUS_VOLTAGE_MON, voltage);
    motor_params_update(motor, PARAM_ID_TEMPERATURE_MON, temperature);
    motor_params_update(motor, PARAM_ID_CURRENT_POSITION, position);
    motor_params_update(motor, PARAM_ID_CURRENT_VELOCITY, velocity);

    motor_params_update(motor, PARAM_ID_CURRENT_UVW_U, current_u);
    motor_params_update(motor, PARAM_ID_CURRENT_UVW_V, current_v);
    motor_params_update(motor, PARAM_ID_CURRENT_UVW_W, current_w);
    
    // 更新FOC模块的电流数据
    motor->foc.current_uvw.u = current_u;
    motor->foc.current_uvw.v = current_v;
    motor->foc.current_uvw.w = current_w;
    
    // 如果没有启用无感控制，使用外部传感器数据
    if (!motor->foc.sensorless_enabled) {
        motor->foc.current_position = position;
        motor->foc.current_velocity = velocity;
        
        pole_pairs = (int)motor_params_get(motor, PARAM_ID_POLE_PAIRS);
        foc_update_angle(&motor->foc, position, pole_pairs);
    }
    // 否则，无感控制会自动估算角度和速度
}

/**
  * @brief  初始化无感控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_sensorless_init(motor_params_t *motor) {
    if (motor == NULL) return;
    
    // 更新参数字典中的无感控制参数
    motor_params_update(motor, PARAM_ID_SENSORLESS_ENABLE, 0);
    motor_params_update(motor, PARAM_ID_SENSORLESS_K1, 0.1);
    motor_params_update(motor, PARAM_ID_SENSORLESS_K2, 0.01);
    motor_params_update(motor, PARAM_ID_SENSORLESS_K3, 0.001);
    motor_params_update(motor, PARAM_ID_SENSORLESS_MIN_SPEED, 100.0);
    motor_params_update(motor, PARAM_ID_SENSORLESS_MAX_SPEED, 3000.0);
    motor_params_update(motor, PARAM_ID_SENSORLESS_ESTIMATOR_GAIN, 0.5);
    
    // 初始化无感控制参数
    foc_sensorless_init(&motor->foc, motor);
}

/**
  * @brief  使能无感控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_sensorless_enable(motor_params_t *motor) {
    if (motor == NULL) return;
    
    // 使能无感控制
    foc_sensorless_enable(&motor->foc);
    
    // 更新参数字典
    motor_params_update(motor, PARAM_ID_SENSORLESS_ENABLE, 1);
}

/**
  * @brief  禁用无感控制
  * @param  motor: 电机参数结构体指针
  * @retval 无
  */
void motor_control_sensorless_disable(motor_params_t *motor) {
    if (motor == NULL) return;
    
    // 禁用无感控制
    foc_sensorless_disable(&motor->foc);
    
    // 更新参数字典
    motor_params_update(motor, PARAM_ID_SENSORLESS_ENABLE, 0);
}

/**
  * @brief  设置无感控制参数
  * @param  motor: 电机参数结构体指针
  * @param  k1: 参数K1
  * @param  k2: 参数K2
  * @param  k3: 参数K3
  * @retval 无
  */
void motor_control_sensorless_set_params(motor_params_t *motor, float k1, float k2, float k3) {
    if (motor == NULL) return;
    
    // 设置无感控制参数
    foc_sensorless_set_params(&motor->foc, k1, k2, k3);
    
    // 更新参数字典
    motor_params_update(motor, PARAM_ID_SENSORLESS_K1, k1);
    motor_params_update(motor, PARAM_ID_SENSORLESS_K2, k2);
    motor_params_update(motor, PARAM_ID_SENSORLESS_K3, k3);
}