#include "motor_params.h"
#include <stddef.h>
#include <string.h>

/**
  * @brief  初始化电机参数
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_params_init(motor_params_t *params) {
    int i;
    if (params == NULL) return;
    
    /* 初始化基本参数 */
    params->basic.pole_pairs = MOTOR_POLE_PAIRS;
    params->basic.bus_voltage = MOTOR_VBUS_VOLTS;
    params->basic.max_current = MOTOR_MAX_CURRENT;
    params->basic.max_velocity = MOTOR_MAX_VELOCITY;
    params->basic.max_torque = MOTOR_MAX_TORQUE;
    params->basic.control_dt = CONTROL_DT;
    params->basic.pwm_frequency = FOC_PWM_FREQUENCY;
    params->basic.encoder_resolution = ENCODER_RESOLUTION;
    params->basic.encoder_cpr = ENCODER_CPR;
    params->basic.temp_threshold = 85.0f;
    params->basic.temp_hysteresis = 5.0f;
    
    /* 初始化监控参数 */
    params->monitor.u_phase_current = 0.0f;
    params->monitor.v_phase_current = 0.0f;
    params->monitor.w_phase_current = 0.0f;
    params->monitor.bus_voltage = 0.0f;
    params->monitor.temperature = 0.0f;
    params->monitor.sample_freq = 1000;
    
    /* 初始化FOC控制参数 */
    params->foc.max_pwm_value = 2000;
    params->foc.sample_freq = 10000;
    params->foc.control_mode = FOC_CONTROL_MODE_POSITION;
    params->foc.target_position = 0.0f;
    params->foc.target_velocity = 0.0f;
    params->foc.target_torque = 0.0f;
    params->foc.current_position = 0.0f;
    params->foc.current_velocity = 0.0f;
    params->foc.current_torque = 0.0f;
    params->foc.kp_pos = 10.0f;
    params->foc.ki_pos = 0.0f;
    params->foc.kp_vel = 0.1f;
    params->foc.ki_vel = 0.01f;
    params->foc.kp_current = 0.5f;
    params->foc.ki_current = 0.1f;
    params->foc.pos_error_integral = 0.0f;
    params->foc.vel_error_integral = 0.0f;
    params->foc.d_current_error_integral = 0.0f;
    params->foc.q_current_error_integral = 0.0f;
    params->foc.enabled = 0;
    
    /* 初始化无感控制参数 */
    params->foc.sensorless_enabled = 0;
    params->foc.sensorless_k1 = 0.1f;
    params->foc.sensorless_k2 = 0.01f;
    params->foc.sensorless_k3 = 0.001f;
    params->foc.sensorless_min_speed = 100.0f;
    params->foc.sensorless_max_speed = 3000.0f;
    params->foc.sensorless_estimator_gain = 0.5f;
    params->foc.estimated_speed = 0.0f;
    params->foc.estimated_angle = 0.0f;
    params->foc.back_emf_est.d = 0.0f;
    params->foc.back_emf_est.q = 0.0f;
    
    /* 初始化保护参数 */
    params->protection.max_current = 10.0f;
    params->protection.min_voltage = 24.0f;
    params->protection.max_voltage = 36.0f;
    params->protection.max_temperature = 85.0f;
    params->protection.overcurrent_prot = 1;
    params->protection.undervolt_prot = 1;
    params->protection.overvolt_prot = 1;
    params->protection.overtemperature_prot = 1;
    params->protection.fault_status = 0;
    params->protection.fault_active = 0;
    
    /* 初始化PID参数 */
    params->pid.position.kp = 10.0f;
    params->pid.position.ki = 0.0f;
    params->pid.position.kd = 0.0f;
    
    params->pid.velocity.kp = 0.1f;
    params->pid.velocity.ki = 0.01f;
    params->pid.velocity.kd = 0.0f;
    
    params->pid.current.kp = 0.5f;
    params->pid.current.ki = 0.1f;
    params->pid.current.kd = 0.0f;
    
    /* 初始化校准参数 */
    params->calib.phase_offset = 0.0f;
    params->calib.current_offset[0] = 0.0f;
    params->calib.current_offset[1] = 0.0f;
    
    /* 初始化CANopen DS301参数 */
    params->canopen.device_type = 0x00000000;           /* 设备类型 */
    params->canopen.error_register = 0x00;              /* 错误寄存器 */
    for (i = 0; i < 8; i++) {
        params->canopen.error_field[i] = 0x00;          /* 预定义错误字段 */
    }
    params->canopen.sync_cob_id = 0x80;                 /* 同步COB ID */
    params->canopen.sync_period = 1000;                 /* 同步周期 */
    strcpy(params->canopen.device_name, "AT32F403 Joint Motor"); /* 制造商设备名称 */
    strcpy(params->canopen.hardware_version, "V1.0");   /* 制造商硬件版本 */
    strcpy(params->canopen.software_version, "V1.0.0"); /* 制造商软件版本 */
    params->canopen.node_id = 1;                        /* 设备站号 */
    params->canopen.watchdog_time = 1000;               /* 看门狗时间 */
    params->canopen.watchdog_factor = 2;                /* 看门狗时间系数 */
    params->canopen.guard_cob_id = 0x700;               /* 节点保护ID */
    params->canopen.store_params = 0x01;                /* 存储参数 */
    params->canopen.restore_params = 0x01;              /* 恢复默认参数 */
    params->canopen.emergency_cob_id = 0x80;            /* 紧急报文COB ID */
    params->canopen.heartbeat_consumer = 0x00000000;    /* 心跳报文消费者 */
    params->canopen.heartbeat_producer = 1000;          /* 心跳生产者时间 */
    params->canopen.identity_object[0] = 0x0000;        /* 站号组 */
    params->canopen.identity_object[1] = 0x0001;        /* 设备厂商代码 */
    params->canopen.identity_object[2] = 0x0001;        /* 产品代码 */
    params->canopen.identity_object[3] = 0x0100;        /* 产品版本 */
    params->canopen.identity_object[4] = 0x00000001;    /* 序列号 */
    
    /* 初始化通信参数 */
    memset(&params->comm, 0, sizeof(motor_comm_params_t));
    
    /* 初始化制造商特定参数 */
    memset(&params->mfg, 0, sizeof(motor_mfg_params_t));
    
    /* 初始化PN相关参数 */
    memset(&params->pn, 0, sizeof(motor_pn_params_t));
    
    /* 初始化DS402 驱动配置对象区参数 */
    memset(&params->ds402, 0, sizeof(motor_ds402_params_t));
    
    /* 初始化电机参数区 */
    memset(&params->param_area, 0, sizeof(motor_param_area_t));
}

/**
  * @brief  加载默认电机参数
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_params_load_default(motor_params_t *params) {
    motor_params_init(params);
}

/**
  * @brief  更新电机参数
  * @param  params: 电机参数结构体指针
  * @param  param_id: 参数ID
  * @param  value: 参数值
  * @retval 无
  */
void motor_params_update(motor_params_t *params, uint16_t param_id, float value) {
    if (params == NULL) return;
    
    switch (param_id) {
        case PARAM_ID_POLE_PAIRS:
            params->basic.pole_pairs = (uint16_t)value;
            break;
        case PARAM_ID_BUS_VOLTAGE:
            params->basic.bus_voltage = value;
            break;
        case PARAM_ID_MAX_CURRENT:
            params->basic.max_current = value;
            break;
        case PARAM_ID_MAX_VELOCITY:
            params->basic.max_velocity = value;
            break;
        case PARAM_ID_MAX_TORQUE:
            params->basic.max_torque = value;
            break;
        case PARAM_ID_CONTROL_DT:
            params->basic.control_dt = value;
            break;
        case PARAM_ID_PWM_FREQUENCY:
            params->basic.pwm_frequency = value;
            break;
        case PARAM_ID_ENCODER_RESOLUTION:
            params->basic.encoder_resolution = (uint16_t)value;
            params->basic.encoder_cpr = params->basic.encoder_resolution * 4;
            break;
        case PARAM_ID_POSITION_KP:
            params->pid.position.kp = value;
            params->foc.kp_pos = value;
            break;
        case PARAM_ID_POSITION_KI:
            params->pid.position.ki = value;
            params->foc.ki_pos = value;
            break;
        case PARAM_ID_POSITION_KD:
            params->pid.position.kd = value;
            break;
        case PARAM_ID_VELOCITY_KP:
            params->pid.velocity.kp = value;
            params->foc.kp_vel = value;
            break;
        case PARAM_ID_VELOCITY_KI:
            params->pid.velocity.ki = value;
            params->foc.ki_vel = value;
            break;
        case PARAM_ID_VELOCITY_KD:
            params->pid.velocity.kd = value;
            break;
        case PARAM_ID_CURRENT_KP:
            params->pid.current.kp = value;
            params->foc.kp_current = value;
            break;
        case PARAM_ID_CURRENT_KI:
            params->pid.current.ki = value;
            params->foc.ki_current = value;
            break;
        case PARAM_ID_CURRENT_KD:
            params->pid.current.kd = value;
            break;
        
        /* 嵌套结构体参数 */
        /* FOC角度结构体参数 */
        case PARAM_ID_ANGLE_ANGLE:
            params->foc.angle.angle = value;
            break;
        case PARAM_ID_ANGLE_SIN:
            params->foc.angle.sin = value;
            break;
        case PARAM_ID_ANGLE_COS:
            params->foc.angle.cos = value;
            break;
        
        /* FOC三相电流结构体参数 */
        case PARAM_ID_CURRENT_UVW_U:
            params->foc.current_uvw.u = value;
            break;
        case PARAM_ID_CURRENT_UVW_V:
            params->foc.current_uvw.v = value;
            break;
        case PARAM_ID_CURRENT_UVW_W:
            params->foc.current_uvw.w = value;
            break;
        
        /* FOC Alpha-Beta电流结构体参数 */
        case PARAM_ID_CURRENT_ALPHA:
            params->foc.current_alpha_beta.alpha = value;
            break;
        case PARAM_ID_CURRENT_BETA:
            params->foc.current_alpha_beta.beta = value;
            break;
        
        /* FOC DQ电流结构体参数 */
        case PARAM_ID_CURRENT_D:
            params->foc.current_dq.d = value;
            break;
        case PARAM_ID_CURRENT_Q:
            params->foc.current_dq.q = value;
            break;
        
        /* FOC DQ电压结构体参数 */
        case PARAM_ID_VOLTAGE_D:
            params->foc.voltage_dq.d = value;
            break;
        case PARAM_ID_VOLTAGE_Q:
            params->foc.voltage_dq.q = value;
            break;
        
        /* FOC PWM结构体参数 */
        case PARAM_ID_PWM_U:
            params->foc.pwm.u = value;
            break;
        case PARAM_ID_PWM_V:
            params->foc.pwm.v = value;
            break;
        case PARAM_ID_PWM_W:
            params->foc.pwm.w = value;
            break;
        
        /* 无感控制反电动势估计结构体参数 */
        case PARAM_ID_BACK_EMF_D:
            params->foc.back_emf_est.d = value;
            break;
        case PARAM_ID_BACK_EMF_Q:
            params->foc.back_emf_est.q = value;
            break;
        case PARAM_ID_PHASE_OFFSET:
            params->calib.phase_offset = value;
            break;
        case PARAM_ID_CURRENT_OFFSET_A:
            params->calib.current_offset[0] = value;
            break;
        case PARAM_ID_CURRENT_OFFSET_B:
            params->calib.current_offset[1] = value;
            break;
        case PARAM_ID_TEMP_THRESHOLD:
            params->basic.temp_threshold = value;
            break;
        case PARAM_ID_TEMP_HYSTERESIS:
            params->basic.temp_hysteresis = value;
            break;
        
        /* 监控参数 */
        case PARAM_ID_U_PHASE_CURRENT:
            params->monitor.u_phase_current = value;
            break;
        case PARAM_ID_V_PHASE_CURRENT:
            params->monitor.v_phase_current = value;
            break;
        case PARAM_ID_W_PHASE_CURRENT:
            params->monitor.w_phase_current = value;
            break;
        case PARAM_ID_BUS_VOLTAGE_MON:
            params->monitor.bus_voltage = value;
            break;
        case PARAM_ID_TEMPERATURE_MON:
            params->monitor.temperature = value;
            break;
        
        /* FOC控制参数 */
        case PARAM_ID_FOC_MAX_PWM_VALUE:
            params->foc.max_pwm_value = (uint16_t)value;
            break;
        case PARAM_ID_FOC_SAMPLE_FREQ:
            params->foc.sample_freq = (uint32_t)value;
            break;
        case PARAM_ID_CONTROL_MODE:
            params->foc.control_mode = (foc_control_mode_t)((uint8_t)value);
            break;
        case PARAM_ID_TARGET_POSITION:
            params->foc.target_position = value;
            break;
        case PARAM_ID_TARGET_VELOCITY_MOTOR:
            params->foc.target_velocity = value;
            break;
        case PARAM_ID_TARGET_TORQUE:
            params->foc.target_torque = value;
            break;
        case PARAM_ID_CURRENT_POSITION:
            params->foc.current_position = value;
            break;
        case PARAM_ID_CURRENT_VELOCITY:
            params->foc.current_velocity = value;
            break;
        case PARAM_ID_CURRENT_TORQUE:
            params->foc.current_torque = value;
            break;
        case PARAM_ID_FOC_ENABLED:
            params->foc.enabled = (uint8_t)value;
            break;
        
        /* 保护参数 */
        case PARAM_ID_MAX_CURRENT_PROT:
            params->protection.max_current = value;
            break;
        case PARAM_ID_MIN_VOLTAGE_PROT:
            params->protection.min_voltage = value;
            break;
        case PARAM_ID_MAX_VOLTAGE_PROT:
            params->protection.max_voltage = value;
            break;
        case PARAM_ID_MAX_TEMPERATURE_PROT:
            params->protection.max_temperature = value;
            break;
        case PARAM_ID_OVERCURRENT_PROT:
            params->protection.overcurrent_prot = (uint8_t)value;
            break;
        case PARAM_ID_UNDERVOLT_PROT:
            params->protection.undervolt_prot = (uint8_t)value;
            break;
        case PARAM_ID_OVERVOLT_PROT:
            params->protection.overvolt_prot = (uint8_t)value;
            break;
        case PARAM_ID_OVERTEMPERATURE_PROT:
            params->protection.overtemperature_prot = (uint8_t)value;
            break;
        case PARAM_ID_FAULT_STATUS:
            params->protection.fault_status = (uint8_t)value;
            break;
        case PARAM_ID_FAULT_ACTIVE:
            params->protection.fault_active = (uint8_t)value;
            break;
        
        /* 采样频率参数 */
        case PARAM_ID_MONITOR_SAMPLE_FREQ:
            params->monitor.sample_freq = (uint32_t)value;
            break;
        
        /* 无感控制参数 */
        case PARAM_ID_SENSORLESS_ENABLE:
            params->foc.sensorless_enabled = (uint8_t)value;
            break;
        case PARAM_ID_SENSORLESS_K1:
            params->foc.sensorless_k1 = value;
            break;
        case PARAM_ID_SENSORLESS_K2:
            params->foc.sensorless_k2 = value;
            break;
        case PARAM_ID_SENSORLESS_K3:
            params->foc.sensorless_k3 = value;
            break;
        case PARAM_ID_SENSORLESS_MIN_SPEED:
            params->foc.sensorless_min_speed = value;
            break;
        case PARAM_ID_SENSORLESS_MAX_SPEED:
            params->foc.sensorless_max_speed = value;
            break;
        case PARAM_ID_SENSORLESS_ESTIMATOR_GAIN:
            params->foc.sensorless_estimator_gain = value;
            break;
        case PARAM_ID_ESTIMATED_SPEED:
            params->foc.estimated_speed = value;
            break;
        case PARAM_ID_ESTIMATED_ANGLE:
            params->foc.estimated_angle = value;
            break;
        
        /* CANopen DS301参数 */
        case PARAM_ID_DEVICE_TYPE:
            params->canopen.device_type = (uint32_t)value;
            break;
        case PARAM_ID_ERROR_REGISTER:
            params->canopen.error_register = (uint8_t)value;
            break;
        case PARAM_ID_SYNC_COB_ID:
            params->canopen.sync_cob_id = (uint32_t)value;
            break;
        case PARAM_ID_SYNC_PERIOD:
            params->canopen.sync_period = (uint32_t)value;
            break;
        case PARAM_ID_NODE_ID:
            params->canopen.node_id = (uint8_t)value;
            break;
        case PARAM_ID_WATCHDOG_TIME:
            params->canopen.watchdog_time = (uint16_t)value;
            break;
        case PARAM_ID_WATCHDOG_FACTOR:
            params->canopen.watchdog_factor = (uint8_t)value;
            break;
        case PARAM_ID_GUARD_COB_ID:
            params->canopen.guard_cob_id = (uint32_t)value;
            break;
        case PARAM_ID_STORE_PARAMS:
            params->canopen.store_params = (uint8_t)value;
            break;
        case PARAM_ID_RESTORE_PARAMS:
            params->canopen.restore_params = (uint8_t)value;
            break;
        case PARAM_ID_EMERGENCY_COB_ID:
            params->canopen.emergency_cob_id = (uint32_t)value;
            break;
        case PARAM_ID_HEARTBEAT_CONSUMER:
            params->canopen.heartbeat_consumer = (uint32_t)value;
            break;
        case PARAM_ID_HEARTBEAT_PRODUCER:
            params->canopen.heartbeat_producer = (uint16_t)value;
            break;
        
        /* RPDO通信参数 */
        case PARAM_ID_RPDO1_COMM:
        case PARAM_ID_RPDO2_COMM:
        case PARAM_ID_RPDO3_COMM:
        case PARAM_ID_RPDO4_COMM:
        case PARAM_ID_RPDO5_COMM:
        case PARAM_ID_RPDO6_COMM:
        case PARAM_ID_RPDO7_COMM:
        case PARAM_ID_RPDO8_COMM:
        case PARAM_ID_RPDO1_MAP:
        case PARAM_ID_RPDO2_MAP:
        case PARAM_ID_RPDO3_MAP:
        case PARAM_ID_RPDO4_MAP:
        case PARAM_ID_RPDO5_MAP:
        case PARAM_ID_RPDO6_MAP:
        case PARAM_ID_RPDO7_MAP:
        case PARAM_ID_RPDO8_MAP:
        case PARAM_ID_TPDO1_COMM:
        case PARAM_ID_TPDO2_COMM:
        case PARAM_ID_TPDO3_COMM:
        case PARAM_ID_TPDO4_COMM:
        case PARAM_ID_TPDO5_COMM:
        case PARAM_ID_TPDO6_COMM:
        case PARAM_ID_TPDO7_COMM:
        case PARAM_ID_TPDO8_COMM:
        case PARAM_ID_TPDO1_MAP:
        case PARAM_ID_TPDO2_MAP:
        case PARAM_ID_TPDO3_MAP:
        case PARAM_ID_TPDO4_MAP:
        case PARAM_ID_TPDO5_MAP:
        case PARAM_ID_TPDO6_MAP:
        case PARAM_ID_TPDO7_MAP:
        case PARAM_ID_TPDO8_MAP:
        case PARAM_ID_NMT_START_STOP:
        case PARAM_ID_NMT_START_STOP_CH0:
        case PARAM_ID_NMT_START_STOP_CH1:
        case PARAM_ID_NMT_START_STOP_CH2:
        case PARAM_ID_NMT_START_STOP_CH3:
        case PARAM_ID_SYNC_OUTPUT:
        case PARAM_ID_SYNC_INPUT:
        case PARAM_ID_POWER_ON_ENABLE:
        case PARAM_ID_IO_GROUP:
        case PARAM_ID_INPUT_FILTER:
        case PARAM_ID_FAST_CAPTURE:
        case PARAM_ID_RISING_CAPTURE_COUNT:
        case PARAM_ID_RISING_CAPTURE_DATA:
        case PARAM_ID_FALLING_CAPTURE_COUNT:
        case PARAM_ID_FALLING_CAPTURE_DATA:
        case PARAM_ID_INPUT_DELAY:
        case PARAM_ID_CAPTURE_COMPENSATION:
        case PARAM_ID_DIN_CONTROL:
        case PARAM_ID_INDEX_WINDOW:
        case PARAM_ID_OSCILLOSCOPE:
        case PARAM_ID_AUTO_FLIP:
        case PARAM_ID_PHASE_CORRECTION:
        case PARAM_ID_SYSTEM_TEST:
        case PARAM_ID_SPECIAL_FUNCTION:
        case PARAM_ID_HIGH_VOLTAGE_SAMPLING:
        case PARAM_ID_ADC_GROUP:
        case PARAM_ID_ANALOG_INPUT:
        case PARAM_ID_PULSE_GROUP2:
        case PARAM_ID_PULSE_GROUP:
        case PARAM_ID_PULSE_GROUP2B:
        case PARAM_ID_FULL_CLOSED_LOOP:
        case PARAM_ID_NOTCH_FILTER:
        case PARAM_ID_NOTCH_FILTER2:
        case PARAM_ID_FIELD_WEAKENING:
        case PARAM_ID_FILTER_EXTENSION:
        case PARAM_ID_ONLINE_TUNING:
        case PARAM_ID_ERROR_MASK:
        case PARAM_ID_ERROR_STATUS:
        case PARAM_ID_ERROR_STATUS2:
        case PARAM_ID_ERROR_MASK_GROUP:
        case PARAM_ID_ERROR_GROUP0:
        case PARAM_ID_ERROR_GROUP1:
        case PARAM_ID_ERROR_GROUP2:
        case PARAM_ID_ERROR_GROUP3:
        case PARAM_ID_ERROR_GROUP4:
        case PARAM_ID_ERROR_GROUP5:
        case PARAM_ID_ERROR_GROUP6:
        case PARAM_ID_ERROR_GROUP7:
            /* 这些参数是数组类型或需要通过子索引访问 */
            break;
        default:
            break;
    }
}

/**
  * @brief  获取电机参数
  * @param  params: 电机参数结构体指针
  * @param  param_id: 参数ID
  * @retval 参数值
  */
float motor_params_get(motor_params_t *params, uint16_t param_id) {
    if (params == NULL) return 0.0f;
    
    switch (param_id) {
        case PARAM_ID_POLE_PAIRS:
            return (float)params->basic.pole_pairs;
        case PARAM_ID_BUS_VOLTAGE:
            return params->basic.bus_voltage;
        case PARAM_ID_MAX_CURRENT:
            return params->basic.max_current;
        case PARAM_ID_MAX_VELOCITY:
            return params->basic.max_velocity;
        case PARAM_ID_MAX_TORQUE:
            return params->basic.max_torque;
        case PARAM_ID_CONTROL_DT:
            return params->basic.control_dt;
        case PARAM_ID_PWM_FREQUENCY:
            return params->basic.pwm_frequency;
        case PARAM_ID_ENCODER_RESOLUTION:
            return (float)params->basic.encoder_resolution;
        case PARAM_ID_POSITION_KP:
            return params->pid.position.kp;
        case PARAM_ID_POSITION_KI:
            return params->pid.position.ki;
        case PARAM_ID_POSITION_KD:
            return params->pid.position.kd;
        case PARAM_ID_VELOCITY_KP:
            return params->pid.velocity.kp;
        case PARAM_ID_VELOCITY_KI:
            return params->pid.velocity.ki;
        case PARAM_ID_VELOCITY_KD:
            return params->pid.velocity.kd;
        case PARAM_ID_CURRENT_KP:
            return params->pid.current.kp;
        case PARAM_ID_CURRENT_KI:
            return params->pid.current.ki;
        case PARAM_ID_CURRENT_KD:
            return params->pid.current.kd;
        
        /* 嵌套结构体参数 */
        /* FOC角度结构体参数 */
        case PARAM_ID_ANGLE_ANGLE:
            return params->foc.angle.angle;
        case PARAM_ID_ANGLE_SIN:
            return params->foc.angle.sin;
        case PARAM_ID_ANGLE_COS:
            return params->foc.angle.cos;
        /* 三相电流结构体 */
        case PARAM_ID_CURRENT_UVW_U:
            return params->foc.current_uvw.u;
        case PARAM_ID_CURRENT_UVW_V:
            return params->foc.current_uvw.v;
        case PARAM_ID_CURRENT_UVW_W:
            return params->foc.current_uvw.w;
        /* Alpha-Beta电流结构体 */
        case PARAM_ID_CURRENT_ALPHA:
            return params->foc.current_alpha_beta.alpha;
        case PARAM_ID_CURRENT_BETA:
            return params->foc.current_alpha_beta.beta;
        /* DQ电流结构体 */
        case PARAM_ID_CURRENT_D:
            return params->foc.current_dq.d;
        case PARAM_ID_CURRENT_Q:
            return params->foc.current_dq.q;
        /* DQ电压结构体 */
        case PARAM_ID_VOLTAGE_D:
            return params->foc.voltage_dq.d;
        case PARAM_ID_VOLTAGE_Q:
            return params->foc.voltage_dq.q;
        /* PWM结构体 */
        case PARAM_ID_PWM_U:
            return params->foc.pwm.u;
        case PARAM_ID_PWM_V:
            return params->foc.pwm.v;
        case PARAM_ID_PWM_W:
            return params->foc.pwm.w;
        /* 反电动势估计结构体 */
        case PARAM_ID_BACK_EMF_D:
            return params->foc.back_emf_est.d;
        case PARAM_ID_BACK_EMF_Q:
            return params->foc.back_emf_est.q;
        case PARAM_ID_PHASE_OFFSET:
            return params->calib.phase_offset;
        case PARAM_ID_CURRENT_OFFSET_A:
            return params->calib.current_offset[0];
        case PARAM_ID_CURRENT_OFFSET_B:
            return params->calib.current_offset[1];
        case PARAM_ID_TEMP_THRESHOLD:
            return params->basic.temp_threshold;
        case PARAM_ID_TEMP_HYSTERESIS:
            return params->basic.temp_hysteresis;
        
        /* 监控参数 */
        case PARAM_ID_U_PHASE_CURRENT:
            return params->monitor.u_phase_current;
        case PARAM_ID_V_PHASE_CURRENT:
            return params->monitor.v_phase_current;
        case PARAM_ID_BUS_VOLTAGE_MON:
            return params->monitor.bus_voltage;
        case PARAM_ID_TEMPERATURE_MON:
            return params->monitor.temperature;
        case PARAM_ID_W_PHASE_CURRENT:
            return params->monitor.w_phase_current;
        
        /* FOC控制参数 */
        case PARAM_ID_FOC_MAX_PWM_VALUE:
            return (float)params->foc.max_pwm_value;
        case PARAM_ID_FOC_SAMPLE_FREQ:
            return (float)params->foc.sample_freq;
        case PARAM_ID_CONTROL_MODE:
            return (float)params->foc.control_mode;
        case PARAM_ID_TARGET_POSITION:
            return params->foc.target_position;
        case PARAM_ID_TARGET_VELOCITY_MOTOR:
            return params->foc.target_velocity;
        case PARAM_ID_TARGET_TORQUE:
            return params->foc.target_torque;
        case PARAM_ID_CURRENT_POSITION:
            return params->foc.current_position;
        case PARAM_ID_CURRENT_VELOCITY:
            return params->foc.current_velocity;
        case PARAM_ID_CURRENT_TORQUE:
            return params->foc.current_torque;
        case PARAM_ID_FOC_ENABLED:
            return (float)params->foc.enabled;
        
        /* 保护参数 */
        case PARAM_ID_MAX_CURRENT_PROT:
            return params->protection.max_current;
        case PARAM_ID_MIN_VOLTAGE_PROT:
            return params->protection.min_voltage;
        case PARAM_ID_MAX_VOLTAGE_PROT:
            return params->protection.max_voltage;
        case PARAM_ID_MAX_TEMPERATURE_PROT:
            return params->protection.max_temperature;
        case PARAM_ID_OVERCURRENT_PROT:
            return (float)params->protection.overcurrent_prot;
        case PARAM_ID_UNDERVOLT_PROT:
            return (float)params->protection.undervolt_prot;
        case PARAM_ID_OVERVOLT_PROT:
            return (float)params->protection.overvolt_prot;
        case PARAM_ID_OVERTEMPERATURE_PROT:
            return (float)params->protection.overtemperature_prot;
        case PARAM_ID_FAULT_STATUS:
            return (float)params->protection.fault_status;
        case PARAM_ID_FAULT_ACTIVE:
            return (float)params->protection.fault_active;
        
        /* 采样频率参数 */
        case PARAM_ID_MONITOR_SAMPLE_FREQ:
            return (float)params->monitor.sample_freq;
        
        /* 无感控制参数 */
        case PARAM_ID_SENSORLESS_ENABLE:
            return (float)params->foc.sensorless_enabled;
        case PARAM_ID_SENSORLESS_K1:
            return params->foc.sensorless_k1;
        case PARAM_ID_SENSORLESS_K2:
            return params->foc.sensorless_k2;
        case PARAM_ID_SENSORLESS_K3:
            return params->foc.sensorless_k3;
        case PARAM_ID_SENSORLESS_MIN_SPEED:
            return params->foc.sensorless_min_speed;
        case PARAM_ID_SENSORLESS_MAX_SPEED:
            return params->foc.sensorless_max_speed;
        case PARAM_ID_SENSORLESS_ESTIMATOR_GAIN:
            return params->foc.sensorless_estimator_gain;
        case PARAM_ID_ESTIMATED_SPEED:
            return params->foc.estimated_speed;
        case PARAM_ID_ESTIMATED_ANGLE:
            return params->foc.estimated_angle;
        
        /* CANopen DS301参数 */
        case PARAM_ID_DEVICE_TYPE:
            return (float)params->canopen.device_type;
        case PARAM_ID_ERROR_REGISTER:
            return (float)params->canopen.error_register;
        case PARAM_ID_SYNC_COB_ID:
            return (float)params->canopen.sync_cob_id;
        case PARAM_ID_SYNC_PERIOD:
            return (float)params->canopen.sync_period;
        case PARAM_ID_NODE_ID:
            return (float)params->canopen.node_id;
        case PARAM_ID_WATCHDOG_TIME:
            return (float)params->canopen.watchdog_time;
        case PARAM_ID_WATCHDOG_FACTOR:
            return (float)params->canopen.watchdog_factor;
        case PARAM_ID_GUARD_COB_ID:
            return (float)params->canopen.guard_cob_id;
        case PARAM_ID_STORE_PARAMS:
            return (float)params->canopen.store_params;
        case PARAM_ID_RESTORE_PARAMS:
            return (float)params->canopen.restore_params;
        case PARAM_ID_EMERGENCY_COB_ID:
            return (float)params->canopen.emergency_cob_id;
        case PARAM_ID_HEARTBEAT_CONSUMER:
            return (float)params->canopen.heartbeat_consumer;
        case PARAM_ID_HEARTBEAT_PRODUCER:
            return (float)params->canopen.heartbeat_producer;
        
        /* RPDO通信参数 */
        case PARAM_ID_RPDO1_COMM:
        case PARAM_ID_RPDO2_COMM:
        case PARAM_ID_RPDO3_COMM:
        case PARAM_ID_RPDO4_COMM:
        case PARAM_ID_RPDO5_COMM:
        case PARAM_ID_RPDO6_COMM:
        case PARAM_ID_RPDO7_COMM:
        case PARAM_ID_RPDO8_COMM:
        case PARAM_ID_RPDO1_MAP:
        case PARAM_ID_RPDO2_MAP:
        case PARAM_ID_RPDO3_MAP:
        case PARAM_ID_RPDO4_MAP:
        case PARAM_ID_RPDO5_MAP:
        case PARAM_ID_RPDO6_MAP:
        case PARAM_ID_RPDO7_MAP:
        case PARAM_ID_RPDO8_MAP:
        case PARAM_ID_TPDO1_COMM:
        case PARAM_ID_TPDO2_COMM:
        case PARAM_ID_TPDO3_COMM:
        case PARAM_ID_TPDO4_COMM:
        case PARAM_ID_TPDO5_COMM:
        case PARAM_ID_TPDO6_COMM:
        case PARAM_ID_TPDO7_COMM:
        case PARAM_ID_TPDO8_COMM:
        case PARAM_ID_TPDO1_MAP:
        case PARAM_ID_TPDO2_MAP:
        case PARAM_ID_TPDO3_MAP:
        case PARAM_ID_TPDO4_MAP:
        case PARAM_ID_TPDO5_MAP:
        case PARAM_ID_TPDO6_MAP:
        case PARAM_ID_TPDO7_MAP:
        case PARAM_ID_TPDO8_MAP:
        case PARAM_ID_NMT_START_STOP:
        case PARAM_ID_NMT_START_STOP_CH0:
        case PARAM_ID_NMT_START_STOP_CH1:
        case PARAM_ID_NMT_START_STOP_CH2:
        case PARAM_ID_NMT_START_STOP_CH3:
        case PARAM_ID_SYNC_OUTPUT:
        case PARAM_ID_SYNC_INPUT:
        case PARAM_ID_POWER_ON_ENABLE:
        case PARAM_ID_IO_GROUP:
        case PARAM_ID_INPUT_FILTER:
        case PARAM_ID_FAST_CAPTURE:
        case PARAM_ID_RISING_CAPTURE_COUNT:
        case PARAM_ID_RISING_CAPTURE_DATA:
        case PARAM_ID_FALLING_CAPTURE_COUNT:
        case PARAM_ID_FALLING_CAPTURE_DATA:
        case PARAM_ID_INPUT_DELAY:
        case PARAM_ID_CAPTURE_COMPENSATION:
        case PARAM_ID_DIN_CONTROL:
        case PARAM_ID_INDEX_WINDOW:
        case PARAM_ID_OSCILLOSCOPE:
        case PARAM_ID_AUTO_FLIP:
        case PARAM_ID_PHASE_CORRECTION:
        case PARAM_ID_SYSTEM_TEST:
        case PARAM_ID_SPECIAL_FUNCTION:
        case PARAM_ID_HIGH_VOLTAGE_SAMPLING:
        case PARAM_ID_ADC_GROUP:
        case PARAM_ID_ANALOG_INPUT:
        case PARAM_ID_PULSE_GROUP2:
        case PARAM_ID_PULSE_GROUP:
        case PARAM_ID_PULSE_GROUP2B:
        case PARAM_ID_FULL_CLOSED_LOOP:
        case PARAM_ID_NOTCH_FILTER:
        case PARAM_ID_NOTCH_FILTER2:
        case PARAM_ID_FIELD_WEAKENING:
        case PARAM_ID_FILTER_EXTENSION:
        case PARAM_ID_ONLINE_TUNING:
        case PARAM_ID_ERROR_MASK:
        case PARAM_ID_ERROR_STATUS:
        case PARAM_ID_ERROR_STATUS2:
        case PARAM_ID_ERROR_MASK_GROUP:
        case PARAM_ID_ERROR_GROUP0:
        case PARAM_ID_ERROR_GROUP1:
        case PARAM_ID_ERROR_GROUP2:
        case PARAM_ID_ERROR_GROUP3:
        case PARAM_ID_ERROR_GROUP4:
        case PARAM_ID_ERROR_GROUP5:
        case PARAM_ID_ERROR_GROUP6:
        case PARAM_ID_ERROR_GROUP7:
            /* 这些参数是数组类型或需要通过子索引访问 */
            return 0.0f;
        default:
            return 0.0f;
    }
}

/**
  * @brief  更新带子索引的电机参数
  * @param  params: 电机参数结构体指针
  * @param  param_id: 参数ID
  * @param  sub_index: 子索引
  * @param  value: 参数值
  * @retval 无
  */
void motor_params_update_sub(motor_params_t *params, uint16_t param_id, uint8_t sub_index, uint32_t value) {
    if (params == NULL) return;
    
    switch (param_id) {
        /* 错误字段 */
        case PARAM_ID_ERROR_FIELD:
            if (sub_index < 8) {
                params->canopen.error_field[sub_index] = (uint8_t)value;
            }
            break;
        
        /* 标识对象 */
        case PARAM_ID_IDENTITY_OBJECT:
            if (sub_index < 5) {
                params->canopen.identity_object[sub_index] = value;
            }
            break;
        
        /* RPDO通信参数 */
        case PARAM_ID_RPDO1_COMM:
            if (sub_index < 4) {
                params->comm.rpdo1_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO2_COMM:
            if (sub_index < 4) {
                params->comm.rpdo2_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO3_COMM:
            if (sub_index < 4) {
                params->comm.rpdo3_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO4_COMM:
            if (sub_index < 4) {
                params->comm.rpdo4_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO5_COMM:
            if (sub_index < 4) {
                params->comm.rpdo5_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO6_COMM:
            if (sub_index < 4) {
                params->comm.rpdo6_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO7_COMM:
            if (sub_index < 4) {
                params->comm.rpdo7_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO8_COMM:
            if (sub_index < 4) {
                params->comm.rpdo8_comm[sub_index] = value;
            }
            break;
        
        /* RPDO映射参数 */
        case PARAM_ID_RPDO1_MAP:
            if (sub_index < 8) {
                params->comm.rpdo1_map[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO2_MAP:
            if (sub_index < 8) {
                params->comm.rpdo2_map[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO3_MAP:
            if (sub_index < 8) {
                params->comm.rpdo3_map[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO4_MAP:
            if (sub_index < 8) {
                params->comm.rpdo4_map[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO5_MAP:
            if (sub_index < 8) {
                params->comm.rpdo5_map[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO6_MAP:
            if (sub_index < 8) {
                params->comm.rpdo6_map[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO7_MAP:
            if (sub_index < 8) {
                params->comm.rpdo7_map[sub_index] = value;
            }
            break;
        case PARAM_ID_RPDO8_MAP:
            if (sub_index < 8) {
                params->comm.rpdo8_map[sub_index] = value;
            }
            break;
        
        /* TPDO通信参数 */
        case PARAM_ID_TPDO1_COMM:
            if (sub_index < 6) {
                params->comm.tpdo1_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO2_COMM:
            if (sub_index < 6) {
                params->comm.tpdo2_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO3_COMM:
            if (sub_index < 6) {
                params->comm.tpdo3_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO4_COMM:
            if (sub_index < 6) {
                params->comm.tpdo4_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO5_COMM:
            if (sub_index < 6) {
                params->comm.tpdo5_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO6_COMM:
            if (sub_index < 6) {
                params->comm.tpdo6_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO7_COMM:
            if (sub_index < 6) {
                params->comm.tpdo7_comm[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO8_COMM:
            if (sub_index < 6) {
                params->comm.tpdo8_comm[sub_index] = value;
            }
            break;
        
        /* TPDO映射参数 */
        case PARAM_ID_TPDO1_MAP:
            if (sub_index < 8) {
                params->comm.tpdo1_map[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO2_MAP:
            if (sub_index < 8) {
                params->comm.tpdo2_map[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO3_MAP:
            if (sub_index < 8) {
                params->comm.tpdo3_map[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO4_MAP:
            if (sub_index < 8) {
                params->comm.tpdo4_map[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO5_MAP:
            if (sub_index < 8) {
                params->comm.tpdo5_map[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO6_MAP:
            if (sub_index < 8) {
                params->comm.tpdo6_map[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO7_MAP:
            if (sub_index < 8) {
                params->comm.tpdo7_map[sub_index] = value;
            }
            break;
        case PARAM_ID_TPDO8_MAP:
            if (sub_index < 8) {
                params->comm.tpdo8_map[sub_index] = value;
            }
            break;
        
        /* NMT启动/停止 */
        case PARAM_ID_NMT_START_STOP:
            if (sub_index < 8) {
                params->comm.nmt_start_stop[sub_index] = value;
            }
            break;
        case PARAM_ID_NMT_START_STOP_CH0:
            if (sub_index < 8) {
                params->comm.nmt_start_stop_ch0[sub_index] = value;
            }
            break;
        case PARAM_ID_NMT_START_STOP_CH1:
            if (sub_index < 8) {
                params->comm.nmt_start_stop_ch1[sub_index] = value;
            }
            break;
        case PARAM_ID_NMT_START_STOP_CH2:
            if (sub_index < 8) {
                params->comm.nmt_start_stop_ch2[sub_index] = value;
            }
            break;
        case PARAM_ID_NMT_START_STOP_CH3:
            if (sub_index < 8) {
                params->comm.nmt_start_stop_ch3[sub_index] = value;
            }
            break;
        
        /* 同步输入输出 */
        case PARAM_ID_SYNC_OUTPUT:
            if (sub_index < 13) {
                params->comm.sync_output[sub_index] = value;
            }
            break;
        case PARAM_ID_SYNC_INPUT:
            if (sub_index < 13) {
                params->comm.sync_input[sub_index] = value;
            }
            break;
        
        /* 制造商特定参数 */
        case PARAM_ID_POWER_ON_ENABLE:
            params->mfg.power_on_enable = value;
            break;
        case PARAM_ID_IO_GROUP:
            if (sub_index < 40) {
                params->mfg.io_group[sub_index] = value;
            }
            break;
        case PARAM_ID_INPUT_FILTER:
            if (sub_index < 8) {
                params->mfg.input_filter[sub_index] = value;
            }
            break;
        case PARAM_ID_FAST_CAPTURE:
            params->mfg.fast_capture = value;
            break;
        case PARAM_ID_RISING_CAPTURE_COUNT:
            params->mfg.rising_capture_count = value;
            break;
        case PARAM_ID_RISING_CAPTURE_DATA:
            if (sub_index < 32) {
                params->mfg.rising_capture_data[sub_index] = value;
            }
            break;
        case PARAM_ID_FALLING_CAPTURE_COUNT:
            params->mfg.falling_capture_count = value;
            break;
        case PARAM_ID_FALLING_CAPTURE_DATA:
            if (sub_index < 32) {
                params->mfg.falling_capture_data[sub_index] = value;
            }
            break;
        case PARAM_ID_INPUT_DELAY:
            if (sub_index < 7) {
                params->mfg.input_delay[sub_index] = value;
            }
            break;
        case PARAM_ID_CAPTURE_COMPENSATION:
            if (sub_index < 4) {
                params->mfg.capture_compensation[sub_index] = value;
            }
            break;
        case PARAM_ID_DIN_CONTROL:
            if (sub_index < 52) {
                params->mfg.din_control[sub_index] = value;
            }
            break;
        case PARAM_ID_INDEX_WINDOW:
            params->mfg.index_window = value;
            break;
        case PARAM_ID_OSCILLOSCOPE:
            if (sub_index < 14) {
                params->mfg.oscilloscope[sub_index] = value;
            }
            break;
        case PARAM_ID_AUTO_FLIP:
            if (sub_index < 6) {
                params->mfg.auto_flip[sub_index] = value;
            }
            break;
        case PARAM_ID_PHASE_CORRECTION:
            if (sub_index < 14) {
                params->mfg.phase_correction[sub_index] = value;
            }
            break;
        case PARAM_ID_SYSTEM_TEST:
            if (sub_index < 13) {
                params->mfg.system_test[sub_index] = value;
            }
            break;
        case PARAM_ID_SPECIAL_FUNCTION:
            if (sub_index < 18) {
                params->mfg.special_function[sub_index] = value;
            }
            break;
        case PARAM_ID_HIGH_VOLTAGE_SAMPLING:
            params->mfg.high_voltage_sampling = value;
            break;
        case PARAM_ID_ADC_GROUP:
            if (sub_index < 14) {
                params->mfg.adc_group[sub_index] = value;
            }
            break;
        case PARAM_ID_ANALOG_INPUT:
            if (sub_index < 17) {
                params->mfg.analog_input[sub_index] = value;
            }
            break;
        case PARAM_ID_PULSE_GROUP2:
            if (sub_index < 3) {
                params->mfg.pulse_group2[sub_index] = value;
            }
            break;
        case PARAM_ID_PULSE_GROUP:
            if (sub_index < 16) {
                params->mfg.pulse_group[sub_index] = value;
            }
            break;
        case PARAM_ID_PULSE_GROUP2B:
            if (sub_index < 15) {
                params->mfg.pulse_group2b[sub_index] = value;
            }
            break;
        case PARAM_ID_FULL_CLOSED_LOOP:
            if (sub_index < 10) {
                params->mfg.full_closed_loop[sub_index] = value;
            }
            break;
        case PARAM_ID_NOTCH_FILTER:
            if (sub_index < 9) {
                params->mfg.notch_filter[sub_index] = value;
            }
            break;
        case PARAM_ID_NOTCH_FILTER2:
            if (sub_index < 9) {
                params->mfg.notch_filter2[sub_index] = value;
            }
            break;
        case PARAM_ID_FIELD_WEAKENING:
            if (sub_index < 9) {
                params->mfg.field_weakening[sub_index] = value;
            }
            break;
        case PARAM_ID_FILTER_EXTENSION:
            if (sub_index < 12) {
                params->mfg.filter_extension[sub_index] = value;
            }
            break;
        case PARAM_ID_ONLINE_TUNING:
            if (sub_index < 14) {
                params->mfg.online_tuning[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_MASK:
            params->mfg.error_mask = value;
            break;
        case PARAM_ID_ERROR_STATUS:
            params->mfg.error_status = value;
            break;
        case PARAM_ID_ERROR_STATUS2:
            params->mfg.error_status2 = value;
            break;
        case PARAM_ID_ERROR_MASK_GROUP:
            if (sub_index < 7) {
                params->mfg.error_mask_group[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_GROUP0:
            if (sub_index < 10) {
                params->mfg.error_group0[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_GROUP1:
            if (sub_index < 10) {
                params->mfg.error_group1[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_GROUP2:
            if (sub_index < 10) {
                params->mfg.error_group2[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_GROUP3:
            if (sub_index < 10) {
                params->mfg.error_group3[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_GROUP4:
            if (sub_index < 10) {
                params->mfg.error_group4[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_GROUP5:
            if (sub_index < 10) {
                params->mfg.error_group5[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_GROUP6:
            if (sub_index < 10) {
                params->mfg.error_group6[sub_index] = value;
            }
            break;
        case PARAM_ID_ERROR_GROUP7:
            if (sub_index < 10) {
                params->mfg.error_group7[sub_index] = value;
            }
            break;
        
        /* 新增制造商特定参数 */
        case PARAM_ID_WARNING_STATUS:
            params->mfg.warning_status = value;
            break;
        case PARAM_ID_ENCODER_RESET:
            params->mfg.encoder_reset = value;
            break;
        case PARAM_ID_ENCODER_TYPE:
            if (sub_index < 1) {
                params->mfg.encoder_type = value;
            }
            break;
        case PARAM_ID_ENCODER_TYPE2:
            if (sub_index < 1) {
                params->mfg.encoder_type2 = value;
            }
            break;
        case PARAM_ID_ENCODER_FLAGS:
            if (sub_index < 1) {
                params->mfg.encoder_flags = value;
            }
            break;
        case PARAM_ID_ENCODER_STATUS:
            if (sub_index < 1) {
                params->mfg.encoder_status = value;
            }
            break;
        case PARAM_ID_PHASE_LOSS_DETECTION:
            params->mfg.phase_loss_detection = value;
            break;
        case PARAM_ID_GROUP_TOUCH:
            if (sub_index < 9) {
                params->mfg.group_touch[sub_index] = value;
            }
            break;
        case PARAM_ID_DEVICE_NODE_GROUP:
            if (sub_index < 2) {
                params->mfg.device_node_group[sub_index] = value;
            }
            break;
        case PARAM_ID_CAN_BAUDRATE:
            params->mfg.can_baudrate = value;
            break;
        case PARAM_ID_RS232_BAUDRATE:
            params->mfg.rs232_baudrate = value;
            break;
        case PARAM_ID_RS232_BAUDRATE_DEBUG:
            if (sub_index < 2) {
                params->mfg.rs232_baudrate_debug[sub_index] = value;
            }
            break;
        case PARAM_ID_RS485_BAUDRATE:
            params->mfg.rs485_baudrate = value;
            break;
        case PARAM_ID_RS485_BAUDRATE_DEBUG:
            params->mfg.rs485_baudrate_debug = value;
            break;
        case PARAM_ID_EEPROM_GROUP:
            if (sub_index < 33) {
                params->mfg.eeprom_group[sub_index] = value;
            }
            break;
        case PARAM_ID_PANEL_GROUP:
            if (sub_index < 36) {
                params->mfg.panel_group[sub_index] = value;
            }
            break;
        case PARAM_ID_PANEL_GROUP2:
            if (sub_index < 4) {
                params->mfg.panel_group2[sub_index] = value;
            }
            break;
        
        /* 新增制造商特定参数 */
        case PARAM_ID_DEVICE_TIME:
            params->mfg.device_time = value;
            break;
        case PARAM_ID_HARDWARE_SERIAL:
            params->mfg.hardware_serial = value;
            break;
        case PARAM_ID_USER_PASSWORD:
            params->mfg.user_password = value;
            break;
        case PARAM_ID_FACTORY_PASSWORD:
            params->mfg.factory_password = value;
            break;
        case PARAM_ID_DEVICE_RESTART:
            params->mfg.device_restart = value;
            break;
        case PARAM_ID_TEST_GROUP:
            if (sub_index < 27) {
                params->mfg.test_group[sub_index] = value;
            }
            break;
        case PARAM_ID_PROCESSOR_GROUP:
            if (sub_index < 29) {
                params->mfg.processor_group[sub_index] = value;
            }
            break;
        case PARAM_ID_ECAN_GROUP:
            if (sub_index < 9) {
                params->mfg.ecan_group[sub_index] = value;
            }
            break;
        case PARAM_ID_INTERPOLATION_GROUP:
            if (sub_index < 11) {
                params->mfg.interpolation_group[sub_index] = value;
            }
            break;
        case PARAM_ID_PARAMETER_CHECKSUM:
            if (sub_index < 4) {
                params->mfg.parameter_checksum[sub_index] = value;
            }
            break;
        case PARAM_ID_SHORT_CIRCUIT_BRAKE:
            if (sub_index < 9) {
                params->mfg.short_circuit_brake[sub_index] = value;
            }
            break;
        case PARAM_ID_LOAD_TEST_SWEEP:
            if (sub_index < 12) {
                params->mfg.load_test_sweep[sub_index] = value;
            }
            break;
        case PARAM_ID_QUICK_SETTING:
            if (sub_index < 7) {
                params->mfg.quick_setting[sub_index] = value;
            }
            break;
        case PARAM_ID_CURRENT_POSITION_TABLE:
            params->mfg.current_position_table = value;
            break;
        case PARAM_ID_ENCODER_COMM_STATUS:
            params->mfg.encoder_communication_status = value;
            break;
        case PARAM_ID_GROUP_RESOLVER:
            if (sub_index < 10) {
                params->mfg.group_resolver[sub_index] = value;
            }
            break;
        case PARAM_ID_ENCODER_INFO:
            params->mfg.encoder_info = value;
            break;
        case PARAM_ID_ENCODER_WARNING:
            params->mfg.encoder_warning = value;
            break;
        case PARAM_ID_ENCODER_TEMPERATURE:
            params->mfg.encoder_temperature = value;
            break;
        case PARAM_ID_INERTIA_IDENTIFICATION:
            if (sub_index < 38) {
                params->mfg.inertia_identification[sub_index] = value;
            }
            break;
        case PARAM_ID_VIBRATION_SUPPRESSION_A:
            if (sub_index < 6) {
                params->mfg.vibration_suppression_a[sub_index] = value;
            }
            break;
        case PARAM_ID_LOW_FREQUENCY_VIBRATION:
            if (sub_index < 11) {
                params->mfg.low_frequency_vibration[sub_index] = value;
            }
            break;
        case PARAM_ID_INPUT_SHAPER:
            if (sub_index < 6) {
                params->mfg.input_shaper[sub_index] = value;
            }
            break;
        case PARAM_ID_MODEL_TRACKING:
            if (sub_index < 3) {
                params->mfg.model_tracking[sub_index] = value;
            }
            break;
        case PARAM_ID_ADAPTIVE_NOTCH_FILTER:
            if (sub_index < 16) {
                params->mfg.adaptive_notch_filter[sub_index] = value;
            }
            break;
        case PARAM_ID_PROBE_FUNCTION:
            if (sub_index < 13) {
                params->mfg.probe_function[sub_index] = value;
            }
            break;
        case PARAM_ID_SYSTEM_FREQUENCY_SWITCH:
            if (sub_index < 3) {
                params->mfg.system_frequency_switch[sub_index] = value;
            }
            break;
        
        /* 新增PN相关参数 */
        case PARAM_ID_PN_SOFTWARE_VERSION:
            params->pn.pn_software_version = value;
            break;
        case PARAM_ID_NETWORK_PORT_STATUS:
            params->pn.network_port_status = value;
            break;
        case PARAM_ID_PN_DEVICE_NAME:
            params->pn.pn_device_name = value;
            break;
        case PARAM_ID_PN_DEVICE_NAME1:
            params->pn.pn_device_name1 = value;
            break;
        case PARAM_ID_PN_DEVICE_NAME2:
            params->pn.pn_device_name2 = value;
            break;
        case PARAM_ID_PN_DEVICE_NAME3:
            params->pn.pn_device_name3 = value;
            break;
        case PARAM_ID_PN_DEVICE_NAME4:
            params->pn.pn_device_name4 = value;
            break;
        case PARAM_ID_IP_ADDRESS:
            params->pn.ip_address = value;
            break;
        case PARAM_ID_SUBNET_MASK:
            params->pn.subnet_mask = value;
            break;
        case PARAM_ID_DEFAULT_GATEWAY:
            params->pn.default_gateway = value;
            break;
        case PARAM_ID_PN_ENABLE:
            params->pn.pn_enable = value;
            break;
        case PARAM_ID_SOFTWARE_LIMIT_ENABLE:
            params->pn.software_limit_enable = value;
            break;
        case PARAM_ID_PN_USER_RX_WORD:
            params->pn.pn_user_rx_word = value;
            break;
        case PARAM_ID_PN_USER_TX_WORD:
            params->pn.pn_user_tx_word = value;
            break;
        case PARAM_ID_MAC_ADDRESS1:
            params->pn.mac_address1 = value;
            break;
        case PARAM_ID_MAC_ADDRESS2:
            params->pn.mac_address2 = value;
            break;
        
        /* DS402 驱动配置对象区参数 */
        case PARAM_ID_MOTOR_ABSOLUTE_POSITION:
            params->ds402.motor_absolute_position = (int32_t)value;
            break;
        case PARAM_ID_COMMUNICATION_INTERRUPT_MODE:
            params->ds402.communication_interrupt_mode = (uint16_t)value;
            break;
        case PARAM_ID_ERROR_CODE:
            params->ds402.error_code = value;
            break;
        case PARAM_ID_DS402_CONTROL_WORD:
            params->ds402.control_word = (uint16_t)value;
            break;
        case PARAM_ID_DS402_STATUS_WORD:
            params->ds402.status_word = (uint16_t)value;
            break;
        case PARAM_ID_QUICK_STOP_MODE:
            params->ds402.quick_stop_mode = (uint8_t)value;
            break;
        case PARAM_ID_SHUTDOWN_STOP_MODE:
            params->ds402.shutdown_stop_mode = (uint8_t)value;
            break;
        case PARAM_ID_DISABLE_STOP_MODE:
            params->ds402.disable_stop_mode = (uint8_t)value;
            break;
        case PARAM_ID_HALT_MODE:
            params->ds402.halt_mode = (uint8_t)value;
            break;
        case PARAM_ID_ERROR_STOP_MODE:
            params->ds402.error_stop_mode = (uint8_t)value;
            break;
        case PARAM_ID_DS402_OPERATION_MODE:
            params->ds402.operation_mode = (uint8_t)value;
            break;
        case PARAM_ID_ACTIVE_OPERATION_MODE:
            params->ds402.active_operation_mode = (uint8_t)value;
            break;
        case PARAM_ID_EFFECTIVE_TARGET_POSITION:
            params->ds402.effective_target_position = (int32_t)value;
            break;
        case PARAM_ID_ACTUAL_POSITION_INC:
            params->ds402.actual_position_inc = (int32_t)value;
            break;
        case PARAM_ID_ACTUAL_POSITION_PULSE:
            params->ds402.actual_position_pulse = (int32_t)value;
            break;
        case PARAM_ID_MAX_FOLLOWING_ERROR:
            params->ds402.max_following_error = (int32_t)value;
            break;
        case PARAM_ID_TARGET_POSITION_WINDOW:
            params->ds402.target_position_window = (int32_t)value;
            break;
        case PARAM_ID_POSITION_WINDOW_TIME:
            params->ds402.position_window_time = value;
            break;
        case PARAM_ID_EFFECTIVE_TARGET_VELOCITY:
            params->ds402.effective_target_velocity = (int32_t)value;
            break;
        case PARAM_ID_ACTUAL_VELOCITY:
            params->ds402.actual_velocity = (int32_t)value;
            break;
        case PARAM_ID_TARGET_TORQUE_PERCENT:
            params->ds402.target_torque_percent = (int16_t)value;
            break;
        case PARAM_ID_MAX_TORQUE_LIMIT_PERCENT:
            params->ds402.max_torque_limit_percent = (int16_t)value;
            break;
        case PARAM_ID_TARGET_CURRENT_LIMIT:
            params->ds402.target_current_limit = (int16_t)value;
            break;
        case PARAM_ID_MOTOR_RATED_CURRENT:
            params->ds402.motor_rated_current = (int16_t)value;
            break;
        case PARAM_ID_MOTOR_RATED_TORQUE:
            params->ds402.motor_rated_torque = (int16_t)value;
            break;
        case PARAM_ID_ACTUAL_TORQUE:
            params->ds402.actual_torque = (int16_t)value;
            break;
        case PARAM_ID_ACTUAL_CURRENT:
            params->ds402.actual_current = (int16_t)value;
            break;
        case PARAM_ID_ACTUAL_BUS_VOLTAGE_MV:
            params->ds402.actual_bus_voltage_mv = (uint16_t)value;
            break;
        case PARAM_ID_DS402_TARGET_POSITION:
        case PARAM_ID_TARGET_POSITION:
            params->ds402.target_position = (int32_t)value;
            break;
        case PARAM_ID_HOME_OFFSET:
            params->ds402.home_offset = (int32_t)value;
            break;
        case PARAM_ID_SOFT_LIMIT:
            if (sub_index < 3) {
                params->ds402.soft_limit[sub_index] = (int32_t)value;
            }
            break;
        case PARAM_ID_VELOCITY_POSITION_DIRECTION:
            params->ds402.velocity_position_direction = (uint8_t)value;
            break;
        case PARAM_ID_MAX_VELOCITY_LIMIT:
            params->ds402.max_velocity_limit = (int32_t)value;
            break;
        case PARAM_ID_MAX_VELOCITY_LIMIT_RPM:
            params->ds402.max_velocity_limit_rpm = (int32_t)value;
            break;
        case PARAM_ID_TRAPEZOIDAL_VELOCITY:
            params->ds402.trapezoidal_velocity = (int32_t)value;
            break;
        case PARAM_ID_TRAPEZOIDAL_ACCELERATION:
            params->ds402.trapezoidal_acceleration = (int32_t)value;
            break;
        case PARAM_ID_TRAPEZOIDAL_DECELERATION:
            params->ds402.trapezoidal_deceleration = (int32_t)value;
            break;
        case PARAM_ID_QUICK_STOP_DECELERATION:
            params->ds402.quick_stop_deceleration = (int32_t)value;
            break;
        case PARAM_ID_POSITION_ENCODER_RESOLUTION:
            params->ds402.position_encoder_resolution = value;
            break;
        case PARAM_ID_DS402_ENCODER_RESOLUTION:
            params->ds402.position_encoder_resolution = value;
            break;
        case PARAM_ID_MOTOR_REVOLUTIONS:
            params->ds402.motor_revolutions = value;
            break;
        case PARAM_ID_BUS_GEAR_RATIO:
            if (sub_index < 4) {
                params->ds402.bus_gear_ratio[sub_index] = value;
            }
            break;
        case PARAM_ID_FEED_CONSTANT:
            if (sub_index < 3) {
                params->ds402.feed_constant[sub_index] = value;
            }
            break;
        case PARAM_ID_POS_FACTOR:
            if (sub_index < 3) {
                params->ds402.pos_factor[sub_index] = value;
            }
            break;
        case PARAM_ID_SPEED_FACTOR:
            if (sub_index < 3) {
                params->ds402.speed_factor[sub_index] = value;
            }
            break;
        case PARAM_ID_ACC_FACTOR:
            if (sub_index < 3) {
                params->ds402.acc_factor[sub_index] = value;
            }
            break;
        case PARAM_ID_HOME_MODE:
            params->ds402.home_mode = (uint8_t)value;
            break;
        case PARAM_ID_HOME_SPEED:
            if (sub_index < 7) {
                params->ds402.home_speed[sub_index] = value;
            }
            break;
        case PARAM_ID_HOME_ACCELERATION:
            params->ds402.home_acceleration = (int32_t)value;
            break;
        
        /* 新增DS402 驱动配置对象区参数 */
        case PARAM_ID_CAPTURE_FUNCTION:
            params->ds402.capture_function = (uint8_t)value;
            break;
        case PARAM_ID_CAPTURE_STATUS:
            params->ds402.capture_status = (uint8_t)value;
            break;
        case PARAM_ID_RISING_EDGE_CAPTURE_POSITION1:
            params->ds402.rising_edge_capture_position1 = (int32_t)value;
            break;
        case PARAM_ID_FALLING_EDGE_CAPTURE_POSITION1:
            params->ds402.falling_edge_capture_position1 = (int32_t)value;
            break;
        case PARAM_ID_RISING_EDGE_CAPTURE_POSITION2:
            params->ds402.rising_edge_capture_position2 = (int32_t)value;
            break;
        case PARAM_ID_FALLING_EDGE_CAPTURE_POSITION2:
            params->ds402.falling_edge_capture_position2 = (int32_t)value;
            break;
        case PARAM_ID_INTERPOLATION_SUBMODE:
            params->ds402.interpolation_submode = (uint8_t)value;
            break;
        case PARAM_ID_INTERPOLATION_DATA:
            if (sub_index < 3) {
                params->ds402.interpolation_data[sub_index] = value;
            }
            break;
        case PARAM_ID_INTERPOLATION_TIME:
            if (sub_index < 3) {
                params->ds402.interpolation_time[sub_index] = value;
            }
            break;
        case PARAM_ID_IP_SYNC_DEFINITION:
            if (sub_index < 3) {
                params->ds402.ip_sync_definition[sub_index] = value;
            }
            break;
        case PARAM_ID_GROUP_IP_DATA_CFG:
            if (sub_index < 7) {
                params->ds402.group_ip_data_cfg[sub_index] = value;
            }
            break;
        case PARAM_ID_POSITION_FOLLOWING_ERROR:
            params->ds402.position_following_error = (int32_t)value;
            break;
        case PARAM_ID_CURRENT_LOOP_GROUP2:
            if (sub_index < 9) {
                params->ds402.current_loop_group2[sub_index] = value;
            }
            break;
        case PARAM_ID_CURRENT_GROUP:
            if (sub_index < 43) {
                params->ds402.current_group[sub_index] = value;
            }
            break;
        case PARAM_ID_POWER_GROUP:
            if (sub_index < 16) {
                params->ds402.power_group[sub_index] = value;
            }
            break;
        case PARAM_ID_SPEED_GROUP:
            if (sub_index < 44) {
                params->ds402.speed_group[sub_index] = value;
            }
            break;
        case PARAM_ID_POSITION_GROUP:
            if (sub_index < 20) {
                params->ds402.position_group[sub_index] = value;
            }
            break;
        case PARAM_ID_EFFECTIVE_TARGET_POSITION2:
            params->ds402.effective_target_position2 = (int32_t)value;
            break;
        case PARAM_ID_INPUT_STATUS:
            params->ds402.input_status = (uint16_t)value;
            break;
        case PARAM_ID_OUTPUT_GROUP:
            if (sub_index < 2) {
                params->ds402.output_group[sub_index] = value;
            }
            break;
        case PARAM_ID_TARGET_VELOCITY:
            params->ds402.target_velocity = (int32_t)value;
            break;
        
        /* 电机参数区 (0x6400 - 0x6FFF) */
        case PARAM_ID_MOTOR_CATALOG_CODE:
            params->param_area.motor_catalog_code = value;
            break;
        case PARAM_ID_MOTOR_GROUP:
            if (sub_index < 32) {
                params->param_area.motor_group[sub_index] = value;
            }
            break;
        case PARAM_ID_SUPPORT_MODE:
            params->param_area.support_mode = value;
            break;
        case PARAM_ID_MANUFACTURER:
            params->param_area.manufacturer = value;
            break;
        case PARAM_ID_DEVICE_GROUP:
            if (sub_index < 17) {
                params->param_area.device_group[sub_index] = value;
            }
            break;
        default:
            break;
    }
}

/**
  * @brief  获取带子索引的电机参数
  * @param  params: 电机参数结构体指针
  * @param  param_id: 参数ID
  * @param  sub_index: 子索引
  * @retval 参数值
  */
uint32_t motor_params_get_sub(motor_params_t *params, uint16_t param_id, uint8_t sub_index) {
    if (params == NULL) return 0;
    
    switch (param_id) {
        /* 错误字段 */
        case PARAM_ID_ERROR_FIELD:
            if (sub_index < 8) {
                return params->canopen.error_field[sub_index];
            }
            break;
        
        /* 标识对象 */
        case PARAM_ID_IDENTITY_OBJECT:
            if (sub_index < 5) {
                return params->canopen.identity_object[sub_index];
            }
            break;
        
        /* RPDO通信参数 */
        case PARAM_ID_RPDO1_COMM:
            if (sub_index < 4) {
                return params->comm.rpdo1_comm[sub_index];
            }
            break;
        case PARAM_ID_RPDO2_COMM:
            if (sub_index < 4) {
                return params->comm.rpdo2_comm[sub_index];
            }
            break;
        case PARAM_ID_RPDO3_COMM:
            if (sub_index < 4) {
                return params->comm.rpdo3_comm[sub_index];
            }
            break;
        case PARAM_ID_RPDO4_COMM:
            if (sub_index < 4) {
                return params->comm.rpdo4_comm[sub_index];
            }
            break;
        case PARAM_ID_RPDO5_COMM:
            if (sub_index < 4) {
                return params->comm.rpdo5_comm[sub_index];
            }
            break;
        case PARAM_ID_RPDO6_COMM:
            if (sub_index < 4) {
                return params->comm.rpdo6_comm[sub_index];
            }
            break;
        case PARAM_ID_RPDO7_COMM:
            if (sub_index < 4) {
                return params->comm.rpdo7_comm[sub_index];
            }
            break;
        case PARAM_ID_RPDO8_COMM:
            if (sub_index < 4) {
                return params->comm.rpdo8_comm[sub_index];
            }
            break;
        
        /* RPDO映射参数 */
        case PARAM_ID_RPDO1_MAP:
            if (sub_index < 8) {
                return params->comm.rpdo1_map[sub_index];
            }
            break;
        case PARAM_ID_RPDO2_MAP:
            if (sub_index < 8) {
                return params->comm.rpdo2_map[sub_index];
            }
            break;
        case PARAM_ID_RPDO3_MAP:
            if (sub_index < 8) {
                return params->comm.rpdo3_map[sub_index];
            }
            break;
        case PARAM_ID_RPDO4_MAP:
            if (sub_index < 8) {
                return params->comm.rpdo4_map[sub_index];
            }
            break;
        case PARAM_ID_RPDO5_MAP:
            if (sub_index < 8) {
                return params->comm.rpdo5_map[sub_index];
            }
            break;
        case PARAM_ID_RPDO6_MAP:
            if (sub_index < 8) {
                return params->comm.rpdo6_map[sub_index];
            }
            break;
        case PARAM_ID_RPDO7_MAP:
            if (sub_index < 8) {
                return params->comm.rpdo7_map[sub_index];
            }
            break;
        case PARAM_ID_RPDO8_MAP:
            if (sub_index < 8) {
                return params->comm.rpdo8_map[sub_index];
            }
            break;
        
        /* TPDO通信参数 */
        case PARAM_ID_TPDO1_COMM:
            if (sub_index < 6) {
                return params->comm.tpdo1_comm[sub_index];
            }
            break;
        case PARAM_ID_TPDO2_COMM:
            if (sub_index < 6) {
                return params->comm.tpdo2_comm[sub_index];
            }
            break;
        case PARAM_ID_TPDO3_COMM:
            if (sub_index < 6) {
                return params->comm.tpdo3_comm[sub_index];
            }
            break;
        case PARAM_ID_TPDO4_COMM:
            if (sub_index < 6) {
                return params->comm.tpdo4_comm[sub_index];
            }
            break;
        case PARAM_ID_TPDO5_COMM:
            if (sub_index < 6) {
                return params->comm.tpdo5_comm[sub_index];
            }
            break;
        case PARAM_ID_TPDO6_COMM:
            if (sub_index < 6) {
                return params->comm.tpdo6_comm[sub_index];
            }
            break;
        case PARAM_ID_TPDO7_COMM:
            if (sub_index < 6) {
                return params->comm.tpdo7_comm[sub_index];
            }
            break;
        case PARAM_ID_TPDO8_COMM:
            if (sub_index < 6) {
                return params->comm.tpdo8_comm[sub_index];
            }
            break;
        
        /* TPDO映射参数 */
        case PARAM_ID_TPDO1_MAP:
            if (sub_index < 8) {
                return params->comm.tpdo1_map[sub_index];
            }
            break;
        case PARAM_ID_TPDO2_MAP:
            if (sub_index < 8) {
                return params->comm.tpdo2_map[sub_index];
            }
            break;
        case PARAM_ID_TPDO3_MAP:
            if (sub_index < 8) {
                return params->comm.tpdo3_map[sub_index];
            }
            break;
        case PARAM_ID_TPDO4_MAP:
            if (sub_index < 8) {
                return params->comm.tpdo4_map[sub_index];
            }
            break;
        case PARAM_ID_TPDO5_MAP:
            if (sub_index < 8) {
                return params->comm.tpdo5_map[sub_index];
            }
            break;
        case PARAM_ID_TPDO6_MAP:
            if (sub_index < 8) {
                return params->comm.tpdo6_map[sub_index];
            }
            break;
        case PARAM_ID_TPDO7_MAP:
            if (sub_index < 8) {
                return params->comm.tpdo7_map[sub_index];
            }
            break;
        case PARAM_ID_TPDO8_MAP:
            if (sub_index < 8) {
                return params->comm.tpdo8_map[sub_index];
            }
            break;
        
        /* NMT启动/停止 */
        case PARAM_ID_NMT_START_STOP:
            if (sub_index < 8) {
                return params->comm.nmt_start_stop[sub_index];
            }
            break;
        case PARAM_ID_NMT_START_STOP_CH0:
            if (sub_index < 8) {
                return params->comm.nmt_start_stop_ch0[sub_index];
            }
            break;
        case PARAM_ID_NMT_START_STOP_CH1:
            if (sub_index < 8) {
                return params->comm.nmt_start_stop_ch1[sub_index];
            }
            break;
        case PARAM_ID_NMT_START_STOP_CH2:
            if (sub_index < 8) {
                return params->comm.nmt_start_stop_ch2[sub_index];
            }
            break;
        case PARAM_ID_NMT_START_STOP_CH3:
            if (sub_index < 8) {
                return params->comm.nmt_start_stop_ch3[sub_index];
            }
            break;
        
        /* 同步输入输出 */
        case PARAM_ID_SYNC_OUTPUT:
            if (sub_index < 13) {
                return params->comm.sync_output[sub_index];
            }
            break;
        case PARAM_ID_SYNC_INPUT:
            if (sub_index < 13) {
                return params->comm.sync_input[sub_index];
            }
            break;
        
        /* 制造商特定参数 */
        case PARAM_ID_POWER_ON_ENABLE:
            return params->mfg.power_on_enable;
        case PARAM_ID_IO_GROUP:
            if (sub_index < 40) {
                return params->mfg.io_group[sub_index];
            }
            break;
        case PARAM_ID_INPUT_FILTER:
            if (sub_index < 8) {
                return params->mfg.input_filter[sub_index];
            }
            break;
        case PARAM_ID_FAST_CAPTURE:
            return params->mfg.fast_capture;
        case PARAM_ID_RISING_CAPTURE_COUNT:
            return params->mfg.rising_capture_count;
        case PARAM_ID_RISING_CAPTURE_DATA:
            if (sub_index < 32) {
                return params->mfg.rising_capture_data[sub_index];
            }
            break;
        case PARAM_ID_FALLING_CAPTURE_COUNT:
            return params->mfg.falling_capture_count;
        case PARAM_ID_FALLING_CAPTURE_DATA:
            if (sub_index < 32) {
                return params->mfg.falling_capture_data[sub_index];
            }
            break;
        case PARAM_ID_INPUT_DELAY:
            if (sub_index < 7) {
                return params->mfg.input_delay[sub_index];
            }
            break;
        case PARAM_ID_CAPTURE_COMPENSATION:
            if (sub_index < 4) {
                return params->mfg.capture_compensation[sub_index];
            }
            break;
        case PARAM_ID_DIN_CONTROL:
            if (sub_index < 52) {
                return params->mfg.din_control[sub_index];
            }
            break;
        case PARAM_ID_INDEX_WINDOW:
            return params->mfg.index_window;
        case PARAM_ID_OSCILLOSCOPE:
            if (sub_index < 14) {
                return params->mfg.oscilloscope[sub_index];
            }
            break;
        case PARAM_ID_AUTO_FLIP:
            if (sub_index < 6) {
                return params->mfg.auto_flip[sub_index];
            }
            break;
        case PARAM_ID_PHASE_CORRECTION:
            if (sub_index < 14) {
                return params->mfg.phase_correction[sub_index];
            }
            break;
        case PARAM_ID_SYSTEM_TEST:
            if (sub_index < 13) {
                return params->mfg.system_test[sub_index];
            }
            break;
        case PARAM_ID_SPECIAL_FUNCTION:
            if (sub_index < 18) {
                return params->mfg.special_function[sub_index];
            }
            break;
        case PARAM_ID_HIGH_VOLTAGE_SAMPLING:
            return params->mfg.high_voltage_sampling;
        case PARAM_ID_ADC_GROUP:
            if (sub_index < 14) {
                return params->mfg.adc_group[sub_index];
            }
            break;
        case PARAM_ID_ANALOG_INPUT:
            if (sub_index < 17) {
                return params->mfg.analog_input[sub_index];
            }
            break;
        case PARAM_ID_PULSE_GROUP2:
            if (sub_index < 3) {
                return params->mfg.pulse_group2[sub_index];
            }
            break;
        case PARAM_ID_PULSE_GROUP:
            if (sub_index < 16) {
                return params->mfg.pulse_group[sub_index];
            }
            break;
        case PARAM_ID_PULSE_GROUP2B:
            if (sub_index < 15) {
                return params->mfg.pulse_group2b[sub_index];
            }
            break;
        case PARAM_ID_FULL_CLOSED_LOOP:
            if (sub_index < 10) {
                return params->mfg.full_closed_loop[sub_index];
            }
            break;
        case PARAM_ID_NOTCH_FILTER:
            if (sub_index < 9) {
                return params->mfg.notch_filter[sub_index];
            }
            break;
        case PARAM_ID_NOTCH_FILTER2:
            if (sub_index < 9) {
                return params->mfg.notch_filter2[sub_index];
            }
            break;
        case PARAM_ID_FIELD_WEAKENING:
            if (sub_index < 9) {
                return params->mfg.field_weakening[sub_index];
            }
            break;
        case PARAM_ID_FILTER_EXTENSION:
            if (sub_index < 12) {
                return params->mfg.filter_extension[sub_index];
            }
            break;
        case PARAM_ID_ONLINE_TUNING:
            if (sub_index < 14) {
                return params->mfg.online_tuning[sub_index];
            }
            break;
        case PARAM_ID_ERROR_MASK:
            return params->mfg.error_mask;
        case PARAM_ID_ERROR_STATUS:
            return params->mfg.error_status;
        case PARAM_ID_ERROR_STATUS2:
            return params->mfg.error_status2;
        case PARAM_ID_ERROR_MASK_GROUP:
            if (sub_index < 7) {
                return params->mfg.error_mask_group[sub_index];
            }
            break;
        case PARAM_ID_ERROR_GROUP0:
            if (sub_index < 10) {
                return params->mfg.error_group0[sub_index];
            }
            break;
        case PARAM_ID_ERROR_GROUP1:
            if (sub_index < 10) {
                return params->mfg.error_group1[sub_index];
            }
            break;
        case PARAM_ID_ERROR_GROUP2:
            if (sub_index < 10) {
                return params->mfg.error_group2[sub_index];
            }
            break;
        case PARAM_ID_ERROR_GROUP3:
            if (sub_index < 10) {
                return params->mfg.error_group3[sub_index];
            }
            break;
        case PARAM_ID_ERROR_GROUP4:
            if (sub_index < 10) {
                return params->mfg.error_group4[sub_index];
            }
            break;
        case PARAM_ID_ERROR_GROUP5:
            if (sub_index < 10) {
                return params->mfg.error_group5[sub_index];
            }
            break;
        case PARAM_ID_ERROR_GROUP6:
            if (sub_index < 10) {
                return params->mfg.error_group6[sub_index];
            }
            break;
        case PARAM_ID_ERROR_GROUP7:
            if (sub_index < 10) {
                return params->mfg.error_group7[sub_index];
            }
            break;
        
        /* 新增制造商特定参数 */
        case PARAM_ID_WARNING_STATUS:
            return params->mfg.warning_status;
        case PARAM_ID_ENCODER_RESET:
            return params->mfg.encoder_reset;
        case PARAM_ID_ENCODER_TYPE:
            return params->mfg.encoder_type;
        case PARAM_ID_ENCODER_TYPE2:
            return params->mfg.encoder_type2;
        case PARAM_ID_ENCODER_FLAGS:
            return params->mfg.encoder_flags;
        case PARAM_ID_ENCODER_STATUS:
            return params->mfg.encoder_status;
        case PARAM_ID_PHASE_LOSS_DETECTION:
            return params->mfg.phase_loss_detection;
        case PARAM_ID_GROUP_TOUCH:
            if (sub_index < 9) {
                return params->mfg.group_touch[sub_index];
            }
            break;
        case PARAM_ID_DEVICE_NODE_GROUP:
            if (sub_index < 2) {
                return params->mfg.device_node_group[sub_index];
            }
            break;
        case PARAM_ID_CAN_BAUDRATE:
            return params->mfg.can_baudrate;
        case PARAM_ID_RS232_BAUDRATE:
            return params->mfg.rs232_baudrate;
        case PARAM_ID_RS232_BAUDRATE_DEBUG:
            if (sub_index < 2) {
                return params->mfg.rs232_baudrate_debug[sub_index];
            }
            break;
        case PARAM_ID_RS485_BAUDRATE:
            return params->mfg.rs485_baudrate;
        case PARAM_ID_RS485_BAUDRATE_DEBUG:
            return params->mfg.rs485_baudrate_debug;
        case PARAM_ID_EEPROM_GROUP:
            if (sub_index < 33) {
                return params->mfg.eeprom_group[sub_index];
            }
            break;
        case PARAM_ID_PANEL_GROUP:
            if (sub_index < 36) {
                return params->mfg.panel_group[sub_index];
            }
            break;
        case PARAM_ID_PANEL_GROUP2:
            if (sub_index < 4) {
                return params->mfg.panel_group2[sub_index];
            }
            break;
        
        /* 新增制造商特定参数 */
        case PARAM_ID_DEVICE_TIME:
            return params->mfg.device_time;
        case PARAM_ID_HARDWARE_SERIAL:
            return params->mfg.hardware_serial;
        case PARAM_ID_USER_PASSWORD:
            return params->mfg.user_password;
        case PARAM_ID_FACTORY_PASSWORD:
            return params->mfg.factory_password;
        case PARAM_ID_DEVICE_RESTART:
            return params->mfg.device_restart;
        case PARAM_ID_TEST_GROUP:
            if (sub_index < 27) {
                return params->mfg.test_group[sub_index];
            }
            break;
        case PARAM_ID_PROCESSOR_GROUP:
            if (sub_index < 29) {
                return params->mfg.processor_group[sub_index];
            }
            break;
        case PARAM_ID_ECAN_GROUP:
            if (sub_index < 9) {
                return params->mfg.ecan_group[sub_index];
            }
            break;
        case PARAM_ID_INTERPOLATION_GROUP:
            if (sub_index < 11) {
                return params->mfg.interpolation_group[sub_index];
            }
            break;
        case PARAM_ID_PARAMETER_CHECKSUM:
            if (sub_index < 4) {
                return params->mfg.parameter_checksum[sub_index];
            }
            break;
        case PARAM_ID_SHORT_CIRCUIT_BRAKE:
            if (sub_index < 9) {
                return params->mfg.short_circuit_brake[sub_index];
            }
            break;
        case PARAM_ID_LOAD_TEST_SWEEP:
            if (sub_index < 12) {
                return params->mfg.load_test_sweep[sub_index];
            }
            break;
        case PARAM_ID_QUICK_SETTING:
            if (sub_index < 7) {
                return params->mfg.quick_setting[sub_index];
            }
            break;
        case PARAM_ID_CURRENT_POSITION_TABLE:
            return params->mfg.current_position_table;
        case PARAM_ID_ENCODER_COMM_STATUS:
            return params->mfg.encoder_communication_status;
        case PARAM_ID_GROUP_RESOLVER:
            if (sub_index < 10) {
                return params->mfg.group_resolver[sub_index];
            }
            break;
        case PARAM_ID_ENCODER_INFO:
            return params->mfg.encoder_info;
        case PARAM_ID_ENCODER_WARNING:
            return params->mfg.encoder_warning;
        case PARAM_ID_ENCODER_TEMPERATURE:
            return params->mfg.encoder_temperature;
        case PARAM_ID_INERTIA_IDENTIFICATION:
            if (sub_index < 38) {
                return params->mfg.inertia_identification[sub_index];
            }
            break;
        case PARAM_ID_VIBRATION_SUPPRESSION_A:
            if (sub_index < 6) {
                return params->mfg.vibration_suppression_a[sub_index];
            }
            break;
        case PARAM_ID_LOW_FREQUENCY_VIBRATION:
            if (sub_index < 11) {
                return params->mfg.low_frequency_vibration[sub_index];
            }
            break;
        case PARAM_ID_INPUT_SHAPER:
            if (sub_index < 6) {
                return params->mfg.input_shaper[sub_index];
            }
            break;
        case PARAM_ID_MODEL_TRACKING:
            if (sub_index < 3) {
                return params->mfg.model_tracking[sub_index];
            }
            break;
        case PARAM_ID_ADAPTIVE_NOTCH_FILTER:
            if (sub_index < 16) {
                return params->mfg.adaptive_notch_filter[sub_index];
            }
            break;
        case PARAM_ID_PROBE_FUNCTION:
            if (sub_index < 13) {
                return params->mfg.probe_function[sub_index];
            }
            break;
        case PARAM_ID_SYSTEM_FREQUENCY_SWITCH:
            if (sub_index < 3) {
                return params->mfg.system_frequency_switch[sub_index];
            }
            break;
        
        /* 新增PN相关参数 */
        case PARAM_ID_PN_SOFTWARE_VERSION:
            return params->pn.pn_software_version;
        case PARAM_ID_NETWORK_PORT_STATUS:
            return params->pn.network_port_status;
        case PARAM_ID_PN_DEVICE_NAME:
            return params->pn.pn_device_name;
        case PARAM_ID_PN_DEVICE_NAME1:
            return params->pn.pn_device_name1;
        case PARAM_ID_PN_DEVICE_NAME2:
            return params->pn.pn_device_name2;
        case PARAM_ID_PN_DEVICE_NAME3:
            return params->pn.pn_device_name3;
        case PARAM_ID_PN_DEVICE_NAME4:
            return params->pn.pn_device_name4;
        case PARAM_ID_IP_ADDRESS:
            return params->pn.ip_address;
        case PARAM_ID_SUBNET_MASK:
            return params->pn.subnet_mask;
        case PARAM_ID_DEFAULT_GATEWAY:
            return params->pn.default_gateway;
        case PARAM_ID_PN_ENABLE:
            return params->pn.pn_enable;
        case PARAM_ID_SOFTWARE_LIMIT_ENABLE:
            return params->pn.software_limit_enable;
        case PARAM_ID_PN_USER_RX_WORD:
            return params->pn.pn_user_rx_word;
        case PARAM_ID_PN_USER_TX_WORD:
            return params->pn.pn_user_tx_word;
        case PARAM_ID_MAC_ADDRESS1:
            return params->pn.mac_address1;
        case PARAM_ID_MAC_ADDRESS2:
            return params->pn.mac_address2;
        
        /* DS402 驱动配置对象区参数 */
        case PARAM_ID_MOTOR_ABSOLUTE_POSITION:
            return (uint32_t)params->ds402.motor_absolute_position;
        case PARAM_ID_COMMUNICATION_INTERRUPT_MODE:
            return (uint32_t)params->ds402.communication_interrupt_mode;
        case PARAM_ID_ERROR_CODE:
            return params->ds402.error_code;
        case PARAM_ID_DS402_CONTROL_WORD:
            return (uint32_t)params->ds402.control_word;
        case PARAM_ID_DS402_STATUS_WORD:
            return (uint32_t)params->ds402.status_word;
        case PARAM_ID_QUICK_STOP_MODE:
            return (uint32_t)params->ds402.quick_stop_mode;
        case PARAM_ID_SHUTDOWN_STOP_MODE:
            return (uint32_t)params->ds402.shutdown_stop_mode;
        case PARAM_ID_DISABLE_STOP_MODE:
            return (uint32_t)params->ds402.disable_stop_mode;
        case PARAM_ID_HALT_MODE:
            return (uint32_t)params->ds402.halt_mode;
        case PARAM_ID_ERROR_STOP_MODE:
            return (uint32_t)params->ds402.error_stop_mode;
        case PARAM_ID_DS402_OPERATION_MODE:
            return (uint32_t)params->ds402.operation_mode;
        case PARAM_ID_ACTIVE_OPERATION_MODE:
            return (uint32_t)params->ds402.active_operation_mode;
        case PARAM_ID_EFFECTIVE_TARGET_POSITION:
            return (uint32_t)params->ds402.effective_target_position;
        case PARAM_ID_ACTUAL_POSITION_INC:
            return (uint32_t)params->ds402.actual_position_inc;
        case PARAM_ID_ACTUAL_POSITION_PULSE:
            return (uint32_t)params->ds402.actual_position_pulse;
        case PARAM_ID_MAX_FOLLOWING_ERROR:
            return (uint32_t)params->ds402.max_following_error;
        case PARAM_ID_TARGET_POSITION_WINDOW:
            return (uint32_t)params->ds402.target_position_window;
        case PARAM_ID_POSITION_WINDOW_TIME:
            return params->ds402.position_window_time;
        case PARAM_ID_EFFECTIVE_TARGET_VELOCITY:
            return (uint32_t)params->ds402.effective_target_velocity;
        case PARAM_ID_ACTUAL_VELOCITY:
            return (uint32_t)params->ds402.actual_velocity;
        case PARAM_ID_TARGET_TORQUE_PERCENT:
            return (uint32_t)params->ds402.target_torque_percent;
        case PARAM_ID_MAX_TORQUE_LIMIT_PERCENT:
            return (uint32_t)params->ds402.max_torque_limit_percent;
        case PARAM_ID_TARGET_CURRENT_LIMIT:
            return (uint32_t)params->ds402.target_current_limit;
        case PARAM_ID_MOTOR_RATED_CURRENT:
            return (uint32_t)params->ds402.motor_rated_current;
        case PARAM_ID_MOTOR_RATED_TORQUE:
            return (uint32_t)params->ds402.motor_rated_torque;
        case PARAM_ID_ACTUAL_TORQUE:
            return (uint32_t)params->ds402.actual_torque;
        case PARAM_ID_ACTUAL_CURRENT:
            return (uint32_t)params->ds402.actual_current;
        case PARAM_ID_ACTUAL_BUS_VOLTAGE_MV:
            return (uint32_t)params->ds402.actual_bus_voltage_mv;
        case PARAM_ID_DS402_TARGET_POSITION:
        case PARAM_ID_TARGET_POSITION:
            return (uint32_t)params->ds402.target_position;
        case PARAM_ID_HOME_OFFSET:
            return (uint32_t)params->ds402.home_offset;
        case PARAM_ID_SOFT_LIMIT:
            if (sub_index < 3) {
                return (uint32_t)params->ds402.soft_limit[sub_index];
            }
            break;
        case PARAM_ID_VELOCITY_POSITION_DIRECTION:
            return (uint32_t)params->ds402.velocity_position_direction;
        case PARAM_ID_MAX_VELOCITY_LIMIT:
            return (uint32_t)params->ds402.max_velocity_limit;
        case PARAM_ID_MAX_VELOCITY_LIMIT_RPM:
            return (uint32_t)params->ds402.max_velocity_limit_rpm;
        case PARAM_ID_TRAPEZOIDAL_VELOCITY:
            return (uint32_t)params->ds402.trapezoidal_velocity;
        case PARAM_ID_TRAPEZOIDAL_ACCELERATION:
            return (uint32_t)params->ds402.trapezoidal_acceleration;
        case PARAM_ID_TRAPEZOIDAL_DECELERATION:
            return (uint32_t)params->ds402.trapezoidal_deceleration;
        case PARAM_ID_QUICK_STOP_DECELERATION:
            return (uint32_t)params->ds402.quick_stop_deceleration;
        case PARAM_ID_POSITION_ENCODER_RESOLUTION:
            return params->ds402.position_encoder_resolution;
        case PARAM_ID_DS402_ENCODER_RESOLUTION:
            return params->ds402.position_encoder_resolution;
        case PARAM_ID_MOTOR_REVOLUTIONS:
            return params->ds402.motor_revolutions;
        case PARAM_ID_BUS_GEAR_RATIO:
            if (sub_index < 4) {
                return params->ds402.bus_gear_ratio[sub_index];
            }
            break;
        case PARAM_ID_FEED_CONSTANT:
            if (sub_index < 3) {
                return params->ds402.feed_constant[sub_index];
            }
            break;
        case PARAM_ID_POS_FACTOR:
            if (sub_index < 3) {
                return params->ds402.pos_factor[sub_index];
            }
            break;
        case PARAM_ID_SPEED_FACTOR:
            if (sub_index < 3) {
                return params->ds402.speed_factor[sub_index];
            }
            break;
        case PARAM_ID_ACC_FACTOR:
            if (sub_index < 3) {
                return params->ds402.acc_factor[sub_index];
            }
            break;
        case PARAM_ID_HOME_MODE:
            return (uint32_t)params->ds402.home_mode;
        case PARAM_ID_HOME_SPEED:
            if (sub_index < 7) {
                return params->ds402.home_speed[sub_index];
            }
            break;
        case PARAM_ID_HOME_ACCELERATION:
            return (uint32_t)params->ds402.home_acceleration;
        
        /* 新增DS402 驱动配置对象区参数 */
        case PARAM_ID_CAPTURE_FUNCTION:
            return (uint32_t)params->ds402.capture_function;
        case PARAM_ID_CAPTURE_STATUS:
            return (uint32_t)params->ds402.capture_status;
        case PARAM_ID_RISING_EDGE_CAPTURE_POSITION1:
            return (uint32_t)params->ds402.rising_edge_capture_position1;
        case PARAM_ID_FALLING_EDGE_CAPTURE_POSITION1:
            return (uint32_t)params->ds402.falling_edge_capture_position1;
        case PARAM_ID_RISING_EDGE_CAPTURE_POSITION2:
            return (uint32_t)params->ds402.rising_edge_capture_position2;
        case PARAM_ID_FALLING_EDGE_CAPTURE_POSITION2:
            return (uint32_t)params->ds402.falling_edge_capture_position2;
        case PARAM_ID_INTERPOLATION_SUBMODE:
            return (uint32_t)params->ds402.interpolation_submode;
        case PARAM_ID_INTERPOLATION_DATA:
            if (sub_index < 3) {
                return params->ds402.interpolation_data[sub_index];
            }
            break;
        case PARAM_ID_INTERPOLATION_TIME:
            if (sub_index < 3) {
                return params->ds402.interpolation_time[sub_index];
            }
            break;
        case PARAM_ID_IP_SYNC_DEFINITION:
            if (sub_index < 3) {
                return params->ds402.ip_sync_definition[sub_index];
            }
            break;
        case PARAM_ID_GROUP_IP_DATA_CFG:
            if (sub_index < 7) {
                return params->ds402.group_ip_data_cfg[sub_index];
            }
            break;
        case PARAM_ID_POSITION_FOLLOWING_ERROR:
            return (uint32_t)params->ds402.position_following_error;
        case PARAM_ID_CURRENT_LOOP_GROUP2:
            if (sub_index < 9) {
                return params->ds402.current_loop_group2[sub_index];
            }
            break;
        case PARAM_ID_CURRENT_GROUP:
            if (sub_index < 43) {
                return params->ds402.current_group[sub_index];
            }
            break;
        case PARAM_ID_POWER_GROUP:
            if (sub_index < 16) {
                return params->ds402.power_group[sub_index];
            }
            break;
        case PARAM_ID_SPEED_GROUP:
            if (sub_index < 44) {
                return params->ds402.speed_group[sub_index];
            }
            break;
        case PARAM_ID_POSITION_GROUP:
            if (sub_index < 20) {
                return params->ds402.position_group[sub_index];
            }
            break;
        case PARAM_ID_EFFECTIVE_TARGET_POSITION2:
            return (uint32_t)params->ds402.effective_target_position2;
        case PARAM_ID_INPUT_STATUS:
            return (uint32_t)params->ds402.input_status;
        case PARAM_ID_OUTPUT_GROUP:
            if (sub_index < 2) {
                return params->ds402.output_group[sub_index];
            }
            break;
        case PARAM_ID_TARGET_VELOCITY:
            return (uint32_t)params->ds402.target_velocity;
        
        /* 电机参数区 (0x6400 - 0x6FFF) */
        case PARAM_ID_MOTOR_CATALOG_CODE:
            return params->param_area.motor_catalog_code;
        case PARAM_ID_MOTOR_GROUP:
            if (sub_index < 32) {
                return params->param_area.motor_group[sub_index];
            }
            break;
        case PARAM_ID_SUPPORT_MODE:
            return params->param_area.support_mode;
        case PARAM_ID_MANUFACTURER:
            return params->param_area.manufacturer;
        case PARAM_ID_DEVICE_GROUP:
            if (sub_index < 17) {
                return params->param_area.device_group[sub_index];
            }
            break;
        default:
            break;
    }
    return 0;
}

/**
  * @brief  清除电机参数中的所有错误状态
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void motor_params_clear_errors(motor_params_t *params) {
    int i;
    if (params == NULL) return;
    
    /* 清除CANopen DS301错误 */
    params->canopen.error_register = 0x00;              /* 错误寄存器 */
    for (i = 0; i < 8; i++) {
        params->canopen.error_field[i] = 0x00;          /* 预定义错误字段 */
    }
    
    /* 清除DS402错误代码 */
    params->ds402.error_code = 0x00000000;              /* 错误代码 */
    
    /* 清除保护故障状态 */
    params->protection.fault_status = 0;                /* 故障状态 */
    params->protection.fault_active = 0;                /* 故障激活 */
    
    /* 清除制造商特定错误状态 */
    params->mfg.error_status = 0x00000000;            /* 错误状态 */
    params->mfg.error_status2 = 0x00000000;           /* 错误状态2 */
    params->mfg.warning_status = 0x00000000;            /* 警告状态字 */
    
    /* 清除错误组 */
    for (i = 0; i < 10; i++) {
        params->mfg.error_group0[i] = 0x00000000;      /* 错误组0 */
        params->mfg.error_group1[i] = 0x00000000;      /* 错误组1 */
        params->mfg.error_group2[i] = 0x00000000;      /* 错误组2 */
        params->mfg.error_group3[i] = 0x00000000;      /* 错误组3 */
        params->mfg.error_group4[i] = 0x00000000;      /* 错误组4 */
        params->mfg.error_group5[i] = 0x00000000;      /* 错误组5 */
        params->mfg.error_group6[i] = 0x00000000;      /* 错误组6 */
        params->mfg.error_group7[i] = 0x00000000;      /* 错误组7 */
    }
}
