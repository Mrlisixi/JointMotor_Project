/* 电机面向对象编程接口 */
#ifndef MOTOR_OBJECT_H
#define MOTOR_OBJECT_H

#include "motor_params.h"

/**
 * @brief  电机基类
 */
typedef struct Motor {
    /* 数据成员 */
    motor_params_t params;
    
    /* 方法 */
    void (*init)(struct Motor *self);
    void (*loadDefault)(struct Motor *self);
    void (*updateParam)(struct Motor *self, uint16_t param_id, float value);
    float (*getParam)(struct Motor *self, uint16_t param_id);
    void (*updateParamSub)(struct Motor *self, uint16_t param_id, uint8_t sub_index, uint32_t value);
    uint32_t (*getParamSub)(struct Motor *self, uint16_t param_id, uint8_t sub_index);
    void (*clearErrors)(struct Motor *self);
    
    /* 状态管理 */
    void (*setState)(struct Motor *self, motor_state_t state);
    motor_state_t (*getState)(struct Motor *self);
    
    /* 控制方法 */
    void (*enable)(struct Motor *self);
    void (*disable)(struct Motor *self);
    void (*setControlMode)(struct Motor *self, foc_control_mode_t mode);
    void (*setTargetPosition)(struct Motor *self, float position);
    void (*setTargetVelocity)(struct Motor *self, float velocity);
    void (*setTargetTorque)(struct Motor *self, float torque);
    
    /* 监控方法 */
    float (*getUPhaseCurrent)(struct Motor *self);
    float (*getVPhaseCurrent)(struct Motor *self);
    float (*getWPhaseCurrent)(struct Motor *self);
    float (*getBusVoltage)(struct Motor *self);
    float (*getTemperature)(struct Motor *self);
    
    /* 保护方法 */
    uint8_t (*checkFault)(struct Motor *self);
    void (*resetFault)(struct Motor *self);
    
    /* 调试方法 */
    void (*printStatus)(struct Motor *self);
} Motor;

/**
 * @brief  基本参数类
 */
typedef struct BasicParams {
    motor_basic_params_t *params;
    
    void (*setPolePairs)(struct BasicParams *self, uint16_t pole_pairs);
    uint16_t (*getPolePairs)(struct BasicParams *self);
    
    void (*setBusVoltage)(struct BasicParams *self, float voltage);
    float (*getBusVoltage)(struct BasicParams *self);
    
    void (*setMaxCurrent)(struct BasicParams *self, float current);
    float (*getMaxCurrent)(struct BasicParams *self);
    
    void (*setMaxVelocity)(struct BasicParams *self, float velocity);
    float (*getMaxVelocity)(struct BasicParams *self);
    
    void (*setMaxTorque)(struct BasicParams *self, float torque);
    float (*getMaxTorque)(struct BasicParams *self);
    
    void (*setControlDt)(struct BasicParams *self, float dt);
    float (*getControlDt)(struct BasicParams *self);
    
    void (*setPwmFrequency)(struct BasicParams *self, float frequency);
    float (*getPwmFrequency)(struct BasicParams *self);
    
    void (*setEncoderResolution)(struct BasicParams *self, uint16_t resolution);
    uint16_t (*getEncoderResolution)(struct BasicParams *self);
} BasicParams;

/**
 * @brief  FOC控制参数类
 */
typedef struct FocParams {
    motor_foc_params_t *params;
    
    void (*setControlMode)(struct FocParams *self, foc_control_mode_t mode);
    foc_control_mode_t (*getControlMode)(struct FocParams *self);
    
    void (*setTargetPosition)(struct FocParams *self, float position);
    float (*getTargetPosition)(struct FocParams *self);
    
    void (*setTargetVelocity)(struct FocParams *self, float velocity);
    float (*getTargetVelocity)(struct FocParams *self);
    
    void (*setTargetTorque)(struct FocParams *self, float torque);
    float (*getTargetTorque)(struct FocParams *self);
    
    float (*getCurrentPosition)(struct FocParams *self);
    float (*getCurrentVelocity)(struct FocParams *self);
    float (*getCurrentTorque)(struct FocParams *self);
    
    void (*setPositionKp)(struct FocParams *self, float kp);
    float (*getPositionKp)(struct FocParams *self);
    
    void (*setPositionKi)(struct FocParams *self, float ki);
    float (*getPositionKi)(struct FocParams *self);
    
    void (*setVelocityKp)(struct FocParams *self, float kp);
    float (*getVelocityKp)(struct FocParams *self);
    
    void (*setVelocityKi)(struct FocParams *self, float ki);
    float (*getVelocityKi)(struct FocParams *self);
    
    void (*setCurrentKp)(struct FocParams *self, float kp);
    float (*getCurrentKp)(struct FocParams *self);
    
    void (*setCurrentKi)(struct FocParams *self, float ki);
    float (*getCurrentKi)(struct FocParams *self);
    
    void (*enable)(struct FocParams *self);
    void (*disable)(struct FocParams *self);
    uint8_t (*isEnabled)(struct FocParams *self);
    
    /* 无感控制 */
    void (*enableSensorless)(struct FocParams *self);
    void (*disableSensorless)(struct FocParams *self);
    uint8_t (*isSensorlessEnabled)(struct FocParams *self);
    
    void (*setSensorlessParams)(struct FocParams *self, float k1, float k2, float k3);
    void (*setSensorlessSpeedRange)(struct FocParams *self, float min_speed, float max_speed);
    
    float (*getEstimatedSpeed)(struct FocParams *self);
    float (*getEstimatedAngle)(struct FocParams *self);
} FocParams;

/**
 * @brief  保护参数类
 */
typedef struct ProtectionParams {
    motor_protection_params_t *params;
    
    void (*setMaxCurrent)(struct ProtectionParams *self, float current);
    float (*getMaxCurrent)(struct ProtectionParams *self);
    
    void (*setMinVoltage)(struct ProtectionParams *self, float voltage);
    float (*getMinVoltage)(struct ProtectionParams *self);
    
    void (*setMaxVoltage)(struct ProtectionParams *self, float voltage);
    float (*getMaxVoltage)(struct ProtectionParams *self);
    
    void (*setMaxTemperature)(struct ProtectionParams *self, float temperature);
    float (*getMaxTemperature)(struct ProtectionParams *self);
    
    void (*enableOvercurrentProtection)(struct ProtectionParams *self);
    void (*disableOvercurrentProtection)(struct ProtectionParams *self);
    uint8_t (*isOvercurrentProtectionEnabled)(struct ProtectionParams *self);
    
    void (*enableUndervoltageProtection)(struct ProtectionParams *self);
    void (*disableUndervoltageProtection)(struct ProtectionParams *self);
    uint8_t (*isUndervoltageProtectionEnabled)(struct ProtectionParams *self);
    
    void (*enableOvervoltageProtection)(struct ProtectionParams *self);
    void (*disableOvervoltageProtection)(struct ProtectionParams *self);
    uint8_t (*isOvervoltageProtectionEnabled)(struct ProtectionParams *self);
    
    void (*enableOvertemperatureProtection)(struct ProtectionParams *self);
    void (*disableOvertemperatureProtection)(struct ProtectionParams *self);
    uint8_t (*isOvertemperatureProtectionEnabled)(struct ProtectionParams *self);
    
    uint8_t (*getFaultStatus)(struct ProtectionParams *self);
    uint8_t (*isFaultActive)(struct ProtectionParams *self);
    void (*clearFault)(struct ProtectionParams *self);
} ProtectionParams;

/**
 * @brief  监控参数类
 */
typedef struct MonitorParams {
    motor_monitor_params_t *params;
    
    void (*setUPhaseCurrent)(struct MonitorParams *self, float current);
    float (*getUPhaseCurrent)(struct MonitorParams *self);
    
    void (*setVPhaseCurrent)(struct MonitorParams *self, float current);
    float (*getVPhaseCurrent)(struct MonitorParams *self);
    
    void (*setWPhaseCurrent)(struct MonitorParams *self, float current);
    float (*getWPhaseCurrent)(struct MonitorParams *self);
    
    void (*setBusVoltage)(struct MonitorParams *self, float voltage);
    float (*getBusVoltage)(struct MonitorParams *self);
    
    void (*setTemperature)(struct MonitorParams *self, float temperature);
    float (*getTemperature)(struct MonitorParams *self);
    
    void (*setSampleFreq)(struct MonitorParams *self, uint32_t freq);
    uint32_t (*getSampleFreq)(struct MonitorParams *self);
} MonitorParams;

/**
 * @brief  PID参数类
 */
typedef struct PidParams {
    motor_pid_params_t *params;
    
    void (*setPositionPid)(struct PidParams *self, float kp, float ki, float kd);
    void (*getPositionPid)(struct PidParams *self, float *kp, float *ki, float *kd);
    
    void (*setVelocityPid)(struct PidParams *self, float kp, float ki, float kd);
    void (*getVelocityPid)(struct PidParams *self, float *kp, float *ki, float *kd);
    
    void (*setCurrentPid)(struct PidParams *self, float kp, float ki, float kd);
    void (*getCurrentPid)(struct PidParams *self, float *kp, float *ki, float *kd);
} PidParams;

/**
 * @brief  校准参数类
 */
typedef struct CalibParams {
    motor_calib_params_t *params;
    
    void (*setPhaseOffset)(struct CalibParams *self, float offset);
    float (*getPhaseOffset)(struct CalibParams *self);
    
    void (*setCurrentOffset)(struct CalibParams *self, uint8_t channel, float offset);
    float (*getCurrentOffset)(struct CalibParams *self, uint8_t channel);
} CalibParams;

/**
 * @brief  CANopen参数类
 */
typedef struct CanopenParams {
    motor_canopen_params_t *params;
    
    void (*setNodeId)(struct CanopenParams *self, uint8_t node_id);
    uint8_t (*getNodeId)(struct CanopenParams *self);
    
    void (*setSyncPeriod)(struct CanopenParams *self, uint32_t period);
    uint32_t (*getSyncPeriod)(struct CanopenParams *self);
    
    void (*setHeartbeatProducer)(struct CanopenParams *self, uint16_t time);
    uint16_t (*getHeartbeatProducer)(struct CanopenParams *self);
    
    void (*setDeviceName)(struct CanopenParams *self, const char *name);
    const char* (*getDeviceName)(struct CanopenParams *self);
    
    void (*setHardwareVersion)(struct CanopenParams *self, const char *version);
    const char* (*getHardwareVersion)(struct CanopenParams *self);
    
    void (*setSoftwareVersion)(struct CanopenParams *self, const char *version);
    const char* (*getSoftwareVersion)(struct CanopenParams *self);
} CanopenParams;

/* 函数声明 */
void Motor_Create(Motor *motor);
void BasicParams_Create(BasicParams *basic, motor_basic_params_t *params);
void FocParams_Create(FocParams *foc, motor_foc_params_t *params);
void ProtectionParams_Create(ProtectionParams *protection, motor_protection_params_t *params);
void MonitorParams_Create(MonitorParams *monitor, motor_monitor_params_t *params);
void PidParams_Create(PidParams *pid, motor_pid_params_t *params);
void CalibParams_Create(CalibParams *calib, motor_calib_params_t *params);
void CanopenParams_Create(CanopenParams *canopen, motor_canopen_params_t *params);

#endif /* MOTOR_OBJECT_H */
