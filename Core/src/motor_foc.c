#include "motor_foc.h"
#include <math.h>
#include <stddef.h>
#include "at32f403a_407_tmr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
  * @brief  初始化FOC控制参数
  * @param  foc: FOC控制参数结构体指针
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void foc_init(motor_foc_params_t *foc, motor_params_t *params) {
    if (foc == NULL) return;
    
    // 初始化基本值
    foc->target_position = 0.0;
    foc->target_velocity = 0.0;
    foc->target_torque = 0.0;
    foc->current_position = 0.0;
    foc->current_velocity = 0.0;
    foc->current_torque = 0.0;
    foc->pos_error_integral = 0.0;
    foc->vel_error_integral = 0.0;
    foc->d_current_error_integral = 0.0;
    foc->q_current_error_integral = 0.0;
    foc->angle.angle = 0.0;
    foc->angle.sin = 0.0;
    foc->angle.cos = 1.0;
    foc->current_uvw.u = 0.0;
    foc->current_uvw.v = 0.0;
    foc->current_uvw.w = 0.0;
    foc->current_alpha_beta.alpha = 0.0;
    foc->current_alpha_beta.beta = 0.0;
    foc->current_dq.d = 0.0;
    foc->current_dq.q = 0.0;
    foc->voltage_dq.d = 0.0;
    foc->voltage_dq.q = 0.0;
    foc->pwm.u = 0.5;
    foc->pwm.v = 0.5;
    foc->pwm.w = 0.5;
    foc->enabled = 0;
    
    // 初始化无感控制参数
    foc->sensorless_enabled = 0;
    foc->sensorless_k1 = 0.1;
    foc->sensorless_k2 = 0.01;
    foc->sensorless_k3 = 0.001;
    foc->sensorless_min_speed = 100.0;
    foc->sensorless_max_speed = 3000.0;
    foc->sensorless_estimator_gain = 0.5;
    foc->estimated_speed = 0.0;
    foc->estimated_angle = 0.0;
    foc->back_emf_est.d = 0.0;
    foc->back_emf_est.q = 0.0;
    
    // 从参数字典获取参数
    if (params != NULL) {
        foc->control_mode = (foc_control_mode_t)motor_params_get(params, PARAM_ID_CONTROL_MODE);
        foc->max_pwm_value = (uint16_t)motor_params_get(params, PARAM_ID_FOC_MAX_PWM_VALUE);
        foc->sample_freq = (uint32_t)motor_params_get(params, PARAM_ID_FOC_SAMPLE_FREQ);
        foc->kp_pos = motor_params_get(params, PARAM_ID_POSITION_KP);
        foc->ki_pos = motor_params_get(params, PARAM_ID_POSITION_KI);
        foc->kp_vel = motor_params_get(params, PARAM_ID_VELOCITY_KP);
        foc->ki_vel = motor_params_get(params, PARAM_ID_VELOCITY_KI);
        foc->kp_current = motor_params_get(params, PARAM_ID_CURRENT_KP);
        foc->ki_current = motor_params_get(params, PARAM_ID_CURRENT_KI);
        
        // 从参数字典获取无感控制参数
        foc->sensorless_enabled = (uint8_t)motor_params_get(params, PARAM_ID_SENSORLESS_ENABLE);
        foc->sensorless_k1 = motor_params_get(params, PARAM_ID_SENSORLESS_K1);
        foc->sensorless_k2 = motor_params_get(params, PARAM_ID_SENSORLESS_K2);
        foc->sensorless_k3 = motor_params_get(params, PARAM_ID_SENSORLESS_K3);
        foc->sensorless_min_speed = motor_params_get(params, PARAM_ID_SENSORLESS_MIN_SPEED);
        foc->sensorless_max_speed = motor_params_get(params, PARAM_ID_SENSORLESS_MAX_SPEED);
        foc->sensorless_estimator_gain = motor_params_get(params, PARAM_ID_SENSORLESS_ESTIMATOR_GAIN);
    } else {
        // 使用默认值
        foc->control_mode = FOC_CONTROL_MODE_POSITION;
        foc->max_pwm_value = 2000;
        foc->sample_freq = 10000;
        foc->kp_pos = 10.0;
        foc->ki_pos = 0.0;
        foc->kp_vel = 0.1;
        foc->ki_vel = 0.01;
        foc->kp_current = 0.5;
        foc->ki_current = 0.1;
    }
}

/**
  * @brief  设置FOC控制模式
  * @param  foc: FOC控制参数结构体指针
  * @param  mode: 控制模式
  * @retval 无
  */
void foc_set_control_mode(motor_foc_params_t *foc, foc_control_mode_t mode) {
    if (foc == NULL) return;
    foc->control_mode = mode;
}

/**
  * @brief  设置目标位置
  * @param  foc: FOC控制参数结构体指针
  * @param  position: 目标位置
  * @retval 无
  */
void foc_set_target_position(motor_foc_params_t *foc, float position) {
    if (foc == NULL) return;
    foc->target_position = position;
}

/**
  * @brief  设置目标速度
  * @param  foc: FOC控制参数结构体指针
  * @param  velocity: 目标速度
  * @retval 无
  */
void foc_set_target_velocity(motor_foc_params_t *foc, float velocity) {
    if (foc == NULL) return;
    foc->target_velocity = velocity;
}

/**
  * @brief  设置目标转矩
  * @param  foc: FOC控制参数结构体指针
  * @param  torque: 目标转矩
  * @retval 无
  */
void foc_set_target_torque(motor_foc_params_t *foc, float torque) {
    if (foc == NULL) return;
    foc->target_torque = torque;
}

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
void foc_set_pid_gains(motor_foc_params_t *foc, float kp_pos, float ki_pos, float kp_vel, float ki_vel, float kp_current, float ki_current) {
    if (foc == NULL) return;
    foc->kp_pos = kp_pos;
    foc->ki_pos = ki_pos;
    foc->kp_vel = kp_vel;
    foc->ki_vel = ki_vel;
    foc->kp_current = kp_current;
    foc->ki_current = ki_current;
}

/**
  * @brief  更新电机角度
  * @param  foc: FOC控制参数结构体指针
  * @param  mechanical_angle: 机械角度
  * @param  pole_pairs: 极对数
  * @retval 无
  */
void foc_update_angle(motor_foc_params_t *foc, float mechanical_angle, int pole_pairs) {
    if (foc == NULL) return;
    foc->angle.angle = mechanical_angle * pole_pairs;
    foc->angle.sin = sin(foc->angle.angle);
    foc->angle.cos = cos(foc->angle.angle);
}



/**
  * @brief  Clark变换 (三相到两相)
  * @param  uvw: 三相电流结构体
  * @param  alpha_beta: 两相电流结构体
  * @retval 无
  */
static void clark_transform(uvw_t *uvw, alpha_beta_t *alpha_beta) {
    if (uvw == NULL || alpha_beta == NULL) return;
    alpha_beta->alpha = uvw->u;
    alpha_beta->beta = (uvw->u + 2 * uvw->v) / sqrt(3);
}

/**
  * @brief  Park变换 (两相静止到两相旋转)
  * @param  alpha_beta: 两相电流结构体
  * @param  angle: 角度结构体
  * @param  dq: DQ电流结构体
  * @retval 无
  */
static void park_transform(alpha_beta_t *alpha_beta, angle_t *angle, dq_t *dq) {
    if (alpha_beta == NULL || angle == NULL || dq == NULL) return;
    dq->d = alpha_beta->alpha * angle->cos + alpha_beta->beta * angle->sin;
    dq->q = -alpha_beta->alpha * angle->sin + alpha_beta->beta * angle->cos;
}

/**
  * @brief  逆Park变换 (两相旋转到两相静止)
  * @param  dq: DQ电压结构体
  * @param  angle: 角度结构体
  * @param  alpha_beta: 两相电压结构体
  * @retval 无
  */
static void inverse_park_transform(dq_t *dq, angle_t *angle, alpha_beta_t *alpha_beta) {
    if (dq == NULL || angle == NULL || alpha_beta == NULL) return;
    alpha_beta->alpha = dq->d * angle->cos - dq->q * angle->sin;
    alpha_beta->beta = dq->d * angle->sin + dq->q * angle->cos;
}

/**
  * @brief  SVPWM计算
  * @param  foc: FOC控制参数结构体指针
  * @param  alpha_beta: 两相电压结构体
  * @param  pwm: PWM结构体
  * @retval 无
  */
static void svpwm_calculate(motor_foc_params_t *foc, alpha_beta_t *alpha_beta, pwm_t *pwm) {
    float u_alpha;
    float u_beta;
    float u_ref;
    float theta;
    uint8_t sector;
    float t1;
    float t2;
    float t0;
    
    if (foc == NULL || alpha_beta == NULL || pwm == NULL) return;
    
    u_alpha = alpha_beta->alpha;
    u_beta = alpha_beta->beta;
    u_ref = sqrt(u_alpha * u_alpha + u_beta * u_beta);
    
    // 避免除以零或无效输入
    if (fabs(u_alpha) < 0.0001f && fabs(u_beta) < 0.0001f) {
        pwm->u = 0.5f;
        pwm->v = 0.5f;
        pwm->w = 0.5f;
        return;
    }
    
    theta = atan2(u_beta, u_alpha);
    
    if (theta < 0) {
        theta += 2 * M_PI;
    }
    
    sector = (uint8_t)(theta / (M_PI / 3)) + 1;
    t1 = (sqrt(3) * u_ref / foc->max_pwm_value) * sin((M_PI / 3) - theta);
    t2 = (sqrt(3) * u_ref / foc->max_pwm_value) * sin(theta);
    t0 = 1.0f - t1 - t2;
    
    switch (sector) {
        case 1:
            pwm->u = (t1 + t2 + t0 / 2);
            pwm->v = (t2 + t0 / 2);
            pwm->w = t0 / 2;
            break;
        case 2:
            pwm->u = (t1 + t0 / 2);
            pwm->v = (t1 + t2 + t0 / 2);
            pwm->w = t0 / 2;
            break;
        case 3:
            pwm->u = t0 / 2;
            pwm->v = (t1 + t2 + t0 / 2);
            pwm->w = (t2 + t0 / 2);
            break;
        case 4:
            pwm->u = t0 / 2;
            pwm->v = (t1 + t0 / 2);
            pwm->w = (t1 + t2 + t0 / 2);
            break;
        case 5:
            pwm->u = (t2 + t0 / 2);
            pwm->v = t0 / 2;
            pwm->w = (t1 + t2 + t0 / 2);
            break;
        case 6:
            pwm->u = (t1 + t2 + t0 / 2);
            pwm->v = t0 / 2;
            pwm->w = (t1 + t0 / 2);
            break;
        default:
            pwm->u = 0.5;
            pwm->v = 0.5;
            pwm->w = 0.5;
            break;
    }
}

/**
  * @brief  处理FOC控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_process(motor_foc_params_t *foc) {
    float target_q_current;
    float pos_error;
    float vel_setpoint;
    float vel_error;
    float d_current_error;
    float q_current_error;
    alpha_beta_t voltage_alpha_beta;
    
    if (foc == NULL || !foc->enabled) {
        return;
    }
    
    clark_transform(&foc->current_uvw, &foc->current_alpha_beta);
    park_transform(&foc->current_alpha_beta, &foc->angle, &foc->current_dq);
    
    target_q_current = 0.0f;
    
    switch (foc->control_mode) {
        case FOC_CONTROL_MODE_POSITION:
        {
            pos_error = foc->target_position - foc->current_position;
            foc->pos_error_integral += pos_error / foc->sample_freq;
            vel_setpoint = foc->kp_pos * pos_error + foc->ki_pos * foc->pos_error_integral;
            
            vel_error = vel_setpoint - foc->current_velocity;
            foc->vel_error_integral += vel_error / foc->sample_freq;
            target_q_current = foc->kp_vel * vel_error + foc->ki_vel * foc->vel_error_integral;
            break;
        }
        case FOC_CONTROL_MODE_VELOCITY:
        {
            vel_error = foc->target_velocity - foc->current_velocity;
            foc->vel_error_integral += vel_error / foc->sample_freq;
            target_q_current = foc->kp_vel * vel_error + foc->ki_vel * foc->vel_error_integral;
            break;
        }
        case FOC_CONTROL_MODE_TORQUE:
        {
            target_q_current = foc->target_torque;
            break;
        }
    }
    
    d_current_error = 0.0f - foc->current_dq.d;
    q_current_error = target_q_current - foc->current_dq.q;
    
    foc->d_current_error_integral += d_current_error / foc->sample_freq;
    foc->q_current_error_integral += q_current_error / foc->sample_freq;
    
    foc->voltage_dq.d = foc->kp_current * d_current_error + foc->ki_current * foc->d_current_error_integral;
    foc->voltage_dq.q = foc->kp_current * q_current_error + foc->ki_current * foc->q_current_error_integral;
    
    inverse_park_transform(&foc->voltage_dq, &foc->angle, &voltage_alpha_beta);
    
    svpwm_calculate(foc, &voltage_alpha_beta, &foc->pwm);
    
    // 更新PWM值到定时器
    uint16_t pwm_u = (uint16_t)(foc->pwm.u * foc->max_pwm_value);
    uint16_t pwm_v = (uint16_t)(foc->pwm.v * foc->max_pwm_value);
    uint16_t pwm_w = (uint16_t)(foc->pwm.w * foc->max_pwm_value);
    
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, pwm_u);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, pwm_v);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, pwm_w);
}

/**
  * @brief  使能FOC控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_enable(motor_foc_params_t *foc) {
    if (foc == NULL) return;
    foc->enabled = 1;
}

/**
  * @brief  禁用FOC控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_disable(motor_foc_params_t *foc) {
    if (foc == NULL) return;
    foc->enabled = 0;
    foc->pwm.u = 0.5;
    foc->pwm.v = 0.5;
    foc->pwm.w = 0.5;
}

/**
  * @brief  初始化无感控制
  * @param  foc: FOC控制参数结构体指针
  * @param  params: 电机参数结构体指针
  * @retval 无
  */
void foc_sensorless_init(motor_foc_params_t *foc, motor_params_t *params) {
    if (foc == NULL) return;
    
    // 初始化无感控制参数
    foc->sensorless_enabled = 0;
    foc->sensorless_k1 = 0.1;
    foc->sensorless_k2 = 0.01;
    foc->sensorless_k3 = 0.001;
    foc->sensorless_min_speed = 100.0;
    foc->sensorless_max_speed = 3000.0;
    foc->sensorless_estimator_gain = 0.5;
    foc->estimated_speed = 0.0;
    foc->estimated_angle = 0.0;
    foc->back_emf_est.d = 0.0;
    foc->back_emf_est.q = 0.0;
    
    // 从参数字典获取参数
    if (params != NULL) {
        foc->sensorless_enabled = (uint8_t)motor_params_get(params, PARAM_ID_SENSORLESS_ENABLE);
        foc->sensorless_k1 = motor_params_get(params, PARAM_ID_SENSORLESS_K1);
        foc->sensorless_k2 = motor_params_get(params, PARAM_ID_SENSORLESS_K2);
        foc->sensorless_k3 = motor_params_get(params, PARAM_ID_SENSORLESS_K3);
        foc->sensorless_min_speed = motor_params_get(params, PARAM_ID_SENSORLESS_MIN_SPEED);
        foc->sensorless_max_speed = motor_params_get(params, PARAM_ID_SENSORLESS_MAX_SPEED);
        foc->sensorless_estimator_gain = motor_params_get(params, PARAM_ID_SENSORLESS_ESTIMATOR_GAIN);
    }
}

/**
  * @brief  使能无感控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_sensorless_enable(motor_foc_params_t *foc) {
    if (foc == NULL) return;
    foc->sensorless_enabled = 1;
}

/**
  * @brief  禁用无感控制
  * @param  foc: FOC控制参数结构体指针
  * @retval 无
  */
void foc_sensorless_disable(motor_foc_params_t *foc) {
    if (foc == NULL) return;
    foc->sensorless_enabled = 0;
}

/**
  * @brief  更新无感控制参数
  * @param  foc: FOC控制参数结构体指针
  * @param  k1: 参数K1
  * @param  k2: 参数K2
  * @param  k3: 参数K3
  * @retval 无
  */
void foc_sensorless_set_params(motor_foc_params_t *foc, float k1, float k2, float k3) {
    if (foc == NULL) return;
    foc->sensorless_k1 = k1;
    foc->sensorless_k2 = k2;
    foc->sensorless_k3 = k3;
}

/**
  * @brief  估算电机角度和速度
  * @param  foc: FOC控制参数结构体指针
  * @param  bus_voltage: 母线电压
  * @retval 无
  */
void foc_sensorless_estimate(motor_foc_params_t *foc, float bus_voltage) {
    float dt, speed_est;
    
    if (foc == NULL || !foc->sensorless_enabled) return;
    
    // 简单的反电动势观测器实现
    // 1. 计算反电动势估计值
    dt = 1.0f / foc->sample_freq;
    
    // 基于电压和电流计算反电动势
    foc->back_emf_est.d = foc->voltage_dq.d - (foc->kp_current * foc->current_dq.d);
    foc->back_emf_est.q = foc->voltage_dq.q - (foc->kp_current * foc->current_dq.q);
    
    // 2. 估算速度
    speed_est = foc->back_emf_est.q / (foc->current_dq.q + 0.0001f);
    foc->estimated_speed = foc->sensorless_k1 * speed_est + (1.0f - foc->sensorless_k1) * foc->estimated_speed;
    
    // 3. 估算角度
    foc->estimated_angle += foc->estimated_speed * dt;
    
    // 4. 限制角度在0-2π范围内
    while (foc->estimated_angle > 2 * M_PI) {
        foc->estimated_angle -= 2 * M_PI;
    }
    while (foc->estimated_angle < 0) {
        foc->estimated_angle += 2 * M_PI;
    }
    
    // 5. 更新角度结构体
    foc->angle.angle = foc->estimated_angle;
    foc->angle.sin = sin(foc->angle.angle);
    foc->angle.cos = cos(foc->angle.angle);
    
    // 6. 更新当前速度和位置
    foc->current_velocity = foc->estimated_speed;
    foc->current_position += foc->estimated_speed * dt;
}

/**
  * @brief  处理FOC控制（带无感控制）
  * @param  foc: FOC控制参数结构体指针
  * @param  bus_voltage: 母线电压
  * @retval 无
  */
void foc_process_with_sensorless(motor_foc_params_t *foc, float bus_voltage) {
    float target_q_current = 0.0f;
    float d_current_error, q_current_error;
    alpha_beta_t voltage_alpha_beta;
    
    if (foc == NULL || !foc->enabled) {
        return;
    }
    
    // 先进行电流转换，确保有有效的电流数据
    clark_transform(&foc->current_uvw, &foc->current_alpha_beta);
    park_transform(&foc->current_alpha_beta, &foc->angle, &foc->current_dq);
    
    switch (foc->control_mode) {
        case FOC_CONTROL_MODE_POSITION:
        {
            float pos_error = foc->target_position - foc->current_position;
            foc->pos_error_integral += pos_error / foc->sample_freq;
            float vel_setpoint = foc->kp_pos * pos_error + foc->ki_pos * foc->pos_error_integral;
            
            float vel_error = vel_setpoint - foc->current_velocity;
            foc->vel_error_integral += vel_error / foc->sample_freq;
            target_q_current = foc->kp_vel * vel_error + foc->ki_vel * foc->vel_error_integral;
            break;
        }
        case FOC_CONTROL_MODE_VELOCITY:
        {
            float vel_error = foc->target_velocity - foc->current_velocity;
            foc->vel_error_integral += vel_error / foc->sample_freq;
            target_q_current = foc->kp_vel * vel_error + foc->ki_vel * foc->vel_error_integral;
            break;
        }
        case FOC_CONTROL_MODE_TORQUE:
        {
            target_q_current = foc->target_torque;
            break;
        }
    }
    
    d_current_error = 0.0f - foc->current_dq.d;
    q_current_error = target_q_current - foc->current_dq.q;
    
    foc->d_current_error_integral += d_current_error / foc->sample_freq;
    foc->q_current_error_integral += q_current_error / foc->sample_freq;
    
    foc->voltage_dq.d = foc->kp_current * d_current_error + foc->ki_current * foc->d_current_error_integral;
    foc->voltage_dq.q = foc->kp_current * q_current_error + foc->ki_current * foc->q_current_error_integral;
    inverse_park_transform(&foc->voltage_dq, &foc->angle, &voltage_alpha_beta);
    
    svpwm_calculate(foc, &voltage_alpha_beta, &foc->pwm);
    
    // 如果启用了无感控制，估算角度和速度
    if (foc->sensorless_enabled) {
        foc_sensorless_estimate(foc, bus_voltage);
    }
    
    // 更新PWM值到定时器
    uint16_t pwm_u = (uint16_t)(foc->pwm.u * foc->max_pwm_value);
    uint16_t pwm_v = (uint16_t)(foc->pwm.v * foc->max_pwm_value);
    uint16_t pwm_w = (uint16_t)(foc->pwm.w * foc->max_pwm_value);
    
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, pwm_u);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, pwm_v);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, pwm_w);
}