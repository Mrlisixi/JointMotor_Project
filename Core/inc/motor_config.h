/* 简单的电机配置和常量（按需调整） */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#define MOTOR_POLE_PAIRS 4
#define MOTOR_VBUS_VOLTS 24.0f

/* 控制周期（秒） - 例如 FOC ISR 频率 10kHz -> 0.0001 */
#define CONTROL_DT 0.0001f

/* ADC 转换系数：把 ADC 原始值转换为安培（由硬件决定） */
#define ADC_TO_AMP_FACTOR  (1.0f)

#endif /* MOTOR_CONFIG_H */
