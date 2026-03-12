/* add user code begin Header */
/**
  **************************************************************************
  * @file     motor_control.c
  * @brief    motor control source file
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include "wk_adc.h"

/* private variables ---------------------------------------------------------*/
static uint16_t adc_raw_data[6]; /* ADC raw data: phase current A, B, C, phase voltage A, B, C, MOS temperature */

/* hardware abstraction layer functions --------------------------------------*/

/**
  * @brief  Initialize ADC for current, voltage and temperature measurement
  * @param  none
  * @retval none
  */
static void hal_adc_init(void)
{
  /* Add ADC initialization code here */
  /* This function should be implemented based on the actual hardware */
}

/**
  * @brief  Read ADC raw data
  * @param  none
  * @retval none
  */
static void hal_adc_read(void)
{
  /* Add ADC reading code here */
  /* This function should be implemented based on the actual hardware */
  
  /* For simulation purposes, we'll use dummy values */
  adc_raw_data[0] = 2048; /* Phase A current */
  adc_raw_data[1] = 2048; /* Phase B current */
  adc_raw_data[2] = 2048; /* Phase C current */
  adc_raw_data[3] = 3072; /* Phase A voltage */
  adc_raw_data[4] = 3072; /* Phase B voltage */
  adc_raw_data[5] = 2560; /* MOS temperature */
}

/**
  * @brief  Convert ADC raw data to actual values
  * @param  raw_value: ADC raw value
  * @param  gain: Sensor gain
  * @param  offset: Sensor offset
  * @retval Converted value
  */
static float hal_adc_convert(uint16_t raw_value, float gain, float offset)
{
  /* Convert ADC raw value (0-4095) to voltage (0-3.3V) */
  float voltage = (float)raw_value * 3.3f / 4095.0f;
  /* Convert voltage to actual value */
  return voltage * gain + offset;
}

/* public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize motor control
  * @param  motor: Motor parameters structure
  * @retval none
  */
void motor_control_init(motor_params_t *motor)
{
  /* Initialize motor parameters */
  motor->state = MOTOR_STATE_IDLE;
  motor->direction = MOTOR_DIR_CW;
  motor->comm_state = COMMUTATION_STATE_1;
  motor->speed = 0;
  motor->duty_cycle = 0;
  motor->commutation_interval = 0;
  motor->last_commutation_time = 0;
  
  /* Initialize monitoring data */
  for (uint8_t i = 0; i < 3; i++)
  {
    motor->monitor_data.phase_current[i] = 0.0f;
    motor->monitor_data.phase_voltage[i] = 0.0f;
  }
  motor->monitor_data.dc_link_voltage = 0.0f;
  motor->monitor_data.mos_temperature = 0.0f;
  motor->monitor_data.timestamp = 0;
  
  /* Initialize monitoring system */
  motor_monitor_init();
}

/**
  * @brief  Process motor control
  * @param  motor: Motor parameters structure
  * @retval none
  */
void motor_control_process(motor_params_t *motor)
{
  /* Add motor control process code here */
  
  /* Update monitoring data */
  motor_monitor_update(motor);
}

/**
  * @brief  Start motor
  * @param  motor: Motor parameters structure
  * @param  speed: Motor speed (RPM)
  * @param  direction: Motor direction
  * @retval none
  */
void motor_start(motor_params_t *motor, uint16_t speed, motor_direction_t direction)
{
  motor->state = MOTOR_STATE_RUNNING;
  motor->speed = speed;
  motor->direction = direction;
  motor->duty_cycle = 500; /* Default duty cycle */
  motor->commutation_interval = 1000 / speed * 60 / (MOTOR_POLE_PAIRS * 2);
  motor->last_commutation_time = 0;
}

/**
  * @brief  Stop motor
  * @param  motor: Motor parameters structure
  * @retval none
  */
void motor_stop(motor_params_t *motor)
{
  motor->state = MOTOR_STATE_STOPPING;
  motor->speed = 0;
  motor->duty_cycle = 0;
}

/**
  * @brief  Set motor speed
  * @param  motor: Motor parameters structure
  * @param  speed: Motor speed (RPM)
  * @retval none
  */
void motor_set_speed(motor_params_t *motor, uint16_t speed)
{
  motor->speed = speed;
  motor->commutation_interval = 1000 / speed * 60 / (MOTOR_POLE_PAIRS * 2);
}

/**
  * @brief  Set motor duty cycle
  * @param  motor: Motor parameters structure
  * @param  duty_cycle: PWM duty cycle
  * @retval none
  */
void motor_set_duty_cycle(motor_params_t *motor, uint16_t duty_cycle)
{
  motor->duty_cycle = duty_cycle;
}

/* monitoring functions -------------------------------------------------------*/

/**
  * @brief  Initialize motor monitoring system
  * @param  none
  * @retval none
  */
void motor_monitor_init(void)
{
  /* Initialize ADC for current, voltage and temperature measurement */
  hal_adc_init();
}

/**
  * @brief  Update motor monitoring data
  * @param  motor: Motor parameters structure
  * @retval none
  */
void motor_monitor_update(motor_params_t *motor)
{
  /* Read ADC data */
  hal_adc_read();
  
  /* Convert ADC raw data to actual values */
  motor->monitor_data.phase_current[0] = hal_adc_convert(adc_raw_data[0], CURRENT_SENSOR_GAIN, 0.0f);
  motor->monitor_data.phase_current[1] = hal_adc_convert(adc_raw_data[1], CURRENT_SENSOR_GAIN, 0.0f);
  motor->monitor_data.phase_current[2] = hal_adc_convert(adc_raw_data[2], CURRENT_SENSOR_GAIN, 0.0f);
  
  motor->monitor_data.phase_voltage[0] = hal_adc_convert(adc_raw_data[3], VOLTAGE_SENSOR_GAIN, 0.0f);
  motor->monitor_data.phase_voltage[1] = hal_adc_convert(adc_raw_data[4], VOLTAGE_SENSOR_GAIN, 0.0f);
  motor->monitor_data.phase_voltage[2] = hal_adc_convert(adc_raw_data[5], VOLTAGE_SENSOR_GAIN, 0.0f);
  
  /* Calculate DC link voltage (simplified) */
  motor->monitor_data.dc_link_voltage = (motor->monitor_data.phase_voltage[0] + 
                                       motor->monitor_data.phase_voltage[1] + 
                                       motor->monitor_data.phase_voltage[2]) / 3.0f;
  
  /* Convert MOS temperature */
  motor->monitor_data.mos_temperature = hal_adc_convert(adc_raw_data[5], TEMPERATURE_SENSOR_GAIN, TEMPERATURE_SENSOR_OFFSET);
  
  /* Update timestamp */
  motor->monitor_data.timestamp = HAL_GetTick();
}

/**
  * @brief  Get motor monitoring data
  * @param  motor: Motor parameters structure
  * @param  data: Monitoring data structure
  * @retval none
  */
void motor_monitor_get_data(motor_params_t *motor, motor_monitor_data_t *data)
{
  /* Copy monitoring data */
  data->phase_current[0] = motor->monitor_data.phase_current[0];
  data->phase_current[1] = motor->monitor_data.phase_current[1];
  data->phase_current[2] = motor->monitor_data.phase_current[2];
  
  data->phase_voltage[0] = motor->monitor_data.phase_voltage[0];
  data->phase_voltage[1] = motor->monitor_data.phase_voltage[1];
  data->phase_voltage[2] = motor->monitor_data.phase_voltage[2];
  
  data->dc_link_voltage = motor->monitor_data.dc_link_voltage;
  data->mos_temperature = motor->monitor_data.mos_temperature;
  data->timestamp = motor->monitor_data.timestamp;
}
