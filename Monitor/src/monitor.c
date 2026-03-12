/* add user code begin Header */
/**
  **************************************************************************
  * @file     monitor.c
  * @brief    monitor implementation file
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
#include "monitor.h"
#include "wk_adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* external variables --------------------------------------------------------*/
extern uint16_t adc_data_buffer[];

/* private variables ---------------------------------------------------------*/
static uint16_t adc_raw_data[6]; /* ADC raw data: phase current A, B, C, DC link voltage, unused, MOS temperature */

/* private function prototypes -----------------------------------------------*/
static void hal_adc_init(void);
static void hal_adc_read(void);
static float hal_adc_convert(uint16_t raw_value, float gain, float offset, float vref);
static float hal_adc_convert_temperature(uint16_t raw_value);

/* hardware abstraction layer functions --------------------------------------*/

/**
  * @brief  Initialize ADC for current, voltage and temperature measurement
  * @param  none
  * @retval none
  */
static void hal_adc_init(void)
{
  /* ADC is already initialized in main.c */
  /* Start ADC conversion */
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

/**
  * @brief  Read ADC raw data
  * @param  none
  * @retval none
  */
static void hal_adc_read(void)
{
  /* Read ADC data from the buffer */
  /* The buffer is filled by DMA in main.c */
  
  /* Get the latest ADC values */
  adc_raw_data[0] = adc_data_buffer[0]; /* Phase A current */
  adc_raw_data[1] = adc_data_buffer[1]; /* Phase B current */
  adc_raw_data[2] = adc_data_buffer[2]; /* Phase C current */
  adc_raw_data[3] = adc_data_buffer[3]; /* DC link voltage */
  adc_raw_data[4] = adc_data_buffer[3]; /* Unused */
  adc_raw_data[5] = adc_data_buffer[4]; /* MOS temperature */
  
  /* Trigger a new conversion */
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

/**
  * @brief  Convert ADC raw data to actual values
  * @param  raw_value: ADC raw value
  * @param  gain: Sensor gain
  * @param  offset: Sensor offset
  * @param  vref: Reference voltage (V)
  * @retval Converted value
  */
static float hal_adc_convert(uint16_t raw_value, float gain, float offset, float vref)
{
  /* Convert ADC raw value (0-4095) to voltage (0-3.3V) */
  float voltage = (float)raw_value * 3.3f / 4095.0f;
  /* Subtract reference voltage for current sensors */
  voltage = voltage - vref;
  /* Convert voltage to actual value */
  return voltage * gain + offset;
}

/**
  * @brief  Convert ADC raw data to temperature using Steinhart-Hart equation
  * @param  raw_value: ADC raw value
  * @retval Temperature in °C
  */
static float hal_adc_convert_temperature(uint16_t raw_value)
{
  // 1. ADC值转电压
    float v_out = (raw_value / ADC_MAX) * VREF;
    // 2. 计算NTC阻值
    float r_ntc = R17 * (VCC / v_out - 1.0f);
    // 3. B值公式计算温度
    float inv_t = 1.0f / T0 + logf(r_ntc / R0) / B;
    float t_k = 1.0f / inv_t;
    return t_k - 273.15f; // 转成℃
}

/* monitor functions -------------------------------------------------------*/

/**
  * @brief  Initialize motor monitoring system
  * @param  none
  * @retval none
  */
void monitor_init(void)
{
  /* Initialize ADC for current, voltage and temperature measurement */
  hal_adc_init();
}

/**
  * @brief  Update motor monitoring data
  * @param  data: Monitoring data structure
  * @retval none
  */
void monitor_update(monitor_data_t *data)
{
  /* Read ADC data */
  hal_adc_read();
  
  /* Convert ADC raw data to actual values */
  /* Current sensors use 1.65V reference */
  data->phase_current[0] = hal_adc_convert(adc_raw_data[0], CURRENT_SENSOR_GAIN, 0.0f, 1.65f);
  data->phase_current[1] = hal_adc_convert(adc_raw_data[1], CURRENT_SENSOR_GAIN, 0.0f, 1.65f);
  data->phase_current[2] = hal_adc_convert(adc_raw_data[2], CURRENT_SENSOR_GAIN, 0.0f, 1.65f);
  
  /* Convert DC link voltage using 3.3V reference */
  data->dc_link_voltage = hal_adc_convert(adc_raw_data[3], VOLTAGE_SENSOR_GAIN, 0.0f, 0.0f);
  
  /* Use DC link voltage for all phases */
  data->phase_voltage[0] = data->dc_link_voltage;
  data->phase_voltage[1] = data->dc_link_voltage;
  data->phase_voltage[2] = data->dc_link_voltage;
  
  /* Convert MOS temperature using Steinhart-Hart equation */
  data->mos_temperature = hal_adc_convert_temperature(adc_raw_data[5]);
  
  /* Update timestamp - using xTaskGetTickCount() which returns tick count */
  /* Note: Tick count unit depends on configTICK_RATE_HZ setting */
  data->timestamp = xTaskGetTickCount();
}
