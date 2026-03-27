/* add user code begin Header */
/**
  **************************************************************************
  * @file     monitor.h
  * @brief    monitor header file
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

#ifndef MONITOR_H
#define MONITOR_H

/* Includes ------------------------------------------------------------------*/
#include "at32f403a_407.h"

/* sensor parameters ---------------------------------------------------------*/
/* Current sensor: 1mΩ shunt resistor, RS724XQ op-amp gain */
/* Differential amplifier gain: R12/R13 = 100kΩ/3.3kΩ ≈ 30.3 */
/* Voltage across shunt: V = I * R = I * 0.001Ω */
/* Op-amp output: Vout = Gain * (SL_C - PGND) = 30.3 * I * 0.001 = I * 0.0303 */
/* So I = Vout / 0.0303 ≈ Vout * 33 */
#define CURRENT_SENSOR_GAIN_A     31.3f      /* Current sensor gain for phase A (A/V) */
#define CURRENT_SENSOR_GAIN_B     31.3f      /* Current sensor gain for phase B (A/V) - reduced due to overcurrent */
#define CURRENT_SENSOR_GAIN_C     31.3f      /* Current sensor gain for phase C (A/V) */
#define CURRENT_SENSOR_OFFSET_A   0.0f       /* Current sensor offset for phase A (A) */
#define CURRENT_SENSOR_OFFSET_B   0.0f       /* Current sensor offset for phase B (A) */
#define CURRENT_SENSOR_OFFSET_C   0.0f       /* Current sensor offset for phase C (A) */

/* Voltage sensor: 18K+1K resistor divider */
/* Vout = Vin * (1K / (18K + 1K)) = Vin * (1/19) ≈ Vin * 0.05263 */
/* So Vin = Vout / 0.05263 ≈ Vout * 19 */
/* If measured voltage is half of actual, double the gain */
#define VOLTAGE_SENSOR_GAIN       19.0f      /* Voltage sensor gain (V/V) */

/* Temperature sensor: 10K F3950 thermistor with 1.65V reference */
/* Using Steinhart-Hart equation for NTC thermistors */
/* B value for F3950 is typically 3950 */
#define TEMPERATURE_SENSOR_B_VALUE 3950.0f   /* B value for F3950 thermistor */
#define TEMPERATURE_SENSOR_R_REF   10000.0f  /* Reference resistor (10K) */
#define TEMPERATURE_SENSOR_V_REF   3.3f     /* Reference voltage (3.3V) */

/* NTC thermistor parameters */
#define R0      10000.0f   // NTC 25℃标称阻值
#define T0      298.15f    // 25℃ 开尔文温度
#define B       3950.0f    // NTC B值
#define R17     3300.0f    // 分压电阻
#define VCC     3.3f       // 供电电压
#define VREF    3.3f       // ADC参考电压
#define ADC_MAX 4095.0f    // 12位ADC最大值(2^12-1)

/* monitoring data structure --------------------------------------------------*/
typedef struct {
  float phase_current[3];    /* Phase currents (A) */
  float phase_voltage[3];    /* Phase voltages (V) */
  float dc_link_voltage;     /* DC link voltage (V) */
  float mos_temperature;     /* MOSFET temperature (°C) */
  uint16_t adc_raw[5];       /* ADC raw data: phase A, B, C current, DC link voltage, MOS temperature */
  uint32_t timestamp;        /* Timestamp (ms) */
} monitor_data_t;

/* function prototypes --------------------------------------------------------*/
void monitor_init(void);
void monitor_update(monitor_data_t *data);

#endif /* MONITOR_H */
