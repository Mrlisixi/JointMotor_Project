/* add user code begin Header */
/**
  **************************************************************************
  * @file     motor_params.h
  * @brief    motor parameters header file
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

#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

/* Includes ------------------------------------------------------------------*/
#include "at32f403a_407.h"

/* motor parameters ----------------------------------------------------------*/
#define MOTOR_RATED_VOLTAGE       24.0f      /* Rated voltage (V) */
#define MOTOR_RATED_CURRENT       2.90f      /* Rated current (A) */
#define MOTOR_RATED_TORQUE        1.20f      /* Rated torque (N.M) */
#define MOTOR_RATED_SPEED         354        /* Rated speed (RPM) */
#define MOTOR_MAX_SPEED           516        /* Maximum speed (RPM) */
#define MOTOR_PEAK_TORQUE         3.70f      /* Peak torque (N.M) */
#define MOTOR_PEAK_CURRENT        16.00f     /* Peak current (A) */
#define MOTOR_INTERPHASE_RESISTANCE 1.00f    /* Interphase resistance (Ω) */
#define MOTOR_INTERPHASE_INDUCTANCE 0.63f    /* Interphase inductance (mH) */
#define MOTOR_SPEED_CONSTANT      22         /* Speed constant (rpm/V) */
#define MOTOR_TORQUE_CONSTANT     0.41f      /* Torque constant (N.M/A) */
#define MOTOR_ROTOR_INERTIA       1555       /* Rotor inertia (gcm^2) */
#define MOTOR_POLE_PAIRS          21         /* Number of pole pairs */

/* motor control parameters ---------------------------------------------------*/
#define PWM_PERIOD                1499       /* PWM period (1500 counts) */
#define PWM_FREQUENCY             50000      /* PWM frequency (50kHz) */
#define MAX_DUTY_CYCLE            1400       /* Maximum duty cycle (93.3%) */
#define MIN_DUTY_CYCLE            100        /* Minimum duty cycle (6.7%) */

/* sensor parameters ---------------------------------------------------------*/
#define CURRENT_SENSOR_GAIN       0.1f       /* Current sensor gain (A/V) */
#define VOLTAGE_SENSOR_GAIN       0.01f      /* Voltage sensor gain (V/V) */
#define TEMPERATURE_SENSOR_GAIN   0.5f       /* Temperature sensor gain (°C/V) */
#define TEMPERATURE_SENSOR_OFFSET 25.0f      /* Temperature sensor offset (°C) */

/* six-step commutation states -----------------------------------------------*/
typedef enum {
  COMMUTATION_STATE_1 = 0,  /* A+ B- */
  COMMUTATION_STATE_2,      /* A+ C- */
  COMMUTATION_STATE_3,      /* B+ C- */
  COMMUTATION_STATE_4,      /* B+ A- */
  COMMUTATION_STATE_5,      /* C+ A- */
  COMMUTATION_STATE_6,      /* C+ B- */
  COMMUTATION_STATE_MAX
} commutation_state_t;

/* motor direction -----------------------------------------------------------*/
typedef enum {
  MOTOR_DIR_CW = 0,         /* Clockwise */
  MOTOR_DIR_CCW             /* Counter-clockwise */
} motor_direction_t;

/* motor state ---------------------------------------------------------------*/
typedef enum {
  MOTOR_STATE_IDLE = 0,     /* Idle state */
  MOTOR_STATE_RUNNING,      /* Running state */
  MOTOR_STATE_STOPPING      /* Stopping state */
} motor_state_t;

/* monitoring data structure --------------------------------------------------*/
typedef struct {
  float phase_current[3];    /* Phase currents (A) */
  float phase_voltage[3];    /* Phase voltages (V) */
  float dc_link_voltage;     /* DC link voltage (V) */
  float mos_temperature;     /* MOSFET temperature (°C) */
  uint32_t timestamp;        /* Timestamp (ms) */
} motor_monitor_data_t;

/* motor parameters structure -------------------------------------------------*/
typedef struct {
  motor_state_t state;              /* Motor state */
  motor_direction_t direction;      /* Motor direction */
  commutation_state_t comm_state;   /* Commutation state */
  uint16_t speed;                   /* Motor speed (RPM) */
  uint16_t duty_cycle;              /* PWM duty cycle */
  uint32_t commutation_interval;    /* Commutation interval (ms) */
  uint32_t last_commutation_time;   /* Last commutation time (ms) */
  motor_monitor_data_t monitor_data; /* Monitoring data */
} motor_params_t;

#endif /* MOTOR_PARAMS_H */