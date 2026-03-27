/* add user code begin Header */
/**
  **************************************************************************
  * @file     foc.h
  * @brief    FOC control header file
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

#ifndef FOC_H
#define FOC_H

/* Includes ------------------------------------------------------------------*/
#include "at32f403a_407.h"
#include "motor_params.h"

/* FOC control parameters -----------------------------------------------------*/
#define FOC_CONTROL_FREQUENCY     30000     /* FOC control frequency (30kHz) */
#define FOC_CURRENT_KP            1.0f      /* Current controller proportional gain */
#define FOC_CURRENT_KI            0.5f      /* Current controller integral gain */
#define FOC_SPEED_KP              2.0f      /* Speed controller proportional gain */
#define FOC_SPEED_KI              0.5f      /* Speed controller integral gain */
#define M_PI                    3.1415926f
/* Back-EMF observer parameters */
#define OBSERVER_K1               2.0f      /* Observer gain K1 */
#define OBSERVER_K2               1.0f      /* Observer gain K2 */

/* Motor start parameters */
#define START_DURATION            2000      /* Start duration (ms) */
#define START_SPEED               3.0f      /* Start speed (rad/s) */
#define START_CURRENT             5.0f      /* Start current (A) */

/* FOC control structure -----------------------------------------------------*/
typedef struct {
  /* Motor state variables */
  float phase_current[3];     /* Phase currents (A) */
  float phase_voltage[3];     /* Phase voltages (V) */
  float i_alpha;              /* Alpha axis current (A) */
  float i_beta;               /* Beta axis current (A) */
  float i_d;                  /* D axis current (A) */
  float i_q;                  /* Q axis current (A) */
  float v_d;                  /* D axis voltage (V) */
  float v_q;                  /* Q axis voltage (V) */
  float v_alpha;              /* Alpha axis voltage (V) */
  float v_beta;               /* Beta axis voltage (V) */
  float angle;                /* Rotor angle (rad) */
  float speed;                /* Rotor speed (rad/s) */
  float target_speed;         /* Target speed (rad/s) */
  
  /* Controller variables */
  float i_d_error;            /* D axis current error */
  float i_q_error;            /* Q axis current error */
  float i_d_integral;         /* D axis current integral */
  float i_q_integral;         /* Q axis current integral */
  float speed_error;          /* Speed error */
  float speed_integral;       /* Speed integral */
  
  /* SVPWM variables */
  float t1;                   /* Time for vector 1 */
  float t2;                   /* Time for vector 2 */
  float t0;                   /* Zero vector time */
  uint8_t sector;             /* Current sector */
  
  /* Back-EMF observer variables */
  float emf_alpha;            /* Alpha axis back-EMF (V) */
  float emf_beta;             /* Beta axis back-EMF (V) */
  float i_alpha_est;          /* Estimated alpha axis current (A) */
  float i_beta_est;           /* Estimated beta axis current (A) */
  float angle_est;             /* Estimated rotor angle (rad) */
  float speed_est;             /* Estimated rotor speed (rad/s) */
  
  /* Motor start variables */
  uint8_t start_state;        /* Start state: 0 = idle, 1 = align, 2 = ramp up, 3 = running */
  uint32_t start_time;        /* Start time (ms) */
  float start_angle;          /* Start angle (rad) */
} foc_state_t;

/* function prototypes --------------------------------------------------------*/
void foc_init(foc_state_t *foc);
void foc_control(foc_state_t *foc, float *phase_current, float angle);
void foc_sensorless_control(foc_state_t *foc, float *phase_current, float *phase_voltage);
void back_emf_observer(foc_state_t *foc);
void motor_start_sequence(foc_state_t *foc);
void clark_transform(float i_a, float i_b, float i_c, float *i_alpha, float *i_beta);
void park_transform(float i_alpha, float i_beta, float angle, float *i_d, float *i_q);
void inverse_park_transform(float v_d, float v_q, float angle, float *v_alpha, float *v_beta);
void svpwm_calculate(float v_alpha, float v_beta, uint16_t *duty_a, uint16_t *duty_b, uint16_t *duty_c);

#endif /* FOC_H */
