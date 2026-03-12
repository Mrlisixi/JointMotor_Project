/* add user code begin Header */
/**
  **************************************************************************
  * @file     motor_control.h
  * @brief    motor control header file
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

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "motor_params.h"

/* function prototypes --------------------------------------------------------*/
void motor_control_init(motor_params_t *motor);
void motor_control_process(motor_params_t *motor);
void motor_start(motor_params_t *motor, uint16_t speed, motor_direction_t direction);
void motor_stop(motor_params_t *motor);
void motor_set_speed(motor_params_t *motor, uint16_t speed);
void motor_set_duty_cycle(motor_params_t *motor, uint16_t duty_cycle);

/* monitoring functions -------------------------------------------------------*/
void motor_monitor_init(void);
void motor_monitor_update(motor_params_t *motor);
void motor_monitor_get_data(motor_params_t *motor, motor_monitor_data_t *data);

#endif /* MOTOR_CONTROL_H */