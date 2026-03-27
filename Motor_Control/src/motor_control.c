/* add user code begin Header */
/**
  **************************************************************************
  * @file     motor_control.c
  * @brief    motor control implementation file
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
#include "monitor.h"
#include "foc.h"
#include "wk_tmr.h"
#include "wk_system.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <math.h>



/* forward declaration */
uint32_t wk_timebase_get(void);

/* external variables --------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function prototypes -----------------------------------------------*/
static void motor_set_commutation_state(motor_params_t *motor, commutation_state_t state);
static void motor_next_commutation_state(motor_params_t *motor);
static uint32_t calculate_commutation_interval(uint16_t speed);

/**
  * @brief  motor control initialization
  * @param  motor: motor parameters structure
  * @retval none
  */
void motor_control_init(motor_params_t *motor)
{
  /* initialize motor parameters */
  motor->state = MOTOR_STATE_IDLE;
  motor->direction = MOTOR_DIR_CW;
  motor->comm_state = COMMUTATION_STATE_1;
  motor->speed = 0;
  motor->duty_cycle = 0;
  motor->commutation_interval = 0;
  motor->last_commutation_time = 0;
  motor->control_mode = JointParam_Read(2000); /* Control mode from params */
  motor->foc_state = NULL;
  
  /* set all outputs to low */
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1C, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2C, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3C, TMR_OUTPUT_CONTROL_LOW);
}

/**
  * @brief  Initialize FOC control
  * @param  motor: motor parameters structure
  * @retval none
  */
void motor_foc_init(motor_params_t *motor)
{
  /* Allocate FOC state structure */
  motor->foc_state = (foc_state_t *)malloc(sizeof(foc_state_t));
  if (motor->foc_state != NULL)
  {
    /* Initialize FOC state */
    foc_init((foc_state_t *)motor->foc_state);
    /* Set control mode to FOC */
    motor->control_mode = 1;
    
    /* Set PWM channels to PWM mode for FOC control */
    tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);
    tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2, TMR_OUTPUT_CONTROL_PWM_MODE_A);
    tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3, TMR_OUTPUT_CONTROL_PWM_MODE_A);
    tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
    tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
    tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
  }
}

/**
  * @brief  FOC control process
  * @param  motor: motor parameters structure
  * @param  phase_current: Phase currents (A)
  * @param  angle: Rotor angle (rad)
  * @retval none
  */
void motor_foc_control(motor_params_t *motor, float *phase_current, float angle)
{
  if (motor->foc_state != NULL && motor->control_mode == 1)
  {
    /* Execute FOC control */
    foc_control((foc_state_t *)motor->foc_state, phase_current, angle);
  }
}

/**
  * @brief  Start motor with sensorless FOC control
  * @param  motor: motor parameters structure
  * @param  speed: target speed (RPM)
  * @retval none
  */
void motor_foc_sensorless_start(motor_params_t *motor, uint16_t speed)
{
  /* Initialize FOC if not already initialized */
  if (motor->foc_state == NULL)
  {
    motor_foc_init(motor);
  }
  
  if (motor->foc_state != NULL)
  {
    /* Set control mode to FOC */
    motor->control_mode = 1;
    
    /* Reset start state to begin startup sequence */
    ((foc_state_t *)motor->foc_state)->start_state = 0;
    
    /* Set target speed */
    float target_speed_rad = (float)speed * 2.0f * M_PI / 60.0f;
    ((foc_state_t *)motor->foc_state)->target_speed = target_speed_rad;
    
    /* Set motor state to running */
    motor->state = MOTOR_STATE_RUNNING;
  }
}

/**
  * @brief  Sensorless FOC control process
  * @param  motor: motor parameters structure
  * @param  phase_current: Phase currents (A)
  * @param  phase_voltage: Phase voltages (V)
  * @retval none
  */
void motor_foc_sensorless_control(motor_params_t *motor, float *phase_current, float *phase_voltage)
{
  if (motor->foc_state != NULL && motor->control_mode == 1)
  {
    /* Execute sensorless FOC control */
    foc_sensorless_control((foc_state_t *)motor->foc_state, phase_current, phase_voltage);
  }
}

/**
  * @brief  motor control process
  * @param  motor: motor parameters structure
  * @retval none
  */
void motor_control_process(motor_params_t *motor)
{
  uint32_t current_time;
  
  /* process only if motor is running */
  if (motor->state == MOTOR_STATE_RUNNING)
  {
    if (motor->control_mode == 0)
    {
      /* Six-step commutation mode */
      /* get current time */
      current_time = wk_timebase_get();
      
      /* check if commutation is needed */
      if (current_time - motor->last_commutation_time >= motor->commutation_interval)
      {
        /* update last commutation time */
        motor->last_commutation_time = current_time;
        
        /* advance to next commutation state */
        motor_next_commutation_state(motor);
      }
    }
    else if (motor->control_mode == 1)
    {
      /* FOC control mode */
      /* FOC control is handled externally with sensor feedback */
      /* This function can be used to trigger FOC control periodically */
    }
  }
}

/**
  * @brief  start motor
  * @param  motor: motor parameters structure
  * @param  speed: target speed (RPM)
  * @param  direction: motor direction
  * @retval none
  */
void motor_start(motor_params_t *motor, uint16_t speed, motor_direction_t direction)
{
  /* set motor parameters */
  motor->speed = speed;
  motor->direction = direction;
  motor->commutation_interval = calculate_commutation_interval(speed);
  motor->last_commutation_time = wk_timebase_get();
  motor->state = MOTOR_STATE_RUNNING;
  
  /* set initial commutation state with minimum duty cycle */
  motor->duty_cycle = MIN_DUTY_CYCLE;
  motor_set_commutation_state(motor, motor->comm_state);
  
  /* delay to ensure motor starts */
  vTaskDelay(pdMS_TO_TICKS(50));
  
  /* increase duty cycle to desired value */
  motor->duty_cycle = 400; /* Increase duty cycle for better torque */
  motor_set_commutation_state(motor, motor->comm_state);
}

/**
  * @brief  stop motor
  * @param  motor: motor parameters structure
  * @retval none
  */
void motor_stop(motor_params_t *motor)
{
  /* set all outputs to low */
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);
  
  /* update motor state */
  motor->state = MOTOR_STATE_IDLE;
  motor->speed = 0;
  motor->duty_cycle = 0;
}

/**
  * @brief  set motor speed
  * @param  motor: motor parameters structure
  * @param  speed: target speed (RPM)
  * @retval none
  */
void motor_set_speed(motor_params_t *motor, uint16_t speed)
{
  motor->speed = speed;
  motor->commutation_interval = calculate_commutation_interval(speed);
}

/**
  * @brief  set motor duty cycle
  * @param  motor: motor parameters structure
  * @param  duty_cycle: PWM duty cycle
  * @retval none
  */
void motor_set_duty_cycle(motor_params_t *motor, uint16_t duty_cycle)
{
  /* limit duty cycle */
  uint16_t max_duty = JointParam_Read(2002); /* Max duty cycle from params */
  if (max_duty == 0) max_duty = 1400; /* Default value */
  
  uint16_t min_duty = JointParam_Read(2003); /* Min duty cycle from params */
  if (min_duty == 0) min_duty = 100; /* Default value */
  
  if (duty_cycle > max_duty)
  {
    duty_cycle = max_duty;
  }
  else if (duty_cycle < min_duty)
  {
    duty_cycle = min_duty;
  }
  
  motor->duty_cycle = duty_cycle;
  
  /* update PWM duty cycle based on current commutation state */
  motor_set_commutation_state(motor, motor->comm_state);
}

/**
  * @brief  set commutation state
  * @param  motor: motor parameters structure
  * @param  state: commutation state
  * @retval none
  */
static void motor_set_commutation_state(motor_params_t *motor, commutation_state_t state)
{
  /* update commutation state */
  motor->comm_state = state;
  
  /* set all channels to low first to avoid transient states */
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1C, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2C, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3, TMR_OUTPUT_CONTROL_LOW);
  tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3C, TMR_OUTPUT_CONTROL_LOW);
  
  /* set PWM outputs based on commutation state */
  switch(state)
  {
    case COMMUTATION_STATE_1: /* A+ B- */
      /* A+ */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, motor->duty_cycle);
      /* B- */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2C, motor->duty_cycle);
      break;
    case COMMUTATION_STATE_2: /* A+ C- */
      /* A+ */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, motor->duty_cycle);
      /* C- */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3C, motor->duty_cycle);
      break;
    case COMMUTATION_STATE_3: /* B+ C- */
      /* B+ */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, motor->duty_cycle);
      /* C- */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3C, motor->duty_cycle);
      break;
    case COMMUTATION_STATE_4: /* B+ A- */
      /* B+ */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, motor->duty_cycle);
      /* A- */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1C, motor->duty_cycle);
      break;
    case COMMUTATION_STATE_5: /* C+ A- */
      /* C+ */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, motor->duty_cycle);
      /* A- */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1C, motor->duty_cycle);
      break;
    case COMMUTATION_STATE_6: /* C+ B- */
      /* C+ */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_3, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, motor->duty_cycle);
      /* B- */
      tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_2C, TMR_OUTPUT_CONTROL_PWM_MODE_A);
      tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2C, motor->duty_cycle);
      break;
    default:
      /* all outputs already set to low */
      break;
  }
}

/**
  * @brief  advance to next commutation state
  * @param  motor: motor parameters structure
  * @retval none
  */
static void motor_next_commutation_state(motor_params_t *motor)
{
  if (motor->direction == MOTOR_DIR_CW)
  {
    /* clockwise direction: increment state */
    motor->comm_state = (commutation_state_t)((motor->comm_state + 1) % COMMUTATION_STATE_MAX);
  } else {
    /* counter-clockwise direction: decrement state */
    if (motor->comm_state == 0)
    {
      motor->comm_state = COMMUTATION_STATE_MAX - 1;
    } else {
      motor->comm_state = (commutation_state_t)(motor->comm_state - 1);
    }
  }
  
  /* set new commutation state */
  motor_set_commutation_state(motor, motor->comm_state);
}

/**
  * @brief  calculate commutation interval based on speed
  * @param  speed: motor speed (RPM)
  * @retval commutation interval (ms)
  */
static uint32_t calculate_commutation_interval(uint16_t speed)
{
  if (speed == 0)
  {
    return 0;
  }
  
  /* limit speed to maximum value */
  if (speed > MOTOR_MAX_SPEED)
  {
    speed = MOTOR_MAX_SPEED;
  }
  
  /* calculate commutation interval in milliseconds */
  /* formula: interval = 60,000 / (speed * pole_pairs * 6) */
  uint16_t pole_pairs = JointParam_Read(2001); /* Pole pairs from params */
  if (pole_pairs == 0) pole_pairs = 21; /* Default value */
  float interval = 60000.0f / (speed * pole_pairs * 6.0f);
  
  /* ensure minimum interval of 1ms */
  if (interval < 1.0f)
  {
    return 1;
  }
  
  return (uint32_t)interval;
}

/* monitoring functions -------------------------------------------------------*/
/* Moved to Monitor module */