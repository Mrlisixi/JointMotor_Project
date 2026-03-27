/* add user code begin Header */
/**
  **************************************************************************
  * @file     foc.c
  * @brief    FOC control implementation file
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
#include "foc.h"
#include "wk_tmr.h"
#include "wk_system.h"
#include <math.h>

/* forward declaration */
uint32_t wk_timebase_get(void);

/* private function prototypes -----------------------------------------------*/
static uint8_t svpwm_calculate_sector(float v_alpha, float v_beta);

/**
  * @brief  Initialize FOC control
  * @param  foc: FOC state structure
  * @retval none
  */
void foc_init(foc_state_t *foc)
{
  /* Initialize state variables */
  foc->phase_current[0] = 0.0f;
  foc->phase_current[1] = 0.0f;
  foc->phase_current[2] = 0.0f;
  foc->phase_voltage[0] = 0.0f;
  foc->phase_voltage[1] = 0.0f;
  foc->phase_voltage[2] = 0.0f;
  foc->i_alpha = 0.0f;
  foc->i_beta = 0.0f;
  foc->i_d = 0.0f;
  foc->i_q = 0.0f;
  foc->v_d = 0.0f;
  foc->v_q = 0.0f;
  foc->v_alpha = 0.0f;
  foc->v_beta = 0.0f;
  foc->angle = 0.0f;
  foc->speed = 0.0f;
  foc->target_speed = 0.0f;
  
  /* Initialize controller variables */
  foc->i_d_error = 0.0f;
  foc->i_q_error = 0.0f;
  foc->i_d_integral = 0.0f;
  foc->i_q_integral = 0.0f;
  foc->speed_error = 0.0f;
  foc->speed_integral = 0.0f;
  
  /* Initialize SVPWM variables */
  foc->t1 = 0.0f;
  foc->t2 = 0.0f;
  foc->t0 = 0.0f;
  foc->sector = 0;
  
  /* Initialize back-EMF observer variables */
  foc->emf_alpha = 0.0f;
  foc->emf_beta = 0.0f;
  foc->i_alpha_est = 0.0f;
  foc->i_beta_est = 0.0f;
  foc->angle_est = 0.0f;
  foc->speed_est = 0.0f;
  
  /* Initialize motor start variables */
  foc->start_state = 0;
  foc->start_time = 0;
  foc->start_angle = 0.0f;
}

/**
  * @brief  FOC control process
  * @param  foc: FOC state structure
  * @param  phase_current: Phase currents (A)
  * @param  angle: Rotor angle (rad)
  * @retval none
  */
void foc_control(foc_state_t *foc, float *phase_current, float angle)
{
  /* Update phase currents */
  foc->phase_current[0] = phase_current[0];
  foc->phase_current[1] = phase_current[1];
  foc->phase_current[2] = phase_current[2];
  
  /* Update rotor angle */
  foc->angle = angle;
  
  /* Clark transform: 3-phase current to alpha-beta */
  clark_transform(foc->phase_current[0], foc->phase_current[1], foc->phase_current[2], &foc->i_alpha, &foc->i_beta);
  
  /* Park transform: alpha-beta to d-q */
  park_transform(foc->i_alpha, foc->i_beta, foc->angle, &foc->i_d, &foc->i_q);
  
  /* Speed controller */
  foc->speed_error = foc->target_speed - foc->speed;
  foc->speed_integral += foc->speed_error * (1.0f / FOC_CONTROL_FREQUENCY);
  
  /* Limit integral windup */
  if (foc->speed_integral > 10.0f)
    foc->speed_integral = 10.0f;
  if (foc->speed_integral < -10.0f)
    foc->speed_integral = -10.0f;
  
  /* Calculate q-axis current reference */
  float i_q_ref = FOC_SPEED_KP * foc->speed_error + FOC_SPEED_KI * foc->speed_integral;
  
  /* Limit q-axis current */
  if (i_q_ref > MOTOR_PEAK_CURRENT)
    i_q_ref = MOTOR_PEAK_CURRENT;
  if (i_q_ref < -MOTOR_PEAK_CURRENT)
    i_q_ref = -MOTOR_PEAK_CURRENT;
  
  /* d-axis current reference (usually 0 for surface mount PMSM) */
  float i_d_ref = 0.0f;
  
  /* Current controllers */
  foc->i_d_error = i_d_ref - foc->i_d;
  foc->i_q_error = i_q_ref - foc->i_q;
  
  foc->i_d_integral += foc->i_d_error * (1.0f / FOC_CONTROL_FREQUENCY);
  foc->i_q_integral += foc->i_q_error * (1.0f / FOC_CONTROL_FREQUENCY);
  
  /* Limit integral windup */
  if (foc->i_d_integral > 10.0f)
    foc->i_d_integral = 10.0f;
  if (foc->i_d_integral < -10.0f)
    foc->i_d_integral = -10.0f;
  if (foc->i_q_integral > 10.0f)
    foc->i_q_integral = 10.0f;
  if (foc->i_q_integral < -10.0f)
    foc->i_q_integral = -10.0f;
  
  /* Calculate d-q voltages */
  foc->v_d = FOC_CURRENT_KP * foc->i_d_error + FOC_CURRENT_KI * foc->i_d_integral;
  foc->v_q = FOC_CURRENT_KP * foc->i_q_error + FOC_CURRENT_KI * foc->i_q_integral;
  
  /* Inverse Park transform: d-q to alpha-beta */
  inverse_park_transform(foc->v_d, foc->v_q, foc->angle, &foc->v_alpha, &foc->v_beta);
  
  /* SVPWM calculation */
  uint16_t duty_a, duty_b, duty_c;
  svpwm_calculate(foc->v_alpha, foc->v_beta, &duty_a, &duty_b, &duty_c);
  
  /* Update phase voltages */
  foc->phase_voltage[0] = (float)duty_a * 24.0f / (float)PWM_PERIOD;
  foc->phase_voltage[1] = (float)duty_b * 24.0f / (float)PWM_PERIOD;
  foc->phase_voltage[2] = (float)duty_c * 24.0f / (float)PWM_PERIOD;
  
  /* Set PWM duty cycles */
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, duty_a);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, duty_b);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, duty_c);
}

/**
  * @brief  Clark transform: 3-phase current to alpha-beta
  * @param  i_a: Phase A current (A)
  * @param  i_b: Phase B current (A)
  * @param  i_c: Phase C current (A)
  * @param  i_alpha: Alpha axis current (A)
  * @param  i_beta: Beta axis current (A)
  * @retval none
  */
void clark_transform(float i_a, float i_b, float i_c, float *i_alpha, float *i_beta)
{
  /* Clark transform assuming balanced 3-phase system (i_a + i_b + i_c = 0) */
  *i_alpha = i_a;
  *i_beta = (i_a + 2.0f * i_b) / sqrtf(3.0f);
}

/**
  * @brief  Park transform: alpha-beta to d-q
  * @param  i_alpha: Alpha axis current (A)
  * @param  i_beta: Beta axis current (A)
  * @param  angle: Rotor angle (rad)
  * @param  i_d: D axis current (A)
  * @param  i_q: Q axis current (A)
  * @retval none
  */
void park_transform(float i_alpha, float i_beta, float angle, float *i_d, float *i_q)
{
  float cos_theta = cosf(angle);
  float sin_theta = sinf(angle);
  
  *i_d = i_alpha * cos_theta + i_beta * sin_theta;
  *i_q = -i_alpha * sin_theta + i_beta * cos_theta;
}

/**
  * @brief  Inverse Park transform: d-q to alpha-beta
  * @param  v_d: D axis voltage (V)
  * @param  v_q: Q axis voltage (V)
  * @param  angle: Rotor angle (rad)
  * @param  v_alpha: Alpha axis voltage (V)
  * @param  v_beta: Beta axis voltage (V)
  * @retval none
  */
void inverse_park_transform(float v_d, float v_q, float angle, float *v_alpha, float *v_beta)
{
  float cos_theta = cosf(angle);
  float sin_theta = sinf(angle);
  
  *v_alpha = v_d * cos_theta - v_q * sin_theta;
  *v_beta = v_d * sin_theta + v_q * cos_theta;
}

/**
  * @brief  Calculate SVPWM sector
  * @param  v_alpha: Alpha axis voltage (V)
  * @param  v_beta: Beta axis voltage (V)
  * @retval Sector number (0-5)
  */
static uint8_t svpwm_calculate_sector(float v_alpha, float v_beta)
{
  uint8_t sector = 0;
  float v0 = v_beta;
  float v1 = (sqrtf(3.0f) * v_alpha - v_beta) / 2.0f;
  float v2 = (-sqrtf(3.0f) * v_alpha - v_beta) / 2.0f;
  
  if (v0 > 0) sector |= 1;
  if (v1 > 0) sector |= 2;
  if (v2 > 0) sector |= 4;
  
  return sector;
}

/**
  * @brief  SVPWM calculation
  * @param  v_alpha: Alpha axis voltage (V)
  * @param  v_beta: Beta axis voltage (V)
  * @param  duty_a: Duty cycle for phase A
  * @param  duty_b: Duty cycle for phase B
  * @param  duty_c: Duty cycle for phase C
  * @retval none
  */
void svpwm_calculate(float v_alpha, float v_beta, uint16_t *duty_a, uint16_t *duty_b, uint16_t *duty_c)
{
  float vdc = 24.0f; /* DC link voltage */
  float Ts = 1.0f / PWM_FREQUENCY; /* PWM period */
  
  /* Calculate normalized voltages */
  float v_alpha_norm = v_alpha / vdc;
  float v_beta_norm = v_beta / vdc;
  
  /* Calculate sector */
  uint8_t sector = svpwm_calculate_sector(v_alpha_norm, v_beta_norm);
  
  /* Calculate time for vectors */
  float t1 = 0.0f, t2 = 0.0f;
  
  switch (sector)
  {
    case 0: /* Sector 1 */
      t1 = sqrtf(3.0f) * v_alpha_norm + v_beta_norm;
      t2 = 2.0f * v_beta_norm;
      break;
    case 1: /* Sector 2 */
      t1 = sqrtf(3.0f) * v_alpha_norm - v_beta_norm;
      t2 = -2.0f * v_alpha_norm;
      break;
    case 2: /* Sector 3 */
      t1 = -sqrtf(3.0f) * v_alpha_norm + v_beta_norm;
      t2 = -2.0f * v_beta_norm;
      break;
    case 3: /* Sector 4 */
      t1 = -sqrtf(3.0f) * v_alpha_norm - v_beta_norm;
      t2 = 2.0f * v_alpha_norm;
      break;
    case 4: /* Sector 5 */
      t1 = sqrtf(3.0f) * v_alpha_norm - v_beta_norm;
      t2 = 2.0f * v_beta_norm;
      break;
    case 5: /* Sector 6 */
      t1 = sqrtf(3.0f) * v_alpha_norm + v_beta_norm;
      t2 = -2.0f * v_alpha_norm;
      break;
    default:
      t1 = 0.0f;
      t2 = 0.0f;
      break;
  }
  
  /* Calculate zero vector time */
  float t0 = Ts - t1 - t2;
  
  /* Limit times to prevent overmodulation */
  if (t0 < 0.0f)
  {
    float scale = Ts / (t1 + t2);
    t1 *= scale;
    t2 *= scale;
    t0 = 0.0f;
  }
  
  /* Calculate duty cycles */
  float ta, tb, tc;
  
  switch (sector)
  {
    case 0: /* Sector 1 */
      ta = (t0 + t1 + t2) / 2.0f;
      tb = (t0 + t2) / 2.0f;
      tc = t0 / 2.0f;
      break;
    case 1: /* Sector 2 */
      ta = (t0 + t1) / 2.0f;
      tb = (t0 + t1 + t2) / 2.0f;
      tc = t0 / 2.0f;
      break;
    case 2: /* Sector 3 */
      ta = t0 / 2.0f;
      tb = (t0 + t1 + t2) / 2.0f;
      tc = (t0 + t1) / 2.0f;
      break;
    case 3: /* Sector 4 */
      ta = t0 / 2.0f;
      tb = (t0 + t1) / 2.0f;
      tc = (t0 + t1 + t2) / 2.0f;
      break;
    case 4: /* Sector 5 */
      ta = (t0 + t2) / 2.0f;
      tb = t0 / 2.0f;
      tc = (t0 + t1 + t2) / 2.0f;
      break;
    case 5: /* Sector 6 */
      ta = (t0 + t1 + t2) / 2.0f;
      tb = t0 / 2.0f;
      tc = (t0 + t2) / 2.0f;
      break;
    default:
      ta = 0.5f;
      tb = 0.5f;
      tc = 0.5f;
      break;
  }
  
  /* Convert to duty cycle counts */
  *duty_a = (uint16_t)(ta * PWM_PERIOD);
  *duty_b = (uint16_t)(tb * PWM_PERIOD);
  *duty_c = (uint16_t)(tc * PWM_PERIOD);
  
  /* Limit duty cycles */
  if (*duty_a > PWM_PERIOD)
    *duty_a = PWM_PERIOD;
  if (*duty_a < 0)
    *duty_a = 0;
  if (*duty_b > PWM_PERIOD)
    *duty_b = PWM_PERIOD;
  if (*duty_b < 0)
    *duty_b = 0;
  if (*duty_c > PWM_PERIOD)
    *duty_c = PWM_PERIOD;
  if (*duty_c < 0)
    *duty_c = 0;
}

/**
  * @brief  Sensorless FOC control process
  * @param  foc: FOC state structure
  * @param  phase_current: Phase currents (A)
  * @param  phase_voltage: Phase voltages (V)
  * @retval none
  */
void foc_sensorless_control(foc_state_t *foc, float *phase_current, float *phase_voltage)
{
  /* Update phase currents */
  foc->phase_current[0] = phase_current[0];
  foc->phase_current[1] = phase_current[1];
  foc->phase_current[2] = phase_current[2];
  
  /* Execute motor start sequence if not running */
  if (foc->start_state < 4)
  {
    motor_start_sequence(foc);
  }
  else
  {
    /* Clark transform: 3-phase current to alpha-beta */
    clark_transform(foc->phase_current[0], foc->phase_current[1], foc->phase_current[2], &foc->i_alpha, &foc->i_beta);
    
    /* Back-EMF observer for sensorless control */
    back_emf_observer(foc);
    
    /* Use estimated angle from observer */
    foc->angle = foc->angle_est;
    foc->speed = foc->speed_est;
    
    /* Park transform: alpha-beta to d-q */
    park_transform(foc->i_alpha, foc->i_beta, foc->angle, &foc->i_d, &foc->i_q);
    
    /* Speed controller */
    foc->speed_error = foc->target_speed - foc->speed;
    foc->speed_integral += foc->speed_error * (1.0f / FOC_CONTROL_FREQUENCY);
    
    /* Limit integral windup */
    if (foc->speed_integral > 10.0f)
      foc->speed_integral = 10.0f;
    if (foc->speed_integral < -10.0f)
      foc->speed_integral = -10.0f;
    
    /* Calculate q-axis current reference */
    float i_q_ref = FOC_SPEED_KP * foc->speed_error + FOC_SPEED_KI * foc->speed_integral;
    
    /* Limit q-axis current */
    if (i_q_ref > MOTOR_PEAK_CURRENT)
      i_q_ref = MOTOR_PEAK_CURRENT;
    if (i_q_ref < -MOTOR_PEAK_CURRENT)
      i_q_ref = -MOTOR_PEAK_CURRENT;
    
    /* d-axis current reference (usually 0 for surface mount PMSM) */
    float i_d_ref = 0.0f;
    
    /* Current controllers */
    foc->i_d_error = i_d_ref - foc->i_d;
    foc->i_q_error = i_q_ref - foc->i_q;
    
    foc->i_d_integral += foc->i_d_error * (1.0f / FOC_CONTROL_FREQUENCY);
    foc->i_q_integral += foc->i_q_error * (1.0f / FOC_CONTROL_FREQUENCY);
    
    /* Limit integral windup */
    if (foc->i_d_integral > 10.0f)
      foc->i_d_integral = 10.0f;
    if (foc->i_d_integral < -10.0f)
      foc->i_d_integral = -10.0f;
    if (foc->i_q_integral > 10.0f)
      foc->i_q_integral = 10.0f;
    if (foc->i_q_integral < -10.0f)
      foc->i_q_integral = -10.0f;
    
    /* Calculate d-q voltages */
    foc->v_d = FOC_CURRENT_KP * foc->i_d_error + FOC_CURRENT_KI * foc->i_d_integral;
    foc->v_q = FOC_CURRENT_KP * foc->i_q_error + FOC_CURRENT_KI * foc->i_q_integral;
    
    /* Inverse Park transform: d-q to alpha-beta */
    inverse_park_transform(foc->v_d, foc->v_q, foc->angle, &foc->v_alpha, &foc->v_beta);
    
    /* SVPWM calculation */
    uint16_t duty_a, duty_b, duty_c;
    svpwm_calculate(foc->v_alpha, foc->v_beta, &duty_a, &duty_b, &duty_c);
    
    /* Update phase voltages */
    foc->phase_voltage[0] = (float)duty_a * 24.0f / (float)PWM_PERIOD;
    foc->phase_voltage[1] = (float)duty_b * 24.0f / (float)PWM_PERIOD;
    foc->phase_voltage[2] = (float)duty_c * 24.0f / (float)PWM_PERIOD;
    
    /* Set PWM duty cycles */
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, duty_a);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, duty_b);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, duty_c);
  }
}

/**
  * @brief  Back-EMF observer
  * @param  foc: FOC state structure
  * @retval none
  */
void back_emf_observer(foc_state_t *foc)
{
  float R = MOTOR_INTERPHASE_RESISTANCE; /* Motor resistance */
  float L = MOTOR_INTERPHASE_INDUCTANCE / 1000.0f; /* Motor inductance in H */
  float Ts = 1.0f / FOC_CONTROL_FREQUENCY; /* Sampling time */
  
  /* Current error */
  float i_alpha_error = foc->i_alpha - foc->i_alpha_est;
  float i_beta_error = foc->i_beta - foc->i_beta_est;
  
  /* Update back-EMF estimates */
  foc->emf_alpha += Ts * (-OBSERVER_K1 * i_alpha_error - OBSERVER_K2 * foc->emf_alpha);
  foc->emf_beta += Ts * (-OBSERVER_K1 * i_beta_error - OBSERVER_K2 * foc->emf_beta);
  
  /* Update current estimates */
  foc->i_alpha_est += Ts * ((foc->v_alpha - R * foc->i_alpha_est - foc->emf_alpha) / L);
  foc->i_beta_est += Ts * ((foc->v_beta - R * foc->i_beta_est - foc->emf_beta) / L);
  
  /* Calculate estimated speed and angle */
  static float angle_prev = 0.0f;
  
  /* Always calculate angle from back-EMF */
  foc->angle_est = atan2f(foc->emf_beta, foc->emf_alpha);
  
  /* Calculate estimated speed */
  float angle_diff = foc->angle_est - angle_prev;
  if (angle_diff > M_PI)
    angle_diff -= 2.0f * M_PI;
  if (angle_diff < -M_PI)
    angle_diff += 2.0f * M_PI;
  foc->speed_est = angle_diff / Ts;
  
  /* Update previous angle */
  angle_prev = foc->angle_est;
}

/**
  * @brief  Motor start sequence for sensorless FOC
  * @param  foc: FOC state structure
  * @retval none
  */
void motor_start_sequence(foc_state_t *foc)
{
  static uint32_t start_time = 0;
  
  switch (foc->start_state)
  {
    case 0: /* Idle state */
      /* Initialize start time */
      start_time = wk_timebase_get();
      foc->start_time = start_time;
      foc->start_angle = 0.0f;
      foc->start_state = 1;
      break;
      
    case 1: /* Alignment state */
      /* Align rotor to known position */
      if (wk_timebase_get() - start_time < 500) /* 500ms alignment */
      {
        /* Set d-axis current to start current */
        float i_d_ref = START_CURRENT;
        float i_q_ref = 0.0f;
        
        /* Set angle to fixed position */
        foc->angle = 0.0f;
        
        /* Inverse Park transform: d-q to alpha-beta */
        inverse_park_transform(i_d_ref, i_q_ref, foc->angle, &foc->v_alpha, &foc->v_beta);
        
        /* SVPWM calculation */
        uint16_t duty_a, duty_b, duty_c;
        svpwm_calculate(foc->v_alpha, foc->v_beta, &duty_a, &duty_b, &duty_c);
        
        /* Update phase voltages */
        foc->phase_voltage[0] = (float)duty_a * 24.0f / (float)PWM_PERIOD;
        foc->phase_voltage[1] = (float)duty_b * 24.0f / (float)PWM_PERIOD;
        foc->phase_voltage[2] = (float)duty_c * 24.0f / (float)PWM_PERIOD;
        
        /* Set PWM duty cycles */
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, duty_a);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, duty_b);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, duty_c);
      }
      else
      {
        /* Alignment complete, move to ramp up */
        start_time = wk_timebase_get();
        foc->start_state = 2;
      }
      break;
      
    case 2: /* Ramp up state */
      /* Ramp up speed to minimum operating speed */
      if (wk_timebase_get() - start_time < START_DURATION)
      {
        /* Calculate ramped speed */
        float ramp_factor = (float)(wk_timebase_get() - start_time) / START_DURATION;
        foc->speed = START_SPEED * ramp_factor;
        foc->angle += foc->speed * (1.0f / FOC_CONTROL_FREQUENCY);
        
        /* Keep angle within 0-2pi */
        if (foc->angle > 2.0f * M_PI)
          foc->angle -= 2.0f * M_PI;
        
        /* Set q-axis current for torque */
        float i_d_ref = 0.0f;
        float i_q_ref = START_CURRENT * 0.5f; /* Use q-axis current for torque */
        
        /* Inverse Park transform: d-q to alpha-beta */
        inverse_park_transform(i_d_ref, i_q_ref, foc->angle, &foc->v_alpha, &foc->v_beta);
        
        /* SVPWM calculation */
        uint16_t duty_a, duty_b, duty_c;
        svpwm_calculate(foc->v_alpha, foc->v_beta, &duty_a, &duty_b, &duty_c);
        
        /* Update phase voltages */
        foc->phase_voltage[0] = (float)duty_a * 24.0f / (float)PWM_PERIOD;
        foc->phase_voltage[1] = (float)duty_b * 24.0f / (float)PWM_PERIOD;
        foc->phase_voltage[2] = (float)duty_c * 24.0f / (float)PWM_PERIOD;
        
        /* Set PWM duty cycles */
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, duty_a);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, duty_b);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, duty_c);
      }
      else
      {
        /* Ramp up complete, move to transition state */
        start_time = wk_timebase_get();
        foc->start_state = 3;
      }
      break;
      
    case 3: /* Transition state */
      /* Transition from open-loop to closed-loop control */
      if (wk_timebase_get() - start_time < 1000) /* 1000ms transition */
      {
        /* Use ramped speed for angle calculation */
        foc->speed = START_SPEED;
        foc->angle += foc->speed * (1.0f / FOC_CONTROL_FREQUENCY);
        
        /* Keep angle within 0-2pi */
        if (foc->angle > 2.0f * M_PI)
          foc->angle -= 2.0f * M_PI;
        
        /* Set q-axis current for torque */
        float i_d_ref = 0.0f;
        float i_q_ref = START_CURRENT * 0.5f;
        
        /* Inverse Park transform: d-q to alpha-beta */
        inverse_park_transform(i_d_ref, i_q_ref, foc->angle, &foc->v_alpha, &foc->v_beta);
        
        /* SVPWM calculation */
        uint16_t duty_a, duty_b, duty_c;
        svpwm_calculate(foc->v_alpha, foc->v_beta, &duty_a, &duty_b, &duty_c);
        
        /* Update phase voltages */
        foc->phase_voltage[0] = (float)duty_a * 24.0f / (float)PWM_PERIOD;
        foc->phase_voltage[1] = (float)duty_b * 24.0f / (float)PWM_PERIOD;
        foc->phase_voltage[2] = (float)duty_c * 24.0f / (float)PWM_PERIOD;
        
        /* Set PWM duty cycles */
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, duty_a);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, duty_b);
        tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, duty_c);
      }
      else
      {
        /* Transition complete, move to running */
        foc->start_state = 4;
      }
      break;
      
    case 4: /* Running state */
      /* Sensorless FOC control */
      break;
  }
}
