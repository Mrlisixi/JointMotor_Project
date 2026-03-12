/* Includes ------------------------------------------------------------------*/
#include "at32f403a_407.h"

/* global variable */
volatile uint32_t timebase_ticks;

/* Function stubs for missing wk_* functions */

/**
  * @brief  this function is called to increment a global variable "timebase_ticks"
  *         used as application time base.
  * @param  none
  * @retval none
  */
void wk_timebase_increase(void)
{
  timebase_ticks++;
}

/**
  * @brief  provides a tick value in millisecond.
  * @param  none
  * @retval tick value
  */
uint32_t wk_timebase_get(void)
{
  return timebase_ticks;
}

/**
  * @brief  this function provides minimum delay (in milliseconds) based
  *         on variable incremented.
  * @param  delay variable specifies the delay time length, in milliseconds.
  * @retval none
  */
void wk_delay_ms(uint32_t delay)
{
  uint32_t start_tick = wk_timebase_get();

  if(delay < 0xFFFFFFFFU)
  {
    delay += 1;
  }

  while((wk_timebase_get() - start_tick) < delay)
  {
  }
}

/**
  * @brief  this function configures the source of the time base
  *         the time source is configured to have 1ms time base
  * @param  none
  * @retval none
  */
void wk_timebase_init(void)
{
  crm_clocks_freq_type crm_clocks;
  uint32_t frequency = 0;

  /* get crm_clocks */
  crm_clocks_freq_get(&crm_clocks);

  frequency = crm_clocks.ahb_freq;
  /* config systick clock source */
  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
  /* system tick config */
  SysTick->LOAD  = (uint32_t)((frequency / 1000) - 1UL);
  SysTick->VAL   = 0UL;
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk |
                   SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief  this function is called at timebase handler, eg: SysTick_Handler
  * @param  none
  * @retval none
  */
void wk_timebase_handler(void)
{
  wk_timebase_increase();
}
