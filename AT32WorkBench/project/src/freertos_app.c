/* add user code begin Header */
/**
  ******************************************************************************
  * File Name          : freertos_app.c
  * Description        : Code for freertos applications
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "freertos_app.h"
#include "usb_app.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* task handler */
TaskHandle_t MC_Task_handle;
TaskHandle_t CANCom_Task_handle;
TaskHandle_t Monitor_Task_handle;
TaskHandle_t Debug_Task_handle;

/* Idle task control block and stack */
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

static StaticTask_t idle_task_tcb;
static StaticTask_t timer_task_tcb;

/* External Idle and Timer task static memory allocation functions */
extern void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer, StackType_t ** ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
extern void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t * pulTimerTaskStackSize );

/*
  vApplicationGetIdleTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
  equals to 1 and is required for static memory allocation support.
*/
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer, StackType_t ** ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &idle_task_tcb;
  *ppxIdleTaskStackBuffer = &idle_task_stack[0];
  *pulIdleTaskStackSize = (uint32_t)configMINIMAL_STACK_SIZE;
}
/*
  vApplicationGetTimerTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
  equals to 1 and is required for static memory allocation support.
*/
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t * pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &timer_task_tcb;
  *ppxTimerTaskStackBuffer = &timer_task_stack[0];
  *pulTimerTaskStackSize = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}

/* add user code begin 1 */

/* add user code end 1 */

/**
  * @brief  initializes all task.
  * @param  none
  * @retval none
  */
void freertos_task_create(void)
{
  /* create MC_Task task */
  xTaskCreate(MC_Task_func,
              "MC_Task",
              512,
              NULL,
              3,
              &MC_Task_handle);

  /* create CANCom_Task task */
  xTaskCreate(CANCom_Task_func,
              "CANCom_Task",
              512,
              NULL,
              2,
              &CANCom_Task_handle);

  /* create Monitor_Task task */
  xTaskCreate(Monitor_Task_func,
              "Monitor_Task",
              512,
              NULL,
              1,
              &Monitor_Task_handle);

  /* create Debug_Task task */
  xTaskCreate(Debug_Task_func,
              "Debug_Task",
              512,
              NULL,
              0,
              &Debug_Task_handle);
}

/**
  * @brief  freertos init and begin run.
  * @param  none
  * @retval none
  */
void wk_freertos_init(void)
{
  /* add user code begin freertos_init 0 */

  /* add user code end freertos_init 0 */

  /* enter critical */
  taskENTER_CRITICAL();

  freertos_task_create();
	
  /* add user code begin freertos_init 1 */

  /* add user code end freertos_init 1 */

  /* exit critical */
  taskEXIT_CRITICAL();

  /* start scheduler */
  vTaskStartScheduler();
}

/**
  * @brief MC_Task function.
  * @param  none
  * @retval none
  */
void MC_Task_func(void *pvParameters)
{
  /* add user code begin MC_Task_func 0 */

  /* add user code end MC_Task_func 0 */

  /* add user code begin MC_Task_func 2 */

  /* add user code end MC_Task_func 2 */

  /* Infinite loop */
  while(1)
  {
    /* when use usb,the function wk_usb_app_task() will be generated,
       which is the usb application layer code that users can improve themselves */
    wk_usb_app_task();

  /* add user code begin MC_Task_func 1 */

    vTaskDelay(1);

  /* add user code end MC_Task_func 1 */
  }
}


/**
  * @brief CANCom_Task function.
  * @param  none
  * @retval none
  */
void CANCom_Task_func(void *pvParameters)
{
  /* add user code begin CANCom_Task_func 0 */

  /* add user code end CANCom_Task_func 0 */

  /* add user code begin CANCom_Task_func 2 */

  /* add user code end CANCom_Task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin CANCom_Task_func 1 */

    vTaskDelay(1);

  /* add user code end CANCom_Task_func 1 */
  }
}


/**
  * @brief Monitor_Task function.
  * @param  none
  * @retval none
  */
void Monitor_Task_func(void *pvParameters)
{
  /* add user code begin Monitor_Task_func 0 */

  /* add user code end Monitor_Task_func 0 */

  /* add user code begin Monitor_Task_func 2 */

  /* add user code end Monitor_Task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin Monitor_Task_func 1 */

    vTaskDelay(1);

  /* add user code end Monitor_Task_func 1 */
  }
}


/**
  * @brief Debug_Task function.
  * @param  none
  * @retval none
  */
void Debug_Task_func(void *pvParameters)
{
  /* add user code begin Debug_Task_func 0 */

  /* add user code end Debug_Task_func 0 */

  /* add user code begin Debug_Task_func 2 */

  /* add user code end Debug_Task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin Debug_Task_func 1 */

    vTaskDelay(1);

  /* add user code end Debug_Task_func 1 */
  }
}


/* add user code begin 2 */

/* add user code end 2 */

