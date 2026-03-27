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
#include "motor_control.h"
#include "foc.h"
#include "usb_printf.h"
#include "monitor.h"
#include <stdio.h>
#include "modbus_slave.h"

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
/* extern motor parameters structure */
extern motor_params_t motor;

/* add user code end private variables */

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
TaskHandle_t Monitor_Task_handle;
TaskHandle_t Community_Task_handle;
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
  xTaskCreate(MC_Task_Func,
              "MC_Task",
              512,
              NULL,
              0,
              &MC_Task_handle);

  /* create Monitor_Task task */
  xTaskCreate(Monitor_Task_Func,
              "Monitor_Task",
              512,
              NULL,
              0,
              &Monitor_Task_handle);

  /* create Community_Task task */
  xTaskCreate(Community_Task_Func,
              "Community_Task",
              512,
              NULL,
              0,
              &Community_Task_handle);

  /* create Debug_Task task */
  xTaskCreate(Debug_Task_Func,
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
void MC_Task_Func(void *pvParameters)
{
  /* add user code begin MC_Task_Func 0 */
  Modbus_Init(1);
  /* initialize motor */
  motor_control_init(&motor);
  
  /* initialize FOC control */
  motor_foc_init(&motor);
  
  /* start motor with FOC sensorless control */
  motor_foc_sensorless_start(&motor, 50);

  /* add user code end MC_Task_Func 0 */

  /* add user code begin MC_Task_Func 2 */
  
  /* add user code end MC_Task_Func 2 */

  /* Infinite loop */
  while(1)
  {
    /* when use usb,the function wk_usb_app_task() will be generated,
       which is the usb application layer code that users can improve themselves */
    wk_usb_app_task();

  /* add user code begin MC_Task_Func 1 */
    // Modbus_Process();
    vTaskDelay(10);  // Longer delay since FOC control is handled in interrupt

  /* add user code end MC_Task_Func 1 */
  }
}


/**
  * @brief Monitor_Task function.
  * @param  none
  * @retval none
  */
void Monitor_Task_Func(void *pvParameters)
{
  /* add user code begin Monitor_Task_Func 0 */
  monitor_data_t monitor_data;
  uint32_t last_print_time = 0;
  const uint32_t print_interval = 1000; /* 1 second */
  
  /* Threshold values for异常检测 */
  const float OVERCURRENT_THRESHOLD = 10.0f; /* 10A */
  const float OVERVOLTAGE_THRESHOLD = 36.0f; /* 36V */
  const float UNDERVOLTAGE_THRESHOLD = 10.0f; /* 10V */
  const float OVERTEMP_THRESHOLD = 80.0f; /* 80°C */
  
  /* Initialize monitor */
  monitor_init();

  /* add user code end Monitor_Task_Func 0 */

  /* add user code begin Monitor_Task_Func 2 */

  /* add user code end Monitor_Task_Func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin Monitor_Task_Func 1 */
    /* Get monitoring data */
    monitor_update(&monitor_data);
    
    /* Check for异常 conditions */
    // uint8_t error_flag = 0;
    
    
    // /* Check overcurrent */
    // for (int i = 0; i < 3; i++)
    // {
    //   if (monitor_data.phase_current[i] > OVERCURRENT_THRESHOLD)
    //   {
    //     usb_send_string("ERROR: Phase ");
    //     char phase_buf[2] = {'A' + i, 0};
    //     usb_send_string(phase_buf);
    //     usb_send_string(" overcurrent: ");
    //     usb_send_float(monitor_data.phase_current[i], 2);
    //     usb_send_string(" A\n");
    //     error_flag = 1;
    //   }
    // }
    
    // /* Check overvoltage */
    // if (monitor_data.dc_link_voltage > OVERVOLTAGE_THRESHOLD)
    // {
    //   usb_send_string("ERROR: DC link overvoltage: ");
    //   usb_send_float(monitor_data.dc_link_voltage, 2);
    //   usb_send_string(" V\n");
    //   error_flag = 1;
    // }
    
    // /* Check undervoltage */
    // if (monitor_data.dc_link_voltage < UNDERVOLTAGE_THRESHOLD)
    // {
    //   usb_send_string("ERROR: DC link undervoltage: ");
    //   usb_send_float(monitor_data.dc_link_voltage, 2);
    //   usb_send_string(" V\n");
    //   error_flag = 1;
    // }
    
    // /* Check overtemperature */
    // if (monitor_data.mos_temperature > OVERTEMP_THRESHOLD)
    // {
    //   usb_send_string("ERROR: MOS overtemperature: ");
    //   usb_send_float(monitor_data.mos_temperature, 2);
    //   usb_send_string(" °C\n");
    //   error_flag = 1;
    // }
    
    // /* Print monitoring data periodically */
    // uint32_t current_time = xTaskGetTickCount();
    // if ((current_time - last_print_time) > (print_interval / portTICK_PERIOD_MS))
    // {
    //   usb_send_string("=== Motor Monitoring Data ===\n");
           
    //   usb_send_string("Phase Currents: A=");
    //   usb_send_float(monitor_data.phase_current[0], 2);
    //   usb_send_string(" A, B=");
    //   usb_send_float(monitor_data.phase_current[1], 2);
    //   usb_send_string(" A, C=");
    //   usb_send_float(monitor_data.phase_current[2], 2);
    //   usb_send_string(" A\n");
      
    //   usb_send_string("Phase Voltages: A=");
    //   usb_send_float(monitor_data.phase_voltage[0], 2);
    //   usb_send_string(" V, B=");
    //   usb_send_float(monitor_data.phase_voltage[1], 2);
    //   usb_send_string(" V, C=");
    //   usb_send_float(monitor_data.phase_voltage[2], 2);
    //   usb_send_string(" V\n");
      
    //   usb_send_string("DC Link Voltage: ");
    //   usb_send_float(monitor_data.dc_link_voltage, 2);
    //   usb_send_string(" V\n");
      
    //   usb_send_string("MOS Temperature: ");
    //   usb_send_float(monitor_data.mos_temperature, 2);
    //   usb_send_string(" °C\n");
      
    //   usb_send_string("Motor Speed: ");
    //   char speed_buf[10];
    //   sprintf(speed_buf, "%d", motor.speed);
    //   usb_send_string(speed_buf);
    //   usb_send_string(" RPM\n");
      
    //   usb_send_string("Commutation State: ");
    //   char comm_buf[10];
    //   sprintf(comm_buf, "%d", motor.comm_state);
    //   usb_send_string(comm_buf);
    //   usb_send_string("\n");
      
    //   usb_send_string("Motor Direction: ");
    //   usb_send_string(motor.direction == MOTOR_DIR_CW ? "CW" : "CCW");
    //   usb_send_string("\n");
      
    //   usb_send_string("Motor State: ");
    //   char state_buf[10];
    //   sprintf(state_buf, "%d", motor.state);
    //   usb_send_string(state_buf);
    //   usb_send_string("\n");
      
    //   /* Print raw ADC values for debugging */
    //   usb_send_string("Raw ADC Values: A=");
    //   char adc_buf[10];
    //   sprintf(adc_buf, "%d", monitor_data.adc_raw[0]);
    //   usb_send_string(adc_buf);
    //   usb_send_string(", B=");
    //   sprintf(adc_buf, "%d", monitor_data.adc_raw[1]);
    //   usb_send_string(adc_buf);
    //   usb_send_string(", C=");
    //   sprintf(adc_buf, "%d", monitor_data.adc_raw[2]);
    //   usb_send_string(adc_buf);
    //   usb_send_string(", Vbus=");
    //   sprintf(adc_buf, "%d", monitor_data.adc_raw[3]);
    //   usb_send_string(adc_buf);
    //   usb_send_string(", Temp=");
    //   sprintf(adc_buf, "%d", monitor_data.adc_raw[4]);
    //   usb_send_string(adc_buf);
    //   usb_send_string("\n");
      
    //   usb_send_string("============================\n\n");
      
    //   last_print_time = current_time;
    // }
    
    // /* If error detected, take action */
    // if (error_flag)
    // {
    //   /* For now, just print error messages, can add more actions later */
    //   usb_send_string("Taking protective action...\n");
    //   /* Example: motor_stop(&motor); */
    // }

    // /* Print additional debug information */
    // usb_send_string("FOC State: ");
    // char foc_state_buf[10];
    // sprintf(foc_state_buf, "%d", motor.foc_state ? ((foc_state_t *)motor.foc_state)->start_state : -1);
    // usb_send_string(foc_state_buf);
    // usb_send_string("\n");
    
    // usb_send_string("Control Mode: ");
    // char mode_buf[10];
    // sprintf(mode_buf, "%d", motor.control_mode);
    // usb_send_string(mode_buf);
    // usb_send_string("\n");

    

    vTaskDelay(500); /* 100ms delay for monitoring */

  /* add user code end Monitor_Task_Func 1 */
  }
}


/**
  * @brief Community_Task function.
  * @param  none
  * @retval none
  */
void Community_Task_Func(void *pvParameters)
{
  /* add user code begin Community_Task_Func 0 */
  
  /* add user code end Community_Task_Func 0 */

  /* add user code begin Community_Task_Func 2 */

  /* add user code end Community_Task_Func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin Community_Task_Func 1 */

    vTaskDelay(1);
    
  /* add user code end Community_Task_Func 1 */
  }
}


/**
  * @brief Debug_Task function.
  * @param  none
  * @retval none
  */
void Debug_Task_Func(void *pvParameters)
{
  /* add user code begin Debug_Task_Func 0 */

  /* add user code end Debug_Task_Func 0 */

  /* add user code begin Debug_Task_Func 2 */

  /* add user code end Debug_Task_Func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin Debug_Task_Func 1 */

    vTaskDelay(1);

  /* add user code end Debug_Task_Func 1 */
  }
}


/* add user code begin 2 */

/* add user code end 2 */

