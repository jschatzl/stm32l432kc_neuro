/**
 * @file ApplicationTask.h
 * @author Jakob Schatzl
 * @brief Task implementation of the \ref ControlTask and \ref ComTask
 * @version 0.1
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 */

 #include <stdint.h>

 #define ADC_MAX      4095u
 #define DAC_MAX      4095u
 #define VREF_INT_MV  3300

#include "ApplicationTask.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "neuralController.h"
#include "limits.h"
#include "queue.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "pt1.h"

double output = 0.0;
double *input = NULL;
control_st control = {0};
double ***weight = NULL;
neuron_st **neuron = NULL;
neuralControllerConfig_st ncConfig = {0};

uint32_t adc_buffer[ADC_BUF_LEN] = {0};
uint16_t dac_value = 0; 

PT1_Handle_t pt1;

volatile bool uart2_tx_done = true;

float i_plant(float yn, float u);
static inline float convertAdcToVolts(uint32_t adcValue);
static inline uint16_t convertVoltsToDac(float volts);

void ControlTask(void* argument){
  (void) argument;
  float yn = 0;
  /* Infinite loop */
  for(;;)
  {
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    // set input for the next run
    uint32_t adc_val = adc_buffer[0];
    input[0] = convertAdcToVolts((float)adc_val);
    // Run through feed forward + backpropagation
    neuralController_Run(&ncConfig, &control, &output, input, weight, neuron);
    yn = PT1_Update(&pt1, output);
    dac_value = convertVoltsToDac(yn);

    while(xQueueSend(controllerToComHandle, &yn, pdMS_TO_TICKS(10)) != pdPASS){
      ;
    }
  }
}

void ComTask(void *argument)
{
  (void)argument;

  float controller_value;
  static uint8_t msg[32] = {0};
  HAL_StatusTypeDef status;
  /* Infinite loop */
  for(;;)
  {
    while(uart2_tx_done != true)
    {
      ;
    }
    while(xQueueReceive(controllerToComHandle, &controller_value, portMAX_DELAY) != pdPASS) {
        /* Format value into a persistent TX buffer, then start UART DMA. */
    }
    int n = snprintf((char *)msg, sizeof msg, "%.9g\n", (float)controller_value);
    uart2_tx_done = false;
    if (n > 0 && n < (int)sizeof msg) {
      status = HAL_UART_Transmit_DMA(&huart2, msg, sizeof(msg) - 1);
    }

    if(status == HAL_ERROR){
      Error_Handler();
    }
  }
}

float i_plant(float yn, float u) {
    float K = 1;
    float T = 0.1;

    return yn + K * u * T;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        uart2_tx_done = true;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    /* Check register DMAx->IFCR register and optionally clear flag */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* End of conversion -> neural controller can start calculating*/
    vTaskNotifyGiveFromISR((TaskHandle_t)controlTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

static inline float convertAdcToVolts(uint32_t adcValue) {
  if(adcValue > ADC_MAX){
    adcValue = ADC_MAX;
  }

  return (float)adcValue * (VREF_INT_MV / 1000.0f) / (float)ADC_MAX;
}

static inline uint16_t convertVoltsToDac(float volts) {
    if (volts > VREF_INT_MV) {
        volts = VREF_INT_MV;
    }

    uint32_t voltsMv = (uint32_t)(volts * 1000.0f + 0.5f);
    uint32_t dacValue = ((uint32_t)voltsMv * DAC_MAX) / VREF_INT_MV;
    if (volts > DAC_MAX) {
        volts = DAC_MAX;
    }
    return (uint16_t)dacValue;
}