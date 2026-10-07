/**
 * @file ApplicationTask.h
 * @author Jakob Schatzl
 * @brief Headers of \ref ControlTask and \ref ComTask
 * @version 0.1
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 */

#ifndef APPLICATION_TASK_H
#define APPLICATION_TASK_H

#include "cmsis_os.h"
#include "neuralController.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "pt1.h"

#define ADC_BUF_LEN 1

extern osThreadId_t comTaskHandle;
extern osThreadId_t controlTaskHandle;
extern osMessageQueueId_t controllerToComHandle;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac_ch1;
extern RNG_HandleTypeDef hrng;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern double output;
extern double *input;
extern control_st control;
extern double ***weight;
extern neuron_st **neuron; 
extern neuralControllerConfig_st ncConfig;

extern uint32_t adc_buffer[];
extern uint16_t dac_value;

extern PT1_Handle_t pt1;

void ControlTask(void* argument);
void ComTask(void *argument);

#endif // APPLICATION_TASK_H