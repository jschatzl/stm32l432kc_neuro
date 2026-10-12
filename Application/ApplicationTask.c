/**
 * @file ApplicationTask.h
 * @author Jakob Schatzl
 * @brief Task implementation of the \ref ControlTask and \ref ComTask
 * @version 0.1
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 */

 #include "cmsis_os2.h"
#include <stdint.h>

 #define ADC_MAX      4095u
 #define DAC_MAX      4095u
 #define VREF_INT_MV  3300
 #define DIAG_N       32

#include "ApplicationTask.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "neuralController.h"
#include "limits.h"
#include "queue.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "pt2.h"
#include "pid.h"

float output = 0.0;
float *input = NULL;
control_st control = {0};
float ***weight = NULL;
neuron_st **neuron = NULL;
neuralControllerConfig_st ncConfig = {0};

uint16_t adc_buffer[ADC_BUF_LEN] = {0};
uint16_t dac_value = 0; 

volatile uint32_t adc_callback_count = 0;
volatile uint32_t last_dwt_cycles = 0;
volatile float measured_dt = 0.001f;
volatile uint32_t diag_elapsed[DIAG_N];
volatile uint32_t diag_index = 0;

PT2_Handle_t pt2;
PID_Handle_t pid;

volatile bool uart2_tx_done = true;

float i_plant(float yn, float u);
static inline float convertAdcToVolts(uint32_t adcValue);
static inline uint16_t convertVoltsToDac(float volts);
float gaussDistribution(float mu, float sigma);
float generateRandomFloatWithLimit(float limit);

void ControlTask(void* argument){
  (void) argument;
  float yn = 0;
  float initialLr = ncConfig.learning_rate;
  uint32_t compute_cycles = 0;
  uint32_t compute_cycles_max = 0;
  uint32_t compute_cycles_sum = 0;
  uint32_t compute_cycles_count = 0;
  /* Infinite loop */
  for(;;)
  {
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    // set input for the next run
    uint16_t adc_val = adc_buffer[0];
    uint32_t start_cycles = DWT->CYCCNT;
    input[0] = convertAdcToVolts((float)adc_val);
    // Run through feed forward + backpropagation
    ncConfig.learning_rate = stepDecayLr(control.epoch, initialLr, 1250, 0.1);
    neuralController_Run(&ncConfig, &control, &output, input, weight, neuron);
    //output = PID_Update(&pid, ncConfig.setpoint, yn);
    uint32_t end = DWT->CYCCNT;
    compute_cycles = end - start_cycles;
    yn = PT2_Update(&pt2, output);
    dac_value = convertVoltsToDac(yn);
    compute_cycles = DWT->CYCCNT - start_cycles;
    if(compute_cycles > compute_cycles_max) {
      compute_cycles_max = compute_cycles;
    }
    compute_cycles_sum += compute_cycles;
    compute_cycles_count++;
    while(xQueueSend(controllerToComHandle, &yn, pdMS_TO_TICKS(10)) != pdPASS){
      ;
    }
  }
}

void ComTask(void *argument)
{
  (void)argument;

  float controller_value;
  static uint8_t msg[26] = {0};

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
    //uint32_t timestamp_ms = osKernelGetTickCount();
    //int n = snprintf((char *)msg, sizeof(msg), "%u,%.6f,%.6f\r\n", control.epoch, ncConfig.learning_rate, (float)controller_value);
    int n = snprintf((char *)msg, sizeof(msg), "%.9g\n", (float)controller_value);
    uart2_tx_done = false;
    if (n > 0 && n < (int)sizeof msg) {
      status = HAL_UART_Transmit_DMA(&huart2, msg, n);
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

float generateXavierFloat(float fanIn, float fanOut){
    static float gain = 0.1;
    float limit = gain * sqrtf(6.0f / (fanIn + fanOut));
    return generateRandomFloatWithLimit(limit);
}

float generateRandomFloatWithLimit(float limit) {
    float rnd = (float)rand() / (float)RAND_MAX;
    return (2.0f * rnd - 1.0f) * limit;
}

float generateRandomFloat(float fanIn, float fanOut) {
  (void) fanIn;
  (void) fanOut;
  return generateRandomFloatWithLimit(0.1f);
}

float gaussDistribution(float mu, float sigma) {
    float u1, u2, r;

    do {
        u1 = (float)rand() / RAND_MAX;
    } while (u1 <= 0.0);  // avoid log(0)

    u2 = (float)rand() / RAND_MAX;

    r = sqrtf(-2.0 * log(u1));

    // Box-Muller: either cos or sin term is a valid N(0,1) sample
    return mu + sigma * r * cos(2.0 * M_PI * u2);
}

float generateGaussFloat(float fanIn, float fanOut) {
    (void)fanIn;
    (void)fanOut;
    return gaussDistribution(0.0, 0.2);
}