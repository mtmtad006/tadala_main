/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
 *
 * Code generated for Simulink model 'MicroMouse_Deploy'.
 *
 * Model version                  : 5.7
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sat Sep 13 19:09:55 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. ROM efficiency
 *    3. RAM efficiency
 * Validation result: Not run
 */

#include <stddef.h>
#include "code_profiling_utility_functions.h"
#include "MicroMouse_Deploy.h"
#include "rtwtypes.h"
#include "MW_target_hardware_resources.h"

unsigned int profilingDataIdx = 0;
unsigned long int _tmwrunningCoreID;
struct _profilingData
{
  unsigned long int sectionID[400];
  unsigned long int timerValue[400];
  unsigned long int coreID[400];
} profilingData;

void store_code_profiling_data_point(void * pData, uint32_T numMemUnits,
  uint32_T sectionId)
{
  uint32_T * pTimerValue = (uint32_T *) pData;
  size_t elNum = 0;
  size_t numEls = numMemUnits/sizeof(uint32_T);
  if (profilingDataIdx==400) {
    return;
  }

  for (elNum=0; elNum<numEls; ++elNum) {
    profilingData.sectionID[profilingDataIdx] = sectionId;
    profilingData.timerValue[profilingDataIdx] = pTimerValue[elNum];
    profilingData.coreID[profilingDataIdx] = _tmwrunningCoreID;
    profilingDataIdx++;
  }
}

void code_profiling_atomic_read_store(uint32_T sectionId)
{
  __disable_irq();

  /* Using a timer that increments on each tick. */
  uint32_T timerValue = (uint32_T)profileTimerRead();
  store_code_profiling_data_point((void *)(&timerValue), (uint32_T)(sizeof
    (uint32_T)), sectionId);
  __enable_irq();
}

volatile int IsrOverrun = 0;
static boolean_T OverrunFlag = 0;
void rt_OneStep(void)
{
  /* Check for overrun. Protect OverrunFlag against preemption */
  if (OverrunFlag++) {
    IsrOverrun = 1;

    /* PROFILE_TASK_OVERRUN */
    OverrunFlag--;
    return;
  }

  __enable_irq();
  taskTimeStart_MicroMouse_Deploy(2U);
  MicroMouse_Deploy_step();
  taskTimeEnd_MicroMouse_Deploy(2U);

  /* Get model outputs here */
  __disable_irq();
  OverrunFlag--;
}

volatile boolean_T stopRequested;
volatile boolean_T runModel;
int main(int argc, char **argv)
{
  float modelBaseRate = 0.01;
  float systemClock = 80.0;

  /* Initialize variables */
  stopRequested = false;
  runModel = false;

#if !defined(MW_FREERTOS) && defined(MW_MULTI_TASKING_MODE) && (MW_MULTI_TASKING_MODE == 1)

  MW_ASM (" SVC #1");

#endif

  ;

  // Start Peripheral initialization imported from STM32CubeMX project;
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();

  // End Peripheral initialization imported from STM32CubeMX project;
  rtmSetErrorStatus(MicroMouse_Deploy_M, 0);
  taskTimeStart_MicroMouse_Deploy(1U);
  MicroMouse_Deploy_initialize();
  taskTimeEnd_MicroMouse_Deploy(1U);
  __disable_irq();
  ARMCM_SysTick_Config(modelBaseRate);
  runModel =
    rtmGetErrorStatus(MicroMouse_Deploy_M) == (NULL)&& !rtmGetStopRequested
    (MicroMouse_Deploy_M);
  __enable_irq();
  __enable_irq();
  while (runModel) {
    stopRequested = !(
                      rtmGetErrorStatus(MicroMouse_Deploy_M) == (NULL)&&
                      !rtmGetStopRequested(MicroMouse_Deploy_M));
    if (stopRequested) {
      SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    }

    ;
  }

  /* Terminate model */
  taskTimeStart_MicroMouse_Deploy(3U);
  MicroMouse_Deploy_terminate();
  taskTimeEnd_MicroMouse_Deploy(3U);

#if !defined(MW_FREERTOS) && !defined(USE_RTX)

  (void) systemClock;

#endif

  ;
  __disable_irq();
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
