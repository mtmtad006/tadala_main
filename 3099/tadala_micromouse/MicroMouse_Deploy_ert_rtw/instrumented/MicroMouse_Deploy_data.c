/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy_data.c
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

#include "MicroMouse_Deploy.h"

/* Block parameters (default storage) */
P_MicroMouse_Deploy_T MicroMouse_Deploy_P = {
  /* Mask Parameter: PIDController_D
   * Referenced by: '<S68>/Derivative Gain'
   */
  0.001,

  /* Mask Parameter: PIDController_I
   * Referenced by: '<S72>/Integral Gain'
   */
  1.0,

  /* Mask Parameter: PIDController_InitialConditionF
   * Referenced by: '<S70>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditio_l
   * Referenced by: '<S75>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_N
   * Referenced by: '<S78>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_P
   * Referenced by: '<S80>/Proportional Gain'
   */
  20.0,

  /* Mask Parameter: LowPassFilterDiscreteorContinuo
   * Referenced by: '<S18>/Time constant'
   */
  0.1,

  /* Mask Parameter: LowPassFilterDiscreteorContin_p
   * Referenced by: '<S25>/Time constant'
   */
  0.1,

  /* Mask Parameter: LowPassFilterDiscreteorContin_e
   * Referenced by: '<S32>/Time constant'
   */
  0.1,

  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S21>/Constant'
   */
  2.0,

  /* Mask Parameter: CompareToConstant_const_j
   * Referenced by: '<S28>/Constant'
   */
  2.0,

  /* Mask Parameter: CompareToConstant_const_o
   * Referenced by: '<S35>/Constant'
   */
  2.0,

  /* Mask Parameter: LowPassFilterDiscreteorContin_k
   * Referenced by: '<S18>/Constant'
   */
  1.0,

  /* Mask Parameter: LowPassFilterDiscreteorContin_c
   * Referenced by: '<S25>/Constant'
   */
  1.0,

  /* Mask Parameter: LowPassFilterDiscreteorConti_cz
   * Referenced by: '<S32>/Constant'
   */
  1.0,

  /* Mask Parameter: LowPassFilterDiscreteorConti_pu
   * Referenced by: '<S11>/K'
   */
  1.0F,

  /* Mask Parameter: LowPassFilterDiscreteorContin_a
   * Referenced by: '<S12>/K'
   */
  1.0F,

  /* Mask Parameter: LowPassFilterDiscreteorConti_cj
   * Referenced by: '<S13>/K'
   */
  1.0F,

  /* Mask Parameter: CompareToConstant_const_l
   * Referenced by: '<S93>/Constant'
   */
  1.4F,

  /* Mask Parameter: DetectRisePositive_vinit
   * Referenced by: '<S94>/Delay Input1'
   */
  0U,

  /* Expression: (2*pi*0.031)
   * Referenced by: '<S17>/Circumference'
   */
  0.19477874452256716,

  /* Expression: 2
   * Referenced by: '<S3>/ref'
   */
  2.0,

  /* Expression: 0
   * Referenced by: '<S14>/Constant2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S14>/Integrator'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain1'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<S3>/Gain'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain'
   */
  1.0,

  /* Expression: 0.3
   * Referenced by: '<S3>/Constant1'
   */
  0.3,

  /* Expression: 27
   * Referenced by: '<S3>/Constant'
   */
  27.0,

  /* Expression: "UCT MICROMOUSE '25"
   * Referenced by: '<S3>/OLED_STRING1'
   */
  "UCT MICROMOUSE '25",

  /* Expression: "Simulink Data Demo"
   * Referenced by: '<S3>/OLED_STRING2'
   */
  "Simulink Data Demo",

  /* Computed Parameter: Constant_Value_l
   * Referenced by: '<S22>/Constant'
   */
  0.0F,

  /* Computed Parameter: Constant_Value_i
   * Referenced by: '<S29>/Constant'
   */
  0.0F,

  /* Computed Parameter: Constant_Value_c
   * Referenced by: '<S36>/Constant'
   */
  0.0F,

  /* Computed Parameter: Gain2_Gain
   * Referenced by: '<S4>/Gain2'
   */
  5.03547708E-5F,

  /* Computed Parameter: Gain_Gain_g
   * Referenced by: '<S10>/Gain'
   */
  0.001F,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S24>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Integrator_UpperSat
   * Referenced by: '<S24>/Integrator'
   */
  INFINITY,

  /* Computed Parameter: Integrator_LowerSat
   * Referenced by: '<S24>/Integrator'
   */
  -INFINITY,

  /* Computed Parameter: Saturation_UpperSat
   * Referenced by: '<S24>/Saturation'
   */
  INFINITY,

  /* Computed Parameter: Saturation_LowerSat
   * Referenced by: '<S24>/Saturation'
   */
  -INFINITY,

  /* Computed Parameter: Integrator_gainval_n
   * Referenced by: '<S31>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Integrator_UpperSat_k
   * Referenced by: '<S31>/Integrator'
   */
  INFINITY,

  /* Computed Parameter: Integrator_LowerSat_d
   * Referenced by: '<S31>/Integrator'
   */
  -INFINITY,

  /* Computed Parameter: Saturation_UpperSat_f
   * Referenced by: '<S31>/Saturation'
   */
  INFINITY,

  /* Computed Parameter: Saturation_LowerSat_b
   * Referenced by: '<S31>/Saturation'
   */
  -INFINITY,

  /* Computed Parameter: Integrator_gainval_k
   * Referenced by: '<S38>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Integrator_UpperSat_l
   * Referenced by: '<S38>/Integrator'
   */
  INFINITY,

  /* Computed Parameter: Integrator_LowerSat_g
   * Referenced by: '<S38>/Integrator'
   */
  -INFINITY,

  /* Computed Parameter: Saturation_UpperSat_c
   * Referenced by: '<S38>/Saturation'
   */
  INFINITY,

  /* Computed Parameter: Saturation_LowerSat_o
   * Referenced by: '<S38>/Saturation'
   */
  -INFINITY,

  /* Computed Parameter: Gain2_Gain_f
   * Referenced by: '<S3>/Gain2'
   */
  0.001F,

  /* Computed Parameter: DataStoreMemory_InitialValue
   * Referenced by: '<S5>/Data Store Memory'
   */
  0.0F,

  /* Computed Parameter: DataStoreMemory1_InitialValue
   * Referenced by: '<S5>/Data Store Memory1'
   */
  0.0F,

  /* Computed Parameter: DataStoreMemory2_InitialValue
   * Referenced by: '<S5>/Data Store Memory2'
   */
  0.0F,

  /* Computed Parameter: DataStoreMemory_InitialValue_o
   * Referenced by: '<S10>/Data Store Memory'
   */
  0U,

  /* Computed Parameter: DataStoreMemory3_InitialValue
   * Referenced by: '<S10>/Data Store Memory3'
   */
  0U,

  /* Computed Parameter: DataStoreMemory_InitialValue_l
   * Referenced by: '<S8>/Data Store Memory'
   */
  0,

  /* Computed Parameter: DataStoreMemory1_InitialValue_p
   * Referenced by: '<S8>/Data Store Memory1'
   */
  0,

  /* Computed Parameter: DataStoreMemory2_InitialValue_n
   * Referenced by: '<S8>/Data Store Memory2'
   */
  0,

  /* Computed Parameter: DataStoreMemory3_InitialValue_g
   * Referenced by: '<S8>/Data Store Memory3'
   */
  0,

  /* Computed Parameter: DataStoreMemory1_InitialValue_h
   * Referenced by: '<S10>/Data Store Memory1'
   */
  0,

  /* Computed Parameter: DataStoreMemory2_InitialValue_g
   * Referenced by: '<S10>/Data Store Memory2'
   */
  0,

  /* Computed Parameter: DataStoreMemory_InitialValue_b
   * Referenced by: '<S4>/Data Store Memory'
   */
  0U,

  /* Computed Parameter: DataStoreMemory1_InitialValue_a
   * Referenced by: '<S4>/Data Store Memory1'
   */
  0U,

  /* Computed Parameter: DataStoreMemory2_InitialValue_e
   * Referenced by: '<S4>/Data Store Memory2'
   */
  0U,

  /* Computed Parameter: DataStoreMemory3_InitialValue_n
   * Referenced by: '<S4>/Data Store Memory3'
   */
  0U,

  /* Computed Parameter: DataStoreMemory4_InitialValue
   * Referenced by: '<S4>/Data Store Memory4'
   */
  0U,

  /* Computed Parameter: Constant_Value_d
   * Referenced by: '<S95>/Constant'
   */
  false,

  /* Computed Parameter: DataStoreMemory_InitialValue_oa
   * Referenced by: '<S1>/Data Store Memory'
   */
  0,

  /* Computed Parameter: DataStoreMemory1_InitialValu_p2
   * Referenced by: '<S1>/Data Store Memory1'
   */
  0,

  /* Computed Parameter: Tick_per_rev_Gain
   * Referenced by: '<S17>/Tick_per_rev'
   */
  128U,

  /* Computed Parameter: Delay_InitialCondition
   * Referenced by: '<S15>/Delay'
   */
  0U,

  /* Computed Parameter: DataStoreMemory_InitialValue_e
   * Referenced by: '<S2>/Data Store Memory'
   */
  0U,

  /* Computed Parameter: DataStoreMemory1_InitialValue_b
   * Referenced by: '<S2>/Data Store Memory1'
   */
  0U,

  /* Computed Parameter: DataStoreMemory2_InitialValue_p
   * Referenced by: '<S2>/Data Store Memory2'
   */
  0U,

  /* Computed Parameter: DataStoreMemory3_InitialValue_c
   * Referenced by: '<S2>/Data Store Memory3'
   */
  0U,

  /* Computed Parameter: DataStoreMemory4_InitialValue_f
   * Referenced by: '<S2>/Data Store Memory4'
   */
  0U,

  /* Computed Parameter: DataStoreMemory_InitialValue_c
   * Referenced by: '<S6>/Data Store Memory'
   */
  0U,

  /* Computed Parameter: DataStoreMemory_InitialValue_n
   * Referenced by: '<S9>/Data Store Memory'
   */
  0U,

  /* Computed Parameter: DataStoreMemory_InitialValue_h
   * Referenced by: '<S7>/Data Store Memory'
   */
  0U,

  /* Computed Parameter: DataStoreMemory4_InitialValue_k
   * Referenced by: '<S8>/Data Store Memory4'
   */
  0U
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
