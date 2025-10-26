/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy_data.c
 *
 * Code generated for Simulink model 'MicroMouse_Deploy'.
 *
 * Model version                  : 5.42
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sun Oct 26 22:36:47 2025
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
  /* Mask Parameter: PIDController1_D
   * Referenced by: '<S267>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController5_D
   * Referenced by: '<S475>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController4_D
   * Referenced by: '<S423>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController2_D
   * Referenced by: '<S319>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController3_D
   * Referenced by: '<S371>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_D
   * Referenced by: '<S102>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_D_e
   * Referenced by: '<S209>/Derivative Gain'
   */
  1.0,

  /* Mask Parameter: PIDController4_D_i
   * Referenced by: '<S154>/Derivative Gain'
   */
  0.001,

  /* Mask Parameter: PIDController1_D_p
   * Referenced by: '<S535>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController2_D_g
   * Referenced by: '<S587>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_I
   * Referenced by: '<S106>/Integral Gain'
   */
  1.0,

  /* Mask Parameter: PIDController1_I
   * Referenced by: '<S271>/Integral Gain'
   */
  1.0,

  /* Mask Parameter: PIDController2_I
   * Referenced by: '<S323>/Integral Gain'
   */
  1.0,

  /* Mask Parameter: PIDController3_I
   * Referenced by: '<S375>/Integral Gain'
   */
  1.0,

  /* Mask Parameter: PIDController4_I
   * Referenced by: '<S427>/Integral Gain'
   */
  5.0,

  /* Mask Parameter: PIDController5_I
   * Referenced by: '<S479>/Integral Gain'
   */
  5.0,

  /* Mask Parameter: PIDController4_I_j
   * Referenced by: '<S158>/Integral Gain'
   */
  1.0,

  /* Mask Parameter: PIDController1_I_g
   * Referenced by: '<S539>/Integral Gain'
   */
  100.0,

  /* Mask Parameter: PIDController2_I_b
   * Referenced by: '<S591>/Integral Gain'
   */
  100.0,

  /* Mask Parameter: PIDController1_InitialCondition
   * Referenced by: '<S269>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController5_InitialCondition
   * Referenced by: '<S477>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController4_InitialCondition
   * Referenced by: '<S425>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialCondition
   * Referenced by: '<S321>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController3_InitialCondition
   * Referenced by: '<S373>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionF
   * Referenced by: '<S104>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditio_a
   * Referenced by: '<S211>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController4_InitialConditi_i
   * Referenced by: '<S156>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditi_h
   * Referenced by: '<S537>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditi_e
   * Referenced by: '<S589>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditi_i
   * Referenced by: '<S274>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController5_InitialConditi_i
   * Referenced by: '<S482>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController4_InitialConditi_d
   * Referenced by: '<S430>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditi_m
   * Referenced by: '<S326>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController3_InitialConditi_n
   * Referenced by: '<S378>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditio_l
   * Referenced by: '<S109>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController4_InitialConditi_e
   * Referenced by: '<S161>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditi_c
   * Referenced by: '<S542>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditi_f
   * Referenced by: '<S594>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController1_N
   * Referenced by: '<S277>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController5_N
   * Referenced by: '<S485>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController4_N
   * Referenced by: '<S433>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController2_N
   * Referenced by: '<S329>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController3_N
   * Referenced by: '<S381>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_N
   * Referenced by: '<S112>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_N_j
   * Referenced by: '<S219>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController4_N_k
   * Referenced by: '<S164>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController1_N_m
   * Referenced by: '<S545>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController2_N_n
   * Referenced by: '<S597>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController1_P
   * Referenced by: '<S279>/Proportional Gain'
   */
  75.0,

  /* Mask Parameter: PIDController5_P
   * Referenced by: '<S487>/Proportional Gain'
   */
  150.0,

  /* Mask Parameter: PIDController4_P
   * Referenced by: '<S435>/Proportional Gain'
   */
  150.0,

  /* Mask Parameter: PIDController2_P
   * Referenced by: '<S331>/Proportional Gain'
   */
  125.0,

  /* Mask Parameter: PIDController3_P
   * Referenced by: '<S383>/Proportional Gain'
   */
  125.0,

  /* Mask Parameter: PIDController_P
   * Referenced by: '<S114>/Proportional Gain'
   */
  2.0,

  /* Mask Parameter: PIDController_P_c
   * Referenced by: '<S221>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: LowPassFilterDiscreteorContinuo
   * Referenced by: '<S54>/Time constant'
   */
  0.1,

  /* Mask Parameter: LowPassFilterDiscreteorContin_e
   * Referenced by: '<S68>/Time constant'
   */
  0.1,

  /* Mask Parameter: LowPassFilterDiscreteorContin_p
   * Referenced by: '<S61>/Time constant'
   */
  0.1,

  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S57>/Constant'
   */
  2.0,

  /* Mask Parameter: CompareToConstant_const_o
   * Referenced by: '<S71>/Constant'
   */
  2.0,

  /* Mask Parameter: CompareToConstant_const_j
   * Referenced by: '<S64>/Constant'
   */
  2.0,

  /* Mask Parameter: LowPassFilterDiscreteorContin_k
   * Referenced by: '<S54>/Constant'
   */
  1.0,

  /* Mask Parameter: LowPassFilterDiscreteorContin_c
   * Referenced by: '<S68>/Constant'
   */
  1.0,

  /* Mask Parameter: LowPassFilterDiscreteorConti_ct
   * Referenced by: '<S61>/Constant'
   */
  1.0,

  /* Mask Parameter: LowPassFilterDiscreteorConti_pu
   * Referenced by: '<S13>/K'
   */
  1.0F,

  /* Mask Parameter: LowPassFilterDiscreteorConti_cj
   * Referenced by: '<S15>/K'
   */
  1.0F,

  /* Mask Parameter: LowPassFilterDiscreteorContin_a
   * Referenced by: '<S14>/K'
   */
  1.0F,

  /* Mask Parameter: CompareToConstant_const_jx
   * Referenced by: '<S34>/Constant'
   */
  1.0F,

  /* Mask Parameter: CompareToConstant_const_c
   * Referenced by: '<S30>/Constant'
   */
  1.0F,

  /* Mask Parameter: CompareToConstant_const_g
   * Referenced by: '<S45>/Constant'
   */
  1.0F,

  /* Mask Parameter: CompareToConstant_const_l
   * Referenced by: '<S49>/Constant'
   */
  1.0F,

  /* Mask Parameter: DetectFallNonpositive_vinit
   * Referenced by: '<S35>/Delay Input1'
   */
  false,

  /* Mask Parameter: DetectFallNonpositive_vinit_a
   * Referenced by: '<S50>/Delay Input1'
   */
  false,

  /* Mask Parameter: DetectRisePositive_vinit
   * Referenced by: '<S36>/Delay Input1'
   */
  0U,

  /* Mask Parameter: DetectRisePositive_vinit_a
   * Referenced by: '<S31>/Delay Input1'
   */
  0U,

  /* Mask Parameter: DetectRisePositive_vinit_h
   * Referenced by: '<S46>/Delay Input1'
   */
  0U,

  /* Mask Parameter: DetectRisePositive_vinit_f
   * Referenced by: '<S51>/Delay Input1'
   */
  0U,

  /* Expression: 0.2
   * Referenced by: '<S3>/Constant1'
   */
  0.2,

  /* Expression: 4.85
   * Referenced by: '<S11>/Rate Limiter2'
   */
  4.85,

  /* Expression: -1
   * Referenced by: '<S11>/Rate Limiter2'
   */
  -1.0,

  /* Expression: 1/8
   * Referenced by: '<S29>/Tick_per_rev'
   */
  0.125,

  /* Expression: (2*pi*0.031)
   * Referenced by: '<S29>/Circumference'
   */
  0.19477874452256716,

  /* Expression: 4.85
   * Referenced by: '<S11>/Rate Limiter4'
   */
  4.85,

  /* Expression: -1
   * Referenced by: '<S11>/Rate Limiter4'
   */
  -1.0,

  /* Expression: 1/8
   * Referenced by: '<S28>/Tick_per_rev'
   */
  0.125,

  /* Expression: (2*pi*0.031)
   * Referenced by: '<S28>/Circumference'
   */
  0.19477874452256716,

  /* Expression: 0.5
   * Referenced by: '<S11>/Gain1'
   */
  0.5,

  /* Expression: 0.05
   * Referenced by: '<S3>/Constant2'
   */
  0.05,

  /* Expression: 0
   * Referenced by: '<S3>/Integrator'
   */
  0.0,

  /* Expression: 45
   * Referenced by: '<S22>/constant'
   */
  45.0,

  /* Expression: 10
   * Referenced by: '<S22>/Gain5'
   */
  10.0,

  /* Expression: 10
   * Referenced by: '<S22>/Gain6'
   */
  10.0,

  /* Expression: 0.07
   * Referenced by: '<S22>/Constant1'
   */
  0.07,

  /* Expression: 0.07
   * Referenced by: '<S22>/Constant'
   */
  0.07,

  /* Expression: 0.11
   * Referenced by: '<S22>/Constant4'
   */
  0.11,

  /* Expression: 0.07
   * Referenced by: '<S22>/Constant2'
   */
  0.07,

  /* Expression: 0.07
   * Referenced by: '<S22>/Constant3'
   */
  0.07,

  /* Expression: 0.11
   * Referenced by: '<S22>/Constant5'
   */
  0.11,

  /* Expression: 1
   * Referenced by: '<Root>/Gain1'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain'
   */
  -1.0,

  /* Expression: 11
   * Referenced by: '<S3>/Constant'
   */
  11.0,

  /* Expression: 1
   * Referenced by: '<S3>/Constant3'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S3>/Constant4'
   */
  0.0,

  /* Expression: 5
   * Referenced by: '<S3>/Constant5'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<S3>/Integrator1'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S3>/Motor_Left2'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S18>/Integrator'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S501>/Unit Delay1'
   */
  0.0,

  /* Expression: tau
   * Referenced by: '<S501>/tau'
   */
  0.005,

  /* Expression: 1
   * Referenced by: '<S503>/Bias'
   */
  1.0,

  /* Expression: 2*zeta
   * Referenced by: '<S503>/Gain'
   */
  0.76536686473017956,

  /* Expression: 0
   * Referenced by: '<S502>/Unit Delay1'
   */
  0.0,

  /* Expression: tau
   * Referenced by: '<S502>/tau'
   */
  0.005,

  /* Expression: 1
   * Referenced by: '<S504>/Bias'
   */
  1.0,

  /* Expression: 2*zeta
   * Referenced by: '<S504>/Gain'
   */
  1.8477590650225735,

  /* Expression: -0.5
   * Referenced by: '<S23>/Dead Zone'
   */
  -0.5,

  /* Expression: 0.5
   * Referenced by: '<S23>/Dead Zone'
   */
  0.5,

  /* Expression: 1
   * Referenced by: '<S503>/Bias1'
   */
  1.0,

  /* Expression: 2
   * Referenced by: '<S501>/Gain1'
   */
  2.0,

  /* Expression: 0
   * Referenced by: '<S501>/Unit Delay2'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S504>/Bias1'
   */
  1.0,

  /* Expression: 2
   * Referenced by: '<S502>/Gain1'
   */
  2.0,

  /* Expression: 0
   * Referenced by: '<S502>/Unit Delay2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S12>/Integrator'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S39>/Delay'
   */
  0.0,

  /* Expression: 4.85
   * Referenced by: '<S12>/Rate Limiter1'
   */
  4.85,

  /* Expression: -1
   * Referenced by: '<S12>/Rate Limiter1'
   */
  -1.0,

  /* Expression: 0
   * Referenced by: '<S12>/Integrator1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S40>/Delay'
   */
  0.0,

  /* Expression: 4.85
   * Referenced by: '<S12>/Rate Limiter2'
   */
  4.85,

  /* Expression: -1
   * Referenced by: '<S12>/Rate Limiter2'
   */
  -1.0,

  /* Expression: 0.0765
   * Referenced by: '<S24>/Constant'
   */
  0.0765,

  /* Expression: "UCT MICROMOUSE '25"
   * Referenced by: '<S3>/OLED_STRING1'
   */
  "UCT MICROMOUSE '25",

  /* Expression: "Simulink Data Demo"
   * Referenced by: '<S3>/OLED_STRING2'
   */
  "Simulink Data Demo",

  /* Computed Parameter: Constant_Value_lf
   * Referenced by: '<S58>/Constant'
   */
  0.0F,

  /* Computed Parameter: Constant_Value_i
   * Referenced by: '<S65>/Constant'
   */
  0.0F,

  /* Computed Parameter: Constant_Value_c
   * Referenced by: '<S72>/Constant'
   */
  0.0F,

  /* Computed Parameter: Gain_Gain_g
   * Referenced by: '<S10>/Gain'
   */
  0.001F,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S60>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Integrator_UpperSat
   * Referenced by: '<S60>/Integrator'
   */
  INFINITY,

  /* Computed Parameter: Integrator_LowerSat
   * Referenced by: '<S60>/Integrator'
   */
  -INFINITY,

  /* Computed Parameter: Saturation_UpperSat
   * Referenced by: '<S60>/Saturation'
   */
  INFINITY,

  /* Computed Parameter: Saturation_LowerSat
   * Referenced by: '<S60>/Saturation'
   */
  -INFINITY,

  /* Computed Parameter: Integrator_gainval_k
   * Referenced by: '<S74>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Integrator_UpperSat_l
   * Referenced by: '<S74>/Integrator'
   */
  INFINITY,

  /* Computed Parameter: Integrator_LowerSat_g
   * Referenced by: '<S74>/Integrator'
   */
  -INFINITY,

  /* Computed Parameter: Saturation_UpperSat_c
   * Referenced by: '<S74>/Saturation'
   */
  INFINITY,

  /* Computed Parameter: Saturation_LowerSat_o
   * Referenced by: '<S74>/Saturation'
   */
  -INFINITY,

  /* Computed Parameter: Gain2_Gain
   * Referenced by: '<S4>/Gain2'
   */
  5.03547708E-5F,

  /* Computed Parameter: Integrator_gainval_n
   * Referenced by: '<S67>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Integrator_UpperSat_k
   * Referenced by: '<S67>/Integrator'
   */
  INFINITY,

  /* Computed Parameter: Integrator_LowerSat_d
   * Referenced by: '<S67>/Integrator'
   */
  -INFINITY,

  /* Computed Parameter: Saturation_UpperSat_f
   * Referenced by: '<S67>/Saturation'
   */
  INFINITY,

  /* Computed Parameter: Saturation_LowerSat_b
   * Referenced by: '<S67>/Saturation'
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

  /* Computed Parameter: Delay_InitialCondition_j
   * Referenced by: '<S27>/Delay'
   */
  0U,

  /* Computed Parameter: Delay_InitialCondition_c
   * Referenced by: '<S26>/Delay'
   */
  0U,

  /* Computed Parameter: Gain1_Gain_k
   * Referenced by: '<S27>/Gain1'
   */
  64000U,

  /* Computed Parameter: Gain1_Gain_m
   * Referenced by: '<S26>/Gain1'
   */
  64000U,

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

  /* Computed Parameter: Constant_Value_m
   * Referenced by: '<S32>/Constant'
   */
  false,

  /* Computed Parameter: Constant_Value_o
   * Referenced by: '<S37>/Constant'
   */
  false,

  /* Computed Parameter: Constant_Value_j
   * Referenced by: '<S38>/Constant'
   */
  false,

  /* Computed Parameter: Constant_Value_f
   * Referenced by: '<S47>/Constant'
   */
  false,

  /* Computed Parameter: Constant_Value_lk
   * Referenced by: '<S52>/Constant'
   */
  false,

  /* Computed Parameter: Constant_Value_fp
   * Referenced by: '<S53>/Constant'
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

  /* Computed Parameter: Gain_Gain_f
   * Referenced by: '<S27>/Gain'
   */
  131U,

  /* Computed Parameter: Gain_Gain_nt
   * Referenced by: '<S26>/Gain'
   */
  131U,

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
