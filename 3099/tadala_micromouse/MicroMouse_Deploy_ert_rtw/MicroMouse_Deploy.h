/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy.h
 *
 * Code generated for Simulink model 'MicroMouse_Deploy'.
 *
 * Model version                  : 5.0
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Fri Sep 12 19:52:49 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. ROM efficiency
 *    3. RAM efficiency
 * Validation result: Not run
 */

#ifndef MicroMouse_Deploy_h_
#define MicroMouse_Deploy_h_
#ifndef MicroMouse_Deploy_COMMON_INCLUDES_
#define MicroMouse_Deploy_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* MicroMouse_Deploy_COMMON_INCLUDES_ */

#include "MicroMouse_Deploy_types.h"
#include "rt_nonfinite.h"
#include <stddef.h>
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* user code (top of header file) */
#include "MicroMouseProgramming_Code\Core\Inc\IMU.h"
#include "MicroMouseProgramming_Code\Core\Inc\Motors.h"
#include "MicroMouseProgramming_Code\Core\Inc\SSD1306.h"
#include "MicroMouseProgramming_Code\Core\Inc\VL53L0X.h"
#include "MicroMouseProgramming_Code\Core\Inc\ADCs.h"
#include "MicroMouseProgramming_Code\Core\Inc\Buttons.h"
#include "MicroMouseProgramming_Code\Core\Inc\LEDs.h"
#include "MicroMouseProgramming_Code\Core\Inc\INA219.h"
#include "MicroMouseProgramming_Code\Core\Inc\preformatted_flash.h"
#include <math.h>

/* Block signals (default storage) */
typedef struct {
  real32_T Probe[2];                   /* '<S14>/Probe' */
  real32_T Probe_h[2];                 /* '<S21>/Probe' */
  real32_T Probe_g[2];                 /* '<S28>/Probe' */
} B_MicroMouse_Deploy_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Integrator_DSTATE;          /* '<S20>/Integrator' */
  real32_T Integrator_DSTATE_m;        /* '<S27>/Integrator' */
  real32_T Integrator_DSTATE_i;        /* '<S34>/Integrator' */
  int8_T Integrator_PrevResetState;    /* '<S20>/Integrator' */
  int8_T Integrator_PrevResetState_a;  /* '<S27>/Integrator' */
  int8_T Integrator_PrevResetState_j;  /* '<S34>/Integrator' */
  uint8_T Integrator_IC_LOADING;       /* '<S20>/Integrator' */
  uint8_T Integrator_IC_LOADING_k;     /* '<S27>/Integrator' */
  uint8_T Integrator_IC_LOADING_a;     /* '<S34>/Integrator' */
} DW_MicroMouse_Deploy_T;

/* Parameters (default storage) */
struct P_MicroMouse_Deploy_T_ {
  real_T LowPassFilterDiscreteorContinuo;
                              /* Mask Parameter: LowPassFilterDiscreteorContinuo
                               * Referenced by: '<S14>/Time constant'
                               */
  real_T LowPassFilterDiscreteorContin_p;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_p
                               * Referenced by: '<S21>/Time constant'
                               */
  real_T LowPassFilterDiscreteorContin_e;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_e
                               * Referenced by: '<S28>/Time constant'
                               */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S17>/Constant'
                                       */
  real_T CompareToConstant_const_j; /* Mask Parameter: CompareToConstant_const_j
                                     * Referenced by: '<S24>/Constant'
                                     */
  real_T CompareToConstant_const_o; /* Mask Parameter: CompareToConstant_const_o
                                     * Referenced by: '<S31>/Constant'
                                     */
  real_T LowPassFilterDiscreteorContin_k;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_k
                               * Referenced by: '<S14>/Constant'
                               */
  real_T LowPassFilterDiscreteorContin_c;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_c
                               * Referenced by: '<S21>/Constant'
                               */
  real_T LowPassFilterDiscreteorConti_cz;
                              /* Mask Parameter: LowPassFilterDiscreteorConti_cz
                               * Referenced by: '<S28>/Constant'
                               */
  real32_T LowPassFilterDiscreteorConti_pu;
                              /* Mask Parameter: LowPassFilterDiscreteorConti_pu
                               * Referenced by: '<S11>/K'
                               */
  real32_T LowPassFilterDiscreteorContin_a;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_a
                               * Referenced by: '<S12>/K'
                               */
  real32_T LowPassFilterDiscreteorConti_cj;
                              /* Mask Parameter: LowPassFilterDiscreteorConti_cj
                               * Referenced by: '<S13>/K'
                               */
  real_T Motor_Right_Value;            /* Expression: 50
                                        * Referenced by: '<S3>/Motor_Right'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Motor_Left_Value;             /* Expression: -50
                                        * Referenced by: '<S3>/Motor_Left'
                                        */
  real_T Gain_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Constant1_Value;              /* Expression: 0.3
                                        * Referenced by: '<S3>/Constant1'
                                        */
  real_T Constant_Value;               /* Expression: 27
                                        * Referenced by: '<S3>/Constant'
                                        */
  char_T OLED_STRING1_String[256];     /* Expression: "UCT MICROMOUSE '25"
                                        * Referenced by: '<S3>/OLED_STRING1'
                                        */
  char_T OLED_STRING2_String[256];     /* Expression: "Simulink Data Demo"
                                        * Referenced by: '<S3>/OLED_STRING2'
                                        */
  real32_T Constant_Value_l;           /* Computed Parameter: Constant_Value_l
                                        * Referenced by: '<S18>/Constant'
                                        */
  real32_T Constant_Value_i;           /* Computed Parameter: Constant_Value_i
                                        * Referenced by: '<S25>/Constant'
                                        */
  real32_T Constant_Value_c;           /* Computed Parameter: Constant_Value_c
                                        * Referenced by: '<S32>/Constant'
                                        */
  real32_T Gain_Gain_g;                /* Computed Parameter: Gain_Gain_g
                                        * Referenced by: '<S10>/Gain'
                                        */
  real32_T Integrator_gainval;         /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S20>/Integrator'
                                        */
  real32_T Integrator_UpperSat;       /* Computed Parameter: Integrator_UpperSat
                                       * Referenced by: '<S20>/Integrator'
                                       */
  real32_T Integrator_LowerSat;       /* Computed Parameter: Integrator_LowerSat
                                       * Referenced by: '<S20>/Integrator'
                                       */
  real32_T Saturation_UpperSat;       /* Computed Parameter: Saturation_UpperSat
                                       * Referenced by: '<S20>/Saturation'
                                       */
  real32_T Saturation_LowerSat;       /* Computed Parameter: Saturation_LowerSat
                                       * Referenced by: '<S20>/Saturation'
                                       */
  real32_T Integrator_gainval_n;     /* Computed Parameter: Integrator_gainval_n
                                      * Referenced by: '<S27>/Integrator'
                                      */
  real32_T Integrator_UpperSat_k;   /* Computed Parameter: Integrator_UpperSat_k
                                     * Referenced by: '<S27>/Integrator'
                                     */
  real32_T Integrator_LowerSat_d;   /* Computed Parameter: Integrator_LowerSat_d
                                     * Referenced by: '<S27>/Integrator'
                                     */
  real32_T Saturation_UpperSat_f;   /* Computed Parameter: Saturation_UpperSat_f
                                     * Referenced by: '<S27>/Saturation'
                                     */
  real32_T Saturation_LowerSat_b;   /* Computed Parameter: Saturation_LowerSat_b
                                     * Referenced by: '<S27>/Saturation'
                                     */
  real32_T Integrator_gainval_k;     /* Computed Parameter: Integrator_gainval_k
                                      * Referenced by: '<S34>/Integrator'
                                      */
  real32_T Integrator_UpperSat_l;   /* Computed Parameter: Integrator_UpperSat_l
                                     * Referenced by: '<S34>/Integrator'
                                     */
  real32_T Integrator_LowerSat_g;   /* Computed Parameter: Integrator_LowerSat_g
                                     * Referenced by: '<S34>/Integrator'
                                     */
  real32_T Saturation_UpperSat_c;   /* Computed Parameter: Saturation_UpperSat_c
                                     * Referenced by: '<S34>/Saturation'
                                     */
  real32_T Saturation_LowerSat_o;   /* Computed Parameter: Saturation_LowerSat_o
                                     * Referenced by: '<S34>/Saturation'
                                     */
  real32_T Gain2_Gain;                 /* Computed Parameter: Gain2_Gain
                                        * Referenced by: '<S3>/Gain2'
                                        */
  real32_T DataStoreMemory_InitialValue;
                             /* Computed Parameter: DataStoreMemory_InitialValue
                              * Referenced by: '<S5>/Data Store Memory'
                              */
  real32_T DataStoreMemory1_InitialValue;
                            /* Computed Parameter: DataStoreMemory1_InitialValue
                             * Referenced by: '<S5>/Data Store Memory1'
                             */
  real32_T DataStoreMemory2_InitialValue;
                            /* Computed Parameter: DataStoreMemory2_InitialValue
                             * Referenced by: '<S5>/Data Store Memory2'
                             */
  uint32_T DataStoreMemory_InitialValue_o;
                           /* Computed Parameter: DataStoreMemory_InitialValue_o
                            * Referenced by: '<S10>/Data Store Memory'
                            */
  uint32_T DataStoreMemory3_InitialValue;
                            /* Computed Parameter: DataStoreMemory3_InitialValue
                             * Referenced by: '<S10>/Data Store Memory3'
                             */
  int16_T DataStoreMemory_InitialValue_l;
                           /* Computed Parameter: DataStoreMemory_InitialValue_l
                            * Referenced by: '<S8>/Data Store Memory'
                            */
  int16_T DataStoreMemory1_InitialValue_p;
                          /* Computed Parameter: DataStoreMemory1_InitialValue_p
                           * Referenced by: '<S8>/Data Store Memory1'
                           */
  int16_T DataStoreMemory2_InitialValue_n;
                          /* Computed Parameter: DataStoreMemory2_InitialValue_n
                           * Referenced by: '<S8>/Data Store Memory2'
                           */
  int16_T DataStoreMemory3_InitialValue_g;
                          /* Computed Parameter: DataStoreMemory3_InitialValue_g
                           * Referenced by: '<S8>/Data Store Memory3'
                           */
  int16_T DataStoreMemory1_InitialValue_h;
                          /* Computed Parameter: DataStoreMemory1_InitialValue_h
                           * Referenced by: '<S10>/Data Store Memory1'
                           */
  int16_T DataStoreMemory2_InitialValue_g;
                          /* Computed Parameter: DataStoreMemory2_InitialValue_g
                           * Referenced by: '<S10>/Data Store Memory2'
                           */
  uint16_T DataStoreMemory_InitialValue_b;
                           /* Computed Parameter: DataStoreMemory_InitialValue_b
                            * Referenced by: '<S4>/Data Store Memory'
                            */
  uint16_T DataStoreMemory1_InitialValue_a;
                          /* Computed Parameter: DataStoreMemory1_InitialValue_a
                           * Referenced by: '<S4>/Data Store Memory1'
                           */
  uint16_T DataStoreMemory2_InitialValue_e;
                          /* Computed Parameter: DataStoreMemory2_InitialValue_e
                           * Referenced by: '<S4>/Data Store Memory2'
                           */
  uint16_T DataStoreMemory3_InitialValue_n;
                          /* Computed Parameter: DataStoreMemory3_InitialValue_n
                           * Referenced by: '<S4>/Data Store Memory3'
                           */
  uint16_T DataStoreMemory4_InitialValue;
                            /* Computed Parameter: DataStoreMemory4_InitialValue
                             * Referenced by: '<S4>/Data Store Memory4'
                             */
  int8_T DataStoreMemory_InitialValue_oa;
                          /* Computed Parameter: DataStoreMemory_InitialValue_oa
                           * Referenced by: '<S1>/Data Store Memory'
                           */
  int8_T DataStoreMemory1_InitialValu_p2;
                          /* Computed Parameter: DataStoreMemory1_InitialValu_p2
                           * Referenced by: '<S1>/Data Store Memory1'
                           */
  uint8_T DataStoreMemory_InitialValue_e;
                           /* Computed Parameter: DataStoreMemory_InitialValue_e
                            * Referenced by: '<S2>/Data Store Memory'
                            */
  uint8_T DataStoreMemory1_InitialValue_b;
                          /* Computed Parameter: DataStoreMemory1_InitialValue_b
                           * Referenced by: '<S2>/Data Store Memory1'
                           */
  uint8_T DataStoreMemory2_InitialValue_p;
                          /* Computed Parameter: DataStoreMemory2_InitialValue_p
                           * Referenced by: '<S2>/Data Store Memory2'
                           */
  uint8_T DataStoreMemory3_InitialValue_c;
                          /* Computed Parameter: DataStoreMemory3_InitialValue_c
                           * Referenced by: '<S2>/Data Store Memory3'
                           */
  uint8_T DataStoreMemory4_InitialValue_f;
                          /* Computed Parameter: DataStoreMemory4_InitialValue_f
                           * Referenced by: '<S2>/Data Store Memory4'
                           */
  uint8_T DataStoreMemory_InitialValue_c;
                           /* Computed Parameter: DataStoreMemory_InitialValue_c
                            * Referenced by: '<S6>/Data Store Memory'
                            */
  uint8_T DataStoreMemory_InitialValue_n;
                           /* Computed Parameter: DataStoreMemory_InitialValue_n
                            * Referenced by: '<S9>/Data Store Memory'
                            */
  uint8_T DataStoreMemory_InitialValue_h;
                           /* Computed Parameter: DataStoreMemory_InitialValue_h
                            * Referenced by: '<S7>/Data Store Memory'
                            */
  uint8_T DataStoreMemory4_InitialValue_k;
                          /* Computed Parameter: DataStoreMemory4_InitialValue_k
                           * Referenced by: '<S8>/Data Store Memory4'
                           */
};

/* Code_Instrumentation_Declarations_Placeholder */

/* Real-time Model Data Structure */
struct tag_RTM_MicroMouse_Deploy_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P_MicroMouse_Deploy_T MicroMouse_Deploy_P;

/* Block signals (default storage) */
extern B_MicroMouse_Deploy_T MicroMouse_Deploy_B;

/* Block states (default storage) */
extern DW_MicroMouse_Deploy_T MicroMouse_Deploy_DW;

/* Model entry point functions */
extern void MicroMouse_Deploy_initialize(void);
extern void MicroMouse_Deploy_step(void);
extern void MicroMouse_Deploy_terminate(void);

/* Real-time Model object */
extern RT_MODEL_MicroMouse_Deploy_T *const MicroMouse_Deploy_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Pulse Generator' : Unused code path elimination
 * Block '<S3>/LW_enc_signal' : Unused code path elimination
 * Block '<S3>/RW_enc_signal' : Unused code path elimination
 * Block '<S3>/Scope' : Unused code path elimination
 * Block '<S3>/Scope1' : Unused code path elimination
 * Block '<S3>/Scope2' : Unused code path elimination
 * Block '<S4>/Cast To Single' : Unused code path elimination
 * Block '<S4>/Cast To Single1' : Unused code path elimination
 * Block '<S4>/Cast To Single2' : Unused code path elimination
 * Block '<S4>/Cast To Single3' : Unused code path elimination
 * Block '<S4>/Cast To Single4' : Unused code path elimination
 * Block '<S4>/Gain1' : Unused code path elimination
 * Block '<S4>/Gain2' : Unused code path elimination
 * Block '<S4>/Gain4' : Unused code path elimination
 * Block '<S5>/Cast To Single1' : Unused code path elimination
 * Block '<S5>/Cast To Single2' : Unused code path elimination
 * Block '<S7>/NOT' : Unused code path elimination
 * Block '<S7>/NOT1' : Unused code path elimination
 * Block '<S8>/Cast To Single1' : Unused code path elimination
 * Block '<S8>/Cast To Single3' : Unused code path elimination
 * Block '<S10>/Cast To Single1' : Unused code path elimination
 * Block '<S10>/Cast To Single2' : Unused code path elimination
 * Block '<S10>/Cast To Single3' : Unused code path elimination
 * Block '<S3>/Cast' : Eliminate redundant data type conversion
 * Block '<S3>/Cast1' : Eliminate redundant data type conversion
 * Block '<S3>/Cast2' : Eliminate redundant data type conversion
 * Block '<S5>/Cast To Single' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'MicroMouse_Deploy'
 * '<S1>'   : 'MicroMouse_Deploy/Motors'
 * '<S2>'   : 'MicroMouse_Deploy/SSD1306'
 * '<S3>'   : 'MicroMouse_Deploy/StudentTemplate'
 * '<S4>'   : 'MicroMouse_Deploy/Subsystem'
 * '<S5>'   : 'MicroMouse_Deploy/Subsystem1'
 * '<S6>'   : 'MicroMouse_Deploy/Subsystem2'
 * '<S7>'   : 'MicroMouse_Deploy/Subsystem3'
 * '<S8>'   : 'MicroMouse_Deploy/Subsystem4'
 * '<S9>'   : 'MicroMouse_Deploy/Subsystem5'
 * '<S10>'  : 'MicroMouse_Deploy/ToFs'
 * '<S11>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)'
 * '<S12>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1'
 * '<S13>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2'
 * '<S14>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant'
 * '<S15>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Initialization'
 * '<S16>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)'
 * '<S17>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Constant'
 * '<S18>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Zero'
 * '<S19>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Initialization/Init_u'
 * '<S20>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)/Discrete'
 * '<S21>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant'
 * '<S22>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Initialization'
 * '<S23>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)'
 * '<S24>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Constant'
 * '<S25>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Zero'
 * '<S26>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Initialization/Init_u'
 * '<S27>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)/Discrete'
 * '<S28>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant'
 * '<S29>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Initialization'
 * '<S30>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Integrator (Discrete or Continuous)'
 * '<S31>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant/Compare To Constant'
 * '<S32>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant/Compare To Zero'
 * '<S33>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Initialization/Init_u'
 * '<S34>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Integrator (Discrete or Continuous)/Discrete'
 */
#endif                                 /* MicroMouse_Deploy_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
