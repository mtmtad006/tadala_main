/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy.h
 *
 * Code generated for Simulink model 'MicroMouse_Deploy'.
 *
 * Model version                  : 5.10
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Thu Oct  2 21:44:06 2025
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
#include <string.h>
#include <stddef.h>
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
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
  real_T FilterCoefficient;            /* '<S79>/Filter Coefficient' */
  real_T IntegralGain;                 /* '<S73>/Integral Gain' */
  real_T UnitDelay1;                   /* '<S98>/Unit Delay1' */
  real_T Product3;                     /* '<S100>/Product3' */
  real_T UnitDelay1_h;                 /* '<S99>/Unit Delay1' */
  real_T Product3_b;                   /* '<S101>/Product3' */
  real_T DeadZone;                     /* '<S18>/Dead Zone' */
  real_T Product4;                     /* '<S100>/Product4' */
  real_T Product5;                     /* '<S100>/Product5' */
  real_T UnitDelay2;                   /* '<S98>/Unit Delay2' */
  real_T Sum1;                         /* '<S98>/Sum1' */
  real_T Sum3;                         /* '<S98>/Sum3' */
  real_T Product4_l;                   /* '<S101>/Product4' */
  real_T Product5_a;                   /* '<S101>/Product5' */
  real_T UnitDelay2_i;                 /* '<S99>/Unit Delay2' */
  real_T Sum1_k;                       /* '<S99>/Sum1' */
  real_T Sum3_n;                       /* '<S99>/Sum3' */
  real_T ang_vel;                      /* '<S3>/distance_control' */
  real_T LW_w;                         /* '<S14>/turn_adjus' */
  real_T RW_w;                         /* '<S14>/turn_adjus' */
  real32_T Probe[2];                   /* '<S19>/Probe' */
  real32_T Probe_h[2];                 /* '<S26>/Probe' */
  real32_T Probe_g[2];                 /* '<S33>/Probe' */
} B_MicroMouse_Deploy_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  dsp_simulink_MedianFilter_Mic_T obj; /* '<S18>/Median Filter' */
  real_T UnitDelay1_DSTATE;            /* '<S98>/Unit Delay1' */
  real_T UnitDelay1_DSTATE_a;          /* '<S99>/Unit Delay1' */
  real_T UnitDelay2_DSTATE;            /* '<S98>/Unit Delay2' */
  real_T UnitDelay2_DSTATE_o;          /* '<S99>/Unit Delay2' */
  real32_T Integrator_DSTATE;          /* '<S25>/Integrator' */
  real32_T Integrator_DSTATE_m;        /* '<S32>/Integrator' */
  real32_T Integrator_DSTATE_i;        /* '<S39>/Integrator' */
  struct {
    uint_T is_c3_MicroMouse_Deploy : 2;/* '<S3>/distance_control' */
    uint_T is_c1_MicroMouse_Deploy : 2;/* '<S14>/turn_adjus' */
    uint_T is_active_c3_MicroMouse_Deploy : 1;/* '<S3>/distance_control' */
    uint_T is_active_c1_MicroMouse_Deploy : 1;/* '<S14>/turn_adjus' */
  } bitsForTID1;

  uint8_T Delay_DSTATE;                /* '<S15>/Delay' */
  uint8_T DelayInput1_DSTATE;          /* '<S95>/Delay Input1' */
  int8_T Integrator_PrevResetState;    /* '<S25>/Integrator' */
  int8_T Integrator_PrevResetState_a;  /* '<S32>/Integrator' */
  int8_T Integrator_PrevResetState_j;  /* '<S39>/Integrator' */
  uint8_T Integrator_IC_LOADING;       /* '<S25>/Integrator' */
  uint8_T Integrator_IC_LOADING_k;     /* '<S32>/Integrator' */
  uint8_T Integrator_IC_LOADING_a;     /* '<S39>/Integrator' */
} DW_MicroMouse_Deploy_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S14>/Integrator' */
  real_T Integrator_CSTATE_p;          /* '<S76>/Integrator' */
  real_T Filter_CSTATE;                /* '<S71>/Filter' */
} X_MicroMouse_Deploy_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S14>/Integrator' */
  real_T Integrator_CSTATE_p;          /* '<S76>/Integrator' */
  real_T Filter_CSTATE;                /* '<S71>/Filter' */
} XDot_MicroMouse_Deploy_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S14>/Integrator' */
  boolean_T Integrator_CSTATE_p;       /* '<S76>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S71>/Filter' */
} XDis_MicroMouse_Deploy_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Parameters (default storage) */
struct P_MicroMouse_Deploy_T_ {
  real_T PIDController_D;              /* Mask Parameter: PIDController_D
                                        * Referenced by: '<S69>/Derivative Gain'
                                        */
  real_T PIDController_I;              /* Mask Parameter: PIDController_I
                                        * Referenced by: '<S73>/Integral Gain'
                                        */
  real_T PIDController_InitialConditionF;
                              /* Mask Parameter: PIDController_InitialConditionF
                               * Referenced by: '<S71>/Filter'
                               */
  real_T PIDController_InitialConditio_l;
                              /* Mask Parameter: PIDController_InitialConditio_l
                               * Referenced by: '<S76>/Integrator'
                               */
  real_T PIDController_N;              /* Mask Parameter: PIDController_N
                                        * Referenced by: '<S79>/Filter Coefficient'
                                        */
  real_T PIDController_P;              /* Mask Parameter: PIDController_P
                                        * Referenced by: '<S81>/Proportional Gain'
                                        */
  real_T LowPassFilterDiscreteorContinuo;
                              /* Mask Parameter: LowPassFilterDiscreteorContinuo
                               * Referenced by: '<S19>/Time constant'
                               */
  real_T LowPassFilterDiscreteorContin_p;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_p
                               * Referenced by: '<S26>/Time constant'
                               */
  real_T LowPassFilterDiscreteorContin_e;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_e
                               * Referenced by: '<S33>/Time constant'
                               */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S22>/Constant'
                                       */
  real_T CompareToConstant_const_j; /* Mask Parameter: CompareToConstant_const_j
                                     * Referenced by: '<S29>/Constant'
                                     */
  real_T CompareToConstant_const_o; /* Mask Parameter: CompareToConstant_const_o
                                     * Referenced by: '<S36>/Constant'
                                     */
  real_T LowPassFilterDiscreteorContin_k;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_k
                               * Referenced by: '<S19>/Constant'
                               */
  real_T LowPassFilterDiscreteorContin_c;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_c
                               * Referenced by: '<S26>/Constant'
                               */
  real_T LowPassFilterDiscreteorConti_cz;
                              /* Mask Parameter: LowPassFilterDiscreteorConti_cz
                               * Referenced by: '<S33>/Constant'
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
  real32_T CompareToConstant_const_l;
                                    /* Mask Parameter: CompareToConstant_const_l
                                     * Referenced by: '<S94>/Constant'
                                     */
  uint8_T DetectRisePositive_vinit;  /* Mask Parameter: DetectRisePositive_vinit
                                      * Referenced by: '<S95>/Delay Input1'
                                      */
  real_T Circumference_Value;          /* Expression: (2*pi*0.032)
                                        * Referenced by: '<S17>/Circumference'
                                        */
  real_T ref_Value;                    /* Expression: 2
                                        * Referenced by: '<S3>/ref'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S14>/Constant'
                                        */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S14>/Integrator'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Gain_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Constant1_Value;              /* Expression: 0.3
                                        * Referenced by: '<S3>/Constant1'
                                        */
  real_T Constant_Value_l;             /* Expression: 27
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Constant2_Value;              /* Expression: 5
                                        * Referenced by: '<S3>/Constant2'
                                        */
  real_T UnitDelay1_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<S98>/Unit Delay1'
                                        */
  real_T tau_Gain;                     /* Expression: tau
                                        * Referenced by: '<S98>/tau'
                                        */
  real_T Bias_Bias;                    /* Expression: 1
                                        * Referenced by: '<S100>/Bias'
                                        */
  real_T Gain_Gain_i;                  /* Expression: 2*zeta
                                        * Referenced by: '<S100>/Gain'
                                        */
  real_T UnitDelay1_InitialCondition_p;/* Expression: 0
                                        * Referenced by: '<S99>/Unit Delay1'
                                        */
  real_T tau_Gain_m;                   /* Expression: tau
                                        * Referenced by: '<S99>/tau'
                                        */
  real_T Bias_Bias_l;                  /* Expression: 1
                                        * Referenced by: '<S101>/Bias'
                                        */
  real_T Gain_Gain_d;                  /* Expression: 2*zeta
                                        * Referenced by: '<S101>/Gain'
                                        */
  real_T DeadZone_Start;               /* Expression: -0.5
                                        * Referenced by: '<S18>/Dead Zone'
                                        */
  real_T DeadZone_End;                 /* Expression: 0.5
                                        * Referenced by: '<S18>/Dead Zone'
                                        */
  real_T Bias1_Bias;                   /* Expression: 1
                                        * Referenced by: '<S100>/Bias1'
                                        */
  real_T Gain1_Gain_f;                 /* Expression: 2
                                        * Referenced by: '<S98>/Gain1'
                                        */
  real_T UnitDelay2_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<S98>/Unit Delay2'
                                        */
  real_T Bias1_Bias_d;                 /* Expression: 1
                                        * Referenced by: '<S101>/Bias1'
                                        */
  real_T Gain1_Gain_e;                 /* Expression: 2
                                        * Referenced by: '<S99>/Gain1'
                                        */
  real_T UnitDelay2_InitialCondition_n;/* Expression: 0
                                        * Referenced by: '<S99>/Unit Delay2'
                                        */
  char_T OLED_STRING1_String[256];     /* Expression: "UCT MICROMOUSE '25"
                                        * Referenced by: '<S3>/OLED_STRING1'
                                        */
  char_T OLED_STRING2_String[256];     /* Expression: "Simulink Data Demo"
                                        * Referenced by: '<S3>/OLED_STRING2'
                                        */
  real32_T Constant_Value_lf;          /* Computed Parameter: Constant_Value_lf
                                        * Referenced by: '<S23>/Constant'
                                        */
  real32_T Constant_Value_i;           /* Computed Parameter: Constant_Value_i
                                        * Referenced by: '<S30>/Constant'
                                        */
  real32_T Constant_Value_c;           /* Computed Parameter: Constant_Value_c
                                        * Referenced by: '<S37>/Constant'
                                        */
  real32_T Gain2_Gain;                 /* Computed Parameter: Gain2_Gain
                                        * Referenced by: '<S4>/Gain2'
                                        */
  real32_T Gain_Gain_g;                /* Computed Parameter: Gain_Gain_g
                                        * Referenced by: '<S10>/Gain'
                                        */
  real32_T Integrator_gainval;         /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S25>/Integrator'
                                        */
  real32_T Integrator_UpperSat;       /* Computed Parameter: Integrator_UpperSat
                                       * Referenced by: '<S25>/Integrator'
                                       */
  real32_T Integrator_LowerSat;       /* Computed Parameter: Integrator_LowerSat
                                       * Referenced by: '<S25>/Integrator'
                                       */
  real32_T Saturation_UpperSat;       /* Computed Parameter: Saturation_UpperSat
                                       * Referenced by: '<S25>/Saturation'
                                       */
  real32_T Saturation_LowerSat;       /* Computed Parameter: Saturation_LowerSat
                                       * Referenced by: '<S25>/Saturation'
                                       */
  real32_T Integrator_gainval_n;     /* Computed Parameter: Integrator_gainval_n
                                      * Referenced by: '<S32>/Integrator'
                                      */
  real32_T Integrator_UpperSat_k;   /* Computed Parameter: Integrator_UpperSat_k
                                     * Referenced by: '<S32>/Integrator'
                                     */
  real32_T Integrator_LowerSat_d;   /* Computed Parameter: Integrator_LowerSat_d
                                     * Referenced by: '<S32>/Integrator'
                                     */
  real32_T Saturation_UpperSat_f;   /* Computed Parameter: Saturation_UpperSat_f
                                     * Referenced by: '<S32>/Saturation'
                                     */
  real32_T Saturation_LowerSat_b;   /* Computed Parameter: Saturation_LowerSat_b
                                     * Referenced by: '<S32>/Saturation'
                                     */
  real32_T Integrator_gainval_k;     /* Computed Parameter: Integrator_gainval_k
                                      * Referenced by: '<S39>/Integrator'
                                      */
  real32_T Integrator_UpperSat_l;   /* Computed Parameter: Integrator_UpperSat_l
                                     * Referenced by: '<S39>/Integrator'
                                     */
  real32_T Integrator_LowerSat_g;   /* Computed Parameter: Integrator_LowerSat_g
                                     * Referenced by: '<S39>/Integrator'
                                     */
  real32_T Saturation_UpperSat_c;   /* Computed Parameter: Saturation_UpperSat_c
                                     * Referenced by: '<S39>/Saturation'
                                     */
  real32_T Saturation_LowerSat_o;   /* Computed Parameter: Saturation_LowerSat_o
                                     * Referenced by: '<S39>/Saturation'
                                     */
  real32_T Gain2_Gain_f;               /* Computed Parameter: Gain2_Gain_f
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
  boolean_T Constant_Value_d;          /* Computed Parameter: Constant_Value_d
                                        * Referenced by: '<S96>/Constant'
                                        */
  int8_T DataStoreMemory_InitialValue_oa;
                          /* Computed Parameter: DataStoreMemory_InitialValue_oa
                           * Referenced by: '<S1>/Data Store Memory'
                           */
  int8_T DataStoreMemory1_InitialValu_p2;
                          /* Computed Parameter: DataStoreMemory1_InitialValu_p2
                           * Referenced by: '<S1>/Data Store Memory1'
                           */
  uint8_T Tick_per_rev_Gain;           /* Computed Parameter: Tick_per_rev_Gain
                                        * Referenced by: '<S17>/Tick_per_rev'
                                        */
  uint8_T Delay_InitialCondition;  /* Computed Parameter: Delay_InitialCondition
                                    * Referenced by: '<S15>/Delay'
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
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_MicroMouse_Deploy_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_MicroMouse_Deploy_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[3];
  real_T odeF[3][3];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_MicroMouse_Deploy_T MicroMouse_Deploy_P;

/* Block signals (default storage) */
extern B_MicroMouse_Deploy_T MicroMouse_Deploy_B;

/* Continuous states (default storage) */
extern X_MicroMouse_Deploy_T MicroMouse_Deploy_X;

/* Disabled states (default storage) */
extern XDis_MicroMouse_Deploy_T MicroMouse_Deploy_XDis;

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
 * Block '<S3>/Motor_Left' : Unused code path elimination
 * Block '<S3>/Motor_Right' : Unused code path elimination
 * Block '<S3>/RW_enc_signal' : Unused code path elimination
 * Block '<S3>/Scope' : Unused code path elimination
 * Block '<S3>/Scope1' : Unused code path elimination
 * Block '<S3>/Scope2' : Unused code path elimination
 * Block '<S3>/Scope3' : Unused code path elimination
 * Block '<S3>/Scope4' : Unused code path elimination
 * Block '<S3>/Scope5' : Unused code path elimination
 * Block '<S3>/Scope6' : Unused code path elimination
 * Block '<S14>/Scope' : Unused code path elimination
 * Block '<S3>/dist_travelled' : Unused code path elimination
 * Block '<S3>/encoder_ticks' : Unused code path elimination
 * Block '<S3>/velocity' : Unused code path elimination
 * Block '<S4>/Cast To Single2' : Unused code path elimination
 * Block '<S4>/Cast To Single3' : Unused code path elimination
 * Block '<S4>/Cast To Single4' : Unused code path elimination
 * Block '<S4>/Gain1' : Unused code path elimination
 * Block '<S4>/Gain4' : Unused code path elimination
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
 * Block '<S5>/Cast To Single1' : Eliminate redundant data type conversion
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
 * '<S14>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2'
 * '<S15>'  : 'MicroMouse_Deploy/StudentTemplate/convert_encoder_signal_to_ticks'
 * '<S16>'  : 'MicroMouse_Deploy/StudentTemplate/distance_control'
 * '<S17>'  : 'MicroMouse_Deploy/StudentTemplate/encoder_tick_to_dist'
 * '<S18>'  : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing '
 * '<S19>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant'
 * '<S20>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Initialization'
 * '<S21>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)'
 * '<S22>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Constant'
 * '<S23>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Zero'
 * '<S24>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Initialization/Init_u'
 * '<S25>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)/Discrete'
 * '<S26>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant'
 * '<S27>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Initialization'
 * '<S28>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)'
 * '<S29>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Constant'
 * '<S30>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Zero'
 * '<S31>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Initialization/Init_u'
 * '<S32>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)/Discrete'
 * '<S33>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant'
 * '<S34>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Initialization'
 * '<S35>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Integrator (Discrete or Continuous)'
 * '<S36>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant/Compare To Constant'
 * '<S37>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant/Compare To Zero'
 * '<S38>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Initialization/Init_u'
 * '<S39>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Integrator (Discrete or Continuous)/Discrete'
 * '<S40>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller'
 * '<S41>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/turn_adjus'
 * '<S42>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Anti-windup'
 * '<S43>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/D Gain'
 * '<S44>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/External Derivative'
 * '<S45>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Filter'
 * '<S46>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Filter ICs'
 * '<S47>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/I Gain'
 * '<S48>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Ideal P Gain'
 * '<S49>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Ideal P Gain Fdbk'
 * '<S50>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Integrator'
 * '<S51>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Integrator ICs'
 * '<S52>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/N Copy'
 * '<S53>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/N Gain'
 * '<S54>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/P Copy'
 * '<S55>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Parallel P Gain'
 * '<S56>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Reset Signal'
 * '<S57>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Saturation'
 * '<S58>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Saturation Fdbk'
 * '<S59>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Sum'
 * '<S60>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Sum Fdbk'
 * '<S61>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tracking Mode'
 * '<S62>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tracking Mode Sum'
 * '<S63>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tsamp - Integral'
 * '<S64>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tsamp - Ngain'
 * '<S65>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/postSat Signal'
 * '<S66>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/preInt Signal'
 * '<S67>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/preSat Signal'
 * '<S68>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Anti-windup/Passthrough'
 * '<S69>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/D Gain/Internal Parameters'
 * '<S70>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/External Derivative/Error'
 * '<S71>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Filter/Cont. Filter'
 * '<S72>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S73>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/I Gain/Internal Parameters'
 * '<S74>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Ideal P Gain/Passthrough'
 * '<S75>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S76>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Integrator/Continuous'
 * '<S77>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Integrator ICs/Internal IC'
 * '<S78>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/N Copy/Disabled'
 * '<S79>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/N Gain/Internal Parameters'
 * '<S80>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/P Copy/Disabled'
 * '<S81>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S82>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Reset Signal/Disabled'
 * '<S83>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Saturation/Passthrough'
 * '<S84>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Saturation Fdbk/Disabled'
 * '<S85>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Sum/Sum_PID'
 * '<S86>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Sum Fdbk/Disabled'
 * '<S87>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tracking Mode/Disabled'
 * '<S88>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S89>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S90>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S91>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/postSat Signal/Forward_Path'
 * '<S92>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/preInt Signal/Internal PreInt'
 * '<S93>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/preSat Signal/Forward_Path'
 * '<S94>'  : 'MicroMouse_Deploy/StudentTemplate/convert_encoder_signal_to_ticks/Compare To Constant'
 * '<S95>'  : 'MicroMouse_Deploy/StudentTemplate/convert_encoder_signal_to_ticks/Detect Rise Positive'
 * '<S96>'  : 'MicroMouse_Deploy/StudentTemplate/convert_encoder_signal_to_ticks/Detect Rise Positive/Positive'
 * '<S97>'  : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass'
 * '<S98>'  : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass/SOS1'
 * '<S99>'  : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass/SOS2'
 * '<S100>' : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass/SOS1/Arithmetic'
 * '<S101>' : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass/SOS2/Arithmetic'
 */
#endif                                 /* MicroMouse_Deploy_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
