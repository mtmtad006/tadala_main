/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy.h
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

#ifndef MicroMouse_Deploy_h_
#define MicroMouse_Deploy_h_
#ifndef MicroMouse_Deploy_COMMON_INCLUDES_
#define MicroMouse_Deploy_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* MicroMouse_Deploy_COMMON_INCLUDES_ */

#include "MicroMouse_Deploy_types.h"
#include "rtGetInf.h"
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
  real_T RateLimiter2;                 /* '<S11>/Rate Limiter2' */
  real_T RateLimiter4;                 /* '<S11>/Rate Limiter4' */
  real_T FilterCoefficient;            /* '<S277>/Filter Coefficient' */
  real_T FilterCoefficient_g;          /* '<S485>/Filter Coefficient' */
  real_T FilterCoefficient_j;          /* '<S433>/Filter Coefficient' */
  real_T FilterCoefficient_b;          /* '<S329>/Filter Coefficient' */
  real_T FilterCoefficient_n;          /* '<S381>/Filter Coefficient' */
  real_T CastToDouble[3];              /* '<S3>/Cast To Double' */
  real_T IntegralGain;                 /* '<S106>/Integral Gain' */
  real_T FilterCoefficient_k;          /* '<S112>/Filter Coefficient' */
  real_T FilterCoefficient_f;          /* '<S219>/Filter Coefficient' */
  real_T IntegralGain_c;               /* '<S271>/Integral Gain' */
  real_T IntegralGain_a;               /* '<S323>/Integral Gain' */
  real_T IntegralGain_j;               /* '<S375>/Integral Gain' */
  real_T IntegralGain_m;               /* '<S427>/Integral Gain' */
  real_T IntegralGain_o;               /* '<S479>/Integral Gain' */
  real_T UnitDelay1;                   /* '<S501>/Unit Delay1' */
  real_T Product3_k;                   /* '<S503>/Product3' */
  real_T UnitDelay1_n;                 /* '<S502>/Unit Delay1' */
  real_T Product3_d;                   /* '<S504>/Product3' */
  real_T DeadZone;                     /* '<S23>/Dead Zone' */
  real_T Product4;                     /* '<S503>/Product4' */
  real_T Product5;                     /* '<S503>/Product5' */
  real_T UnitDelay2;                   /* '<S501>/Unit Delay2' */
  real_T Sum1;                         /* '<S501>/Sum1' */
  real_T Sum3;                         /* '<S501>/Sum3' */
  real_T Product4_k;                   /* '<S504>/Product4' */
  real_T Product5_a;                   /* '<S504>/Product5' */
  real_T UnitDelay2_b;                 /* '<S502>/Unit Delay2' */
  real_T Sum1_k;                       /* '<S502>/Sum1' */
  real_T Sum3_o;                       /* '<S502>/Sum3' */
  real_T RateLimiter1;                 /* '<S12>/Rate Limiter1' */
  real_T RateLimiter2_j;               /* '<S12>/Rate Limiter2' */
  real_T DerivativeGain;               /* '<S154>/Derivative Gain' */
  real_T IntegralGain_i;               /* '<S158>/Integral Gain' */
  real_T FilterCoefficient_o;          /* '<S164>/Filter Coefficient' */
  real_T DerivativeGain_c;             /* '<S535>/Derivative Gain' */
  real_T IntegralGain_e;               /* '<S539>/Integral Gain' */
  real_T FilterCoefficient_ja;         /* '<S545>/Filter Coefficient' */
  real_T DerivativeGain_k;             /* '<S587>/Derivative Gain' */
  real_T IntegralGain_k;               /* '<S591>/Integral Gain' */
  real_T FilterCoefficient_nz;         /* '<S597>/Filter Coefficient' */
  real_T error;                      /* '<S24>/engage_disengage_straightnert' */
  real_T TOF_L_out;                  /* '<S24>/engage_disengage_straightnert' */
  real_T TOF_R_out;                  /* '<S24>/engage_disengage_straightnert' */
  real_T Left_distance;                /* '<S22>/Chart' */
  real_T Right_distance;               /* '<S22>/Chart' */
  real_T Z_angle;                      /* '<S3>/draft2_Maze_exploration2' */
  real_T Z_angle_l;                    /* '<S3>/draft2_Maze_exploration1' */
  real_T LW_c;                         /* '<S3>/draft2_Maze_exploration' */
  real_T RW_d;                         /* '<S3>/draft2_Maze_exploration' */
  real_T turn_indicator_i;             /* '<S3>/draft2_Maze_exploration' */
  real_T Z_angle_f;                    /* '<S3>/draft2_Maze_exploration' */
  real_T offset;                       /* '<S18>/Chart' */
  real32_T Probe[2];                   /* '<S54>/Probe' */
  real32_T Saturation;                 /* '<S60>/Saturation' */
  real32_T Probe_g[2];                 /* '<S68>/Probe' */
  real32_T Saturation_m;               /* '<S74>/Saturation' */
  real32_T Probe_h[2];                 /* '<S61>/Probe' */
  real32_T Saturation_d;               /* '<S67>/Saturation' */
  real32_T AvoidDividebyZero;          /* '<S54>/Avoid Divide by Zero' */
  real32_T uT;                         /* '<S13>/1//T' */
  real32_T AvoidDividebyZero_k;        /* '<S61>/Avoid Divide by Zero' */
  real32_T uT_p;                       /* '<S14>/1//T' */
  real32_T AvoidDividebyZero_n;        /* '<S68>/Avoid Divide by Zero' */
  real32_T uT_e;                       /* '<S15>/1//T' */
  uint16_T Delay;                      /* '<S27>/Delay' */
  uint16_T Add_f;                      /* '<S27>/Add' */
  uint16_T Delay_f;                    /* '<S26>/Delay' */
  uint16_T Add_h;                      /* '<S26>/Add' */
  uint8_T Compare;                     /* '<S38>/Compare' */
  uint8_T Uk1;                         /* '<S36>/Delay Input1' */
  uint8_T Compare_a;                   /* '<S32>/Compare' */
  uint8_T Uk1_k;                       /* '<S31>/Delay Input1' */
  boolean_T TOF_LEFT;                  /* '<S3>/GreaterThan' */
  boolean_T TOF_RIGHT;                 /* '<S3>/GreaterThan2' */
  boolean_T TOF_FRONT;                 /* '<S3>/GreaterThan1' */
  boolean_T RelationalOperator;        /* '<S22>/Relational Operator' */
  boolean_T RelationalOperator1;       /* '<S22>/Relational Operator1' */
  boolean_T RelationalOperator4;       /* '<S22>/Relational Operator4' */
  boolean_T RelationalOperator2;       /* '<S22>/Relational Operator2' */
  boolean_T RelationalOperator3;       /* '<S22>/Relational Operator3' */
  boolean_T RelationalOperator5;       /* '<S22>/Relational Operator5' */
  boolean_T Compare_n;                 /* '<S37>/Compare' */
  boolean_T RelationalOperator_n;      /* '<S3>/Relational Operator' */
} B_MicroMouse_Deploy_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  dsp_simulink_MovingAverage_Mi_T obj; /* '<S24>/Moving Average1' */
  dsp_simulink_MovingAverage_Mi_T obj_b;/* '<S24>/Moving Average' */
  dsp_simulink_MedianFilter_Mic_T obj_o;/* '<S23>/Median Filter' */
  real_T UnitDelay1_DSTATE;            /* '<S501>/Unit Delay1' */
  real_T UnitDelay1_DSTATE_n;          /* '<S502>/Unit Delay1' */
  real_T UnitDelay2_DSTATE;            /* '<S501>/Unit Delay2' */
  real_T UnitDelay2_DSTATE_j;          /* '<S502>/Unit Delay2' */
  real_T Delay_DSTATE;                 /* '<S39>/Delay' */
  real_T Delay_DSTATE_k;               /* '<S40>/Delay' */
  real_T PrevY;                        /* '<S11>/Rate Limiter2' */
  real_T LastMajorTime;                /* '<S11>/Rate Limiter2' */
  real_T PrevY_i;                      /* '<S11>/Rate Limiter4' */
  real_T LastMajorTime_b;              /* '<S11>/Rate Limiter4' */
  real_T PrevY_l;                      /* '<S12>/Rate Limiter1' */
  real_T LastMajorTime_c;              /* '<S12>/Rate Limiter1' */
  real_T PrevY_b;                      /* '<S12>/Rate Limiter2' */
  real_T LastMajorTime_k;              /* '<S12>/Rate Limiter2' */
  real_T difference;                   /* '<S22>/Chart' */
  real_T end_dist;                     /* '<S3>/draft2_Maze_exploration' */
  real_T turn_dist;                    /* '<S3>/draft2_Maze_exploration' */
  real_T temporalCounter_i1;           /* '<S3>/draft2_Maze_exploration' */
  real_T previousTime;                 /* '<S3>/draft2_Maze_exploration' */
  real32_T Integrator_DSTATE;          /* '<S60>/Integrator' */
  real32_T Integrator_DSTATE_i;        /* '<S74>/Integrator' */
  real32_T Integrator_DSTATE_m;        /* '<S67>/Integrator' */
  real32_T end_dist_o;                 /* '<S3>/draft2_Maze_exploration2' */
  real32_T end_dist_a;                 /* '<S3>/draft2_Maze_exploration1' */
  real32_T theta;                      /* '<S3>/draft2_Maze_exploration1' */
  real32_T TLR_dist;                   /* '<S3>/draft2_Maze_exploration' */
  real32_T TFR_dist;                   /* '<S3>/draft2_Maze_exploration' */
  struct {
    uint_T is_c8_MicroMouse_Deploy : 5;/* '<S3>/draft2_Maze_exploration' */
    uint_T is_c10_MicroMouse_Deploy : 4;/* '<S3>/draft2_Maze_exploration2' */
    uint_T is_c6_MicroMouse_Deploy : 4;/* '<S3>/draft2_Maze_exploration1' */
    uint_T is_active_c10_MicroMouse_Deploy : 1;/* '<S3>/draft2_Maze_exploration2' */
    uint_T is_active_c6_MicroMouse_Deploy : 1;/* '<S3>/draft2_Maze_exploration1' */
    uint_T is_active_c8_MicroMouse_Deploy : 1;/* '<S3>/draft2_Maze_exploration' */
  } bitsForTID0;

  struct {
    uint_T is_c5_MicroMouse_Deploy : 2;/* '<S3>/turn_adjus' */
    uint_T is_c3_MicroMouse_Deploy : 2;
                                     /* '<S24>/engage_disengage_straightnert' */
    uint_T is_c9_MicroMouse_Deploy : 2;/* '<S22>/Chart' */
    uint_T is_c1_MicroMouse_Deploy : 2;/* '<S18>/turn_adjus' */
    uint_T is_c2_MicroMouse_Deploy : 2;/* '<S18>/Chart' */
    uint_T is_c4_MicroMouse_Deploy : 2;/* '<S40>/Chart' */
    uint_T is_c7_MicroMouse_Deploy : 2;/* '<S27>/Chart' */
    uint_T is_active_c5_MicroMouse_Deploy : 1;/* '<S3>/turn_adjus' */
    uint_T is_active_c3_MicroMouse_Deploy : 1;
                                     /* '<S24>/engage_disengage_straightnert' */
    uint_T is_active_c9_MicroMouse_Deploy : 1;/* '<S22>/Chart' */
    uint_T is_active_c1_MicroMouse_Deploy : 1;/* '<S18>/turn_adjus' */
    uint_T is_active_c2_MicroMouse_Deploy : 1;/* '<S18>/Chart' */
    uint_T is_active_c4_MicroMouse_Deploy : 1;/* '<S40>/Chart' */
    uint_T is_active_c7_MicroMouse_Deploy : 1;/* '<S27>/Chart' */
  } bitsForTID1;

  uint16_T Delay_DSTATE_d;             /* '<S27>/Delay' */
  uint16_T Delay_DSTATE_f;             /* '<S26>/Delay' */
  uint8_T DelayInput1_DSTATE;          /* '<S36>/Delay Input1' */
  uint8_T DelayInput1_DSTATE_c;        /* '<S31>/Delay Input1' */
  uint8_T DelayInput1_DSTATE_b;        /* '<S46>/Delay Input1' */
  uint8_T DelayInput1_DSTATE_k;        /* '<S51>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_e;      /* '<S35>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_i;      /* '<S50>/Delay Input1' */
  int8_T Integrator_PrevResetState;    /* '<S60>/Integrator' */
  int8_T Integrator_PrevResetState_j;  /* '<S74>/Integrator' */
  int8_T Integrator_PrevResetState_a;  /* '<S67>/Integrator' */
  uint8_T Integrator_IC_LOADING;       /* '<S60>/Integrator' */
  uint8_T Integrator_IC_LOADING_a;     /* '<S74>/Integrator' */
  uint8_T Integrator_IC_LOADING_k;     /* '<S67>/Integrator' */
  boolean_T PrevLimited;               /* '<S11>/Rate Limiter2' */
  boolean_T PrevLimited_o;             /* '<S11>/Rate Limiter4' */
  boolean_T PrevLimited_e;             /* '<S12>/Rate Limiter1' */
  boolean_T PrevLimited_l;             /* '<S12>/Rate Limiter2' */
} DW_MicroMouse_Deploy_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S3>/Integrator' */
  real_T Integrator_CSTATE_o;          /* '<S274>/Integrator' */
  real_T Filter_CSTATE;                /* '<S269>/Filter' */
  real_T Integrator_CSTATE_k;          /* '<S482>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S477>/Filter' */
  real_T Integrator_CSTATE_a;          /* '<S430>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S425>/Filter' */
  real_T Integrator_CSTATE_l;          /* '<S326>/Integrator' */
  real_T Filter_CSTATE_e;              /* '<S321>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S378>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S373>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S3>/Integrator1' */
  real_T Filter_CSTATE_k;              /* '<S104>/Filter' */
  real_T Integrator_CSTATE_e;          /* '<S109>/Integrator' */
  real_T Integrator_CSTATE_j;          /* '<S18>/Integrator' */
  real_T Filter_CSTATE_lq;             /* '<S211>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S12>/Integrator' */
  real_T Integrator1_CSTATE_g;         /* '<S12>/Integrator1' */
  real_T Filter_CSTATE_n;              /* '<S156>/Filter' */
  real_T Integrator_CSTATE_kx;         /* '<S161>/Integrator' */
  real_T Filter_CSTATE_c;              /* '<S537>/Filter' */
  real_T Integrator_CSTATE_g;          /* '<S542>/Integrator' */
  real_T Filter_CSTATE_fe;             /* '<S589>/Filter' */
  real_T Integrator_CSTATE_am;         /* '<S594>/Integrator' */
} X_MicroMouse_Deploy_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S3>/Integrator' */
  real_T Integrator_CSTATE_o;          /* '<S274>/Integrator' */
  real_T Filter_CSTATE;                /* '<S269>/Filter' */
  real_T Integrator_CSTATE_k;          /* '<S482>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S477>/Filter' */
  real_T Integrator_CSTATE_a;          /* '<S430>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S425>/Filter' */
  real_T Integrator_CSTATE_l;          /* '<S326>/Integrator' */
  real_T Filter_CSTATE_e;              /* '<S321>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S378>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S373>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S3>/Integrator1' */
  real_T Filter_CSTATE_k;              /* '<S104>/Filter' */
  real_T Integrator_CSTATE_e;          /* '<S109>/Integrator' */
  real_T Integrator_CSTATE_j;          /* '<S18>/Integrator' */
  real_T Filter_CSTATE_lq;             /* '<S211>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S12>/Integrator' */
  real_T Integrator1_CSTATE_g;         /* '<S12>/Integrator1' */
  real_T Filter_CSTATE_n;              /* '<S156>/Filter' */
  real_T Integrator_CSTATE_kx;         /* '<S161>/Integrator' */
  real_T Filter_CSTATE_c;              /* '<S537>/Filter' */
  real_T Integrator_CSTATE_g;          /* '<S542>/Integrator' */
  real_T Filter_CSTATE_fe;             /* '<S589>/Filter' */
  real_T Integrator_CSTATE_am;         /* '<S594>/Integrator' */
} XDot_MicroMouse_Deploy_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S3>/Integrator' */
  boolean_T Integrator_CSTATE_o;       /* '<S274>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S269>/Filter' */
  boolean_T Integrator_CSTATE_k;       /* '<S482>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S477>/Filter' */
  boolean_T Integrator_CSTATE_a;       /* '<S430>/Integrator' */
  boolean_T Filter_CSTATE_m;           /* '<S425>/Filter' */
  boolean_T Integrator_CSTATE_l;       /* '<S326>/Integrator' */
  boolean_T Filter_CSTATE_e;           /* '<S321>/Filter' */
  boolean_T Integrator_CSTATE_p;       /* '<S378>/Integrator' */
  boolean_T Filter_CSTATE_l;           /* '<S373>/Filter' */
  boolean_T Integrator1_CSTATE;        /* '<S3>/Integrator1' */
  boolean_T Filter_CSTATE_k;           /* '<S104>/Filter' */
  boolean_T Integrator_CSTATE_e;       /* '<S109>/Integrator' */
  boolean_T Integrator_CSTATE_j;       /* '<S18>/Integrator' */
  boolean_T Filter_CSTATE_lq;          /* '<S211>/Filter' */
  boolean_T Integrator_CSTATE_f;       /* '<S12>/Integrator' */
  boolean_T Integrator1_CSTATE_g;      /* '<S12>/Integrator1' */
  boolean_T Filter_CSTATE_n;           /* '<S156>/Filter' */
  boolean_T Integrator_CSTATE_kx;      /* '<S161>/Integrator' */
  boolean_T Filter_CSTATE_c;           /* '<S537>/Filter' */
  boolean_T Integrator_CSTATE_g;       /* '<S542>/Integrator' */
  boolean_T Filter_CSTATE_fe;          /* '<S589>/Filter' */
  boolean_T Integrator_CSTATE_am;      /* '<S594>/Integrator' */
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
  real_T PIDController1_D;             /* Mask Parameter: PIDController1_D
                                        * Referenced by: '<S267>/Derivative Gain'
                                        */
  real_T PIDController5_D;             /* Mask Parameter: PIDController5_D
                                        * Referenced by: '<S475>/Derivative Gain'
                                        */
  real_T PIDController4_D;             /* Mask Parameter: PIDController4_D
                                        * Referenced by: '<S423>/Derivative Gain'
                                        */
  real_T PIDController2_D;             /* Mask Parameter: PIDController2_D
                                        * Referenced by: '<S319>/Derivative Gain'
                                        */
  real_T PIDController3_D;             /* Mask Parameter: PIDController3_D
                                        * Referenced by: '<S371>/Derivative Gain'
                                        */
  real_T PIDController_D;              /* Mask Parameter: PIDController_D
                                        * Referenced by: '<S102>/Derivative Gain'
                                        */
  real_T PIDController_D_e;            /* Mask Parameter: PIDController_D_e
                                        * Referenced by: '<S209>/Derivative Gain'
                                        */
  real_T PIDController4_D_i;           /* Mask Parameter: PIDController4_D_i
                                        * Referenced by: '<S154>/Derivative Gain'
                                        */
  real_T PIDController1_D_p;           /* Mask Parameter: PIDController1_D_p
                                        * Referenced by: '<S535>/Derivative Gain'
                                        */
  real_T PIDController2_D_g;           /* Mask Parameter: PIDController2_D_g
                                        * Referenced by: '<S587>/Derivative Gain'
                                        */
  real_T PIDController_I;              /* Mask Parameter: PIDController_I
                                        * Referenced by: '<S106>/Integral Gain'
                                        */
  real_T PIDController1_I;             /* Mask Parameter: PIDController1_I
                                        * Referenced by: '<S271>/Integral Gain'
                                        */
  real_T PIDController2_I;             /* Mask Parameter: PIDController2_I
                                        * Referenced by: '<S323>/Integral Gain'
                                        */
  real_T PIDController3_I;             /* Mask Parameter: PIDController3_I
                                        * Referenced by: '<S375>/Integral Gain'
                                        */
  real_T PIDController4_I;             /* Mask Parameter: PIDController4_I
                                        * Referenced by: '<S427>/Integral Gain'
                                        */
  real_T PIDController5_I;             /* Mask Parameter: PIDController5_I
                                        * Referenced by: '<S479>/Integral Gain'
                                        */
  real_T PIDController4_I_j;           /* Mask Parameter: PIDController4_I_j
                                        * Referenced by: '<S158>/Integral Gain'
                                        */
  real_T PIDController1_I_g;           /* Mask Parameter: PIDController1_I_g
                                        * Referenced by: '<S539>/Integral Gain'
                                        */
  real_T PIDController2_I_b;           /* Mask Parameter: PIDController2_I_b
                                        * Referenced by: '<S591>/Integral Gain'
                                        */
  real_T PIDController1_InitialCondition;
                              /* Mask Parameter: PIDController1_InitialCondition
                               * Referenced by: '<S269>/Filter'
                               */
  real_T PIDController5_InitialCondition;
                              /* Mask Parameter: PIDController5_InitialCondition
                               * Referenced by: '<S477>/Filter'
                               */
  real_T PIDController4_InitialCondition;
                              /* Mask Parameter: PIDController4_InitialCondition
                               * Referenced by: '<S425>/Filter'
                               */
  real_T PIDController2_InitialCondition;
                              /* Mask Parameter: PIDController2_InitialCondition
                               * Referenced by: '<S321>/Filter'
                               */
  real_T PIDController3_InitialCondition;
                              /* Mask Parameter: PIDController3_InitialCondition
                               * Referenced by: '<S373>/Filter'
                               */
  real_T PIDController_InitialConditionF;
                              /* Mask Parameter: PIDController_InitialConditionF
                               * Referenced by: '<S104>/Filter'
                               */
  real_T PIDController_InitialConditio_a;
                              /* Mask Parameter: PIDController_InitialConditio_a
                               * Referenced by: '<S211>/Filter'
                               */
  real_T PIDController4_InitialConditi_i;
                              /* Mask Parameter: PIDController4_InitialConditi_i
                               * Referenced by: '<S156>/Filter'
                               */
  real_T PIDController1_InitialConditi_h;
                              /* Mask Parameter: PIDController1_InitialConditi_h
                               * Referenced by: '<S537>/Filter'
                               */
  real_T PIDController2_InitialConditi_e;
                              /* Mask Parameter: PIDController2_InitialConditi_e
                               * Referenced by: '<S589>/Filter'
                               */
  real_T PIDController1_InitialConditi_i;
                              /* Mask Parameter: PIDController1_InitialConditi_i
                               * Referenced by: '<S274>/Integrator'
                               */
  real_T PIDController5_InitialConditi_i;
                              /* Mask Parameter: PIDController5_InitialConditi_i
                               * Referenced by: '<S482>/Integrator'
                               */
  real_T PIDController4_InitialConditi_d;
                              /* Mask Parameter: PIDController4_InitialConditi_d
                               * Referenced by: '<S430>/Integrator'
                               */
  real_T PIDController2_InitialConditi_m;
                              /* Mask Parameter: PIDController2_InitialConditi_m
                               * Referenced by: '<S326>/Integrator'
                               */
  real_T PIDController3_InitialConditi_n;
                              /* Mask Parameter: PIDController3_InitialConditi_n
                               * Referenced by: '<S378>/Integrator'
                               */
  real_T PIDController_InitialConditio_l;
                              /* Mask Parameter: PIDController_InitialConditio_l
                               * Referenced by: '<S109>/Integrator'
                               */
  real_T PIDController4_InitialConditi_e;
                              /* Mask Parameter: PIDController4_InitialConditi_e
                               * Referenced by: '<S161>/Integrator'
                               */
  real_T PIDController1_InitialConditi_c;
                              /* Mask Parameter: PIDController1_InitialConditi_c
                               * Referenced by: '<S542>/Integrator'
                               */
  real_T PIDController2_InitialConditi_f;
                              /* Mask Parameter: PIDController2_InitialConditi_f
                               * Referenced by: '<S594>/Integrator'
                               */
  real_T PIDController1_N;             /* Mask Parameter: PIDController1_N
                                        * Referenced by: '<S277>/Filter Coefficient'
                                        */
  real_T PIDController5_N;             /* Mask Parameter: PIDController5_N
                                        * Referenced by: '<S485>/Filter Coefficient'
                                        */
  real_T PIDController4_N;             /* Mask Parameter: PIDController4_N
                                        * Referenced by: '<S433>/Filter Coefficient'
                                        */
  real_T PIDController2_N;             /* Mask Parameter: PIDController2_N
                                        * Referenced by: '<S329>/Filter Coefficient'
                                        */
  real_T PIDController3_N;             /* Mask Parameter: PIDController3_N
                                        * Referenced by: '<S381>/Filter Coefficient'
                                        */
  real_T PIDController_N;              /* Mask Parameter: PIDController_N
                                        * Referenced by: '<S112>/Filter Coefficient'
                                        */
  real_T PIDController_N_j;            /* Mask Parameter: PIDController_N_j
                                        * Referenced by: '<S219>/Filter Coefficient'
                                        */
  real_T PIDController4_N_k;           /* Mask Parameter: PIDController4_N_k
                                        * Referenced by: '<S164>/Filter Coefficient'
                                        */
  real_T PIDController1_N_m;           /* Mask Parameter: PIDController1_N_m
                                        * Referenced by: '<S545>/Filter Coefficient'
                                        */
  real_T PIDController2_N_n;           /* Mask Parameter: PIDController2_N_n
                                        * Referenced by: '<S597>/Filter Coefficient'
                                        */
  real_T PIDController1_P;             /* Mask Parameter: PIDController1_P
                                        * Referenced by: '<S279>/Proportional Gain'
                                        */
  real_T PIDController5_P;             /* Mask Parameter: PIDController5_P
                                        * Referenced by: '<S487>/Proportional Gain'
                                        */
  real_T PIDController4_P;             /* Mask Parameter: PIDController4_P
                                        * Referenced by: '<S435>/Proportional Gain'
                                        */
  real_T PIDController2_P;             /* Mask Parameter: PIDController2_P
                                        * Referenced by: '<S331>/Proportional Gain'
                                        */
  real_T PIDController3_P;             /* Mask Parameter: PIDController3_P
                                        * Referenced by: '<S383>/Proportional Gain'
                                        */
  real_T PIDController_P;              /* Mask Parameter: PIDController_P
                                        * Referenced by: '<S114>/Proportional Gain'
                                        */
  real_T PIDController_P_c;            /* Mask Parameter: PIDController_P_c
                                        * Referenced by: '<S221>/Proportional Gain'
                                        */
  real_T LowPassFilterDiscreteorContinuo;
                              /* Mask Parameter: LowPassFilterDiscreteorContinuo
                               * Referenced by: '<S54>/Time constant'
                               */
  real_T LowPassFilterDiscreteorContin_e;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_e
                               * Referenced by: '<S68>/Time constant'
                               */
  real_T LowPassFilterDiscreteorContin_p;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_p
                               * Referenced by: '<S61>/Time constant'
                               */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S57>/Constant'
                                       */
  real_T CompareToConstant_const_o; /* Mask Parameter: CompareToConstant_const_o
                                     * Referenced by: '<S71>/Constant'
                                     */
  real_T CompareToConstant_const_j; /* Mask Parameter: CompareToConstant_const_j
                                     * Referenced by: '<S64>/Constant'
                                     */
  real_T LowPassFilterDiscreteorContin_k;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_k
                               * Referenced by: '<S54>/Constant'
                               */
  real_T LowPassFilterDiscreteorContin_c;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_c
                               * Referenced by: '<S68>/Constant'
                               */
  real_T LowPassFilterDiscreteorConti_ct;
                              /* Mask Parameter: LowPassFilterDiscreteorConti_ct
                               * Referenced by: '<S61>/Constant'
                               */
  real32_T LowPassFilterDiscreteorConti_pu;
                              /* Mask Parameter: LowPassFilterDiscreteorConti_pu
                               * Referenced by: '<S13>/K'
                               */
  real32_T LowPassFilterDiscreteorConti_cj;
                              /* Mask Parameter: LowPassFilterDiscreteorConti_cj
                               * Referenced by: '<S15>/K'
                               */
  real32_T LowPassFilterDiscreteorContin_a;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_a
                               * Referenced by: '<S14>/K'
                               */
  real32_T CompareToConstant_const_jx;
                                   /* Mask Parameter: CompareToConstant_const_jx
                                    * Referenced by: '<S34>/Constant'
                                    */
  real32_T CompareToConstant_const_c;
                                    /* Mask Parameter: CompareToConstant_const_c
                                     * Referenced by: '<S30>/Constant'
                                     */
  real32_T CompareToConstant_const_g;
                                    /* Mask Parameter: CompareToConstant_const_g
                                     * Referenced by: '<S45>/Constant'
                                     */
  real32_T CompareToConstant_const_l;
                                    /* Mask Parameter: CompareToConstant_const_l
                                     * Referenced by: '<S49>/Constant'
                                     */
  boolean_T DetectFallNonpositive_vinit;
                                  /* Mask Parameter: DetectFallNonpositive_vinit
                                   * Referenced by: '<S35>/Delay Input1'
                                   */
  boolean_T DetectFallNonpositive_vinit_a;
                                /* Mask Parameter: DetectFallNonpositive_vinit_a
                                 * Referenced by: '<S50>/Delay Input1'
                                 */
  uint8_T DetectRisePositive_vinit;  /* Mask Parameter: DetectRisePositive_vinit
                                      * Referenced by: '<S36>/Delay Input1'
                                      */
  uint8_T DetectRisePositive_vinit_a;
                                   /* Mask Parameter: DetectRisePositive_vinit_a
                                    * Referenced by: '<S31>/Delay Input1'
                                    */
  uint8_T DetectRisePositive_vinit_h;
                                   /* Mask Parameter: DetectRisePositive_vinit_h
                                    * Referenced by: '<S46>/Delay Input1'
                                    */
  uint8_T DetectRisePositive_vinit_f;
                                   /* Mask Parameter: DetectRisePositive_vinit_f
                                    * Referenced by: '<S51>/Delay Input1'
                                    */
  real_T Constant1_Value;              /* Expression: 0.2
                                        * Referenced by: '<S3>/Constant1'
                                        */
  real_T RateLimiter2_RisingLim;       /* Expression: 4.85
                                        * Referenced by: '<S11>/Rate Limiter2'
                                        */
  real_T RateLimiter2_FallingLim;      /* Expression: -1
                                        * Referenced by: '<S11>/Rate Limiter2'
                                        */
  real_T Tick_per_rev_Gain;            /* Expression: 1/8
                                        * Referenced by: '<S29>/Tick_per_rev'
                                        */
  real_T Circumference_Value;          /* Expression: (2*pi*0.031)
                                        * Referenced by: '<S29>/Circumference'
                                        */
  real_T RateLimiter4_RisingLim;       /* Expression: 4.85
                                        * Referenced by: '<S11>/Rate Limiter4'
                                        */
  real_T RateLimiter4_FallingLim;      /* Expression: -1
                                        * Referenced by: '<S11>/Rate Limiter4'
                                        */
  real_T Tick_per_rev_Gain_l;          /* Expression: 1/8
                                        * Referenced by: '<S28>/Tick_per_rev'
                                        */
  real_T Circumference_Value_b;        /* Expression: (2*pi*0.031)
                                        * Referenced by: '<S28>/Circumference'
                                        */
  real_T Gain1_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S11>/Gain1'
                                        */
  real_T Constant2_Value;              /* Expression: 0.05
                                        * Referenced by: '<S3>/Constant2'
                                        */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S3>/Integrator'
                                        */
  real_T constant_Value;               /* Expression: 45
                                        * Referenced by: '<S22>/constant'
                                        */
  real_T Gain5_Gain;                   /* Expression: 10
                                        * Referenced by: '<S22>/Gain5'
                                        */
  real_T Gain6_Gain;                   /* Expression: 10
                                        * Referenced by: '<S22>/Gain6'
                                        */
  real_T Constant1_Value_a;            /* Expression: 0.07
                                        * Referenced by: '<S22>/Constant1'
                                        */
  real_T Constant_Value;               /* Expression: 0.07
                                        * Referenced by: '<S22>/Constant'
                                        */
  real_T Constant4_Value;              /* Expression: 0.11
                                        * Referenced by: '<S22>/Constant4'
                                        */
  real_T Constant2_Value_a;            /* Expression: 0.07
                                        * Referenced by: '<S22>/Constant2'
                                        */
  real_T Constant3_Value;              /* Expression: 0.07
                                        * Referenced by: '<S22>/Constant3'
                                        */
  real_T Constant5_Value;              /* Expression: 0.11
                                        * Referenced by: '<S22>/Constant5'
                                        */
  real_T Gain1_Gain_d;                 /* Expression: 1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Constant_Value_l;             /* Expression: 11
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Constant3_Value_g;            /* Expression: 1
                                        * Referenced by: '<S3>/Constant3'
                                        */
  real_T Constant4_Value_d;            /* Expression: 0
                                        * Referenced by: '<S3>/Constant4'
                                        */
  real_T Constant5_Value_n;            /* Expression: 5
                                        * Referenced by: '<S3>/Constant5'
                                        */
  real_T Integrator1_IC;               /* Expression: 0
                                        * Referenced by: '<S3>/Integrator1'
                                        */
  real_T Motor_Left2_Value;            /* Expression: 1
                                        * Referenced by: '<S3>/Motor_Left2'
                                        */
  real_T Integrator_IC_b;              /* Expression: 0
                                        * Referenced by: '<S18>/Integrator'
                                        */
  real_T UnitDelay1_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<S501>/Unit Delay1'
                                        */
  real_T tau_Gain;                     /* Expression: tau
                                        * Referenced by: '<S501>/tau'
                                        */
  real_T Bias_Bias;                    /* Expression: 1
                                        * Referenced by: '<S503>/Bias'
                                        */
  real_T Gain_Gain_n;                  /* Expression: 2*zeta
                                        * Referenced by: '<S503>/Gain'
                                        */
  real_T UnitDelay1_InitialCondition_k;/* Expression: 0
                                        * Referenced by: '<S502>/Unit Delay1'
                                        */
  real_T tau_Gain_b;                   /* Expression: tau
                                        * Referenced by: '<S502>/tau'
                                        */
  real_T Bias_Bias_b;                  /* Expression: 1
                                        * Referenced by: '<S504>/Bias'
                                        */
  real_T Gain_Gain_l;                  /* Expression: 2*zeta
                                        * Referenced by: '<S504>/Gain'
                                        */
  real_T DeadZone_Start;               /* Expression: -0.5
                                        * Referenced by: '<S23>/Dead Zone'
                                        */
  real_T DeadZone_End;                 /* Expression: 0.5
                                        * Referenced by: '<S23>/Dead Zone'
                                        */
  real_T Bias1_Bias;                   /* Expression: 1
                                        * Referenced by: '<S503>/Bias1'
                                        */
  real_T Gain1_Gain_b;                 /* Expression: 2
                                        * Referenced by: '<S501>/Gain1'
                                        */
  real_T UnitDelay2_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<S501>/Unit Delay2'
                                        */
  real_T Bias1_Bias_f;                 /* Expression: 1
                                        * Referenced by: '<S504>/Bias1'
                                        */
  real_T Gain1_Gain_h;                 /* Expression: 2
                                        * Referenced by: '<S502>/Gain1'
                                        */
  real_T UnitDelay2_InitialCondition_e;/* Expression: 0
                                        * Referenced by: '<S502>/Unit Delay2'
                                        */
  real_T Integrator_IC_p;              /* Expression: 0
                                        * Referenced by: '<S12>/Integrator'
                                        */
  real_T Delay_InitialCondition;       /* Expression: 0
                                        * Referenced by: '<S39>/Delay'
                                        */
  real_T RateLimiter1_RisingLim;       /* Expression: 4.85
                                        * Referenced by: '<S12>/Rate Limiter1'
                                        */
  real_T RateLimiter1_FallingLim;      /* Expression: -1
                                        * Referenced by: '<S12>/Rate Limiter1'
                                        */
  real_T Integrator1_IC_o;             /* Expression: 0
                                        * Referenced by: '<S12>/Integrator1'
                                        */
  real_T Delay_InitialCondition_d;     /* Expression: 0
                                        * Referenced by: '<S40>/Delay'
                                        */
  real_T RateLimiter2_RisingLim_h;     /* Expression: 4.85
                                        * Referenced by: '<S12>/Rate Limiter2'
                                        */
  real_T RateLimiter2_FallingLim_n;    /* Expression: -1
                                        * Referenced by: '<S12>/Rate Limiter2'
                                        */
  real_T Constant_Value_g;             /* Expression: 0.0765
                                        * Referenced by: '<S24>/Constant'
                                        */
  char_T OLED_STRING1_String[256];     /* Expression: "UCT MICROMOUSE '25"
                                        * Referenced by: '<S3>/OLED_STRING1'
                                        */
  char_T OLED_STRING2_String[256];     /* Expression: "Simulink Data Demo"
                                        * Referenced by: '<S3>/OLED_STRING2'
                                        */
  real32_T Constant_Value_lf;          /* Computed Parameter: Constant_Value_lf
                                        * Referenced by: '<S58>/Constant'
                                        */
  real32_T Constant_Value_i;           /* Computed Parameter: Constant_Value_i
                                        * Referenced by: '<S65>/Constant'
                                        */
  real32_T Constant_Value_c;           /* Computed Parameter: Constant_Value_c
                                        * Referenced by: '<S72>/Constant'
                                        */
  real32_T Gain_Gain_g;                /* Computed Parameter: Gain_Gain_g
                                        * Referenced by: '<S10>/Gain'
                                        */
  real32_T Integrator_gainval;         /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S60>/Integrator'
                                        */
  real32_T Integrator_UpperSat;       /* Computed Parameter: Integrator_UpperSat
                                       * Referenced by: '<S60>/Integrator'
                                       */
  real32_T Integrator_LowerSat;       /* Computed Parameter: Integrator_LowerSat
                                       * Referenced by: '<S60>/Integrator'
                                       */
  real32_T Saturation_UpperSat;       /* Computed Parameter: Saturation_UpperSat
                                       * Referenced by: '<S60>/Saturation'
                                       */
  real32_T Saturation_LowerSat;       /* Computed Parameter: Saturation_LowerSat
                                       * Referenced by: '<S60>/Saturation'
                                       */
  real32_T Integrator_gainval_k;     /* Computed Parameter: Integrator_gainval_k
                                      * Referenced by: '<S74>/Integrator'
                                      */
  real32_T Integrator_UpperSat_l;   /* Computed Parameter: Integrator_UpperSat_l
                                     * Referenced by: '<S74>/Integrator'
                                     */
  real32_T Integrator_LowerSat_g;   /* Computed Parameter: Integrator_LowerSat_g
                                     * Referenced by: '<S74>/Integrator'
                                     */
  real32_T Saturation_UpperSat_c;   /* Computed Parameter: Saturation_UpperSat_c
                                     * Referenced by: '<S74>/Saturation'
                                     */
  real32_T Saturation_LowerSat_o;   /* Computed Parameter: Saturation_LowerSat_o
                                     * Referenced by: '<S74>/Saturation'
                                     */
  real32_T Gain2_Gain;                 /* Computed Parameter: Gain2_Gain
                                        * Referenced by: '<S4>/Gain2'
                                        */
  real32_T Integrator_gainval_n;     /* Computed Parameter: Integrator_gainval_n
                                      * Referenced by: '<S67>/Integrator'
                                      */
  real32_T Integrator_UpperSat_k;   /* Computed Parameter: Integrator_UpperSat_k
                                     * Referenced by: '<S67>/Integrator'
                                     */
  real32_T Integrator_LowerSat_d;   /* Computed Parameter: Integrator_LowerSat_d
                                     * Referenced by: '<S67>/Integrator'
                                     */
  real32_T Saturation_UpperSat_f;   /* Computed Parameter: Saturation_UpperSat_f
                                     * Referenced by: '<S67>/Saturation'
                                     */
  real32_T Saturation_LowerSat_b;   /* Computed Parameter: Saturation_LowerSat_b
                                     * Referenced by: '<S67>/Saturation'
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
  uint16_T Delay_InitialCondition_j;
                                 /* Computed Parameter: Delay_InitialCondition_j
                                  * Referenced by: '<S27>/Delay'
                                  */
  uint16_T Delay_InitialCondition_c;
                                 /* Computed Parameter: Delay_InitialCondition_c
                                  * Referenced by: '<S26>/Delay'
                                  */
  uint16_T Gain1_Gain_k;               /* Computed Parameter: Gain1_Gain_k
                                        * Referenced by: '<S27>/Gain1'
                                        */
  uint16_T Gain1_Gain_m;               /* Computed Parameter: Gain1_Gain_m
                                        * Referenced by: '<S26>/Gain1'
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
  boolean_T Constant_Value_m;          /* Computed Parameter: Constant_Value_m
                                        * Referenced by: '<S32>/Constant'
                                        */
  boolean_T Constant_Value_o;          /* Computed Parameter: Constant_Value_o
                                        * Referenced by: '<S37>/Constant'
                                        */
  boolean_T Constant_Value_j;          /* Computed Parameter: Constant_Value_j
                                        * Referenced by: '<S38>/Constant'
                                        */
  boolean_T Constant_Value_f;          /* Computed Parameter: Constant_Value_f
                                        * Referenced by: '<S47>/Constant'
                                        */
  boolean_T Constant_Value_lk;         /* Computed Parameter: Constant_Value_lk
                                        * Referenced by: '<S52>/Constant'
                                        */
  boolean_T Constant_Value_fp;         /* Computed Parameter: Constant_Value_fp
                                        * Referenced by: '<S53>/Constant'
                                        */
  int8_T DataStoreMemory_InitialValue_oa;
                          /* Computed Parameter: DataStoreMemory_InitialValue_oa
                           * Referenced by: '<S1>/Data Store Memory'
                           */
  int8_T DataStoreMemory1_InitialValu_p2;
                          /* Computed Parameter: DataStoreMemory1_InitialValu_p2
                           * Referenced by: '<S1>/Data Store Memory1'
                           */
  uint8_T Gain_Gain_f;                 /* Computed Parameter: Gain_Gain_f
                                        * Referenced by: '<S27>/Gain'
                                        */
  uint8_T Gain_Gain_nt;                /* Computed Parameter: Gain_Gain_nt
                                        * Referenced by: '<S26>/Gain'
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
  real_T odeY[24];
  real_T odeF[3][24];
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
 * Block '<S3>/Add' : Unused code path elimination
 * Block '<S3>/Add1' : Unused code path elimination
 * Block '<S11>/Average_dist_travelled1' : Unused code path elimination
 * Block '<S11>/LW_dist_travelled1' : Unused code path elimination
 * Block '<S11>/LW_enc_signal1' : Unused code path elimination
 * Block '<S11>/LW_encoder_ticks3' : Unused code path elimination
 * Block '<S11>/RW_dist_travelled1' : Unused code path elimination
 * Block '<S11>/RW_enc_signal1' : Unused code path elimination
 * Block '<S11>/RW_encoder_ticks2' : Unused code path elimination
 * Block '<S26>/Scope' : Unused code path elimination
 * Block '<S27>/Scope' : Unused code path elimination
 * Block '<S12>/Average_dist_travelled1' : Unused code path elimination
 * Block '<S12>/Gain1' : Unused code path elimination
 * Block '<S12>/LW_dist_travelled1' : Unused code path elimination
 * Block '<S12>/LW_dist_travelled2' : Unused code path elimination
 * Block '<S12>/LW_dist_travelled3' : Unused code path elimination
 * Block '<S12>/LW_enc_signal1' : Unused code path elimination
 * Block '<S12>/LW_encoder_ticks1' : Unused code path elimination
 * Block '<S12>/LW_encoder_ticks2' : Unused code path elimination
 * Block '<S12>/LW_encoder_ticks3' : Unused code path elimination
 * Block '<S12>/RW_dist_travelled1' : Unused code path elimination
 * Block '<S12>/RW_enc_signal1' : Unused code path elimination
 * Block '<S12>/RW_encoder_ticks2' : Unused code path elimination
 * Block '<S12>/Rate Limiter' : Unused code path elimination
 * Block '<S12>/Rate Limiter3' : Unused code path elimination
 * Block '<S12>/Scope' : Unused code path elimination
 * Block '<S12>/Scope1' : Unused code path elimination
 * Block '<S12>/Scope2' : Unused code path elimination
 * Block '<S12>/Scope5' : Unused code path elimination
 * Block '<S12>/Sum3' : Unused code path elimination
 * Block '<S39>/Scope' : Unused code path elimination
 * Block '<S40>/Scope' : Unused code path elimination
 * Block '<S41>/Circumference' : Unused code path elimination
 * Block '<S41>/Product' : Unused code path elimination
 * Block '<S41>/Tick_per_rev' : Unused code path elimination
 * Block '<S42>/Circumference' : Unused code path elimination
 * Block '<S42>/Product' : Unused code path elimination
 * Block '<S42>/Tick_per_rev' : Unused code path elimination
 * Block '<S43>/Circumference' : Unused code path elimination
 * Block '<S43>/Product' : Unused code path elimination
 * Block '<S43>/Tick_per_rev' : Unused code path elimination
 * Block '<S44>/Circumference' : Unused code path elimination
 * Block '<S44>/Product' : Unused code path elimination
 * Block '<S44>/Tick_per_rev' : Unused code path elimination
 * Block '<S3>/Display' : Unused code path elimination
 * Block '<S3>/Display1' : Unused code path elimination
 * Block '<S3>/Display10' : Unused code path elimination
 * Block '<S3>/Display11' : Unused code path elimination
 * Block '<S3>/Display12' : Unused code path elimination
 * Block '<S3>/Display2' : Unused code path elimination
 * Block '<S3>/Display3' : Unused code path elimination
 * Block '<S3>/Display4' : Unused code path elimination
 * Block '<S3>/Display5' : Unused code path elimination
 * Block '<S3>/Display6' : Unused code path elimination
 * Block '<S3>/Display7' : Unused code path elimination
 * Block '<S3>/Display9' : Unused code path elimination
 * Block '<S3>/Gain' : Unused code path elimination
 * Block '<S3>/Gain1' : Unused code path elimination
 * Block '<S3>/Gain3' : Unused code path elimination
 * Block '<S3>/Gain4' : Unused code path elimination
 * Block '<S3>/Motor_Left1' : Unused code path elimination
 * Block '<S3>/Motor_Left3' : Unused code path elimination
 * Block '<S3>/Motor_Right1' : Unused code path elimination
 * Block '<S3>/NOT1' : Unused code path elimination
 * Block '<S166>/Proportional Gain' : Unused code path elimination
 * Block '<S170>/Sum' : Unused code path elimination
 * Block '<S3>/Scope' : Unused code path elimination
 * Block '<S3>/Scope1' : Unused code path elimination
 * Block '<S3>/Scope2' : Unused code path elimination
 * Block '<S3>/Scope3' : Unused code path elimination
 * Block '<S3>/Scope4' : Unused code path elimination
 * Block '<S3>/Scope5' : Unused code path elimination
 * Block '<S3>/TOF_F' : Unused code path elimination
 * Block '<S3>/TOF_L' : Unused code path elimination
 * Block '<S3>/TOF_R' : Unused code path elimination
 * Block '<S3>/Z_theta' : Unused code path elimination
 * Block '<S3>/Z_w' : Unused code path elimination
 * Block '<S3>/Z_w1' : Unused code path elimination
 * Block '<S3>/Z_w2' : Unused code path elimination
 * Block '<S18>/Gain' : Unused code path elimination
 * Block '<S18>/Gain1' : Unused code path elimination
 * Block '<S18>/Scope' : Unused code path elimination
 * Block '<S18>/Scope1' : Unused code path elimination
 * Block '<S18>/Scope2' : Unused code path elimination
 * Block '<S22>/Display1' : Unused code path elimination
 * Block '<S22>/Display2' : Unused code path elimination
 * Block '<S22>/Display3' : Unused code path elimination
 * Block '<S22>/Display4' : Unused code path elimination
 * Block '<S22>/Display5' : Unused code path elimination
 * Block '<S22>/Display6' : Unused code path elimination
 * Block '<S22>/Display8' : Unused code path elimination
 * Block '<S22>/Scope6' : Unused code path elimination
 * Block '<S24>/Display9' : Unused code path elimination
 * Block '<S24>/Gain' : Unused code path elimination
 * Block '<S547>/Proportional Gain' : Unused code path elimination
 * Block '<S551>/Sum' : Unused code path elimination
 * Block '<S599>/Proportional Gain' : Unused code path elimination
 * Block '<S603>/Sum' : Unused code path elimination
 * Block '<S24>/Scope' : Unused code path elimination
 * Block '<S24>/Scope1' : Unused code path elimination
 * Block '<S24>/Scope6' : Unused code path elimination
 * Block '<S24>/Subtract' : Unused code path elimination
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
 * Block '<S22>/Cast To Double3' : Eliminate redundant data type conversion
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
 * '<S11>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement '
 * '<S12>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1'
 * '<S13>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)'
 * '<S14>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1'
 * '<S15>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2'
 * '<S16>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller'
 * '<S17>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller4'
 * '<S18>'  : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2'
 * '<S19>'  : 'MicroMouse_Deploy/StudentTemplate/draft2_Maze_exploration'
 * '<S20>'  : 'MicroMouse_Deploy/StudentTemplate/draft2_Maze_exploration1'
 * '<S21>'  : 'MicroMouse_Deploy/StudentTemplate/draft2_Maze_exploration2'
 * '<S22>'  : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening'
 * '<S23>'  : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing '
 * '<S24>'  : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1'
 * '<S25>'  : 'MicroMouse_Deploy/StudentTemplate/turn_adjus'
 * '<S26>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks2'
 * '<S27>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks3'
 * '<S28>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /encoder_tick_to_dist2'
 * '<S29>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /encoder_tick_to_dist3'
 * '<S30>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks2/Compare To Constant'
 * '<S31>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks2/Detect Rise Positive'
 * '<S32>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks2/Detect Rise Positive/Positive'
 * '<S33>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks3/Chart'
 * '<S34>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks3/Compare To Constant'
 * '<S35>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks3/Detect Fall Nonpositive'
 * '<S36>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks3/Detect Rise Positive'
 * '<S37>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks3/Detect Fall Nonpositive/Nonpositive'
 * '<S38>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement /convert_encoder_signal_to_ticks3/Detect Rise Positive/Positive'
 * '<S39>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks2'
 * '<S40>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks3'
 * '<S41>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/encoder_tick_to_dist1'
 * '<S42>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/encoder_tick_to_dist2'
 * '<S43>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/encoder_tick_to_dist3'
 * '<S44>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/encoder_tick_to_dist4'
 * '<S45>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks2/Compare To Constant'
 * '<S46>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks2/Detect Rise Positive'
 * '<S47>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks2/Detect Rise Positive/Positive'
 * '<S48>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks3/Chart'
 * '<S49>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks3/Compare To Constant'
 * '<S50>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks3/Detect Fall Nonpositive'
 * '<S51>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks3/Detect Rise Positive'
 * '<S52>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks3/Detect Fall Nonpositive/Nonpositive'
 * '<S53>'  : 'MicroMouse_Deploy/StudentTemplate/DIstance measurement 1/convert_encoder_signal_to_ticks3/Detect Rise Positive/Positive'
 * '<S54>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant'
 * '<S55>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Initialization'
 * '<S56>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)'
 * '<S57>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Constant'
 * '<S58>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Zero'
 * '<S59>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Initialization/Init_u'
 * '<S60>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)/Discrete'
 * '<S61>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant'
 * '<S62>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Initialization'
 * '<S63>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)'
 * '<S64>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Constant'
 * '<S65>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Zero'
 * '<S66>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Initialization/Init_u'
 * '<S67>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)/Discrete'
 * '<S68>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant'
 * '<S69>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Initialization'
 * '<S70>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Integrator (Discrete or Continuous)'
 * '<S71>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant/Compare To Constant'
 * '<S72>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Enable//disable time constant/Compare To Zero'
 * '<S73>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Initialization/Init_u'
 * '<S74>'  : 'MicroMouse_Deploy/StudentTemplate/Low-Pass Filter (Discrete or Continuous)2/Integrator (Discrete or Continuous)/Discrete'
 * '<S75>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Anti-windup'
 * '<S76>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/D Gain'
 * '<S77>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/External Derivative'
 * '<S78>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Filter'
 * '<S79>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Filter ICs'
 * '<S80>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/I Gain'
 * '<S81>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Ideal P Gain'
 * '<S82>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Ideal P Gain Fdbk'
 * '<S83>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Integrator'
 * '<S84>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Integrator ICs'
 * '<S85>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/N Copy'
 * '<S86>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/N Gain'
 * '<S87>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/P Copy'
 * '<S88>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Parallel P Gain'
 * '<S89>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Reset Signal'
 * '<S90>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Saturation'
 * '<S91>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Saturation Fdbk'
 * '<S92>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Sum'
 * '<S93>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Sum Fdbk'
 * '<S94>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Tracking Mode'
 * '<S95>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Tracking Mode Sum'
 * '<S96>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Tsamp - Integral'
 * '<S97>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Tsamp - Ngain'
 * '<S98>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/postSat Signal'
 * '<S99>'  : 'MicroMouse_Deploy/StudentTemplate/PID Controller/preInt Signal'
 * '<S100>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/preSat Signal'
 * '<S101>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Anti-windup/Passthrough'
 * '<S102>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/D Gain/Internal Parameters'
 * '<S103>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/External Derivative/Error'
 * '<S104>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Filter/Cont. Filter'
 * '<S105>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S106>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/I Gain/Internal Parameters'
 * '<S107>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Ideal P Gain/Passthrough'
 * '<S108>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S109>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Integrator/Continuous'
 * '<S110>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Integrator ICs/Internal IC'
 * '<S111>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/N Copy/Disabled'
 * '<S112>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/N Gain/Internal Parameters'
 * '<S113>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/P Copy/Disabled'
 * '<S114>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S115>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Reset Signal/Disabled'
 * '<S116>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Saturation/Passthrough'
 * '<S117>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Saturation Fdbk/Disabled'
 * '<S118>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Sum/Sum_PID'
 * '<S119>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Sum Fdbk/Disabled'
 * '<S120>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Tracking Mode/Disabled'
 * '<S121>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S122>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S123>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S124>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/postSat Signal/Forward_Path'
 * '<S125>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/preInt Signal/Internal PreInt'
 * '<S126>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller/preSat Signal/Forward_Path'
 * '<S127>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Anti-windup'
 * '<S128>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/D Gain'
 * '<S129>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/External Derivative'
 * '<S130>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Filter'
 * '<S131>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Filter ICs'
 * '<S132>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/I Gain'
 * '<S133>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Ideal P Gain'
 * '<S134>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Ideal P Gain Fdbk'
 * '<S135>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Integrator'
 * '<S136>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Integrator ICs'
 * '<S137>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/N Copy'
 * '<S138>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/N Gain'
 * '<S139>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/P Copy'
 * '<S140>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Parallel P Gain'
 * '<S141>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Reset Signal'
 * '<S142>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Saturation'
 * '<S143>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Saturation Fdbk'
 * '<S144>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Sum'
 * '<S145>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Sum Fdbk'
 * '<S146>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Tracking Mode'
 * '<S147>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Tracking Mode Sum'
 * '<S148>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Tsamp - Integral'
 * '<S149>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Tsamp - Ngain'
 * '<S150>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/postSat Signal'
 * '<S151>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/preInt Signal'
 * '<S152>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/preSat Signal'
 * '<S153>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Anti-windup/Passthrough'
 * '<S154>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/D Gain/Internal Parameters'
 * '<S155>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/External Derivative/Error'
 * '<S156>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Filter/Cont. Filter'
 * '<S157>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Filter ICs/Internal IC - Filter'
 * '<S158>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/I Gain/Internal Parameters'
 * '<S159>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Ideal P Gain/Passthrough'
 * '<S160>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Ideal P Gain Fdbk/Disabled'
 * '<S161>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Integrator/Continuous'
 * '<S162>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Integrator ICs/Internal IC'
 * '<S163>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/N Copy/Disabled'
 * '<S164>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/N Gain/Internal Parameters'
 * '<S165>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/P Copy/Disabled'
 * '<S166>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Parallel P Gain/Internal Parameters'
 * '<S167>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Reset Signal/Disabled'
 * '<S168>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Saturation/Passthrough'
 * '<S169>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Saturation Fdbk/Disabled'
 * '<S170>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Sum/Sum_PID'
 * '<S171>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Sum Fdbk/Disabled'
 * '<S172>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Tracking Mode/Disabled'
 * '<S173>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Tracking Mode Sum/Passthrough'
 * '<S174>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Tsamp - Integral/TsSignalSpecification'
 * '<S175>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/Tsamp - Ngain/Passthrough'
 * '<S176>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/postSat Signal/Forward_Path'
 * '<S177>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/preInt Signal/Internal PreInt'
 * '<S178>' : 'MicroMouse_Deploy/StudentTemplate/PID Controller4/preSat Signal/Forward_Path'
 * '<S179>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/Chart'
 * '<S180>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller'
 * '<S181>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/turn_adjus'
 * '<S182>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Anti-windup'
 * '<S183>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/D Gain'
 * '<S184>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/External Derivative'
 * '<S185>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Filter'
 * '<S186>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Filter ICs'
 * '<S187>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/I Gain'
 * '<S188>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Ideal P Gain'
 * '<S189>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Ideal P Gain Fdbk'
 * '<S190>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Integrator'
 * '<S191>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Integrator ICs'
 * '<S192>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/N Copy'
 * '<S193>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/N Gain'
 * '<S194>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/P Copy'
 * '<S195>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Parallel P Gain'
 * '<S196>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Reset Signal'
 * '<S197>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Saturation'
 * '<S198>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Saturation Fdbk'
 * '<S199>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Sum'
 * '<S200>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Sum Fdbk'
 * '<S201>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tracking Mode'
 * '<S202>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tracking Mode Sum'
 * '<S203>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tsamp - Integral'
 * '<S204>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tsamp - Ngain'
 * '<S205>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/postSat Signal'
 * '<S206>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/preInt Signal'
 * '<S207>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/preSat Signal'
 * '<S208>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Anti-windup/Disabled'
 * '<S209>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/D Gain/Internal Parameters'
 * '<S210>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/External Derivative/Error'
 * '<S211>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Filter/Cont. Filter'
 * '<S212>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S213>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/I Gain/Disabled'
 * '<S214>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Ideal P Gain/Passthrough'
 * '<S215>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S216>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Integrator/Disabled'
 * '<S217>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Integrator ICs/Disabled'
 * '<S218>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/N Copy/Disabled'
 * '<S219>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/N Gain/Internal Parameters'
 * '<S220>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/P Copy/Disabled'
 * '<S221>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S222>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Reset Signal/Disabled'
 * '<S223>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Saturation/Passthrough'
 * '<S224>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Saturation Fdbk/Disabled'
 * '<S225>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Sum/Sum_PD'
 * '<S226>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Sum Fdbk/Disabled'
 * '<S227>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tracking Mode/Disabled'
 * '<S228>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S229>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S230>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S231>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/postSat Signal/Forward_Path'
 * '<S232>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/preInt Signal/Internal PreInt'
 * '<S233>' : 'MicroMouse_Deploy/StudentTemplate/angle_adjus_draft2/PID Controller/preSat Signal/Forward_Path'
 * '<S234>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/Chart'
 * '<S235>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1'
 * '<S236>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2'
 * '<S237>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3'
 * '<S238>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4'
 * '<S239>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5'
 * '<S240>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Anti-windup'
 * '<S241>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/D Gain'
 * '<S242>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/External Derivative'
 * '<S243>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Filter'
 * '<S244>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Filter ICs'
 * '<S245>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/I Gain'
 * '<S246>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Ideal P Gain'
 * '<S247>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Ideal P Gain Fdbk'
 * '<S248>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Integrator'
 * '<S249>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Integrator ICs'
 * '<S250>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/N Copy'
 * '<S251>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/N Gain'
 * '<S252>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/P Copy'
 * '<S253>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Parallel P Gain'
 * '<S254>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Reset Signal'
 * '<S255>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Saturation'
 * '<S256>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Saturation Fdbk'
 * '<S257>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Sum'
 * '<S258>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Sum Fdbk'
 * '<S259>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Tracking Mode'
 * '<S260>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Tracking Mode Sum'
 * '<S261>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Tsamp - Integral'
 * '<S262>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Tsamp - Ngain'
 * '<S263>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/postSat Signal'
 * '<S264>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/preInt Signal'
 * '<S265>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/preSat Signal'
 * '<S266>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Anti-windup/Passthrough'
 * '<S267>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/D Gain/Internal Parameters'
 * '<S268>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/External Derivative/Error'
 * '<S269>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Filter/Cont. Filter'
 * '<S270>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S271>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/I Gain/Internal Parameters'
 * '<S272>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Ideal P Gain/Passthrough'
 * '<S273>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S274>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Integrator/Continuous'
 * '<S275>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Integrator ICs/Internal IC'
 * '<S276>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/N Copy/Disabled'
 * '<S277>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/N Gain/Internal Parameters'
 * '<S278>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/P Copy/Disabled'
 * '<S279>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S280>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Reset Signal/Disabled'
 * '<S281>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Saturation/Passthrough'
 * '<S282>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Saturation Fdbk/Disabled'
 * '<S283>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Sum/Sum_PID'
 * '<S284>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Sum Fdbk/Disabled'
 * '<S285>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Tracking Mode/Disabled'
 * '<S286>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S287>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S288>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S289>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/postSat Signal/Forward_Path'
 * '<S290>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/preInt Signal/Internal PreInt'
 * '<S291>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller1/preSat Signal/Forward_Path'
 * '<S292>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Anti-windup'
 * '<S293>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/D Gain'
 * '<S294>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/External Derivative'
 * '<S295>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Filter'
 * '<S296>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Filter ICs'
 * '<S297>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/I Gain'
 * '<S298>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Ideal P Gain'
 * '<S299>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Ideal P Gain Fdbk'
 * '<S300>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Integrator'
 * '<S301>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Integrator ICs'
 * '<S302>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/N Copy'
 * '<S303>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/N Gain'
 * '<S304>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/P Copy'
 * '<S305>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Parallel P Gain'
 * '<S306>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Reset Signal'
 * '<S307>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Saturation'
 * '<S308>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Saturation Fdbk'
 * '<S309>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Sum'
 * '<S310>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Sum Fdbk'
 * '<S311>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Tracking Mode'
 * '<S312>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Tracking Mode Sum'
 * '<S313>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Tsamp - Integral'
 * '<S314>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Tsamp - Ngain'
 * '<S315>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/postSat Signal'
 * '<S316>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/preInt Signal'
 * '<S317>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/preSat Signal'
 * '<S318>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Anti-windup/Passthrough'
 * '<S319>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/D Gain/Internal Parameters'
 * '<S320>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/External Derivative/Error'
 * '<S321>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Filter/Cont. Filter'
 * '<S322>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Filter ICs/Internal IC - Filter'
 * '<S323>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/I Gain/Internal Parameters'
 * '<S324>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Ideal P Gain/Passthrough'
 * '<S325>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S326>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Integrator/Continuous'
 * '<S327>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Integrator ICs/Internal IC'
 * '<S328>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/N Copy/Disabled'
 * '<S329>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/N Gain/Internal Parameters'
 * '<S330>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/P Copy/Disabled'
 * '<S331>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Parallel P Gain/Internal Parameters'
 * '<S332>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Reset Signal/Disabled'
 * '<S333>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Saturation/Passthrough'
 * '<S334>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Saturation Fdbk/Disabled'
 * '<S335>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Sum/Sum_PID'
 * '<S336>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Sum Fdbk/Disabled'
 * '<S337>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Tracking Mode/Disabled'
 * '<S338>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S339>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S340>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S341>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/postSat Signal/Forward_Path'
 * '<S342>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/preInt Signal/Internal PreInt'
 * '<S343>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller2/preSat Signal/Forward_Path'
 * '<S344>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Anti-windup'
 * '<S345>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/D Gain'
 * '<S346>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/External Derivative'
 * '<S347>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Filter'
 * '<S348>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Filter ICs'
 * '<S349>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/I Gain'
 * '<S350>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Ideal P Gain'
 * '<S351>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Ideal P Gain Fdbk'
 * '<S352>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Integrator'
 * '<S353>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Integrator ICs'
 * '<S354>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/N Copy'
 * '<S355>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/N Gain'
 * '<S356>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/P Copy'
 * '<S357>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Parallel P Gain'
 * '<S358>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Reset Signal'
 * '<S359>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Saturation'
 * '<S360>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Saturation Fdbk'
 * '<S361>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Sum'
 * '<S362>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Sum Fdbk'
 * '<S363>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Tracking Mode'
 * '<S364>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Tracking Mode Sum'
 * '<S365>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Tsamp - Integral'
 * '<S366>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Tsamp - Ngain'
 * '<S367>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/postSat Signal'
 * '<S368>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/preInt Signal'
 * '<S369>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/preSat Signal'
 * '<S370>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Anti-windup/Passthrough'
 * '<S371>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/D Gain/Internal Parameters'
 * '<S372>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/External Derivative/Error'
 * '<S373>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Filter/Cont. Filter'
 * '<S374>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Filter ICs/Internal IC - Filter'
 * '<S375>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/I Gain/Internal Parameters'
 * '<S376>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Ideal P Gain/Passthrough'
 * '<S377>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Ideal P Gain Fdbk/Disabled'
 * '<S378>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Integrator/Continuous'
 * '<S379>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Integrator ICs/Internal IC'
 * '<S380>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/N Copy/Disabled'
 * '<S381>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/N Gain/Internal Parameters'
 * '<S382>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/P Copy/Disabled'
 * '<S383>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Parallel P Gain/Internal Parameters'
 * '<S384>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Reset Signal/Disabled'
 * '<S385>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Saturation/Passthrough'
 * '<S386>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Saturation Fdbk/Disabled'
 * '<S387>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Sum/Sum_PID'
 * '<S388>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Sum Fdbk/Disabled'
 * '<S389>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Tracking Mode/Disabled'
 * '<S390>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Tracking Mode Sum/Passthrough'
 * '<S391>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Tsamp - Integral/TsSignalSpecification'
 * '<S392>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/Tsamp - Ngain/Passthrough'
 * '<S393>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/postSat Signal/Forward_Path'
 * '<S394>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/preInt Signal/Internal PreInt'
 * '<S395>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller3/preSat Signal/Forward_Path'
 * '<S396>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Anti-windup'
 * '<S397>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/D Gain'
 * '<S398>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/External Derivative'
 * '<S399>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Filter'
 * '<S400>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Filter ICs'
 * '<S401>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/I Gain'
 * '<S402>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Ideal P Gain'
 * '<S403>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Ideal P Gain Fdbk'
 * '<S404>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Integrator'
 * '<S405>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Integrator ICs'
 * '<S406>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/N Copy'
 * '<S407>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/N Gain'
 * '<S408>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/P Copy'
 * '<S409>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Parallel P Gain'
 * '<S410>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Reset Signal'
 * '<S411>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Saturation'
 * '<S412>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Saturation Fdbk'
 * '<S413>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Sum'
 * '<S414>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Sum Fdbk'
 * '<S415>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Tracking Mode'
 * '<S416>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Tracking Mode Sum'
 * '<S417>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Tsamp - Integral'
 * '<S418>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Tsamp - Ngain'
 * '<S419>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/postSat Signal'
 * '<S420>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/preInt Signal'
 * '<S421>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/preSat Signal'
 * '<S422>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Anti-windup/Passthrough'
 * '<S423>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/D Gain/Internal Parameters'
 * '<S424>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/External Derivative/Error'
 * '<S425>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Filter/Cont. Filter'
 * '<S426>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Filter ICs/Internal IC - Filter'
 * '<S427>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/I Gain/Internal Parameters'
 * '<S428>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Ideal P Gain/Passthrough'
 * '<S429>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Ideal P Gain Fdbk/Disabled'
 * '<S430>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Integrator/Continuous'
 * '<S431>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Integrator ICs/Internal IC'
 * '<S432>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/N Copy/Disabled'
 * '<S433>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/N Gain/Internal Parameters'
 * '<S434>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/P Copy/Disabled'
 * '<S435>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Parallel P Gain/Internal Parameters'
 * '<S436>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Reset Signal/Disabled'
 * '<S437>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Saturation/Passthrough'
 * '<S438>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Saturation Fdbk/Disabled'
 * '<S439>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Sum/Sum_PID'
 * '<S440>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Sum Fdbk/Disabled'
 * '<S441>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Tracking Mode/Disabled'
 * '<S442>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Tracking Mode Sum/Passthrough'
 * '<S443>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Tsamp - Integral/TsSignalSpecification'
 * '<S444>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/Tsamp - Ngain/Passthrough'
 * '<S445>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/postSat Signal/Forward_Path'
 * '<S446>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/preInt Signal/Internal PreInt'
 * '<S447>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller4/preSat Signal/Forward_Path'
 * '<S448>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Anti-windup'
 * '<S449>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/D Gain'
 * '<S450>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/External Derivative'
 * '<S451>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Filter'
 * '<S452>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Filter ICs'
 * '<S453>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/I Gain'
 * '<S454>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Ideal P Gain'
 * '<S455>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Ideal P Gain Fdbk'
 * '<S456>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Integrator'
 * '<S457>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Integrator ICs'
 * '<S458>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/N Copy'
 * '<S459>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/N Gain'
 * '<S460>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/P Copy'
 * '<S461>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Parallel P Gain'
 * '<S462>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Reset Signal'
 * '<S463>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Saturation'
 * '<S464>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Saturation Fdbk'
 * '<S465>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Sum'
 * '<S466>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Sum Fdbk'
 * '<S467>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Tracking Mode'
 * '<S468>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Tracking Mode Sum'
 * '<S469>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Tsamp - Integral'
 * '<S470>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Tsamp - Ngain'
 * '<S471>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/postSat Signal'
 * '<S472>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/preInt Signal'
 * '<S473>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/preSat Signal'
 * '<S474>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Anti-windup/Passthrough'
 * '<S475>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/D Gain/Internal Parameters'
 * '<S476>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/External Derivative/Error'
 * '<S477>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Filter/Cont. Filter'
 * '<S478>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Filter ICs/Internal IC - Filter'
 * '<S479>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/I Gain/Internal Parameters'
 * '<S480>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Ideal P Gain/Passthrough'
 * '<S481>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Ideal P Gain Fdbk/Disabled'
 * '<S482>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Integrator/Continuous'
 * '<S483>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Integrator ICs/Internal IC'
 * '<S484>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/N Copy/Disabled'
 * '<S485>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/N Gain/Internal Parameters'
 * '<S486>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/P Copy/Disabled'
 * '<S487>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Parallel P Gain/Internal Parameters'
 * '<S488>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Reset Signal/Disabled'
 * '<S489>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Saturation/Passthrough'
 * '<S490>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Saturation Fdbk/Disabled'
 * '<S491>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Sum/Sum_PID'
 * '<S492>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Sum Fdbk/Disabled'
 * '<S493>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Tracking Mode/Disabled'
 * '<S494>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Tracking Mode Sum/Passthrough'
 * '<S495>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Tsamp - Integral/TsSignalSpecification'
 * '<S496>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/Tsamp - Ngain/Passthrough'
 * '<S497>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/postSat Signal/Forward_Path'
 * '<S498>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/preInt Signal/Internal PreInt'
 * '<S499>' : 'MicroMouse_Deploy/StudentTemplate/encoder_straightening/PID Controller5/preSat Signal/Forward_Path'
 * '<S500>' : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass'
 * '<S501>' : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass/SOS1'
 * '<S502>' : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass/SOS2'
 * '<S503>' : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass/SOS1/Arithmetic'
 * '<S504>' : 'MicroMouse_Deploy/StudentTemplate/gyro wave smoothing /Discrete Varying Lowpass/SOS2/Arithmetic'
 * '<S505>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1'
 * '<S506>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2'
 * '<S507>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/engage_disengage_straightnert'
 * '<S508>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Anti-windup'
 * '<S509>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/D Gain'
 * '<S510>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/External Derivative'
 * '<S511>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Filter'
 * '<S512>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Filter ICs'
 * '<S513>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/I Gain'
 * '<S514>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Ideal P Gain'
 * '<S515>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Ideal P Gain Fdbk'
 * '<S516>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Integrator'
 * '<S517>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Integrator ICs'
 * '<S518>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/N Copy'
 * '<S519>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/N Gain'
 * '<S520>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/P Copy'
 * '<S521>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Parallel P Gain'
 * '<S522>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Reset Signal'
 * '<S523>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Saturation'
 * '<S524>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Saturation Fdbk'
 * '<S525>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Sum'
 * '<S526>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Sum Fdbk'
 * '<S527>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Tracking Mode'
 * '<S528>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Tracking Mode Sum'
 * '<S529>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Tsamp - Integral'
 * '<S530>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Tsamp - Ngain'
 * '<S531>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/postSat Signal'
 * '<S532>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/preInt Signal'
 * '<S533>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/preSat Signal'
 * '<S534>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Anti-windup/Passthrough'
 * '<S535>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/D Gain/Internal Parameters'
 * '<S536>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/External Derivative/Error'
 * '<S537>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Filter/Cont. Filter'
 * '<S538>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S539>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/I Gain/Internal Parameters'
 * '<S540>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Ideal P Gain/Passthrough'
 * '<S541>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S542>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Integrator/Continuous'
 * '<S543>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Integrator ICs/Internal IC'
 * '<S544>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/N Copy/Disabled'
 * '<S545>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/N Gain/Internal Parameters'
 * '<S546>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/P Copy/Disabled'
 * '<S547>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S548>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Reset Signal/Disabled'
 * '<S549>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Saturation/Passthrough'
 * '<S550>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Saturation Fdbk/Disabled'
 * '<S551>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Sum/Sum_PID'
 * '<S552>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Sum Fdbk/Disabled'
 * '<S553>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Tracking Mode/Disabled'
 * '<S554>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S555>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S556>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S557>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/postSat Signal/Forward_Path'
 * '<S558>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/preInt Signal/Internal PreInt'
 * '<S559>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller1/preSat Signal/Forward_Path'
 * '<S560>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Anti-windup'
 * '<S561>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/D Gain'
 * '<S562>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/External Derivative'
 * '<S563>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Filter'
 * '<S564>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Filter ICs'
 * '<S565>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/I Gain'
 * '<S566>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Ideal P Gain'
 * '<S567>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Ideal P Gain Fdbk'
 * '<S568>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Integrator'
 * '<S569>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Integrator ICs'
 * '<S570>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/N Copy'
 * '<S571>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/N Gain'
 * '<S572>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/P Copy'
 * '<S573>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Parallel P Gain'
 * '<S574>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Reset Signal'
 * '<S575>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Saturation'
 * '<S576>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Saturation Fdbk'
 * '<S577>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Sum'
 * '<S578>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Sum Fdbk'
 * '<S579>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Tracking Mode'
 * '<S580>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Tracking Mode Sum'
 * '<S581>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Tsamp - Integral'
 * '<S582>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Tsamp - Ngain'
 * '<S583>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/postSat Signal'
 * '<S584>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/preInt Signal'
 * '<S585>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/preSat Signal'
 * '<S586>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Anti-windup/Passthrough'
 * '<S587>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/D Gain/Internal Parameters'
 * '<S588>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/External Derivative/Error'
 * '<S589>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Filter/Cont. Filter'
 * '<S590>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Filter ICs/Internal IC - Filter'
 * '<S591>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/I Gain/Internal Parameters'
 * '<S592>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Ideal P Gain/Passthrough'
 * '<S593>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S594>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Integrator/Continuous'
 * '<S595>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Integrator ICs/Internal IC'
 * '<S596>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/N Copy/Disabled'
 * '<S597>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/N Gain/Internal Parameters'
 * '<S598>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/P Copy/Disabled'
 * '<S599>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Parallel P Gain/Internal Parameters'
 * '<S600>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Reset Signal/Disabled'
 * '<S601>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Saturation/Passthrough'
 * '<S602>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Saturation Fdbk/Disabled'
 * '<S603>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Sum/Sum_PID'
 * '<S604>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Sum Fdbk/Disabled'
 * '<S605>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Tracking Mode/Disabled'
 * '<S606>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S607>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S608>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S609>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/postSat Signal/Forward_Path'
 * '<S610>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/preInt Signal/Internal PreInt'
 * '<S611>' : 'MicroMouse_Deploy/StudentTemplate/maze_straightener_draft1/PID Controller2/preSat Signal/Forward_Path'
 */
#endif                                 /* MicroMouse_Deploy_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
