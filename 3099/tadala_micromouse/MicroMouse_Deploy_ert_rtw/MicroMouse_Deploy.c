/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy.c
 *
 * Code generated for Simulink model 'MicroMouse_Deploy'.
 *
 * Model version                  : 5.30
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sat Oct 25 21:13:17 2025
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
#include "MicroMouse_Deploy_types.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <stdio.h>
#include <string.h>
#include "MicroMouse_Deploy_private.h"

/* Named constants for Chart: '<S27>/Chart' */
#define MicroMouse_D_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define MicroMouse_Deploy_IN_default   ((uint8_T)1U)
#define MicroMouse_IN_encoder_pulse_end ((uint8_T)2U)

/* Named constants for Chart: '<S18>/Chart' */
#define MicroMouse_Deploy_IN_Set_Offset ((uint8_T)1U)
#define MicroMouse_Deploy_IN_Start     ((uint8_T)2U)

/* Named constants for Chart: '<S18>/turn_adjus' */
#define MicroMouse_Deploy_IN_deafult   ((uint8_T)1U)
#define MicroMouse_Deploy_IN_turnLeft  ((uint8_T)2U)
#define MicroMouse_Deploy_IN_turnRight ((uint8_T)3U)

/* Named constants for Chart: '<S3>/draft2_Maze_exploration' */
#define MicroMouse_D_IN_left_turn_setup ((uint8_T)8U)
#define MicroMouse_Dep_IN_Full_rotation ((uint8_T)1U)
#define MicroMouse_Depl_IN_Go_straight1 ((uint8_T)3U)
#define MicroMouse_Deplo_IN_Go_straight ((uint8_T)2U)
#define MicroMouse_Deplo_IN_set_Z_angle ((uint8_T)11U)
#define MicroMouse_Deploy_IN_Left_turn ((uint8_T)4U)
#define MicroMouse_Deploy_IN_Left_turn1 ((uint8_T)5U)
#define MicroMouse_Deploy_IN_Left_turn2 ((uint8_T)6U)
#define MicroMouse_Deploy_IN_Start_p   ((uint8_T)7U)
#define MicroMouse_Deploy_IN_right_turn ((uint8_T)10U)
#define MicroMouse__IN_left_turn_setup1 ((uint8_T)9U)

/* Named constants for Chart: '<S22>/Chart' */
#define MicroMou_IN_distance_corrcetion ((uint8_T)2U)
#define MicroMouse_Deploy_IN_after_turn ((uint8_T)1U)

/* Named constants for Chart: '<S24>/engage_disengage_straightnert' */
#define MicroMouse_De_IN_Going_straight ((uint8_T)2U)
#define MicroMouse_Depl_IN_Turn_is_seen ((uint8_T)3U)
#define MicroMouse_Deploy_IN_Deafult   ((uint8_T)1U)

/* Block signals (default storage) */
B_MicroMouse_Deploy_T MicroMouse_Deploy_B;

/* Continuous states */
X_MicroMouse_Deploy_T MicroMouse_Deploy_X;

/* Disabled State Vector */
XDis_MicroMouse_Deploy_T MicroMouse_Deploy_XDis;

/* Block states (default storage) */
DW_MicroMouse_Deploy_T MicroMouse_Deploy_DW;

/* Real-time model */
static RT_MODEL_MicroMouse_Deploy_T MicroMouse_Deploy_M_;
RT_MODEL_MicroMouse_Deploy_T *const MicroMouse_Deploy_M = &MicroMouse_Deploy_M_;

/* Forward declaration for local functions */
static void MicroMou_MedianFilter_resetImpl(e_dsp_internal_codegen_Median_T *obj);
static void Mic_MedianFilter_trickleDownMax(e_dsp_internal_codegen_Median_T *obj,
  real_T i);
static void Mic_MedianFilter_trickleDownMin(e_dsp_internal_codegen_Median_T *obj,
  real_T i);
static void MicroMouse_Dep_SystemCore_setup(dsp_simulink_MovingAverage_Mi_T *obj);

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 16;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  MicroMouse_Deploy_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  MicroMouse_Deploy_step();
  MicroMouse_Deploy_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  MicroMouse_Deploy_step();
  MicroMouse_Deploy_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void MicroMou_MedianFilter_resetImpl(e_dsp_internal_codegen_Median_T *obj)
{
  real_T cnt2;
  int32_T cnt1;
  int32_T i;
  for (i = 0; i < 5; i++) {
    /* Start for MATLABSystem: '<S23>/Median Filter' */
    obj->pBuf[i] = 0.0;
    obj->pPos[i] = 0.0;
    obj->pHeap[i] = 0.0;
  }

  /* Start for MATLABSystem: '<S23>/Median Filter' */
  obj->pWinLen = 5.0;
  obj->pIdx = obj->pWinLen;

  /* Start for MATLABSystem: '<S23>/Median Filter' */
  obj->pMidHeap = ceil((obj->pWinLen + 1.0) / 2.0);
  obj->pMinHeapLength = trunc((obj->pWinLen - 1.0) / 2.0);
  obj->pMaxHeapLength = trunc(obj->pWinLen / 2.0);
  cnt1 = 1;
  cnt2 = obj->pWinLen;
  for (i = 0; i < 5; i++) {
    /* Start for MATLABSystem: '<S23>/Median Filter' */
    if (fmod(5.0 - (real_T)i, 2.0) == 0.0) {
      obj->pPos[4 - i] = cnt1;
      cnt1++;
    } else {
      obj->pPos[4 - i] = cnt2;
      cnt2--;
    }

    obj->pHeap[(int32_T)obj->pPos[4 - i] - 1] = 5.0 - (real_T)i;
  }
}

static void Mic_MedianFilter_trickleDownMax(e_dsp_internal_codegen_Median_T *obj,
  real_T i)
{
  boolean_T exitg1;
  exitg1 = false;
  while ((!exitg1) && (i >= -obj->pMaxHeapLength)) {
    real_T ind1;
    real_T ind2;
    real_T tmp;
    real_T tmp_0;
    if ((i < -1.0) && (i > -obj->pMaxHeapLength) && (obj->pBuf[(int32_T)
         obj->pHeap[(int32_T)(i + obj->pMidHeap) - 1] - 1] < obj->pBuf[(int32_T)
         obj->pHeap[(int32_T)((i - 1.0) + obj->pMidHeap) - 1] - 1])) {
      i--;
    }

    ind1 = trunc(i / 2.0) + obj->pMidHeap;
    ind2 = i + obj->pMidHeap;
    tmp = obj->pHeap[(int32_T)ind1 - 1];
    tmp_0 = obj->pHeap[(int32_T)ind2 - 1];
    if (!(obj->pBuf[(int32_T)tmp - 1] < obj->pBuf[(int32_T)tmp_0 - 1])) {
      exitg1 = true;
    } else {
      obj->pHeap[(int32_T)ind1 - 1] = tmp_0;
      obj->pHeap[(int32_T)ind2 - 1] = tmp;
      obj->pPos[(int32_T)obj->pHeap[(int32_T)ind1 - 1] - 1] = ind1;
      obj->pPos[(int32_T)obj->pHeap[(int32_T)ind2 - 1] - 1] = ind2;
      i *= 2.0;
    }
  }
}

static void Mic_MedianFilter_trickleDownMin(e_dsp_internal_codegen_Median_T *obj,
  real_T i)
{
  boolean_T exitg1;
  exitg1 = false;
  while ((!exitg1) && (i <= obj->pMinHeapLength)) {
    real_T ind1;
    real_T ind2;
    real_T tmp;
    real_T tmp_0;
    if ((i > 1.0) && (i < obj->pMinHeapLength) && (obj->pBuf[(int32_T)obj->
         pHeap[(int32_T)((i + 1.0) + obj->pMidHeap) - 1] - 1] < obj->pBuf
         [(int32_T)obj->pHeap[(int32_T)(i + obj->pMidHeap) - 1] - 1])) {
      i++;
    }

    ind1 = i + obj->pMidHeap;
    ind2 = trunc(i / 2.0) + obj->pMidHeap;
    tmp = obj->pHeap[(int32_T)ind1 - 1];
    tmp_0 = obj->pHeap[(int32_T)ind2 - 1];
    if (!(obj->pBuf[(int32_T)tmp - 1] < obj->pBuf[(int32_T)tmp_0 - 1])) {
      exitg1 = true;
    } else {
      obj->pHeap[(int32_T)ind1 - 1] = tmp_0;
      obj->pHeap[(int32_T)ind2 - 1] = tmp;
      obj->pPos[(int32_T)obj->pHeap[(int32_T)ind1 - 1] - 1] = ind1;
      obj->pPos[(int32_T)obj->pHeap[(int32_T)ind2 - 1] - 1] = ind2;
      i *= 2.0;
    }
  }
}

static void MicroMouse_Dep_SystemCore_setup(dsp_simulink_MovingAverage_Mi_T *obj)
{
  int32_T i;
  obj->isInitialized = 1;

  /* Start for MATLABSystem: '<S24>/Moving Average' incorporates:
   *  MATLABSystem: '<S24>/Moving Average1'
   */
  obj->NumChannels = 1;
  obj->FrameLength = 1;
  obj->pCumSum = 0.0F;
  for (i = 0; i < 299; i++) {
    /* Start for MATLABSystem: '<S24>/Moving Average' incorporates:
     *  MATLABSystem: '<S24>/Moving Average1'
     */
    obj->pCumSumRev[i] = 0.0F;
  }

  /* Start for MATLABSystem: '<S24>/Moving Average' incorporates:
   *  MATLABSystem: '<S24>/Moving Average1'
   */
  obj->pCumRevIndex = 1.0F;
  obj->pModValueRev = 0.0F;
  obj->isSetupComplete = true;
  obj->TunablePropsChanged = false;
}

/* Function for Chart: '<S3>/draft2_Maze_exploration' */
void mdlDerivatives_c8_MicroMouse_Deploy(void)
{
}

/* Function for Chart: '<S3>/draft2_Maze_exploration1' */
void mdlDerivatives_c6_MicroMouse_Deploy(void)
{
}

/* Function for Chart: '<S3>/draft2_Maze_exploration2' */
void mdlDerivatives_c10_MicroMouse_Deploy(void)
{
}

/* Model step function */
void MicroMouse_Deploy_step(void)
{
  /* local block i/o variables */
  uint8_T rtb_Compare;
  uint8_T rtb_Compare_g;
  boolean_T rtb_LogicalOperator;
  boolean_T rtb_LogicalOperator_a;
  boolean_T rtb_LogicalOperator_c;
  boolean_T rtb_Compare_hh;
  real_T Gain1;
  real_T Product;
  real_T Product_g;
  real_T elapsedTime;
  real_T p;
  real_T rtb_Product1_b;
  real_T rtb_Sum2_j;
  real_T rtb_d;
  real_T rtb_p;
  real_T rtb_p_a;
  real_T rtb_q;
  real_T rtb_q_g;
  real_T rtb_r;
  real_T rtb_r_p;
  real_T temp;
  real_T vprev;
  int32_T k;
  real32_T K;
  real32_T K_e;
  real32_T K_p;
  real32_T rtb_Gain1_h;
  real32_T rtb_Gain2_b;
  real32_T rtb_Gain4;
  char_T rtb_ComposeString1_0[256];
  char_T rtb_ComposeString2_0[256];
  char_T rtb_ComposeString4_0[256];
  char_T rtb_ComposeString_0[256];
  boolean_T exitg1;
  boolean_T rtb_Compare_b;
  boolean_T rtb_Compare_c;
  boolean_T rtb_NOT;
  boolean_T tmp;
  boolean_T tmp_0;
  if (rtmIsMajorTimeStep(MicroMouse_Deploy_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&MicroMouse_Deploy_M->solverInfo,
                          ((MicroMouse_Deploy_M->Timing.clockTick0+1)*
      MicroMouse_Deploy_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(MicroMouse_Deploy_M)) {
    MicroMouse_Deploy_M->Timing.t[0] = rtsiGetT(&MicroMouse_Deploy_M->solverInfo);
  }

  tmp = rtmIsMajorTimeStep(MicroMouse_Deploy_M);

  /* Gain: '<S13>/K' incorporates:
   *  DataStoreRead: '<S10>/Data Store Read'
   *  DataTypeConversion: '<S10>/Cast To Single'
   *  Gain: '<S10>/Gain'
   */
  K = MicroMouse_Deploy_P.Gain_Gain_g * (real32_T)TOF_Distance[0] *
    MicroMouse_Deploy_P.LowPassFilterDiscreteorConti_pu;

  /* Gain: '<S15>/K' incorporates:
   *  DataStoreRead: '<S10>/Data Store Read'
   *  DataTypeConversion: '<S10>/Cast To Single'
   *  Gain: '<S10>/Gain'
   */
  K_e = MicroMouse_Deploy_P.Gain_Gain_g * (real32_T)TOF_Distance[2] *
    MicroMouse_Deploy_P.LowPassFilterDiscreteorConti_cj;
  if (tmp) {
    /* Logic: '<S54>/Logical Operator' incorporates:
     *  Constant: '<S54>/Constant'
     *  Constant: '<S54>/Time constant'
     *  Constant: '<S57>/Constant'
     *  Constant: '<S58>/Constant'
     *  RelationalOperator: '<S57>/Compare'
     *  RelationalOperator: '<S58>/Compare'
     *  Sum: '<S54>/Sum1'
     */
    rtb_LogicalOperator = (((real32_T)
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContinuo -
       MicroMouse_Deploy_B.Probe[0]) <= MicroMouse_Deploy_P.Constant_Value_lf) &&
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_k <
       MicroMouse_Deploy_P.CompareToConstant_const));

    /* DiscreteIntegrator: '<S60>/Integrator' */
    if (MicroMouse_Deploy_DW.Integrator_IC_LOADING != 0) {
      MicroMouse_Deploy_DW.Integrator_DSTATE = K;
      if (MicroMouse_Deploy_DW.Integrator_DSTATE >
          MicroMouse_Deploy_P.Integrator_UpperSat) {
        MicroMouse_Deploy_DW.Integrator_DSTATE =
          MicroMouse_Deploy_P.Integrator_UpperSat;
      } else if (MicroMouse_Deploy_DW.Integrator_DSTATE <
                 MicroMouse_Deploy_P.Integrator_LowerSat) {
        MicroMouse_Deploy_DW.Integrator_DSTATE =
          MicroMouse_Deploy_P.Integrator_LowerSat;
      }
    }

    if (rtb_LogicalOperator || (MicroMouse_Deploy_DW.Integrator_PrevResetState
         != 0)) {
      MicroMouse_Deploy_DW.Integrator_DSTATE = K;
      if (MicroMouse_Deploy_DW.Integrator_DSTATE >
          MicroMouse_Deploy_P.Integrator_UpperSat) {
        MicroMouse_Deploy_DW.Integrator_DSTATE =
          MicroMouse_Deploy_P.Integrator_UpperSat;
      } else if (MicroMouse_Deploy_DW.Integrator_DSTATE <
                 MicroMouse_Deploy_P.Integrator_LowerSat) {
        MicroMouse_Deploy_DW.Integrator_DSTATE =
          MicroMouse_Deploy_P.Integrator_LowerSat;
      }
    }

    /* Saturate: '<S60>/Saturation' incorporates:
     *  DiscreteIntegrator: '<S60>/Integrator'
     */
    if (MicroMouse_Deploy_DW.Integrator_DSTATE >
        MicroMouse_Deploy_P.Saturation_UpperSat) {
      /* Saturate: '<S60>/Saturation' */
      MicroMouse_Deploy_B.Saturation = MicroMouse_Deploy_P.Saturation_UpperSat;
    } else if (MicroMouse_Deploy_DW.Integrator_DSTATE <
               MicroMouse_Deploy_P.Saturation_LowerSat) {
      /* Saturate: '<S60>/Saturation' */
      MicroMouse_Deploy_B.Saturation = MicroMouse_Deploy_P.Saturation_LowerSat;
    } else {
      /* Saturate: '<S60>/Saturation' */
      MicroMouse_Deploy_B.Saturation = MicroMouse_Deploy_DW.Integrator_DSTATE;
    }

    /* End of Saturate: '<S60>/Saturation' */

    /* RelationalOperator: '<S3>/GreaterThan' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    MicroMouse_Deploy_B.TOF_LEFT = (MicroMouse_Deploy_B.Saturation <=
      MicroMouse_Deploy_P.Constant1_Value);

    /* Logic: '<S68>/Logical Operator' incorporates:
     *  Constant: '<S68>/Constant'
     *  Constant: '<S68>/Time constant'
     *  Constant: '<S71>/Constant'
     *  Constant: '<S72>/Constant'
     *  RelationalOperator: '<S71>/Compare'
     *  RelationalOperator: '<S72>/Compare'
     *  Sum: '<S68>/Sum1'
     */
    rtb_LogicalOperator_a = (((real32_T)
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_e -
       MicroMouse_Deploy_B.Probe_g[0]) <= MicroMouse_Deploy_P.Constant_Value_c) &&
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_c <
       MicroMouse_Deploy_P.CompareToConstant_const_o));

    /* DiscreteIntegrator: '<S74>/Integrator' */
    if (MicroMouse_Deploy_DW.Integrator_IC_LOADING_a != 0) {
      MicroMouse_Deploy_DW.Integrator_DSTATE_i = K_e;
      if (MicroMouse_Deploy_DW.Integrator_DSTATE_i >
          MicroMouse_Deploy_P.Integrator_UpperSat_l) {
        MicroMouse_Deploy_DW.Integrator_DSTATE_i =
          MicroMouse_Deploy_P.Integrator_UpperSat_l;
      } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_i <
                 MicroMouse_Deploy_P.Integrator_LowerSat_g) {
        MicroMouse_Deploy_DW.Integrator_DSTATE_i =
          MicroMouse_Deploy_P.Integrator_LowerSat_g;
      }
    }

    if (rtb_LogicalOperator_a ||
        (MicroMouse_Deploy_DW.Integrator_PrevResetState_j != 0)) {
      MicroMouse_Deploy_DW.Integrator_DSTATE_i = K_e;
      if (MicroMouse_Deploy_DW.Integrator_DSTATE_i >
          MicroMouse_Deploy_P.Integrator_UpperSat_l) {
        MicroMouse_Deploy_DW.Integrator_DSTATE_i =
          MicroMouse_Deploy_P.Integrator_UpperSat_l;
      } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_i <
                 MicroMouse_Deploy_P.Integrator_LowerSat_g) {
        MicroMouse_Deploy_DW.Integrator_DSTATE_i =
          MicroMouse_Deploy_P.Integrator_LowerSat_g;
      }
    }

    /* Saturate: '<S74>/Saturation' incorporates:
     *  DiscreteIntegrator: '<S74>/Integrator'
     */
    if (MicroMouse_Deploy_DW.Integrator_DSTATE_i >
        MicroMouse_Deploy_P.Saturation_UpperSat_c) {
      /* Saturate: '<S74>/Saturation' */
      MicroMouse_Deploy_B.Saturation_m =
        MicroMouse_Deploy_P.Saturation_UpperSat_c;
    } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_i <
               MicroMouse_Deploy_P.Saturation_LowerSat_o) {
      /* Saturate: '<S74>/Saturation' */
      MicroMouse_Deploy_B.Saturation_m =
        MicroMouse_Deploy_P.Saturation_LowerSat_o;
    } else {
      /* Saturate: '<S74>/Saturation' */
      MicroMouse_Deploy_B.Saturation_m =
        MicroMouse_Deploy_DW.Integrator_DSTATE_i;
    }

    /* End of Saturate: '<S74>/Saturation' */

    /* RelationalOperator: '<S3>/GreaterThan2' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    MicroMouse_Deploy_B.TOF_RIGHT = (MicroMouse_Deploy_B.Saturation_m <=
      MicroMouse_Deploy_P.Constant1_Value);

    /* Delay: '<S27>/Delay' */
    MicroMouse_Deploy_B.Delay = MicroMouse_Deploy_DW.Delay_DSTATE_d;

    /* UnitDelay: '<S36>/Delay Input1'
     *
     * Block description for '<S36>/Delay Input1':
     *
     *  Store in Global RAM
     */
    MicroMouse_Deploy_B.Uk1 = MicroMouse_Deploy_DW.DelayInput1_DSTATE;
  }

  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   *  DataStoreRead: '<S4>/Data Store Read4'
   *  DataTypeConversion: '<S4>/Cast To Single'
   *  Gain: '<S4>/Gain2'
   */
  rtb_Compare_c = (MicroMouse_Deploy_P.Gain2_Gain * (real32_T)V_PHOTO_MOT_RS >=
                   MicroMouse_Deploy_P.CompareToConstant_const_jx);

  /* RelationalOperator: '<S38>/Compare' incorporates:
   *  Constant: '<S38>/Constant'
   */
  MicroMouse_Deploy_B.Compare = (uint8_T)((int32_T)rtb_Compare_c > (int32_T)
    MicroMouse_Deploy_P.Constant_Value_j);

  /* Sum: '<S27>/Add' incorporates:
   *  RelationalOperator: '<S36>/FixPt Relational Operator'
   */
  MicroMouse_Deploy_B.Add_f = (uint8_T)((uint32_T)(MicroMouse_Deploy_B.Compare >
    MicroMouse_Deploy_B.Uk1) + MicroMouse_Deploy_B.Delay);

  /* RateLimiter: '<S11>/Rate Limiter2' incorporates:
   *  DataTypeConversion: '<S11>/Data Type Conversion1'
   */
  if (MicroMouse_Deploy_DW.LastMajorTime == (rtInf)) {
    /* RateLimiter: '<S11>/Rate Limiter2' incorporates:
     *  DataTypeConversion: '<S11>/Data Type Conversion1'
     */
    MicroMouse_Deploy_B.RateLimiter2 = MicroMouse_Deploy_B.Add_f;
  } else {
    Gain1 = MicroMouse_Deploy_M->Timing.t[0];
    Product_g = Gain1 - MicroMouse_Deploy_DW.LastMajorTime;
    if (MicroMouse_Deploy_DW.LastMajorTime == Gain1) {
      if (MicroMouse_Deploy_DW.PrevLimited) {
        /* RateLimiter: '<S11>/Rate Limiter2' */
        MicroMouse_Deploy_B.RateLimiter2 = MicroMouse_Deploy_DW.PrevY;
      } else {
        /* RateLimiter: '<S11>/Rate Limiter2' incorporates:
         *  DataTypeConversion: '<S11>/Data Type Conversion1'
         */
        MicroMouse_Deploy_B.RateLimiter2 = MicroMouse_Deploy_B.Add_f;
      }
    } else {
      elapsedTime = Product_g * MicroMouse_Deploy_P.RateLimiter2_RisingLim;
      Gain1 = (real_T)MicroMouse_Deploy_B.Add_f - MicroMouse_Deploy_DW.PrevY;
      if (Gain1 > elapsedTime) {
        /* RateLimiter: '<S11>/Rate Limiter2' */
        MicroMouse_Deploy_B.RateLimiter2 = MicroMouse_Deploy_DW.PrevY +
          elapsedTime;
        rtb_NOT = true;
      } else {
        Product_g *= MicroMouse_Deploy_P.RateLimiter2_FallingLim;
        if (Gain1 < Product_g) {
          /* RateLimiter: '<S11>/Rate Limiter2' */
          MicroMouse_Deploy_B.RateLimiter2 = MicroMouse_Deploy_DW.PrevY +
            Product_g;
          rtb_NOT = true;
        } else {
          /* RateLimiter: '<S11>/Rate Limiter2' */
          MicroMouse_Deploy_B.RateLimiter2 = MicroMouse_Deploy_B.Add_f;
          rtb_NOT = false;
        }
      }

      if (rtsiIsModeUpdateTimeStep(&MicroMouse_Deploy_M->solverInfo)) {
        MicroMouse_Deploy_DW.PrevLimited = rtb_NOT;
      }
    }
  }

  /* End of RateLimiter: '<S11>/Rate Limiter2' */

  /* Product: '<S29>/Product' incorporates:
   *  Constant: '<S29>/Circumference'
   *  Gain: '<S29>/Tick_per_rev'
   */
  Product = MicroMouse_Deploy_P.Tick_per_rev_Gain *
    MicroMouse_Deploy_B.RateLimiter2 * MicroMouse_Deploy_P.Circumference_Value;
  if (tmp) {
    /* Delay: '<S26>/Delay' */
    MicroMouse_Deploy_B.Delay_f = MicroMouse_Deploy_DW.Delay_DSTATE_f;

    /* UnitDelay: '<S31>/Delay Input1'
     *
     * Block description for '<S31>/Delay Input1':
     *
     *  Store in Global RAM
     */
    MicroMouse_Deploy_B.Uk1_k = MicroMouse_Deploy_DW.DelayInput1_DSTATE_c;
  }

  /* RelationalOperator: '<S32>/Compare' incorporates:
   *  Constant: '<S30>/Constant'
   *  Constant: '<S32>/Constant'
   *  DataStoreRead: '<S4>/Data Store Read3'
   *  DataTypeConversion: '<S4>/Cast To Single1'
   *  Gain: '<S4>/Gain2'
   *  RelationalOperator: '<S30>/Compare'
   */
  MicroMouse_Deploy_B.Compare_a = (uint8_T)((MicroMouse_Deploy_P.Gain2_Gain *
    (real32_T)V_PHOTO_MOT_LS >= MicroMouse_Deploy_P.CompareToConstant_const_c) >
    (int32_T)MicroMouse_Deploy_P.Constant_Value_m);

  /* Sum: '<S26>/Add' incorporates:
   *  RelationalOperator: '<S31>/FixPt Relational Operator'
   */
  MicroMouse_Deploy_B.Add_h = (uint8_T)((uint32_T)(MicroMouse_Deploy_B.Compare_a
    > MicroMouse_Deploy_B.Uk1_k) + MicroMouse_Deploy_B.Delay_f);

  /* RateLimiter: '<S11>/Rate Limiter4' incorporates:
   *  DataTypeConversion: '<S11>/Data Type Conversion'
   */
  if (MicroMouse_Deploy_DW.LastMajorTime_b == (rtInf)) {
    /* RateLimiter: '<S11>/Rate Limiter4' incorporates:
     *  DataTypeConversion: '<S11>/Data Type Conversion'
     */
    MicroMouse_Deploy_B.RateLimiter4 = MicroMouse_Deploy_B.Add_h;
  } else {
    Gain1 = MicroMouse_Deploy_M->Timing.t[0];
    Product_g = Gain1 - MicroMouse_Deploy_DW.LastMajorTime_b;
    if (MicroMouse_Deploy_DW.LastMajorTime_b == Gain1) {
      if (MicroMouse_Deploy_DW.PrevLimited_o) {
        /* RateLimiter: '<S11>/Rate Limiter4' */
        MicroMouse_Deploy_B.RateLimiter4 = MicroMouse_Deploy_DW.PrevY_i;
      } else {
        /* RateLimiter: '<S11>/Rate Limiter4' incorporates:
         *  DataTypeConversion: '<S11>/Data Type Conversion'
         */
        MicroMouse_Deploy_B.RateLimiter4 = MicroMouse_Deploy_B.Add_h;
      }
    } else {
      elapsedTime = Product_g * MicroMouse_Deploy_P.RateLimiter4_RisingLim;
      Gain1 = (real_T)MicroMouse_Deploy_B.Add_h - MicroMouse_Deploy_DW.PrevY_i;
      if (Gain1 > elapsedTime) {
        /* RateLimiter: '<S11>/Rate Limiter4' */
        MicroMouse_Deploy_B.RateLimiter4 = MicroMouse_Deploy_DW.PrevY_i +
          elapsedTime;
        rtb_NOT = true;
      } else {
        Product_g *= MicroMouse_Deploy_P.RateLimiter4_FallingLim;
        if (Gain1 < Product_g) {
          /* RateLimiter: '<S11>/Rate Limiter4' */
          MicroMouse_Deploy_B.RateLimiter4 = MicroMouse_Deploy_DW.PrevY_i +
            Product_g;
          rtb_NOT = true;
        } else {
          /* RateLimiter: '<S11>/Rate Limiter4' */
          MicroMouse_Deploy_B.RateLimiter4 = MicroMouse_Deploy_B.Add_h;
          rtb_NOT = false;
        }
      }

      if (rtsiIsModeUpdateTimeStep(&MicroMouse_Deploy_M->solverInfo)) {
        MicroMouse_Deploy_DW.PrevLimited_o = rtb_NOT;
      }
    }
  }

  /* End of RateLimiter: '<S11>/Rate Limiter4' */

  /* Product: '<S28>/Product' incorporates:
   *  Constant: '<S28>/Circumference'
   *  Gain: '<S28>/Tick_per_rev'
   */
  Product_g = MicroMouse_Deploy_P.Tick_per_rev_Gain_l *
    MicroMouse_Deploy_B.RateLimiter4 * MicroMouse_Deploy_P.Circumference_Value_b;

  /* Gain: '<S11>/Gain1' incorporates:
   *  Sum: '<S11>/Sum3'
   */
  Gain1 = (Product + Product_g) * MicroMouse_Deploy_P.Gain1_Gain;

  /* Gain: '<S14>/K' incorporates:
   *  DataStoreRead: '<S10>/Data Store Read'
   *  DataTypeConversion: '<S10>/Cast To Single'
   *  Gain: '<S10>/Gain'
   */
  K_p = MicroMouse_Deploy_P.Gain_Gain_g * (real32_T)TOF_Distance[1] *
    MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_a;
  if (tmp) {
    /* Logic: '<S61>/Logical Operator' incorporates:
     *  Constant: '<S61>/Constant'
     *  Constant: '<S61>/Time constant'
     *  Constant: '<S64>/Constant'
     *  Constant: '<S65>/Constant'
     *  RelationalOperator: '<S64>/Compare'
     *  RelationalOperator: '<S65>/Compare'
     *  Sum: '<S61>/Sum1'
     */
    rtb_LogicalOperator_c = (((real32_T)
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_p -
       MicroMouse_Deploy_B.Probe_h[0]) <= MicroMouse_Deploy_P.Constant_Value_i) &&
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorConti_ct <
       MicroMouse_Deploy_P.CompareToConstant_const_j));

    /* DiscreteIntegrator: '<S67>/Integrator' */
    if (MicroMouse_Deploy_DW.Integrator_IC_LOADING_k != 0) {
      MicroMouse_Deploy_DW.Integrator_DSTATE_m = K_p;
      if (MicroMouse_Deploy_DW.Integrator_DSTATE_m >
          MicroMouse_Deploy_P.Integrator_UpperSat_k) {
        MicroMouse_Deploy_DW.Integrator_DSTATE_m =
          MicroMouse_Deploy_P.Integrator_UpperSat_k;
      } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_m <
                 MicroMouse_Deploy_P.Integrator_LowerSat_d) {
        MicroMouse_Deploy_DW.Integrator_DSTATE_m =
          MicroMouse_Deploy_P.Integrator_LowerSat_d;
      }
    }

    if (rtb_LogicalOperator_c ||
        (MicroMouse_Deploy_DW.Integrator_PrevResetState_a != 0)) {
      MicroMouse_Deploy_DW.Integrator_DSTATE_m = K_p;
      if (MicroMouse_Deploy_DW.Integrator_DSTATE_m >
          MicroMouse_Deploy_P.Integrator_UpperSat_k) {
        MicroMouse_Deploy_DW.Integrator_DSTATE_m =
          MicroMouse_Deploy_P.Integrator_UpperSat_k;
      } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_m <
                 MicroMouse_Deploy_P.Integrator_LowerSat_d) {
        MicroMouse_Deploy_DW.Integrator_DSTATE_m =
          MicroMouse_Deploy_P.Integrator_LowerSat_d;
      }
    }

    /* Saturate: '<S67>/Saturation' incorporates:
     *  DiscreteIntegrator: '<S67>/Integrator'
     */
    if (MicroMouse_Deploy_DW.Integrator_DSTATE_m >
        MicroMouse_Deploy_P.Saturation_UpperSat_f) {
      /* Saturate: '<S67>/Saturation' */
      MicroMouse_Deploy_B.Saturation_d =
        MicroMouse_Deploy_P.Saturation_UpperSat_f;
    } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_m <
               MicroMouse_Deploy_P.Saturation_LowerSat_b) {
      /* Saturate: '<S67>/Saturation' */
      MicroMouse_Deploy_B.Saturation_d =
        MicroMouse_Deploy_P.Saturation_LowerSat_b;
    } else {
      /* Saturate: '<S67>/Saturation' */
      MicroMouse_Deploy_B.Saturation_d =
        MicroMouse_Deploy_DW.Integrator_DSTATE_m;
    }

    /* End of Saturate: '<S67>/Saturation' */

    /* RelationalOperator: '<S3>/GreaterThan1' incorporates:
     *  Constant: '<S3>/Constant2'
     */
    MicroMouse_Deploy_B.TOF_FRONT = (MicroMouse_Deploy_B.Saturation_d <=
      MicroMouse_Deploy_P.Constant2_Value);
  }

  /* Chart: '<S3>/draft2_Maze_exploration' incorporates:
   *  Chart: '<S3>/draft2_Maze_exploration1'
   *  RateLimiter: '<S12>/Rate Limiter1'
   *  RateLimiter: '<S12>/Rate Limiter2'
   */
  tmp_0 = rtsiIsModeUpdateTimeStep(&MicroMouse_Deploy_M->solverInfo);
  if (tmp_0) {
    elapsedTime = MicroMouse_Deploy_M->Timing.t[0] -
      MicroMouse_Deploy_DW.previousTime;
    MicroMouse_Deploy_DW.previousTime = MicroMouse_Deploy_M->Timing.t[0];
    MicroMouse_Deploy_DW.temporalCounter_i1 += elapsedTime;
    if (MicroMouse_Deploy_DW.bitsForTID0.is_active_c8_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID0.is_active_c8_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
        MicroMouse_Deplo_IN_set_Z_angle;
      MicroMouse_Deploy_B.Z_angle_f = 0.0;
      rtsiSetBlockStateForSolverChangedAtMajorStep
        (&MicroMouse_Deploy_M->solverInfo, true);
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy) {
       case MicroMouse_Dep_IN_Full_rotation:
        if ((MicroMouse_Deploy_X.Integrator_CSTATE <=
             MicroMouse_Deploy_B.Z_angle_f) && (Gain1 >=
             MicroMouse_Deploy_DW.turn_dist)) {
          MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
          MicroMouse_Deploy_B.LW_c = 0.0;
          MicroMouse_Deploy_B.RW_d = 0.0;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse_Deplo_IN_Go_straight:
        if (MicroMouse_Deploy_B.TOF_FRONT) {
          MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
          MicroMouse_Deploy_B.LW_c = 0.0;
          MicroMouse_Deploy_B.RW_d = 0.0;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        } else if ((!MicroMouse_Deploy_B.TOF_FRONT) &&
                   (!MicroMouse_Deploy_B.TOF_LEFT)) {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_D_IN_left_turn_setup;
          MicroMouse_Deploy_DW.end_dist = Gain1 + 0.12;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse_Depl_IN_Go_straight1:
        if (MicroMouse_Deploy_B.TOF_FRONT) {
          MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
          MicroMouse_Deploy_B.LW_c = 0.0;
          MicroMouse_Deploy_B.RW_d = 0.0;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        } else if ((!MicroMouse_Deploy_B.TOF_FRONT) &&
                   (!MicroMouse_Deploy_B.TOF_LEFT)) {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse__IN_left_turn_setup1;
          MicroMouse_Deploy_DW.end_dist = Gain1 + 0.12;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse_Deploy_IN_Left_turn:
        if ((MicroMouse_Deploy_X.Integrator_CSTATE >=
             MicroMouse_Deploy_B.Z_angle_f) && (Gain1 >=
             MicroMouse_Deploy_DW.turn_dist)) {
          MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
          MicroMouse_Deploy_B.LW_c = 0.0;
          MicroMouse_Deploy_B.RW_d = 0.0;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse_Deploy_IN_Left_turn1:
        if ((MicroMouse_Deploy_X.Integrator_CSTATE >=
             MicroMouse_Deploy_B.Z_angle_f) && (Gain1 >=
             MicroMouse_Deploy_DW.turn_dist)) {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Depl_IN_Go_straight1;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse_Deploy_IN_Left_turn2:
        if ((MicroMouse_Deploy_X.Integrator_CSTATE >=
             MicroMouse_Deploy_B.Z_angle_f) && (Gain1 >=
             MicroMouse_Deploy_DW.turn_dist)) {
          MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
          MicroMouse_Deploy_B.LW_c = 0.0;
          MicroMouse_Deploy_B.RW_d = 0.0;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse_Deploy_IN_Start_p:
        if ((MicroMouse_Deploy_B.TOF_LEFT && (!MicroMouse_Deploy_B.TOF_FRONT)) ||
            (MicroMouse_Deploy_B.TOF_RIGHT && (!MicroMouse_Deploy_B.TOF_FRONT)))
        {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deplo_IN_Go_straight;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        } else if ((!MicroMouse_Deploy_B.TOF_LEFT) &&
                   MicroMouse_Deploy_B.TOF_FRONT) {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Left_turn;
          MicroMouse_Deploy_B.Z_angle_f += 1.5707963267948966;
          MicroMouse_Deploy_DW.turn_dist = Gain1;
          MicroMouse_Deploy_DW.turn_dist += 0.086393797973719308;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        } else if ((!MicroMouse_Deploy_B.TOF_RIGHT) &&
                   MicroMouse_Deploy_B.TOF_FRONT) {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_right_turn;
          MicroMouse_Deploy_B.Z_angle_f -= 1.5707963267948966;
          MicroMouse_Deploy_DW.turn_dist = Gain1;
          MicroMouse_Deploy_DW.turn_dist += 0.086393797973719308;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        } else if (((MicroMouse_Deploy_M->Timing.t[0] -
                     MicroMouse_Deploy_DW.previousTime) +
                    MicroMouse_Deploy_DW.temporalCounter_i1 >= 2.0) &&
                   MicroMouse_Deploy_B.TOF_LEFT && MicroMouse_Deploy_B.TOF_RIGHT
                   && MicroMouse_Deploy_B.TOF_FRONT) {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Dep_IN_Full_rotation;
          MicroMouse_Deploy_B.Z_angle_f -= 3.1415926535897931;
          MicroMouse_Deploy_DW.turn_dist = Gain1;
          MicroMouse_Deploy_DW.turn_dist += 0.15707963267948966;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse_D_IN_left_turn_setup:
        if (MicroMouse_Deploy_B.TOF_LEFT) {
          MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
          MicroMouse_Deploy_B.LW_c = 0.0;
          MicroMouse_Deploy_B.RW_d = 0.0;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        } else if ((Gain1 >= MicroMouse_Deploy_DW.end_dist) ||
                   (MicroMouse_Deploy_B.TOF_FRONT &&
                    (!MicroMouse_Deploy_B.TOF_LEFT))) {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Left_turn1;
          MicroMouse_Deploy_B.Z_angle_f += 1.5707963267948966;
          MicroMouse_Deploy_DW.turn_dist = Gain1;
          MicroMouse_Deploy_DW.turn_dist += 0.086393797973719308;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse__IN_left_turn_setup1:
        if (MicroMouse_Deploy_B.TOF_LEFT) {
          MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
          MicroMouse_Deploy_B.LW_c = 0.0;
          MicroMouse_Deploy_B.RW_d = 0.0;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        } else if ((Gain1 >= MicroMouse_Deploy_DW.end_dist) ||
                   (MicroMouse_Deploy_B.TOF_FRONT &&
                    (!MicroMouse_Deploy_B.TOF_LEFT))) {
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Left_turn2;
          MicroMouse_Deploy_B.Z_angle_f += 1.5707963267948966;
          MicroMouse_Deploy_DW.turn_dist = Gain1;
          MicroMouse_Deploy_DW.turn_dist += 0.086393797973719308;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       case MicroMouse_Deploy_IN_right_turn:
        if ((MicroMouse_Deploy_X.Integrator_CSTATE <=
             MicroMouse_Deploy_B.Z_angle_f) && (Gain1 >=
             MicroMouse_Deploy_DW.turn_dist)) {
          MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
          MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
          MicroMouse_Deploy_B.LW_c = 0.0;
          MicroMouse_Deploy_B.RW_d = 0.0;
          MicroMouse_Deploy_B.turn_indicator_i = 0.0;
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&MicroMouse_Deploy_M->solverInfo, true);
        }
        break;

       default:
        /* case IN_set_Z_angle: */
        MicroMouse_Deploy_DW.temporalCounter_i1 = 0.0;
        MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
          MicroMouse_Deploy_IN_Start_p;
        MicroMouse_Deploy_B.LW_c = 0.0;
        MicroMouse_Deploy_B.RW_d = 0.0;
        MicroMouse_Deploy_B.turn_indicator_i = 0.0;
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&MicroMouse_Deploy_M->solverInfo, true);
        break;
      }
    }
  }

  switch (MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy) {
   case MicroMouse_Dep_IN_Full_rotation:
    MicroMouse_Deploy_B.LW_c = 1.0;
    MicroMouse_Deploy_B.RW_d = -1.0;
    break;

   case MicroMouse_Deplo_IN_Go_straight:
    MicroMouse_Deploy_B.LW_c = 1.0;
    MicroMouse_Deploy_B.RW_d = 1.0;
    break;

   case MicroMouse_Depl_IN_Go_straight1:
    MicroMouse_Deploy_B.LW_c = 1.0;
    MicroMouse_Deploy_B.RW_d = 1.0;
    break;

   case MicroMouse_Deploy_IN_Left_turn:
    MicroMouse_Deploy_B.LW_c = -1.0;
    MicroMouse_Deploy_B.RW_d = 1.0;
    break;

   case MicroMouse_Deploy_IN_Left_turn1:
    MicroMouse_Deploy_B.LW_c = -1.0;
    MicroMouse_Deploy_B.RW_d = 1.0;
    break;

   case MicroMouse_Deploy_IN_Left_turn2:
    MicroMouse_Deploy_B.LW_c = -1.0;
    MicroMouse_Deploy_B.RW_d = 1.0;
    break;

   case MicroMouse_Deploy_IN_Start_p:
   case MicroMouse_D_IN_left_turn_setup:
   case MicroMouse__IN_left_turn_setup1:
    break;

   case MicroMouse_Deploy_IN_right_turn:
    MicroMouse_Deploy_B.LW_c = 1.0;
    MicroMouse_Deploy_B.RW_d = -1.0;
    break;

   default:
    /* case IN_set_Z_angle: */
    break;
  }

  /* End of Chart: '<S3>/draft2_Maze_exploration' */

  /* Logic: '<S22>/NOT' */
  rtb_NOT = !(MicroMouse_Deploy_B.turn_indicator_i != 0.0);
  if (tmp) {
    /* Chart: '<S22>/Chart' */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c9_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c9_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c9_MicroMouse_Deploy =
        MicroMou_IN_distance_corrcetion;
    } else if (MicroMouse_Deploy_DW.bitsForTID1.is_c9_MicroMouse_Deploy ==
               MicroMouse_Deploy_IN_after_turn) {
      if (MicroMouse_Deploy_B.turn_indicator_i == 1.0) {
        MicroMouse_Deploy_DW.bitsForTID1.is_c9_MicroMouse_Deploy =
          MicroMou_IN_distance_corrcetion;
      } else if (Product_g > Product) {
        MicroMouse_Deploy_B.Right_distance = MicroMouse_Deploy_DW.difference;
      } else if (Product > Product_g) {
        MicroMouse_Deploy_B.Left_distance = MicroMouse_Deploy_DW.difference;
      }

      /* case IN_distance_corrcetion: */
    } else if (MicroMouse_Deploy_B.turn_indicator_i == 0.0) {
      MicroMouse_Deploy_DW.difference = Product - Product_g;
      MicroMouse_Deploy_DW.difference = fabs(MicroMouse_Deploy_DW.difference);
      MicroMouse_Deploy_DW.bitsForTID1.is_c9_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_after_turn;
    }

    /* End of Chart: '<S22>/Chart' */

    /* RelationalOperator: '<S22>/Relational Operator' incorporates:
     *  Constant: '<S22>/Constant1'
     */
    MicroMouse_Deploy_B.RelationalOperator = (MicroMouse_Deploy_B.Saturation_m <=
      MicroMouse_Deploy_P.Constant1_Value_a);
  }

  /* Product: '<S22>/Product2' incorporates:
   *  Gain: '<S22>/Gain5'
   *  Gain: '<S22>/Gain6'
   *  Sum: '<S22>/Add2'
   *  Sum: '<S22>/Sum'
   *  Sum: '<S22>/Sum1'
   */
  Product = ((Product_g + MicroMouse_Deploy_B.Left_distance) *
             MicroMouse_Deploy_P.Gain5_Gain - (Product +
              MicroMouse_Deploy_B.Right_distance) *
             MicroMouse_Deploy_P.Gain6_Gain) * (real_T)rtb_NOT;

  /* Gain: '<S273>/Filter Coefficient' incorporates:
   *  Gain: '<S263>/Derivative Gain'
   *  Integrator: '<S265>/Filter'
   *  Sum: '<S265>/SumD'
   */
  MicroMouse_Deploy_B.FilterCoefficient = (MicroMouse_Deploy_P.PIDController1_D *
    Product - MicroMouse_Deploy_X.Filter_CSTATE) *
    MicroMouse_Deploy_P.PIDController1_N;

  /* Sum: '<S279>/Sum' incorporates:
   *  Gain: '<S275>/Proportional Gain'
   *  Integrator: '<S270>/Integrator'
   */
  rtb_Sum2_j = (MicroMouse_Deploy_P.PIDController1_P * Product +
                MicroMouse_Deploy_X.Integrator_CSTATE_o) +
    MicroMouse_Deploy_B.FilterCoefficient;

  /* Gain: '<S22>/Gain1' incorporates:
   *  Product: '<S22>/Product3'
   */
  rtb_Gain1_h = (real32_T)rtb_NOT * MicroMouse_Deploy_B.Saturation_m * (real32_T)
    MicroMouse_Deploy_B.RelationalOperator * MicroMouse_Deploy_P.Gain1_Gain_a;
  if (tmp) {
    /* RelationalOperator: '<S22>/Relational Operator1' incorporates:
     *  Constant: '<S22>/Constant'
     */
    MicroMouse_Deploy_B.RelationalOperator1 = (MicroMouse_Deploy_B.Saturation <=
      MicroMouse_Deploy_P.Constant_Value);

    /* RelationalOperator: '<S22>/Relational Operator4' incorporates:
     *  Constant: '<S22>/Constant4'
     */
    MicroMouse_Deploy_B.RelationalOperator4 = (MicroMouse_Deploy_B.Saturation_m <=
      MicroMouse_Deploy_P.Constant4_Value);

    /* RelationalOperator: '<S22>/Relational Operator2' incorporates:
     *  Constant: '<S22>/Constant2'
     */
    MicroMouse_Deploy_B.RelationalOperator2 = (MicroMouse_Deploy_B.Saturation_m >=
      MicroMouse_Deploy_P.Constant2_Value_a);
  }

  /* Gain: '<S22>/Gain2' incorporates:
   *  Product: '<S22>/Product1'
   */
  rtb_Gain2_b = MicroMouse_Deploy_B.Saturation * (real32_T)rtb_NOT * (real32_T)
    MicroMouse_Deploy_B.RelationalOperator1 * MicroMouse_Deploy_P.Gain2_Gain_k;

  /* Gain: '<S22>/Gain3' incorporates:
   *  Constant: '<S22>/Constant2'
   *  Product: '<S22>/Product4'
   */
  Gain1 = (MicroMouse_Deploy_B.RelationalOperator4 ? (real_T)
           MicroMouse_Deploy_B.RelationalOperator2 : 0.0) *
    MicroMouse_Deploy_P.Constant2_Value_a * (real_T)rtb_NOT *
    MicroMouse_Deploy_P.Gain3_Gain;
  if (tmp) {
    /* RelationalOperator: '<S22>/Relational Operator3' incorporates:
     *  Constant: '<S22>/Constant3'
     */
    MicroMouse_Deploy_B.RelationalOperator3 = (MicroMouse_Deploy_B.Saturation >=
      MicroMouse_Deploy_P.Constant3_Value);

    /* RelationalOperator: '<S22>/Relational Operator5' incorporates:
     *  Constant: '<S22>/Constant5'
     */
    MicroMouse_Deploy_B.RelationalOperator5 = (MicroMouse_Deploy_B.Saturation <=
      MicroMouse_Deploy_P.Constant5_Value);
  }

  /* Gain: '<S22>/Gain4' incorporates:
   *  Product: '<S22>/Product5'
   */
  rtb_Gain4 = (real32_T)rtb_NOT * MicroMouse_Deploy_B.Saturation * (real32_T)
    MicroMouse_Deploy_B.RelationalOperator3 * (real32_T)
    MicroMouse_Deploy_B.RelationalOperator5 * MicroMouse_Deploy_P.Gain4_Gain;

  /* DataTypeConversion: '<S1>/Cast1' incorporates:
   *  Constant: '<S22>/constant'
   *  DataStoreWrite: '<S1>/Data Store Write1'
   *  Gain: '<Root>/Gain1'
   *  Product: '<S3>/Product'
   *  Sum: '<S22>/Add4'
   *  Sum: '<S22>/Add5'
   */
  Product_g = floor((((((MicroMouse_Deploy_P.constant_Value + rtb_Sum2_j) +
                        rtb_Gain1_h) - rtb_Gain2_b) - Gain1) + rtb_Gain4) *
                    MicroMouse_Deploy_B.RW_d * MicroMouse_Deploy_P.Gain1_Gain_d);
  if (rtIsNaN(Product_g) || rtIsInf(Product_g)) {
    Product_g = 0.0;
  } else {
    Product_g = fmod(Product_g, 256.0);
  }

  MOTOR_RS = (int8_T)(Product_g < 0.0 ? (int32_T)(int8_T)-(int8_T)(uint8_T)
                      -Product_g : (int32_T)(int8_T)(uint8_T)Product_g);

  /* End of DataTypeConversion: '<S1>/Cast1' */

  /* DataTypeConversion: '<S1>/Cast' incorporates:
   *  Constant: '<S22>/constant'
   *  DataStoreWrite: '<S1>/Data Store Write'
   *  Gain: '<Root>/Gain'
   *  Product: '<S3>/Product1'
   *  Sum: '<S22>/Add1'
   *  Sum: '<S22>/Add3'
   */
  Product_g = floor((((((MicroMouse_Deploy_P.constant_Value - rtb_Sum2_j) -
                        rtb_Gain1_h) + rtb_Gain2_b) + Gain1) - rtb_Gain4) *
                    MicroMouse_Deploy_B.LW_c * MicroMouse_Deploy_P.Gain_Gain);
  if (rtIsNaN(Product_g) || rtIsInf(Product_g)) {
    Product_g = 0.0;
  } else {
    Product_g = fmod(Product_g, 256.0);
  }

  MOTOR_LS = (int8_T)(Product_g < 0.0 ? (int32_T)(int8_T)-(int8_T)(uint8_T)
                      -Product_g : (int32_T)(int8_T)(uint8_T)Product_g);

  /* End of DataTypeConversion: '<S1>/Cast' */

  /* DataTypeConversion: '<S3>/Cast To Double' incorporates:
   *  DataStoreRead: '<S5>/Data Store Read1'
   */
  MicroMouse_Deploy_B.CastToDouble[0] = IMU_Gyro[0];
  MicroMouse_Deploy_B.CastToDouble[1] = IMU_Gyro[1];
  MicroMouse_Deploy_B.CastToDouble[2] = IMU_Gyro[2];

  /* RelationalOperator: '<S37>/Compare' incorporates:
   *  Constant: '<S37>/Constant'
   */
  MicroMouse_Deploy_B.Compare_n = ((int32_T)rtb_Compare_c <= (int32_T)
    MicroMouse_Deploy_P.Constant_Value_o);
  if (tmp) {
    /* ComposeString: '<S2>/Compose String' incorporates:
     *  StringConstant: '<S3>/OLED_STRING1'
     */
    snprintf(&rtb_ComposeString4_0[0], 256U, "%-18s",
             &MicroMouse_Deploy_P.OLED_STRING1_String[0]);

    /* StringToASCII: '<S2>/String to ASCII' incorporates:
     *  ComposeString: '<S2>/Compose String'
     */
    strncpy((char_T *)&oled_string1[0], &rtb_ComposeString4_0[0], 18U);

    /* ComposeString: '<S2>/Compose String1' incorporates:
     *  StringConstant: '<S3>/OLED_STRING2'
     */
    snprintf(&rtb_ComposeString4_0[0], 256U, "%-18s",
             &MicroMouse_Deploy_P.OLED_STRING2_String[0]);

    /* StringToASCII: '<S2>/String to ASCII1' incorporates:
     *  ComposeString: '<S2>/Compose String1'
     */
    strncpy((char_T *)&oled_string2[0], &rtb_ComposeString4_0[0], 18U);

    /* ComposeString: '<S3>/Compose String' incorporates:
     *  Saturate: '<S60>/Saturation'
     *  Saturate: '<S67>/Saturation'
     *  Saturate: '<S74>/Saturation'
     */
    snprintf(&rtb_ComposeString_0[0], 256U, "L%1.2f F%1.2f R%1.2f",
             MicroMouse_Deploy_B.Saturation, MicroMouse_Deploy_B.Saturation_d,
             MicroMouse_Deploy_B.Saturation_m);

    /* ComposeString: '<S2>/Compose String2' incorporates:
     *  ComposeString: '<S3>/Compose String'
     */
    snprintf(&rtb_ComposeString4_0[0], 256U, "%-18s", &rtb_ComposeString_0[0]);

    /* StringToASCII: '<S2>/String to ASCII2' incorporates:
     *  ComposeString: '<S2>/Compose String2'
     */
    strncpy((char_T *)&oled_string3[0], &rtb_ComposeString4_0[0], 18U);

    /* ComposeString: '<S3>/Compose String1' incorporates:
     *  DataStoreRead: '<S5>/Data Store Read'
     */
    snprintf(&rtb_ComposeString1_0[0], 256U, "X%2.1f  Y%2.1f  Z%2.1f",
             IMU_Accel[0], IMU_Accel[1], IMU_Accel[2]);

    /* ComposeString: '<S2>/Compose String3' incorporates:
     *  ComposeString: '<S3>/Compose String1'
     */
    snprintf(&rtb_ComposeString4_0[0], 256U, "%-18s", &rtb_ComposeString1_0[0]);

    /* StringToASCII: '<S2>/String to ASCII3' incorporates:
     *  ComposeString: '<S2>/Compose String3'
     */
    strncpy((char_T *)&oled_string4[0], &rtb_ComposeString4_0[0], 18U);

    /* ComposeString: '<S3>/Compose String2' incorporates:
     *  DataStoreRead: '<S8>/Data Store Read'
     *  DataStoreRead: '<S8>/Data Store Read2'
     *  DataStoreRead: '<S8>/Data Store Read4'
     *  DataTypeConversion: '<S8>/Cast To Single'
     *  DataTypeConversion: '<S8>/Cast To Single2'
     *  DataTypeConversion: '<S8>/Cast To Single4'
     *  Gain: '<S3>/Gain2'
     */
    snprintf(&rtb_ComposeString2_0[0], 256U, "%1.2fV  %3.0fmA  %2.0f%%",
             MicroMouse_Deploy_P.Gain2_Gain_f * (real32_T)Vbattery, (real32_T)
             Current, (real32_T)batteryLife);

    /* ComposeString: '<S2>/Compose String4' incorporates:
     *  ComposeString: '<S3>/Compose String2'
     */
    snprintf(&rtb_ComposeString4_0[0], 256U, "%-18s", &rtb_ComposeString2_0[0]);

    /* StringToASCII: '<S2>/String to ASCII4' incorporates:
     *  ComposeString: '<S2>/Compose String4'
     */
    strncpy((char_T *)&oled_string5[0], &rtb_ComposeString4_0[0], 18U);

    /* DataTypeConversion: '<S6>/Cast To Boolean' incorporates:
     *  DataStoreWrite: '<S6>/Data Store Write'
     */
    LED[0] = MicroMouse_Deploy_B.TOF_LEFT;
    LED[1] = MicroMouse_Deploy_B.TOF_FRONT;
    LED[2] = MicroMouse_Deploy_B.TOF_RIGHT;

    /* DataTypeConversion: '<S9>/Cast To Boolean' incorporates:
     *  Constant: '<S3>/Constant'
     *  DataStoreWrite: '<S9>/Data Store Write'
     */
    Product_g = floor(MicroMouse_Deploy_P.Constant_Value_l);
    if (rtIsNaN(Product_g) || rtIsInf(Product_g)) {
      Product_g = 0.0;
    } else {
      Product_g = fmod(Product_g, 256.0);
    }

    STATE = (uint8_T)(Product_g < 0.0 ? (int32_T)(uint8_T)-(int8_T)(uint8_T)
                      -Product_g : (int32_T)(uint8_T)Product_g);

    /* End of DataTypeConversion: '<S9>/Cast To Boolean' */

    /* Chart: '<S27>/Chart' incorporates:
     *  RelationalOperator: '<S35>/FixPt Relational Operator'
     *  UnitDelay: '<S35>/Delay Input1'
     *
     * Block description for '<S35>/Delay Input1':
     *
     *  Store in Global RAM
     */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c7_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c7_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c7_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_default;
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID1.is_c7_MicroMouse_Deploy) {
       case MicroMouse_Deploy_IN_default:
       case MicroMouse_IN_encoder_pulse_end:
        break;

       default:
        /* case IN_encoder_pulse_start: */
        if ((int32_T)MicroMouse_Deploy_B.Compare_n > (int32_T)
            MicroMouse_Deploy_DW.DelayInput1_DSTATE_e) {
          MicroMouse_Deploy_DW.bitsForTID1.is_c7_MicroMouse_Deploy =
            MicroMouse_IN_encoder_pulse_end;
        }
        break;
      }
    }

    /* End of Chart: '<S27>/Chart' */

    /* MinMax: '<S54>/Max' incorporates:
     *  Constant: '<S54>/Time constant'
     */
    rtb_Gain1_h = (real32_T)fmax(MicroMouse_Deploy_B.Probe[0],
      MicroMouse_Deploy_P.LowPassFilterDiscreteorContinuo);

    /* Fcn: '<S54>/Avoid Divide by Zero' */
    MicroMouse_Deploy_B.AvoidDividebyZero = (real32_T)(rtb_Gain1_h == 0.0F) *
      2.22044605e-16F + rtb_Gain1_h;

    /* MinMax: '<S61>/Max' incorporates:
     *  Constant: '<S61>/Time constant'
     */
    rtb_Gain1_h = (real32_T)fmax(MicroMouse_Deploy_B.Probe_h[0],
      MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_p);

    /* Fcn: '<S61>/Avoid Divide by Zero' */
    MicroMouse_Deploy_B.AvoidDividebyZero_k = (real32_T)(rtb_Gain1_h == 0.0F) *
      2.22044605e-16F + rtb_Gain1_h;

    /* MinMax: '<S68>/Max' incorporates:
     *  Constant: '<S68>/Time constant'
     */
    rtb_Gain1_h = (real32_T)fmax(MicroMouse_Deploy_B.Probe_g[0],
      MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_e);

    /* Fcn: '<S68>/Avoid Divide by Zero' */
    MicroMouse_Deploy_B.AvoidDividebyZero_n = (real32_T)(rtb_Gain1_h == 0.0F) *
      2.22044605e-16F + rtb_Gain1_h;

    /* RelationalOperator: '<S3>/Relational Operator' incorporates:
     *  Constant: '<S3>/Constant3'
     *  Constant: '<S3>/Motor_Left2'
     */
    MicroMouse_Deploy_B.RelationalOperator_n =
      (MicroMouse_Deploy_P.Motor_Left2_Value ==
       MicroMouse_Deploy_P.Constant3_Value_g);
  }

  /* Sum: '<S3>/Error' incorporates:
   *  Constant: '<S3>/Constant4'
   *  Integrator: '<S3>/Integrator1'
   */
  elapsedTime = MicroMouse_Deploy_P.Constant4_Value_d -
    MicroMouse_Deploy_X.Integrator1_CSTATE;

  /* Product: '<S13>/1//T' incorporates:
   *  Sum: '<S13>/Sum1'
   */
  MicroMouse_Deploy_B.uT = 1.0F / MicroMouse_Deploy_B.AvoidDividebyZero * (K -
    MicroMouse_Deploy_B.Saturation);

  /* Product: '<S14>/1//T' incorporates:
   *  Sum: '<S14>/Sum1'
   */
  MicroMouse_Deploy_B.uT_p = 1.0F / MicroMouse_Deploy_B.AvoidDividebyZero_k *
    (K_p - MicroMouse_Deploy_B.Saturation_d);

  /* Product: '<S15>/1//T' incorporates:
   *  Sum: '<S15>/Sum1'
   */
  MicroMouse_Deploy_B.uT_e = 1.0F / MicroMouse_Deploy_B.AvoidDividebyZero_n *
    (K_e - MicroMouse_Deploy_B.Saturation_m);

  /* Gain: '<S106>/Integral Gain' */
  MicroMouse_Deploy_B.IntegralGain = MicroMouse_Deploy_P.PIDController_I *
    elapsedTime;

  /* Gain: '<S112>/Filter Coefficient' incorporates:
   *  Gain: '<S102>/Derivative Gain'
   *  Integrator: '<S104>/Filter'
   *  Sum: '<S104>/SumD'
   */
  MicroMouse_Deploy_B.FilterCoefficient_k = (MicroMouse_Deploy_P.PIDController_D
    * elapsedTime - MicroMouse_Deploy_X.Filter_CSTATE_k) *
    MicroMouse_Deploy_P.PIDController_N;

  /* Product: '<S3>/Product3' incorporates:
   *  Gain: '<S114>/Proportional Gain'
   *  Integrator: '<S109>/Integrator'
   *  Sum: '<S118>/Sum'
   */
  Product_g = ((MicroMouse_Deploy_P.PIDController_P * elapsedTime +
                MicroMouse_Deploy_X.Integrator_CSTATE_e) +
               MicroMouse_Deploy_B.FilterCoefficient_k) * (real_T)
    MicroMouse_Deploy_B.RelationalOperator_n;

  /* Clock: '<S18>/Clock' */
  Gain1 = MicroMouse_Deploy_M->Timing.t[0];
  if (tmp) {
    /* Chart: '<S18>/Chart' incorporates:
     *  Integrator: '<S18>/Integrator'
     */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c2_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c2_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c2_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_Start;
    } else if (MicroMouse_Deploy_DW.bitsForTID1.is_c2_MicroMouse_Deploy ==
               MicroMouse_Deploy_IN_Set_Offset) {
      if (Gain1 > 0.1) {
        MicroMouse_Deploy_DW.bitsForTID1.is_c2_MicroMouse_Deploy =
          MicroMouse_Deploy_IN_Set_Offset;
      } else {
        MicroMouse_Deploy_B.offset = MicroMouse_Deploy_X.Integrator_CSTATE_j;
      }

      /* case IN_Start: */
    } else if (Gain1 < 0.1) {
      MicroMouse_Deploy_DW.bitsForTID1.is_c2_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_Set_Offset;
    } else {
      MicroMouse_Deploy_B.offset = 0.0;
    }

    /* End of Chart: '<S18>/Chart' */
  }

  /* Sum: '<S18>/Error' incorporates:
   *  Integrator: '<S18>/Integrator'
   */
  elapsedTime = MicroMouse_Deploy_B.offset -
    MicroMouse_Deploy_X.Integrator_CSTATE_j;

  /* Gain: '<S219>/Filter Coefficient' incorporates:
   *  Gain: '<S209>/Derivative Gain'
   *  Integrator: '<S211>/Filter'
   *  Sum: '<S211>/SumD'
   */
  MicroMouse_Deploy_B.FilterCoefficient_f =
    (MicroMouse_Deploy_P.PIDController_D_e * elapsedTime -
     MicroMouse_Deploy_X.Filter_CSTATE_l) *
    MicroMouse_Deploy_P.PIDController_N_j;

  /* Sum: '<S225>/Sum' incorporates:
   *  Gain: '<S221>/Proportional Gain'
   */
  Gain1 = MicroMouse_Deploy_P.PIDController_P_c * elapsedTime +
    MicroMouse_Deploy_B.FilterCoefficient_f;
  if (tmp) {
    /* Chart: '<S18>/turn_adjus' */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c1_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c1_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_deafult;
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy) {
       case MicroMouse_Deploy_IN_deafult:
        if (Gain1 < 0.0) {
          MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_turnRight;
        } else if (Gain1 > 0.0) {
          MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_turnLeft;
        }
        break;

       case MicroMouse_Deploy_IN_turnLeft:
        break;

       default:
        /* case IN_turnRight: */
        break;
      }
    }

    /* End of Chart: '<S18>/turn_adjus' */

    /* UnitDelay: '<S289>/Unit Delay1' */
    MicroMouse_Deploy_B.UnitDelay1 = MicroMouse_Deploy_DW.UnitDelay1_DSTATE;

    /* Gain: '<S289>/tau' incorporates:
     *  Constant: '<S3>/Constant5'
     */
    rtb_p = MicroMouse_Deploy_P.tau_Gain * MicroMouse_Deploy_P.Constant5_Value_n;

    /* Product: '<S291>/Product2' */
    rtb_q = rtb_p * rtb_p;

    /* Bias: '<S291>/Bias' */
    rtb_r = rtb_q + MicroMouse_Deploy_P.Bias_Bias;

    /* Gain: '<S291>/Gain' */
    rtb_p *= MicroMouse_Deploy_P.Gain_Gain_n;

    /* Sum: '<S291>/Add1' */
    rtb_d = rtb_r + rtb_p;

    /* Product: '<S291>/Product3' */
    MicroMouse_Deploy_B.Product3_k = rtb_q / rtb_d;

    /* UnitDelay: '<S290>/Unit Delay1' */
    MicroMouse_Deploy_B.UnitDelay1_n = MicroMouse_Deploy_DW.UnitDelay1_DSTATE_n;

    /* Gain: '<S290>/tau' incorporates:
     *  Constant: '<S3>/Constant5'
     */
    rtb_p_a = MicroMouse_Deploy_P.tau_Gain_b *
      MicroMouse_Deploy_P.Constant5_Value_n;

    /* Product: '<S292>/Product2' */
    rtb_q_g = rtb_p_a * rtb_p_a;

    /* Bias: '<S292>/Bias' */
    rtb_r_p = rtb_q_g + MicroMouse_Deploy_P.Bias_Bias_b;

    /* Gain: '<S292>/Gain' */
    rtb_p_a *= MicroMouse_Deploy_P.Gain_Gain_l;

    /* Sum: '<S292>/Add1' */
    rtb_Product1_b = rtb_r_p + rtb_p_a;

    /* Product: '<S292>/Product3' */
    MicroMouse_Deploy_B.Product3_d = rtb_q_g / rtb_Product1_b;
  }

  /* Gain: '<S267>/Integral Gain' */
  MicroMouse_Deploy_B.IntegralGain_c = MicroMouse_Deploy_P.PIDController1_I *
    Product;

  /* Sum: '<S289>/Sum' */
  elapsedTime = MicroMouse_Deploy_B.CastToDouble[2] +
    MicroMouse_Deploy_B.UnitDelay1;

  /* Product: '<S289>/Product1' */
  Gain1 = elapsedTime * MicroMouse_Deploy_B.Product3_k;

  /* Sum: '<S290>/Sum' */
  Product = Gain1 + MicroMouse_Deploy_B.UnitDelay1_n;

  /* Product: '<S290>/Product1' */
  rtb_Sum2_j = Product * MicroMouse_Deploy_B.Product3_d;

  /* MATLABSystem: '<S23>/Median Filter' */
  if (MicroMouse_Deploy_DW.obj_o.pMID.isInitialized != 1) {
    MicroMouse_Deploy_DW.obj_o.pMID.isInitialized = 1;
    MicroMouse_Deploy_DW.obj_o.pMID.isSetupComplete = true;
    MicroMou_MedianFilter_resetImpl(&MicroMouse_Deploy_DW.obj_o.pMID);
  }

  vprev = MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
    MicroMouse_Deploy_DW.obj_o.pMID.pIdx - 1];
  MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
    MicroMouse_Deploy_DW.obj_o.pMID.pIdx - 1] = rtb_Sum2_j;
  p = MicroMouse_Deploy_DW.obj_o.pMID.pPos[(int32_T)
    MicroMouse_Deploy_DW.obj_o.pMID.pIdx - 1];
  MicroMouse_Deploy_DW.obj_o.pMID.pIdx++;
  if (MicroMouse_Deploy_DW.obj_o.pMID.pWinLen + 1.0 ==
      MicroMouse_Deploy_DW.obj_o.pMID.pIdx) {
    MicroMouse_Deploy_DW.obj_o.pMID.pIdx = 1.0;
  }

  if (p > MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap) {
    if (vprev < rtb_Sum2_j) {
      Mic_MedianFilter_trickleDownMin(&MicroMouse_Deploy_DW.obj_o.pMID, (p -
        MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap) * 2.0);
    } else {
      p -= MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap;
      exitg1 = false;
      while ((!exitg1) && (p > 0.0)) {
        rtb_Sum2_j = p / 2.0;
        temp = trunc(rtb_Sum2_j);
        if (!(MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
              MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)(p +
              MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap) - 1] - 1] <
              MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
              MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)(temp +
              MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap) - 1] - 1])) {
          exitg1 = true;
        } else {
          vprev = p + MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap;
          p = temp + MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap;
          temp = MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)vprev - 1];
          MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)vprev - 1] =
            MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)p - 1];
          MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)p - 1] = temp;
          MicroMouse_Deploy_DW.obj_o.pMID.pPos[(int32_T)
            MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)vprev - 1] - 1] =
            vprev;
          MicroMouse_Deploy_DW.obj_o.pMID.pPos[(int32_T)
            MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)p - 1] - 1] = p;
          p = trunc(rtb_Sum2_j);
        }
      }

      if (p == 0.0) {
        Mic_MedianFilter_trickleDownMax(&MicroMouse_Deploy_DW.obj_o.pMID, -1.0);
      }
    }
  } else if (p < MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap) {
    if (rtb_Sum2_j < vprev) {
      Mic_MedianFilter_trickleDownMax(&MicroMouse_Deploy_DW.obj_o.pMID, (p -
        MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap) * 2.0);
    } else {
      p -= MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap;
      exitg1 = false;
      while ((!exitg1) && (p < 0.0)) {
        rtb_Sum2_j = p / 2.0;
        temp = trunc(rtb_Sum2_j);
        if (!(MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
              MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)(temp +
              MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap) - 1] - 1] <
              MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
              MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)(p +
              MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap) - 1] - 1])) {
          exitg1 = true;
        } else {
          vprev = temp + MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap;
          p += MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap;
          temp = MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)vprev - 1];
          MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)vprev - 1] =
            MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)p - 1];
          MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)p - 1] = temp;
          MicroMouse_Deploy_DW.obj_o.pMID.pPos[(int32_T)
            MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)vprev - 1] - 1] =
            vprev;
          MicroMouse_Deploy_DW.obj_o.pMID.pPos[(int32_T)
            MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)p - 1] - 1] = p;
          p = trunc(rtb_Sum2_j);
        }
      }

      if (p == 0.0) {
        Mic_MedianFilter_trickleDownMin(&MicroMouse_Deploy_DW.obj_o.pMID, 1.0);
      }
    }
  } else {
    if (MicroMouse_Deploy_DW.obj_o.pMID.pMaxHeapLength != 0.0) {
      Mic_MedianFilter_trickleDownMax(&MicroMouse_Deploy_DW.obj_o.pMID, -1.0);
    }

    if (MicroMouse_Deploy_DW.obj_o.pMID.pMinHeapLength > 0.0) {
      Mic_MedianFilter_trickleDownMin(&MicroMouse_Deploy_DW.obj_o.pMID, 1.0);
    }
  }

  /* DeadZone: '<S23>/Dead Zone' incorporates:
   *  MATLABSystem: '<S23>/Median Filter'
   * */
  if (MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
      MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)
      MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap - 1] - 1] >
      MicroMouse_Deploy_P.DeadZone_End) {
    /* DeadZone: '<S23>/Dead Zone' */
    MicroMouse_Deploy_B.DeadZone = MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
      MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)
      MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap - 1] - 1] -
      MicroMouse_Deploy_P.DeadZone_End;
  } else if (MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
             MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)
             MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap - 1] - 1] >=
             MicroMouse_Deploy_P.DeadZone_Start) {
    /* DeadZone: '<S23>/Dead Zone' */
    MicroMouse_Deploy_B.DeadZone = 0.0;
  } else {
    /* DeadZone: '<S23>/Dead Zone' */
    MicroMouse_Deploy_B.DeadZone = MicroMouse_Deploy_DW.obj_o.pMID.pBuf[(int32_T)
      MicroMouse_Deploy_DW.obj_o.pMID.pHeap[(int32_T)
      MicroMouse_Deploy_DW.obj_o.pMID.pMidHeap - 1] - 1] -
      MicroMouse_Deploy_P.DeadZone_Start;
  }

  /* End of DeadZone: '<S23>/Dead Zone' */
  if (tmp) {
    /* Product: '<S291>/Product4' incorporates:
     *  Bias: '<S291>/Bias1'
     *  UnaryMinus: '<S291>/Unary Minus'
     */
    MicroMouse_Deploy_B.Product4 = (-rtb_q + MicroMouse_Deploy_P.Bias1_Bias) /
      rtb_d;

    /* Product: '<S291>/Product5' incorporates:
     *  Sum: '<S291>/Add3'
     */
    MicroMouse_Deploy_B.Product5 = 1.0 / rtb_d * (rtb_r - rtb_p);

    /* UnitDelay: '<S289>/Unit Delay2' */
    MicroMouse_Deploy_B.UnitDelay2 = MicroMouse_Deploy_DW.UnitDelay2_DSTATE;

    /* Product: '<S292>/Product4' incorporates:
     *  Bias: '<S292>/Bias1'
     *  UnaryMinus: '<S292>/Unary Minus'
     */
    MicroMouse_Deploy_B.Product4_k = (-rtb_q_g +
      MicroMouse_Deploy_P.Bias1_Bias_f) / rtb_Product1_b;

    /* Product: '<S292>/Product5' incorporates:
     *  Sum: '<S292>/Add3'
     */
    MicroMouse_Deploy_B.Product5_a = 1.0 / rtb_Product1_b * (rtb_r_p - rtb_p_a);

    /* UnitDelay: '<S290>/Unit Delay2' */
    MicroMouse_Deploy_B.UnitDelay2_b = MicroMouse_Deploy_DW.UnitDelay2_DSTATE_j;
  }

  /* Sum: '<S289>/Sum1' incorporates:
   *  Gain: '<S289>/Gain1'
   *  Product: '<S289>/Product5'
   *  Sum: '<S289>/Sum2'
   */
  MicroMouse_Deploy_B.Sum1 = (elapsedTime * MicroMouse_Deploy_B.Product4 +
    MicroMouse_Deploy_B.CastToDouble[2]) * MicroMouse_Deploy_P.Gain1_Gain_b +
    MicroMouse_Deploy_B.UnitDelay2;

  /* Sum: '<S289>/Sum3' incorporates:
   *  Product: '<S289>/Product2'
   */
  MicroMouse_Deploy_B.Sum3 = MicroMouse_Deploy_B.CastToDouble[2] - elapsedTime *
    MicroMouse_Deploy_B.Product5;

  /* Sum: '<S290>/Sum1' incorporates:
   *  Gain: '<S290>/Gain1'
   *  Product: '<S290>/Product5'
   *  Sum: '<S290>/Sum2'
   */
  MicroMouse_Deploy_B.Sum1_k = (Product * MicroMouse_Deploy_B.Product4_k + Gain1)
    * MicroMouse_Deploy_P.Gain1_Gain_h + MicroMouse_Deploy_B.UnitDelay2_b;

  /* Sum: '<S290>/Sum3' incorporates:
   *  Product: '<S290>/Product2'
   */
  MicroMouse_Deploy_B.Sum3_o = Gain1 - Product * MicroMouse_Deploy_B.Product5_a;
  if (tmp) {
    /* Chart: '<S3>/turn_adjus' */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c5_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c5_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c5_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_deafult;
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID1.is_c5_MicroMouse_Deploy) {
       case MicroMouse_Deploy_IN_deafult:
        if (Product_g < 0.0) {
          MicroMouse_Deploy_DW.bitsForTID1.is_c5_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_turnRight;
        } else if (Product_g > 0.0) {
          MicroMouse_Deploy_DW.bitsForTID1.is_c5_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_turnLeft;
        }
        break;

       case MicroMouse_Deploy_IN_turnLeft:
        break;

       default:
        /* case IN_turnRight: */
        break;
      }
    }

    /* End of Chart: '<S3>/turn_adjus' */

    /* RelationalOperator: '<S47>/Compare' incorporates:
     *  Constant: '<S45>/Constant'
     *  Constant: '<S47>/Constant'
     *  RelationalOperator: '<S45>/Compare'
     */
    rtb_Compare = (uint8_T)((MicroMouse_Deploy_P.CompareToConstant_const_g <=
      0.0F) > (int32_T)MicroMouse_Deploy_P.Constant_Value_f);

    /* Sum: '<S39>/Add' incorporates:
     *  Delay: '<S39>/Delay'
     *  RelationalOperator: '<S46>/FixPt Relational Operator'
     *  UnitDelay: '<S46>/Delay Input1'
     *
     * Block description for '<S46>/Delay Input1':
     *
     *  Store in Global RAM
     */
    MicroMouse_Deploy_DW.Delay_DSTATE += (real_T)(rtb_Compare >
      MicroMouse_Deploy_DW.DelayInput1_DSTATE_b);

    /* RelationalOperator: '<S49>/Compare' incorporates:
     *  Constant: '<S49>/Constant'
     */
    rtb_Compare_b = (MicroMouse_Deploy_P.CompareToConstant_const_l <= 0.0F);

    /* RelationalOperator: '<S53>/Compare' incorporates:
     *  Constant: '<S53>/Constant'
     */
    rtb_Compare_g = (uint8_T)((int32_T)rtb_Compare_b > (int32_T)
      MicroMouse_Deploy_P.Constant_Value_fp);

    /* Sum: '<S40>/Add' incorporates:
     *  Delay: '<S40>/Delay'
     *  RelationalOperator: '<S51>/FixPt Relational Operator'
     *  UnitDelay: '<S51>/Delay Input1'
     *
     * Block description for '<S51>/Delay Input1':
     *
     *  Store in Global RAM
     */
    MicroMouse_Deploy_DW.Delay_DSTATE_k += (real_T)(rtb_Compare_g >
      MicroMouse_Deploy_DW.DelayInput1_DSTATE_k);
  }

  /* RateLimiter: '<S12>/Rate Limiter1' incorporates:
   *  Delay: '<S39>/Delay'
   */
  if (MicroMouse_Deploy_DW.LastMajorTime_c == (rtInf)) {
    /* RateLimiter: '<S12>/Rate Limiter1' incorporates:
     *  Delay: '<S39>/Delay'
     */
    MicroMouse_Deploy_B.RateLimiter1 = MicroMouse_Deploy_DW.Delay_DSTATE;
  } else {
    Gain1 = MicroMouse_Deploy_M->Timing.t[0];
    Product_g = Gain1 - MicroMouse_Deploy_DW.LastMajorTime_c;
    if (MicroMouse_Deploy_DW.LastMajorTime_c == Gain1) {
      if (MicroMouse_Deploy_DW.PrevLimited_e) {
        /* RateLimiter: '<S12>/Rate Limiter1' */
        MicroMouse_Deploy_B.RateLimiter1 = MicroMouse_Deploy_DW.PrevY_l;
      } else {
        /* RateLimiter: '<S12>/Rate Limiter1' incorporates:
         *  Delay: '<S39>/Delay'
         */
        MicroMouse_Deploy_B.RateLimiter1 = MicroMouse_Deploy_DW.Delay_DSTATE;
      }
    } else {
      elapsedTime = Product_g * MicroMouse_Deploy_P.RateLimiter1_RisingLim;
      Gain1 = MicroMouse_Deploy_DW.Delay_DSTATE - MicroMouse_Deploy_DW.PrevY_l;
      if (Gain1 > elapsedTime) {
        /* RateLimiter: '<S12>/Rate Limiter1' */
        MicroMouse_Deploy_B.RateLimiter1 = MicroMouse_Deploy_DW.PrevY_l +
          elapsedTime;
        rtb_NOT = true;
      } else {
        Product_g *= MicroMouse_Deploy_P.RateLimiter1_FallingLim;
        if (Gain1 < Product_g) {
          /* RateLimiter: '<S12>/Rate Limiter1' */
          MicroMouse_Deploy_B.RateLimiter1 = MicroMouse_Deploy_DW.PrevY_l +
            Product_g;
          rtb_NOT = true;
        } else {
          /* RateLimiter: '<S12>/Rate Limiter1' */
          MicroMouse_Deploy_B.RateLimiter1 = MicroMouse_Deploy_DW.Delay_DSTATE;
          rtb_NOT = false;
        }
      }

      if (tmp_0) {
        MicroMouse_Deploy_DW.PrevLimited_e = rtb_NOT;
      }
    }
  }

  /* RateLimiter: '<S12>/Rate Limiter2' incorporates:
   *  Delay: '<S40>/Delay'
   */
  if (MicroMouse_Deploy_DW.LastMajorTime_k == (rtInf)) {
    /* RateLimiter: '<S12>/Rate Limiter2' incorporates:
     *  Delay: '<S40>/Delay'
     */
    MicroMouse_Deploy_B.RateLimiter2_j = MicroMouse_Deploy_DW.Delay_DSTATE_k;
  } else {
    Gain1 = MicroMouse_Deploy_M->Timing.t[0];
    Product_g = Gain1 - MicroMouse_Deploy_DW.LastMajorTime_k;
    if (MicroMouse_Deploy_DW.LastMajorTime_k == Gain1) {
      if (MicroMouse_Deploy_DW.PrevLimited_l) {
        /* RateLimiter: '<S12>/Rate Limiter2' */
        MicroMouse_Deploy_B.RateLimiter2_j = MicroMouse_Deploy_DW.PrevY_b;
      } else {
        /* RateLimiter: '<S12>/Rate Limiter2' incorporates:
         *  Delay: '<S40>/Delay'
         */
        MicroMouse_Deploy_B.RateLimiter2_j = MicroMouse_Deploy_DW.Delay_DSTATE_k;
      }
    } else {
      elapsedTime = Product_g * MicroMouse_Deploy_P.RateLimiter2_RisingLim_h;
      Gain1 = MicroMouse_Deploy_DW.Delay_DSTATE_k - MicroMouse_Deploy_DW.PrevY_b;
      if (Gain1 > elapsedTime) {
        /* RateLimiter: '<S12>/Rate Limiter2' */
        MicroMouse_Deploy_B.RateLimiter2_j = MicroMouse_Deploy_DW.PrevY_b +
          elapsedTime;
        rtb_NOT = true;
      } else {
        Product_g *= MicroMouse_Deploy_P.RateLimiter2_FallingLim_n;
        if (Gain1 < Product_g) {
          /* RateLimiter: '<S12>/Rate Limiter2' */
          MicroMouse_Deploy_B.RateLimiter2_j = MicroMouse_Deploy_DW.PrevY_b +
            Product_g;
          rtb_NOT = true;
        } else {
          /* RateLimiter: '<S12>/Rate Limiter2' */
          MicroMouse_Deploy_B.RateLimiter2_j =
            MicroMouse_Deploy_DW.Delay_DSTATE_k;
          rtb_NOT = false;
        }
      }

      if (tmp_0) {
        MicroMouse_Deploy_DW.PrevLimited_l = rtb_NOT;
      }
    }
  }

  if (tmp) {
    /* RelationalOperator: '<S52>/Compare' incorporates:
     *  Constant: '<S52>/Constant'
     */
    rtb_Compare_hh = ((int32_T)rtb_Compare_b <= (int32_T)
                      MicroMouse_Deploy_P.Constant_Value_lk);

    /* Chart: '<S40>/Chart' incorporates:
     *  RelationalOperator: '<S50>/FixPt Relational Operator'
     *  UnitDelay: '<S50>/Delay Input1'
     *
     * Block description for '<S50>/Delay Input1':
     *
     *  Store in Global RAM
     */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c4_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c4_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c4_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_default;
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID1.is_c4_MicroMouse_Deploy) {
       case MicroMouse_Deploy_IN_default:
       case MicroMouse_IN_encoder_pulse_end:
        break;

       default:
        /* case IN_encoder_pulse_start: */
        if ((int32_T)rtb_Compare_hh > (int32_T)
            MicroMouse_Deploy_DW.DelayInput1_DSTATE_i) {
          MicroMouse_Deploy_DW.bitsForTID1.is_c4_MicroMouse_Deploy =
            MicroMouse_IN_encoder_pulse_end;
        }
        break;
      }
    }

    /* End of Chart: '<S40>/Chart' */

    /* Gain: '<S154>/Derivative Gain' */
    MicroMouse_Deploy_B.DerivativeGain = MicroMouse_Deploy_P.PIDController4_D *
      0.0;

    /* Gain: '<S158>/Integral Gain' */
    MicroMouse_Deploy_B.IntegralGain_i = MicroMouse_Deploy_P.PIDController4_I *
      0.0;
  }

  /* Gain: '<S164>/Filter Coefficient' incorporates:
   *  Integrator: '<S156>/Filter'
   *  Sum: '<S156>/SumD'
   */
  MicroMouse_Deploy_B.FilterCoefficient_o = (MicroMouse_Deploy_B.DerivativeGain
    - MicroMouse_Deploy_X.Filter_CSTATE_n) *
    MicroMouse_Deploy_P.PIDController4_N;

  /* Chart: '<S3>/draft2_Maze_exploration1' incorporates:
   *  Chart: '<S3>/draft2_Maze_exploration2'
   */
  if (tmp_0) {
    rtb_Compare_c = false;
    if (MicroMouse_Deploy_DW.bitsForTID0.is_active_c6_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID0.is_active_c6_MicroMouse_Deploy = 1U;
      rtb_Compare_c = true;
      MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
        MicroMouse_Deplo_IN_set_Z_angle;
      MicroMouse_Deploy_B.Z_angle_l = 0.0;
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy) {
       case MicroMouse_Dep_IN_Full_rotation:
        if (MicroMouse_Deploy_DW.theta - 3.14159274F >= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
        }
        break;

       case MicroMouse_Deplo_IN_Go_straight:
        rtb_Compare_c = true;
        MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
          MicroMouse_D_IN_left_turn_setup;
        MicroMouse_Deploy_DW.end_dist_a = 0.14F;
        break;

       case MicroMouse_Depl_IN_Go_straight1:
        rtb_Compare_c = true;
        MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
          MicroMouse__IN_left_turn_setup1;
        MicroMouse_Deploy_DW.end_dist_a = 0.14F;
        break;

       case MicroMouse_Deploy_IN_Left_turn:
        if (MicroMouse_Deploy_DW.theta + 1.57079637F <= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
        }
        break;

       case MicroMouse_Deploy_IN_Left_turn1:
        if (MicroMouse_Deploy_DW.theta + 1.57079637F <= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
            MicroMouse_Depl_IN_Go_straight1;
        }
        break;

       case MicroMouse_Deploy_IN_Left_turn2:
        if (MicroMouse_Deploy_DW.theta + 1.57079637F <= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
        }
        break;

       case MicroMouse_Deploy_IN_Start_p:
        break;

       case MicroMouse_D_IN_left_turn_setup:
        if (MicroMouse_Deploy_DW.end_dist_a <= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Left_turn1;
          MicroMouse_Deploy_DW.theta = 0.0F;
          MicroMouse_Deploy_B.Z_angle_l += 1.5707963267948966;
        }
        break;

       case MicroMouse__IN_left_turn_setup1:
        if (MicroMouse_Deploy_DW.end_dist_a <= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Left_turn2;
          MicroMouse_Deploy_DW.theta = 0.0F;
          MicroMouse_Deploy_B.Z_angle_l += 1.5707963267948966;
        }
        break;

       case MicroMouse_Deploy_IN_right_turn:
        if (MicroMouse_Deploy_DW.theta - 1.57079637F >= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
        }
        break;

       default:
        /* case IN_set_Z_angle: */
        rtb_Compare_c = true;
        MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
          MicroMouse_Deploy_IN_Start_p;
        break;
      }
    }

    if (rtb_Compare_c) {
      rtsiSetBlockStateForSolverChangedAtMajorStep
        (&MicroMouse_Deploy_M->solverInfo, true);
    }

    rtb_Compare_c = false;
    if (MicroMouse_Deploy_DW.bitsForTID0.is_active_c10_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID0.is_active_c10_MicroMouse_Deploy = 1U;
      rtb_Compare_c = true;
      MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
        MicroMouse_Deplo_IN_set_Z_angle;
      MicroMouse_Deploy_B.Z_angle = 0.0;
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy) {
       case MicroMouse_Dep_IN_Full_rotation:
        if (MicroMouse_Deploy_B.Z_angle >= 0.0) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
        }
        break;

       case MicroMouse_Deplo_IN_Go_straight:
        rtb_Compare_c = true;
        MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
          MicroMouse_D_IN_left_turn_setup;
        MicroMouse_Deploy_DW.end_dist_o = 0.145F;
        break;

       case MicroMouse_Depl_IN_Go_straight1:
        rtb_Compare_c = true;
        MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
          MicroMouse__IN_left_turn_setup1;
        MicroMouse_Deploy_DW.end_dist_o = 0.145F;
        break;

       case MicroMouse_Deploy_IN_Left_turn:
        if (MicroMouse_Deploy_B.Z_angle <= 0.0) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
        }
        break;

       case MicroMouse_Deploy_IN_Left_turn1:
        if (MicroMouse_Deploy_B.Z_angle <= 0.0) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
            MicroMouse_Depl_IN_Go_straight1;
        }
        break;

       case MicroMouse_Deploy_IN_Left_turn2:
        if (MicroMouse_Deploy_B.Z_angle <= 0.0) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
        }
        break;

       case MicroMouse_Deploy_IN_Start_p:
        break;

       case MicroMouse_D_IN_left_turn_setup:
        if (MicroMouse_Deploy_DW.end_dist_o <= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Left_turn1;
          MicroMouse_Deploy_B.Z_angle += 1.5707963267948966;
        }
        break;

       case MicroMouse__IN_left_turn_setup1:
        if (MicroMouse_Deploy_DW.end_dist_o <= 0.0F) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Left_turn2;
          MicroMouse_Deploy_B.Z_angle += 1.5707963267948966;
        }
        break;

       case MicroMouse_Deploy_IN_right_turn:
        if (MicroMouse_Deploy_B.Z_angle >= 0.0) {
          rtb_Compare_c = true;
          MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_Start_p;
        }
        break;

       default:
        /* case IN_set_Z_angle: */
        rtb_Compare_c = true;
        MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
          MicroMouse_Deploy_IN_Start_p;
        break;
      }
    }

    if (rtb_Compare_c) {
      rtsiSetBlockStateForSolverChangedAtMajorStep
        (&MicroMouse_Deploy_M->solverInfo, true);
    }
  }

  if (tmp) {
    /* MATLABSystem: '<S24>/Moving Average' */
    if (MicroMouse_Deploy_DW.obj_b.TunablePropsChanged) {
      MicroMouse_Deploy_DW.obj_b.TunablePropsChanged = false;
    }

    K = 0.0F;
    K_e = 0.0F;
    K_p = MicroMouse_Deploy_DW.obj_b.pCumSum + MicroMouse_Deploy_B.Saturation;
    if (MicroMouse_Deploy_DW.obj_b.pModValueRev == 0.0F) {
      K = MicroMouse_Deploy_DW.obj_b.pCumSumRev[(int32_T)
        MicroMouse_Deploy_DW.obj_b.pCumRevIndex - 1] + K_p;
    }

    MicroMouse_Deploy_DW.obj_b.pCumSumRev[(int32_T)
      MicroMouse_Deploy_DW.obj_b.pCumRevIndex - 1] =
      MicroMouse_Deploy_B.Saturation;
    if (MicroMouse_Deploy_DW.obj_b.pCumRevIndex != 299.0F) {
      rtb_Gain1_h = MicroMouse_Deploy_DW.obj_b.pCumRevIndex + 1.0F;
    } else {
      rtb_Gain1_h = 1.0F;
      K_p = 0.0F;
      for (k = 297; k >= 0; k--) {
        MicroMouse_Deploy_DW.obj_b.pCumSumRev[k] +=
          MicroMouse_Deploy_DW.obj_b.pCumSumRev[k + 1];
      }
    }

    if (MicroMouse_Deploy_DW.obj_b.pModValueRev == 0.0F) {
      K_e = K / 300.0F;
    }

    MicroMouse_Deploy_DW.obj_b.pCumSum = K_p;
    MicroMouse_Deploy_DW.obj_b.pCumRevIndex = rtb_Gain1_h;
    if (MicroMouse_Deploy_DW.obj_b.pModValueRev > 0.0F) {
      MicroMouse_Deploy_DW.obj_b.pModValueRev--;
    } else {
      MicroMouse_Deploy_DW.obj_b.pModValueRev = 0.0F;
    }

    /* MATLABSystem: '<S24>/Moving Average1' */
    if (MicroMouse_Deploy_DW.obj.TunablePropsChanged) {
      MicroMouse_Deploy_DW.obj.TunablePropsChanged = false;
    }

    K = 0.0F;
    rtb_Gain2_b = 0.0F;
    K_p = MicroMouse_Deploy_DW.obj.pCumSum + MicroMouse_Deploy_B.Saturation_m;
    if (MicroMouse_Deploy_DW.obj.pModValueRev == 0.0F) {
      K = MicroMouse_Deploy_DW.obj.pCumSumRev[(int32_T)
        MicroMouse_Deploy_DW.obj.pCumRevIndex - 1] + K_p;
    }

    MicroMouse_Deploy_DW.obj.pCumSumRev[(int32_T)
      MicroMouse_Deploy_DW.obj.pCumRevIndex - 1] =
      MicroMouse_Deploy_B.Saturation_m;
    if (MicroMouse_Deploy_DW.obj.pCumRevIndex != 299.0F) {
      rtb_Gain1_h = MicroMouse_Deploy_DW.obj.pCumRevIndex + 1.0F;
    } else {
      rtb_Gain1_h = 1.0F;
      K_p = 0.0F;
      for (k = 297; k >= 0; k--) {
        MicroMouse_Deploy_DW.obj.pCumSumRev[k] +=
          MicroMouse_Deploy_DW.obj.pCumSumRev[k + 1];
      }
    }

    if (MicroMouse_Deploy_DW.obj.pModValueRev == 0.0F) {
      rtb_Gain2_b = K / 300.0F;
    }

    MicroMouse_Deploy_DW.obj.pCumSum = K_p;
    MicroMouse_Deploy_DW.obj.pCumRevIndex = rtb_Gain1_h;
    if (MicroMouse_Deploy_DW.obj.pModValueRev > 0.0F) {
      MicroMouse_Deploy_DW.obj.pModValueRev--;
    } else {
      MicroMouse_Deploy_DW.obj.pModValueRev = 0.0F;
    }

    /* Chart: '<S24>/engage_disengage_straightnert' incorporates:
     *  MATLABSystem: '<S24>/Moving Average'
     *  MATLABSystem: '<S24>/Moving Average1'
     */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c3_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c3_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_Deafult;
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy) {
       case MicroMouse_Deploy_IN_Deafult:
        if ((!MicroMouse_Deploy_B.TOF_LEFT) || (!MicroMouse_Deploy_B.TOF_RIGHT))
        {
          MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy =
            MicroMouse_Depl_IN_Turn_is_seen;
          MicroMouse_Deploy_B.error = 0.0;
          MicroMouse_Deploy_B.TOF_L_out = 0.0;
          MicroMouse_Deploy_B.TOF_R_out = 0.0;
        } else {
          MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy =
            MicroMouse_De_IN_Going_straight;
          MicroMouse_Deploy_B.error = 1.0;
          MicroMouse_Deploy_B.TOF_L_out = K_e;
          MicroMouse_Deploy_B.TOF_R_out = rtb_Gain2_b;
        }
        break;

       case MicroMouse_De_IN_Going_straight:
        MicroMouse_Deploy_B.error = 1.0;
        break;

       default:
        /* case IN_Turn_is_seen: */
        MicroMouse_Deploy_B.error = 0.0;
        MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy =
          MicroMouse_Deploy_IN_Deafult;
        break;
      }
    }

    /* End of Chart: '<S24>/engage_disengage_straightnert' */

    /* Product: '<S24>/Product2' incorporates:
     *  Constant: '<S24>/Constant'
     *  Sum: '<S24>/Subtract1'
     */
    rtb_Product1_b = (MicroMouse_Deploy_P.Constant_Value_g -
                      MicroMouse_Deploy_B.TOF_L_out) * MicroMouse_Deploy_B.error;

    /* Gain: '<S323>/Derivative Gain' */
    MicroMouse_Deploy_B.DerivativeGain_c =
      MicroMouse_Deploy_P.PIDController1_D_p * rtb_Product1_b;

    /* Gain: '<S327>/Integral Gain' */
    MicroMouse_Deploy_B.IntegralGain_e = MicroMouse_Deploy_P.PIDController1_I_g *
      rtb_Product1_b;

    /* Product: '<S24>/Product1' incorporates:
     *  Constant: '<S24>/Constant'
     *  Sum: '<S24>/Subtract2'
     */
    rtb_Product1_b = (MicroMouse_Deploy_P.Constant_Value_g -
                      MicroMouse_Deploy_B.TOF_R_out) * MicroMouse_Deploy_B.error;

    /* Gain: '<S375>/Derivative Gain' */
    MicroMouse_Deploy_B.DerivativeGain_k = MicroMouse_Deploy_P.PIDController2_D *
      rtb_Product1_b;

    /* Gain: '<S379>/Integral Gain' */
    MicroMouse_Deploy_B.IntegralGain_k = MicroMouse_Deploy_P.PIDController2_I *
      rtb_Product1_b;
  }

  /* Gain: '<S333>/Filter Coefficient' incorporates:
   *  Integrator: '<S325>/Filter'
   *  Sum: '<S325>/SumD'
   */
  MicroMouse_Deploy_B.FilterCoefficient_j =
    (MicroMouse_Deploy_B.DerivativeGain_c - MicroMouse_Deploy_X.Filter_CSTATE_c)
    * MicroMouse_Deploy_P.PIDController1_N_m;

  /* Gain: '<S385>/Filter Coefficient' incorporates:
   *  Integrator: '<S377>/Filter'
   *  Sum: '<S377>/SumD'
   */
  MicroMouse_Deploy_B.FilterCoefficient_n =
    (MicroMouse_Deploy_B.DerivativeGain_k - MicroMouse_Deploy_X.Filter_CSTATE_f)
    * MicroMouse_Deploy_P.PIDController2_N;
  if (rtmIsMajorTimeStep(MicroMouse_Deploy_M)) {
    {
      {
        /* user code (Update function Header) */

        /* System '<Root>' */
        refreshTOFValues();

        /* System '<Root>' */
        refreshINA219Values();

        /* System '<Root>' */
        refreshSWValues();

        /* System '<Root>' */
        refreshLEDs();

        /* System '<Root>' */
        refreshIMUValues();

        /* System '<Root>' */
        refreshADCs();

        /* System '<Root>' */
        refreshScreen();

        /* System '<Root>' */
        refreshMotors();

        /* System '<Root>' */
        refreshLoggedData();
        if (rtmIsMajorTimeStep(MicroMouse_Deploy_M)) {
          /* Update for DiscreteIntegrator: '<S60>/Integrator' */
          MicroMouse_Deploy_DW.Integrator_IC_LOADING = 0U;
          MicroMouse_Deploy_DW.Integrator_DSTATE +=
            MicroMouse_Deploy_P.Integrator_gainval * MicroMouse_Deploy_B.uT;
          if (MicroMouse_Deploy_DW.Integrator_DSTATE >
              MicroMouse_Deploy_P.Integrator_UpperSat) {
            MicroMouse_Deploy_DW.Integrator_DSTATE =
              MicroMouse_Deploy_P.Integrator_UpperSat;
          } else if (MicroMouse_Deploy_DW.Integrator_DSTATE <
                     MicroMouse_Deploy_P.Integrator_LowerSat) {
            MicroMouse_Deploy_DW.Integrator_DSTATE =
              MicroMouse_Deploy_P.Integrator_LowerSat;
          }

          MicroMouse_Deploy_DW.Integrator_PrevResetState = (int8_T)
            rtb_LogicalOperator;

          /* End of Update for DiscreteIntegrator: '<S60>/Integrator' */

          /* Update for DiscreteIntegrator: '<S74>/Integrator' */
          MicroMouse_Deploy_DW.Integrator_IC_LOADING_a = 0U;
          MicroMouse_Deploy_DW.Integrator_DSTATE_i +=
            MicroMouse_Deploy_P.Integrator_gainval_k * MicroMouse_Deploy_B.uT_e;
          if (MicroMouse_Deploy_DW.Integrator_DSTATE_i >
              MicroMouse_Deploy_P.Integrator_UpperSat_l) {
            MicroMouse_Deploy_DW.Integrator_DSTATE_i =
              MicroMouse_Deploy_P.Integrator_UpperSat_l;
          } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_i <
                     MicroMouse_Deploy_P.Integrator_LowerSat_g) {
            MicroMouse_Deploy_DW.Integrator_DSTATE_i =
              MicroMouse_Deploy_P.Integrator_LowerSat_g;
          }

          MicroMouse_Deploy_DW.Integrator_PrevResetState_j = (int8_T)
            rtb_LogicalOperator_a;

          /* End of Update for DiscreteIntegrator: '<S74>/Integrator' */

          /* Update for Delay: '<S27>/Delay' */
          MicroMouse_Deploy_DW.Delay_DSTATE_d = MicroMouse_Deploy_B.Add_f;

          /* Update for UnitDelay: '<S36>/Delay Input1'
           *
           * Block description for '<S36>/Delay Input1':
           *
           *  Store in Global RAM
           */
          MicroMouse_Deploy_DW.DelayInput1_DSTATE = MicroMouse_Deploy_B.Compare;

          /* Update for Delay: '<S26>/Delay' */
          MicroMouse_Deploy_DW.Delay_DSTATE_f = MicroMouse_Deploy_B.Add_h;

          /* Update for UnitDelay: '<S31>/Delay Input1'
           *
           * Block description for '<S31>/Delay Input1':
           *
           *  Store in Global RAM
           */
          MicroMouse_Deploy_DW.DelayInput1_DSTATE_c =
            MicroMouse_Deploy_B.Compare_a;

          /* Update for DiscreteIntegrator: '<S67>/Integrator' */
          MicroMouse_Deploy_DW.Integrator_IC_LOADING_k = 0U;
          MicroMouse_Deploy_DW.Integrator_DSTATE_m +=
            MicroMouse_Deploy_P.Integrator_gainval_n * MicroMouse_Deploy_B.uT_p;
          if (MicroMouse_Deploy_DW.Integrator_DSTATE_m >
              MicroMouse_Deploy_P.Integrator_UpperSat_k) {
            MicroMouse_Deploy_DW.Integrator_DSTATE_m =
              MicroMouse_Deploy_P.Integrator_UpperSat_k;
          } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_m <
                     MicroMouse_Deploy_P.Integrator_LowerSat_d) {
            MicroMouse_Deploy_DW.Integrator_DSTATE_m =
              MicroMouse_Deploy_P.Integrator_LowerSat_d;
          }

          MicroMouse_Deploy_DW.Integrator_PrevResetState_a = (int8_T)
            rtb_LogicalOperator_c;

          /* End of Update for DiscreteIntegrator: '<S67>/Integrator' */

          /* Update for UnitDelay: '<S35>/Delay Input1'
           *
           * Block description for '<S35>/Delay Input1':
           *
           *  Store in Global RAM
           */
          MicroMouse_Deploy_DW.DelayInput1_DSTATE_e =
            MicroMouse_Deploy_B.Compare_n;

          /* Update for UnitDelay: '<S289>/Unit Delay1' */
          MicroMouse_Deploy_DW.UnitDelay1_DSTATE = MicroMouse_Deploy_B.Sum1;

          /* Update for UnitDelay: '<S290>/Unit Delay1' */
          MicroMouse_Deploy_DW.UnitDelay1_DSTATE_n = MicroMouse_Deploy_B.Sum1_k;

          /* Update for UnitDelay: '<S289>/Unit Delay2' */
          MicroMouse_Deploy_DW.UnitDelay2_DSTATE = MicroMouse_Deploy_B.Sum3;

          /* Update for UnitDelay: '<S290>/Unit Delay2' */
          MicroMouse_Deploy_DW.UnitDelay2_DSTATE_j = MicroMouse_Deploy_B.Sum3_o;

          /* Update for UnitDelay: '<S46>/Delay Input1'
           *
           * Block description for '<S46>/Delay Input1':
           *
           *  Store in Global RAM
           */
          MicroMouse_Deploy_DW.DelayInput1_DSTATE_b = rtb_Compare;

          /* Update for UnitDelay: '<S51>/Delay Input1'
           *
           * Block description for '<S51>/Delay Input1':
           *
           *  Store in Global RAM
           */
          MicroMouse_Deploy_DW.DelayInput1_DSTATE_k = rtb_Compare_g;

          /* Update for UnitDelay: '<S50>/Delay Input1'
           *
           * Block description for '<S50>/Delay Input1':
           *
           *  Store in Global RAM
           */
          MicroMouse_Deploy_DW.DelayInput1_DSTATE_i = rtb_Compare_hh;
        }

        /* Update for RateLimiter: '<S11>/Rate Limiter2' incorporates:
         *  RateLimiter: '<S11>/Rate Limiter4'
         *  RateLimiter: '<S12>/Rate Limiter1'
         *  RateLimiter: '<S12>/Rate Limiter2'
         */
        MicroMouse_Deploy_DW.PrevY = MicroMouse_Deploy_B.RateLimiter2;
        MicroMouse_Deploy_DW.LastMajorTime = MicroMouse_Deploy_M->Timing.t[0];

        /* Update for RateLimiter: '<S11>/Rate Limiter4' */
        MicroMouse_Deploy_DW.PrevY_i = MicroMouse_Deploy_B.RateLimiter4;
        MicroMouse_Deploy_DW.LastMajorTime_b =
          MicroMouse_Deploy_DW.LastMajorTime;

        /* Update for RateLimiter: '<S12>/Rate Limiter1' */
        MicroMouse_Deploy_DW.PrevY_l = MicroMouse_Deploy_B.RateLimiter1;
        MicroMouse_Deploy_DW.LastMajorTime_c =
          MicroMouse_Deploy_DW.LastMajorTime;

        /* Update for RateLimiter: '<S12>/Rate Limiter2' */
        MicroMouse_Deploy_DW.PrevY_b = MicroMouse_Deploy_B.RateLimiter2_j;
        MicroMouse_Deploy_DW.LastMajorTime_k =
          MicroMouse_Deploy_DW.LastMajorTime;
      }
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(MicroMouse_Deploy_M)) {
    rt_ertODEUpdateContinuousStates(&MicroMouse_Deploy_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++MicroMouse_Deploy_M->Timing.clockTick0;
    MicroMouse_Deploy_M->Timing.t[0] = rtsiGetSolverStopTime
      (&MicroMouse_Deploy_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      MicroMouse_Deploy_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void MicroMouse_Deploy_derivatives(void)
{
  XDot_MicroMouse_Deploy_T *_rtXdot;
  _rtXdot = ((XDot_MicroMouse_Deploy_T *) MicroMouse_Deploy_M->derivs);

  /* Derivatives for Integrator: '<S3>/Integrator' */
  _rtXdot->Integrator_CSTATE = MicroMouse_Deploy_B.CastToDouble[2];

  /* Derivatives for Integrator: '<S270>/Integrator' */
  _rtXdot->Integrator_CSTATE_o = MicroMouse_Deploy_B.IntegralGain_c;

  /* Derivatives for Integrator: '<S265>/Filter' */
  _rtXdot->Filter_CSTATE = MicroMouse_Deploy_B.FilterCoefficient;

  /* Derivatives for Integrator: '<S3>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = MicroMouse_Deploy_B.DeadZone;

  /* Derivatives for Integrator: '<S104>/Filter' */
  _rtXdot->Filter_CSTATE_k = MicroMouse_Deploy_B.FilterCoefficient_k;

  /* Derivatives for Integrator: '<S109>/Integrator' */
  _rtXdot->Integrator_CSTATE_e = MicroMouse_Deploy_B.IntegralGain;

  /* Derivatives for Integrator: '<S18>/Integrator' */
  _rtXdot->Integrator_CSTATE_j = MicroMouse_Deploy_B.CastToDouble[2];

  /* Derivatives for Integrator: '<S211>/Filter' */
  _rtXdot->Filter_CSTATE_l = MicroMouse_Deploy_B.FilterCoefficient_f;

  /* Derivatives for Integrator: '<S12>/Integrator' */
  _rtXdot->Integrator_CSTATE_f = MicroMouse_Deploy_B.RateLimiter1;

  /* Derivatives for Integrator: '<S12>/Integrator1' */
  _rtXdot->Integrator1_CSTATE_g = MicroMouse_Deploy_B.RateLimiter2_j;

  /* Derivatives for Integrator: '<S156>/Filter' */
  _rtXdot->Filter_CSTATE_n = MicroMouse_Deploy_B.FilterCoefficient_o;

  /* Derivatives for Integrator: '<S161>/Integrator' */
  _rtXdot->Integrator_CSTATE_k = MicroMouse_Deploy_B.IntegralGain_i;

  /* Derivatives for Integrator: '<S325>/Filter' */
  _rtXdot->Filter_CSTATE_c = MicroMouse_Deploy_B.FilterCoefficient_j;

  /* Derivatives for Integrator: '<S330>/Integrator' */
  _rtXdot->Integrator_CSTATE_g = MicroMouse_Deploy_B.IntegralGain_e;

  /* Derivatives for Integrator: '<S377>/Filter' */
  _rtXdot->Filter_CSTATE_f = MicroMouse_Deploy_B.FilterCoefficient_n;

  /* Derivatives for Integrator: '<S382>/Integrator' */
  _rtXdot->Integrator_CSTATE_a = MicroMouse_Deploy_B.IntegralGain_k;
}

/* Model initialize function */
void MicroMouse_Deploy_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&MicroMouse_Deploy_M->solverInfo,
                          &MicroMouse_Deploy_M->Timing.simTimeStep);
    rtsiSetTPtr(&MicroMouse_Deploy_M->solverInfo, &rtmGetTPtr
                (MicroMouse_Deploy_M));
    rtsiSetStepSizePtr(&MicroMouse_Deploy_M->solverInfo,
                       &MicroMouse_Deploy_M->Timing.stepSize0);
    rtsiSetdXPtr(&MicroMouse_Deploy_M->solverInfo, &MicroMouse_Deploy_M->derivs);
    rtsiSetContStatesPtr(&MicroMouse_Deploy_M->solverInfo, (real_T **)
                         &MicroMouse_Deploy_M->contStates);
    rtsiSetNumContStatesPtr(&MicroMouse_Deploy_M->solverInfo,
      &MicroMouse_Deploy_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&MicroMouse_Deploy_M->solverInfo,
      &MicroMouse_Deploy_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&MicroMouse_Deploy_M->solverInfo,
      &MicroMouse_Deploy_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&MicroMouse_Deploy_M->solverInfo,
      &MicroMouse_Deploy_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&MicroMouse_Deploy_M->solverInfo, (boolean_T**)
      &MicroMouse_Deploy_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&MicroMouse_Deploy_M->solverInfo, (&rtmGetErrorStatus
      (MicroMouse_Deploy_M)));
    rtsiSetRTModelPtr(&MicroMouse_Deploy_M->solverInfo, MicroMouse_Deploy_M);
  }

  rtsiSetSimTimeStep(&MicroMouse_Deploy_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&MicroMouse_Deploy_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&MicroMouse_Deploy_M->solverInfo, false);
  MicroMouse_Deploy_M->intgData.y = MicroMouse_Deploy_M->odeY;
  MicroMouse_Deploy_M->intgData.f[0] = MicroMouse_Deploy_M->odeF[0];
  MicroMouse_Deploy_M->intgData.f[1] = MicroMouse_Deploy_M->odeF[1];
  MicroMouse_Deploy_M->intgData.f[2] = MicroMouse_Deploy_M->odeF[2];
  MicroMouse_Deploy_M->contStates = ((X_MicroMouse_Deploy_T *)
    &MicroMouse_Deploy_X);
  MicroMouse_Deploy_M->contStateDisabled = ((XDis_MicroMouse_Deploy_T *)
    &MicroMouse_Deploy_XDis);
  MicroMouse_Deploy_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&MicroMouse_Deploy_M->solverInfo, (void *)
                    &MicroMouse_Deploy_M->intgData);
  rtsiSetSolverName(&MicroMouse_Deploy_M->solverInfo,"ode3");
  rtmSetTPtr(MicroMouse_Deploy_M, &MicroMouse_Deploy_M->Timing.tArray[0]);
  MicroMouse_Deploy_M->Timing.stepSize0 = 0.01;

  {
    int32_T i;

    /* Start for Probe: '<S54>/Probe' */
    MicroMouse_Deploy_B.Probe[0] = 0.01F;
    MicroMouse_Deploy_B.Probe[1] = 0.0F;

    /* Start for Probe: '<S68>/Probe' */
    MicroMouse_Deploy_B.Probe_g[0] = 0.01F;
    MicroMouse_Deploy_B.Probe_g[1] = 0.0F;

    /* Start for Probe: '<S61>/Probe' */
    MicroMouse_Deploy_B.Probe_h[0] = 0.01F;
    MicroMouse_Deploy_B.Probe_h[1] = 0.0F;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory' */
    MOTOR_LS = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_oa;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory1' */
    MOTOR_RS = MicroMouse_Deploy_P.DataStoreMemory1_InitialValu_p2;
    for (i = 0; i < 18; i++) {
      /* Start for DataStoreMemory: '<S2>/Data Store Memory' */
      oled_string1[i] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_e;

      /* Start for DataStoreMemory: '<S2>/Data Store Memory1' incorporates:
       *  DataStoreMemory: '<S2>/Data Store Memory'
       */
      oled_string2[i] = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue_b;

      /* Start for DataStoreMemory: '<S2>/Data Store Memory2' incorporates:
       *  DataStoreMemory: '<S2>/Data Store Memory'
       */
      oled_string3[i] = MicroMouse_Deploy_P.DataStoreMemory2_InitialValue_p;

      /* Start for DataStoreMemory: '<S2>/Data Store Memory3' incorporates:
       *  DataStoreMemory: '<S2>/Data Store Memory'
       */
      oled_string4[i] = MicroMouse_Deploy_P.DataStoreMemory3_InitialValue_c;

      /* Start for DataStoreMemory: '<S2>/Data Store Memory4' incorporates:
       *  DataStoreMemory: '<S2>/Data Store Memory'
       */
      oled_string5[i] = MicroMouse_Deploy_P.DataStoreMemory4_InitialValue_f;
    }

    /* Start for DataStoreMemory: '<S9>/Data Store Memory' */
    STATE = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_n;

    /* Start for DataStoreMemory: '<S4>/Data Store Memory' */
    V_BATT = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_b;

    /* Start for DataStoreMemory: '<S4>/Data Store Memory1' */
    V_PHOTO_DOWN_LS = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue_a;

    /* Start for DataStoreMemory: '<S4>/Data Store Memory2' */
    V_PHOTO_DOWN_RS = MicroMouse_Deploy_P.DataStoreMemory2_InitialValue_e;

    /* Start for DataStoreMemory: '<S4>/Data Store Memory3' */
    V_PHOTO_MOT_LS = MicroMouse_Deploy_P.DataStoreMemory3_InitialValue_n;

    /* Start for DataStoreMemory: '<S4>/Data Store Memory4' */
    V_PHOTO_MOT_RS = MicroMouse_Deploy_P.DataStoreMemory4_InitialValue;

    /* Start for DataStoreMemory: '<S6>/Data Store Memory' */
    LED[0] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_c;

    /* Start for DataStoreMemory: '<S5>/Data Store Memory' */
    IMU_Accel[0] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S5>/Data Store Memory1' */
    IMU_Gyro[0] = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<S6>/Data Store Memory' */
    LED[1] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_c;

    /* Start for DataStoreMemory: '<S5>/Data Store Memory' */
    IMU_Accel[1] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S5>/Data Store Memory1' */
    IMU_Gyro[1] = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<S6>/Data Store Memory' */
    LED[2] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_c;

    /* Start for DataStoreMemory: '<S5>/Data Store Memory' */
    IMU_Accel[2] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S5>/Data Store Memory1' */
    IMU_Gyro[2] = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<S5>/Data Store Memory2' */
    IMU_Temp = MicroMouse_Deploy_P.DataStoreMemory2_InitialValue;

    /* Start for DataStoreMemory: '<S7>/Data Store Memory' */
    SW[0] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_h;
    SW[1] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_h;

    /* Start for DataStoreMemory: '<S8>/Data Store Memory' */
    Vbattery = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_l;

    /* Start for DataStoreMemory: '<S8>/Data Store Memory1' */
    Vshunt = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue_p;

    /* Start for DataStoreMemory: '<S8>/Data Store Memory2' */
    Current = MicroMouse_Deploy_P.DataStoreMemory2_InitialValue_n;

    /* Start for DataStoreMemory: '<S8>/Data Store Memory3' */
    Power = MicroMouse_Deploy_P.DataStoreMemory3_InitialValue_g;

    /* Start for DataStoreMemory: '<S8>/Data Store Memory4' */
    batteryLife = MicroMouse_Deploy_P.DataStoreMemory4_InitialValue_k;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory' */
    TOF_Distance[0] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_o;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory1' */
    TOF_Ambient[0] = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue_h;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory2' */
    TOF_Signal[0] = MicroMouse_Deploy_P.DataStoreMemory2_InitialValue_g;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory3' */
    TOF_Status[0] = MicroMouse_Deploy_P.DataStoreMemory3_InitialValue;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory' */
    TOF_Distance[1] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_o;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory1' */
    TOF_Ambient[1] = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue_h;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory2' */
    TOF_Signal[1] = MicroMouse_Deploy_P.DataStoreMemory2_InitialValue_g;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory3' */
    TOF_Status[1] = MicroMouse_Deploy_P.DataStoreMemory3_InitialValue;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory' */
    TOF_Distance[2] = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_o;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory1' */
    TOF_Ambient[2] = MicroMouse_Deploy_P.DataStoreMemory1_InitialValue_h;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory2' */
    TOF_Signal[2] = MicroMouse_Deploy_P.DataStoreMemory2_InitialValue_g;

    /* Start for DataStoreMemory: '<S10>/Data Store Memory3' */
    TOF_Status[2] = MicroMouse_Deploy_P.DataStoreMemory3_InitialValue;

    {
      /* user code (Initialize function Header) */

      /* System '<Root>' */
      initTOFs(1);

      /* System '<Root>' */
      initSW();

      /* System '<Root>' */
      initLEDs();

      /* System '<Root>' */
      initIMU();

      /* System '<Root>' */
      initScreen();

      /* System '<Root>' */
      initMotors();

      /* System '<Root>' */
      I2C_HandleTypeDef hi2c1;
      I2C_HandleTypeDef hi2c2;
      ADC_HandleTypeDef hadc1;
      DMA_HandleTypeDef hdma_adc1;
      initLogs();
      __enable_irq();

      /* user code (Initialize function Body) */

      /* System '<Root>' */
      initINA219();

      /* System '<Root>' */
      initADCs();

      /* InitializeConditions for DiscreteIntegrator: '<S60>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING = 1U;

      /* InitializeConditions for DiscreteIntegrator: '<S74>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING_a = 1U;

      /* InitializeConditions for Delay: '<S27>/Delay' */
      MicroMouse_Deploy_DW.Delay_DSTATE_d =
        MicroMouse_Deploy_P.Delay_InitialCondition_j;

      /* InitializeConditions for UnitDelay: '<S36>/Delay Input1'
       *
       * Block description for '<S36>/Delay Input1':
       *
       *  Store in Global RAM
       */
      MicroMouse_Deploy_DW.DelayInput1_DSTATE =
        MicroMouse_Deploy_P.DetectRisePositive_vinit;

      /* InitializeConditions for RateLimiter: '<S11>/Rate Limiter2' */
      MicroMouse_Deploy_DW.LastMajorTime = (rtInf);

      /* InitializeConditions for Delay: '<S26>/Delay' */
      MicroMouse_Deploy_DW.Delay_DSTATE_f =
        MicroMouse_Deploy_P.Delay_InitialCondition_c;

      /* InitializeConditions for UnitDelay: '<S31>/Delay Input1'
       *
       * Block description for '<S31>/Delay Input1':
       *
       *  Store in Global RAM
       */
      MicroMouse_Deploy_DW.DelayInput1_DSTATE_c =
        MicroMouse_Deploy_P.DetectRisePositive_vinit_a;

      /* InitializeConditions for RateLimiter: '<S11>/Rate Limiter4' */
      MicroMouse_Deploy_DW.LastMajorTime_b = (rtInf);

      /* InitializeConditions for DiscreteIntegrator: '<S67>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING_k = 1U;

      /* InitializeConditions for Integrator: '<S3>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE = MicroMouse_Deploy_P.Integrator_IC;

      /* InitializeConditions for Integrator: '<S270>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE_o =
        MicroMouse_Deploy_P.PIDController1_InitialConditi_i;

      /* InitializeConditions for Integrator: '<S265>/Filter' */
      MicroMouse_Deploy_X.Filter_CSTATE =
        MicroMouse_Deploy_P.PIDController1_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S35>/Delay Input1'
       *
       * Block description for '<S35>/Delay Input1':
       *
       *  Store in Global RAM
       */
      MicroMouse_Deploy_DW.DelayInput1_DSTATE_e =
        MicroMouse_Deploy_P.DetectFallNonpositive_vinit;

      /* InitializeConditions for Integrator: '<S3>/Integrator1' */
      MicroMouse_Deploy_X.Integrator1_CSTATE =
        MicroMouse_Deploy_P.Integrator1_IC;

      /* InitializeConditions for Integrator: '<S104>/Filter' */
      MicroMouse_Deploy_X.Filter_CSTATE_k =
        MicroMouse_Deploy_P.PIDController_InitialConditionF;

      /* InitializeConditions for Integrator: '<S109>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE_e =
        MicroMouse_Deploy_P.PIDController_InitialConditio_l;

      /* InitializeConditions for Integrator: '<S18>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE_j =
        MicroMouse_Deploy_P.Integrator_IC_b;

      /* InitializeConditions for Integrator: '<S211>/Filter' */
      MicroMouse_Deploy_X.Filter_CSTATE_l =
        MicroMouse_Deploy_P.PIDController_InitialConditio_a;

      /* InitializeConditions for UnitDelay: '<S289>/Unit Delay1' */
      MicroMouse_Deploy_DW.UnitDelay1_DSTATE =
        MicroMouse_Deploy_P.UnitDelay1_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S290>/Unit Delay1' */
      MicroMouse_Deploy_DW.UnitDelay1_DSTATE_n =
        MicroMouse_Deploy_P.UnitDelay1_InitialCondition_k;

      /* InitializeConditions for UnitDelay: '<S289>/Unit Delay2' */
      MicroMouse_Deploy_DW.UnitDelay2_DSTATE =
        MicroMouse_Deploy_P.UnitDelay2_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S290>/Unit Delay2' */
      MicroMouse_Deploy_DW.UnitDelay2_DSTATE_j =
        MicroMouse_Deploy_P.UnitDelay2_InitialCondition_e;

      /* InitializeConditions for Integrator: '<S12>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE_f =
        MicroMouse_Deploy_P.Integrator_IC_p;

      /* InitializeConditions for Delay: '<S39>/Delay' */
      MicroMouse_Deploy_DW.Delay_DSTATE =
        MicroMouse_Deploy_P.Delay_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S46>/Delay Input1'
       *
       * Block description for '<S46>/Delay Input1':
       *
       *  Store in Global RAM
       */
      MicroMouse_Deploy_DW.DelayInput1_DSTATE_b =
        MicroMouse_Deploy_P.DetectRisePositive_vinit_h;

      /* InitializeConditions for RateLimiter: '<S12>/Rate Limiter1' */
      MicroMouse_Deploy_DW.LastMajorTime_c = (rtInf);

      /* InitializeConditions for Integrator: '<S12>/Integrator1' */
      MicroMouse_Deploy_X.Integrator1_CSTATE_g =
        MicroMouse_Deploy_P.Integrator1_IC_o;

      /* InitializeConditions for Delay: '<S40>/Delay' */
      MicroMouse_Deploy_DW.Delay_DSTATE_k =
        MicroMouse_Deploy_P.Delay_InitialCondition_d;

      /* InitializeConditions for UnitDelay: '<S51>/Delay Input1'
       *
       * Block description for '<S51>/Delay Input1':
       *
       *  Store in Global RAM
       */
      MicroMouse_Deploy_DW.DelayInput1_DSTATE_k =
        MicroMouse_Deploy_P.DetectRisePositive_vinit_f;

      /* InitializeConditions for RateLimiter: '<S12>/Rate Limiter2' */
      MicroMouse_Deploy_DW.LastMajorTime_k = (rtInf);

      /* InitializeConditions for UnitDelay: '<S50>/Delay Input1'
       *
       * Block description for '<S50>/Delay Input1':
       *
       *  Store in Global RAM
       */
      MicroMouse_Deploy_DW.DelayInput1_DSTATE_i =
        MicroMouse_Deploy_P.DetectFallNonpositive_vinit_a;

      /* InitializeConditions for Integrator: '<S156>/Filter' */
      MicroMouse_Deploy_X.Filter_CSTATE_n =
        MicroMouse_Deploy_P.PIDController4_InitialCondition;

      /* InitializeConditions for Integrator: '<S161>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE_k =
        MicroMouse_Deploy_P.PIDController4_InitialConditi_e;

      /* InitializeConditions for Integrator: '<S325>/Filter' */
      MicroMouse_Deploy_X.Filter_CSTATE_c =
        MicroMouse_Deploy_P.PIDController1_InitialConditi_h;

      /* InitializeConditions for Integrator: '<S330>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE_g =
        MicroMouse_Deploy_P.PIDController1_InitialConditi_c;

      /* InitializeConditions for Integrator: '<S377>/Filter' */
      MicroMouse_Deploy_X.Filter_CSTATE_f =
        MicroMouse_Deploy_P.PIDController2_InitialCondition;

      /* InitializeConditions for Integrator: '<S382>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE_a =
        MicroMouse_Deploy_P.PIDController2_InitialConditi_f;

      /* SystemInitialize for Chart: '<S3>/draft2_Maze_exploration' */
      MicroMouse_Deploy_DW.bitsForTID0.is_active_c8_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID0.is_c8_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S22>/Chart' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c9_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c9_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S27>/Chart' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c7_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c7_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S18>/Chart' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c2_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c2_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S18>/turn_adjus' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c1_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S3>/turn_adjus' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c5_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c5_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S40>/Chart' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c4_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c4_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S3>/draft2_Maze_exploration1' */
      MicroMouse_Deploy_DW.bitsForTID0.is_active_c6_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID0.is_c6_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S3>/draft2_Maze_exploration2' */
      MicroMouse_Deploy_DW.bitsForTID0.is_active_c10_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID0.is_c10_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S24>/engage_disengage_straightnert' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c3_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* Start for MATLABSystem: '<S23>/Median Filter' */
      MicroMouse_Deploy_DW.obj_o.matlabCodegenIsDeleted = false;
      MicroMouse_Deploy_DW.obj_o.isInitialized = 1;
      MicroMouse_Deploy_DW.obj_o.NumChannels = 1;
      MicroMouse_Deploy_DW.obj_o.pMID.isInitialized = 0;
      MicroMouse_Deploy_DW.obj_o.isSetupComplete = true;

      /* Start for MATLABSystem: '<S24>/Moving Average' */
      MicroMouse_Deploy_DW.obj_b.isInitialized = 0;
      MicroMouse_Deploy_DW.obj_b.NumChannels = -1;
      MicroMouse_Deploy_DW.obj_b.FrameLength = -1;
      MicroMouse_Deploy_DW.obj_b.matlabCodegenIsDeleted = false;
      MicroMouse_Dep_SystemCore_setup(&MicroMouse_Deploy_DW.obj_b);

      /* InitializeConditions for MATLABSystem: '<S24>/Moving Average' */
      MicroMouse_Deploy_DW.obj_b.pCumSum = 0.0F;
      for (i = 0; i < 299; i++) {
        MicroMouse_Deploy_DW.obj_b.pCumSumRev[i] = 0.0F;
      }

      MicroMouse_Deploy_DW.obj_b.pCumRevIndex = 1.0F;
      MicroMouse_Deploy_DW.obj_b.pModValueRev = 0.0F;

      /* End of InitializeConditions for MATLABSystem: '<S24>/Moving Average' */

      /* Start for MATLABSystem: '<S24>/Moving Average1' */
      MicroMouse_Deploy_DW.obj.isInitialized = 0;
      MicroMouse_Deploy_DW.obj.NumChannels = -1;
      MicroMouse_Deploy_DW.obj.FrameLength = -1;
      MicroMouse_Deploy_DW.obj.matlabCodegenIsDeleted = false;
      MicroMouse_Dep_SystemCore_setup(&MicroMouse_Deploy_DW.obj);

      /* InitializeConditions for MATLABSystem: '<S24>/Moving Average1' */
      MicroMouse_Deploy_DW.obj.pCumSum = 0.0F;
      for (i = 0; i < 299; i++) {
        MicroMouse_Deploy_DW.obj.pCumSumRev[i] = 0.0F;
      }

      MicroMouse_Deploy_DW.obj.pCumRevIndex = 1.0F;
      MicroMouse_Deploy_DW.obj.pModValueRev = 0.0F;

      /* End of InitializeConditions for MATLABSystem: '<S24>/Moving Average1' */
    }

    /* Enable for Chart: '<S3>/draft2_Maze_exploration' */
    MicroMouse_Deploy_DW.previousTime = MicroMouse_Deploy_M->Timing.t[0];
  }

  {
  }
}

/* Model terminate function */
void MicroMouse_Deploy_terminate(void)
{
  /* Terminate for MATLABSystem: '<S23>/Median Filter' */
  if (!MicroMouse_Deploy_DW.obj_o.matlabCodegenIsDeleted) {
    MicroMouse_Deploy_DW.obj_o.matlabCodegenIsDeleted = true;
    if ((MicroMouse_Deploy_DW.obj_o.isInitialized == 1) &&
        MicroMouse_Deploy_DW.obj_o.isSetupComplete) {
      MicroMouse_Deploy_DW.obj_o.NumChannels = -1;
      if (MicroMouse_Deploy_DW.obj_o.pMID.isInitialized == 1) {
        MicroMouse_Deploy_DW.obj_o.pMID.isInitialized = 2;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S23>/Median Filter' */

  /* Terminate for MATLABSystem: '<S24>/Moving Average' */
  if (!MicroMouse_Deploy_DW.obj_b.matlabCodegenIsDeleted) {
    MicroMouse_Deploy_DW.obj_b.matlabCodegenIsDeleted = true;
    if ((MicroMouse_Deploy_DW.obj_b.isInitialized == 1) &&
        MicroMouse_Deploy_DW.obj_b.isSetupComplete) {
      MicroMouse_Deploy_DW.obj_b.NumChannels = -1;
      MicroMouse_Deploy_DW.obj_b.FrameLength = -1;
    }
  }

  /* End of Terminate for MATLABSystem: '<S24>/Moving Average' */

  /* Terminate for MATLABSystem: '<S24>/Moving Average1' */
  if (!MicroMouse_Deploy_DW.obj.matlabCodegenIsDeleted) {
    MicroMouse_Deploy_DW.obj.matlabCodegenIsDeleted = true;
    if ((MicroMouse_Deploy_DW.obj.isInitialized == 1) &&
        MicroMouse_Deploy_DW.obj.isSetupComplete) {
      MicroMouse_Deploy_DW.obj.NumChannels = -1;
      MicroMouse_Deploy_DW.obj.FrameLength = -1;
    }
  }

  /* End of Terminate for MATLABSystem: '<S24>/Moving Average1' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
