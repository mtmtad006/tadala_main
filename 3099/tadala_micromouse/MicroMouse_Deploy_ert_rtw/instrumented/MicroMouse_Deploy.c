/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy.c
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

#include "MicroMouse_Deploy.h"
#include "MicroMouse_Deploy_types.h"
#include "rtwtypes.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "MicroMouse_Deploy_private.h"
#include "code_profiling_utility_functions.h"

/* Named constants for Chart: '<S14>/turn_adjus' */
#define MicroMouse_D_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define MicroMouse_Deplo_IN_AdjustRight ((uint8_T)2U)
#define MicroMouse_Deploy_IN_AdjustLeft ((uint8_T)1U)
#define MicroMouse_Deploy_IN_deafult   ((uint8_T)3U)

/* Named constants for Chart: '<S3>/distance_control' */
#define MicroMouse_Deploy_IN_Run       ((uint8_T)1U)
#define MicroMouse_Deploy_IN_Stop      ((uint8_T)2U)

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
  int_T nXc = 3;
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
    /* Start for MATLABSystem: '<S18>/Median Filter' */
    obj->pBuf[i] = 0.0;
    obj->pPos[i] = 0.0;
    obj->pHeap[i] = 0.0;
  }

  /* Start for MATLABSystem: '<S18>/Median Filter' */
  obj->pWinLen = 5.0;
  obj->pIdx = obj->pWinLen;

  /* Start for MATLABSystem: '<S18>/Median Filter' */
  obj->pMidHeap = ceil((obj->pWinLen + 1.0) / 2.0);
  obj->pMinHeapLength = trunc((obj->pWinLen - 1.0) / 2.0);
  obj->pMaxHeapLength = trunc(obj->pWinLen / 2.0);
  cnt1 = 1;
  cnt2 = obj->pWinLen;
  for (i = 0; i < 5; i++) {
    /* Start for MATLABSystem: '<S18>/Median Filter' */
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
    if (obj->pBuf[(int32_T)tmp - 1] >= obj->pBuf[(int32_T)tmp_0 - 1]) {
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
    if (obj->pBuf[(int32_T)tmp - 1] >= obj->pBuf[(int32_T)tmp_0 - 1]) {
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

/* Model step function */
void MicroMouse_Deploy_step(void)
{
  /* local block i/o variables */
  real32_T rtb_uT;
  real32_T rtb_uT_p;
  real32_T rtb_uT_e;
  uint8_T rtb_Compare;
  boolean_T rtb_LogicalOperator;
  boolean_T rtb_LogicalOperator_c;
  boolean_T rtb_LogicalOperator_a;
  real_T Sum;
  real_T ind1;
  real_T ind2;
  real_T p;
  real_T rtb_Product1;
  real_T rtb_Product2_d;
  real_T rtb_Sum_l;
  real_T rtb_d;
  real_T rtb_d_n;
  real_T rtb_p;
  real_T rtb_p_p;
  real_T rtb_q;
  real_T rtb_q_m;
  real_T rtb_r;
  real_T rtb_r_l;
  real_T tmp_0;
  real_T vprev;
  real32_T rtb_K;
  real32_T rtb_K_i;
  real32_T rtb_K_n;
  real32_T rtb_Max;
  real32_T rtb_Saturation;
  real32_T rtb_Saturation_e;
  real32_T rtb_Saturation_g;
  char_T rtb_ComposeString1_0[256];
  char_T rtb_ComposeString2_0[256];
  char_T rtb_ComposeString4_0[256];
  char_T rtb_ComposeString_0[256];
  boolean_T exitg1;
  boolean_T tmp;
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
  if (tmp) {
    /* RelationalOperator: '<S96>/Compare' incorporates:
     *  Constant: '<S94>/Constant'
     *  Constant: '<S96>/Constant'
     *  DataStoreRead: '<S4>/Data Store Read4'
     *  DataTypeConversion: '<S4>/Cast To Single'
     *  Gain: '<S4>/Gain2'
     *  RelationalOperator: '<S94>/Compare'
     */
    rtb_Compare = (uint8_T)((MicroMouse_Deploy_P.Gain2_Gain * (real32_T)
      V_PHOTO_MOT_RS >= MicroMouse_Deploy_P.CompareToConstant_const_l) >
      (int32_T)MicroMouse_Deploy_P.Constant_Value_d);

    /* Sum: '<S15>/Add' incorporates:
     *  Delay: '<S15>/Delay'
     *  RelationalOperator: '<S95>/FixPt Relational Operator'
     *  UnitDelay: '<S95>/Delay Input1'
     *
     * Block description for '<S95>/Delay Input1':
     *
     *  Store in Global RAM
     */
    MicroMouse_Deploy_DW.Delay_DSTATE = (uint8_T)((uint32_T)(rtb_Compare >
      MicroMouse_Deploy_DW.DelayInput1_DSTATE) +
      MicroMouse_Deploy_DW.Delay_DSTATE);

    /* Chart: '<S3>/distance_control' incorporates:
     *  Constant: '<S17>/Circumference'
     *  Constant: '<S3>/ref'
     *  Delay: '<S15>/Delay'
     *  Gain: '<S17>/Tick_per_rev'
     *  Product: '<S17>/Product'
     */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c3_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c3_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_Run;
    } else if (MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy ==
               MicroMouse_Deploy_IN_Run) {
      if ((real_T)(MicroMouse_Deploy_P.Tick_per_rev_Gain *
                   MicroMouse_Deploy_DW.Delay_DSTATE) * 0.0009765625 *
          MicroMouse_Deploy_P.Circumference_Value >=
          MicroMouse_Deploy_P.ref_Value) {
        MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy =
          MicroMouse_Deploy_IN_Stop;
      } else {
        MicroMouse_Deploy_B.ang_vel = 70.0;
      }
    } else {
      /* case IN_Stop: */
      MicroMouse_Deploy_B.ang_vel = 0.0;
    }

    /* End of Chart: '<S3>/distance_control' */
  }

  /* Sum: '<S14>/Error' incorporates:
   *  Constant: '<S14>/Constant'
   *  Integrator: '<S14>/Integrator'
   */
  rtb_Sum_l = MicroMouse_Deploy_P.Constant_Value -
    MicroMouse_Deploy_X.Integrator_CSTATE;

  /* Gain: '<S79>/Filter Coefficient' incorporates:
   *  Gain: '<S69>/Derivative Gain'
   *  Integrator: '<S71>/Filter'
   *  Sum: '<S71>/SumD'
   */
  MicroMouse_Deploy_B.FilterCoefficient = (MicroMouse_Deploy_P.PIDController_D *
    rtb_Sum_l - MicroMouse_Deploy_X.Filter_CSTATE) *
    MicroMouse_Deploy_P.PIDController_N;

  /* Sum: '<S85>/Sum' incorporates:
   *  Gain: '<S81>/Proportional Gain'
   *  Integrator: '<S76>/Integrator'
   */
  Sum = (MicroMouse_Deploy_P.PIDController_P * rtb_Sum_l +
         MicroMouse_Deploy_X.Integrator_CSTATE_p) +
    MicroMouse_Deploy_B.FilterCoefficient;
  if (tmp) {
    /* Chart: '<S14>/turn_adjus' */
    if (MicroMouse_Deploy_DW.bitsForTID1.is_active_c1_MicroMouse_Deploy == 0) {
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c1_MicroMouse_Deploy = 1U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_deafult;
    } else {
      switch (MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy) {
       case MicroMouse_Deploy_IN_AdjustLeft:
        MicroMouse_Deploy_B.RW_w = Sum;
        MicroMouse_Deploy_B.LW_w = 0.0;
        break;

       case MicroMouse_Deplo_IN_AdjustRight:
        MicroMouse_Deploy_B.LW_w = Sum;
        MicroMouse_Deploy_B.RW_w = 0.0;
        break;

       default:
        /* case IN_deafult: */
        if (Sum < 0.0) {
          MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy =
            MicroMouse_Deploy_IN_AdjustLeft;
        } else if (Sum > 0.0) {
          MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy =
            MicroMouse_Deplo_IN_AdjustRight;
        } else {
          MicroMouse_Deploy_B.LW_w = 0.0;
          MicroMouse_Deploy_B.RW_w = 0.0;
        }
        break;
      }
    }

    /* End of Chart: '<S14>/turn_adjus' */

    /* DataTypeConversion: '<S1>/Cast1' incorporates:
     *  DataStoreWrite: '<S1>/Data Store Write1'
     *  Gain: '<Root>/Gain1'
     *  Sum: '<S3>/Sum'
     */
    rtb_Product1 = fmod(floor((MicroMouse_Deploy_B.ang_vel +
      MicroMouse_Deploy_B.RW_w) * MicroMouse_Deploy_P.Gain1_Gain), 256.0);
    MOTOR_RS = (int8_T)(rtb_Product1 < 0.0 ? (int32_T)(int8_T)-(int8_T)(uint8_T)
                        -rtb_Product1 : (int32_T)(int8_T)(uint8_T)rtb_Product1);

    /* DataTypeConversion: '<S1>/Cast' incorporates:
     *  DataStoreWrite: '<S1>/Data Store Write'
     *  Gain: '<Root>/Gain'
     *  Sum: '<S3>/Sum1'
     */
    rtb_Product1 = fmod(floor((MicroMouse_Deploy_B.ang_vel +
      MicroMouse_Deploy_B.LW_w) * MicroMouse_Deploy_P.Gain_Gain), 256.0);
    MOTOR_LS = (int8_T)(rtb_Product1 < 0.0 ? (int32_T)(int8_T)-(int8_T)(uint8_T)
                        -rtb_Product1 : (int32_T)(int8_T)(uint8_T)rtb_Product1);

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

    /* Logic: '<S19>/Logical Operator' incorporates:
     *  Constant: '<S19>/Constant'
     *  Constant: '<S19>/Time constant'
     *  Constant: '<S22>/Constant'
     *  Constant: '<S23>/Constant'
     *  RelationalOperator: '<S22>/Compare'
     *  RelationalOperator: '<S23>/Compare'
     *  Sum: '<S19>/Sum1'
     */
    rtb_LogicalOperator = (((real32_T)
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContinuo -
       MicroMouse_Deploy_B.Probe[0]) <= MicroMouse_Deploy_P.Constant_Value_lf) &&
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_k <
       MicroMouse_Deploy_P.CompareToConstant_const));

    /* Gain: '<S11>/K' incorporates:
     *  DataStoreRead: '<S10>/Data Store Read'
     *  DataTypeConversion: '<S10>/Cast To Single'
     *  Gain: '<S10>/Gain'
     */
    rtb_K = MicroMouse_Deploy_P.Gain_Gain_g * (real32_T)TOF_Distance[0] *
      MicroMouse_Deploy_P.LowPassFilterDiscreteorConti_pu;

    /* DiscreteIntegrator: '<S25>/Integrator' */
    if (MicroMouse_Deploy_DW.Integrator_IC_LOADING != 0) {
      MicroMouse_Deploy_DW.Integrator_DSTATE = rtb_K;
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
      MicroMouse_Deploy_DW.Integrator_DSTATE = rtb_K;
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

    /* Saturate: '<S25>/Saturation' incorporates:
     *  DiscreteIntegrator: '<S25>/Integrator'
     */
    if (MicroMouse_Deploy_DW.Integrator_DSTATE >
        MicroMouse_Deploy_P.Saturation_UpperSat) {
      /* Saturate: '<S25>/Saturation' */
      rtb_Saturation = MicroMouse_Deploy_P.Saturation_UpperSat;
    } else if (MicroMouse_Deploy_DW.Integrator_DSTATE <
               MicroMouse_Deploy_P.Saturation_LowerSat) {
      /* Saturate: '<S25>/Saturation' */
      rtb_Saturation = MicroMouse_Deploy_P.Saturation_LowerSat;
    } else {
      /* Saturate: '<S25>/Saturation' */
      rtb_Saturation = MicroMouse_Deploy_DW.Integrator_DSTATE;
    }

    /* End of Saturate: '<S25>/Saturation' */

    /* Logic: '<S26>/Logical Operator' incorporates:
     *  Constant: '<S26>/Constant'
     *  Constant: '<S26>/Time constant'
     *  Constant: '<S29>/Constant'
     *  Constant: '<S30>/Constant'
     *  RelationalOperator: '<S29>/Compare'
     *  RelationalOperator: '<S30>/Compare'
     *  Sum: '<S26>/Sum1'
     */
    rtb_LogicalOperator_c = (((real32_T)
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_p -
       MicroMouse_Deploy_B.Probe_h[0]) <= MicroMouse_Deploy_P.Constant_Value_i) &&
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_c <
       MicroMouse_Deploy_P.CompareToConstant_const_j));

    /* Gain: '<S12>/K' incorporates:
     *  DataStoreRead: '<S10>/Data Store Read'
     *  DataTypeConversion: '<S10>/Cast To Single'
     *  Gain: '<S10>/Gain'
     */
    rtb_K_i = MicroMouse_Deploy_P.Gain_Gain_g * (real32_T)TOF_Distance[1] *
      MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_a;

    /* DiscreteIntegrator: '<S32>/Integrator' */
    if (MicroMouse_Deploy_DW.Integrator_IC_LOADING_k != 0) {
      MicroMouse_Deploy_DW.Integrator_DSTATE_m = rtb_K_i;
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
      MicroMouse_Deploy_DW.Integrator_DSTATE_m = rtb_K_i;
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

    /* Saturate: '<S32>/Saturation' incorporates:
     *  DiscreteIntegrator: '<S32>/Integrator'
     */
    if (MicroMouse_Deploy_DW.Integrator_DSTATE_m >
        MicroMouse_Deploy_P.Saturation_UpperSat_f) {
      /* Saturate: '<S32>/Saturation' */
      rtb_Saturation_e = MicroMouse_Deploy_P.Saturation_UpperSat_f;
    } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_m <
               MicroMouse_Deploy_P.Saturation_LowerSat_b) {
      /* Saturate: '<S32>/Saturation' */
      rtb_Saturation_e = MicroMouse_Deploy_P.Saturation_LowerSat_b;
    } else {
      /* Saturate: '<S32>/Saturation' */
      rtb_Saturation_e = MicroMouse_Deploy_DW.Integrator_DSTATE_m;
    }

    /* End of Saturate: '<S32>/Saturation' */

    /* Logic: '<S33>/Logical Operator' incorporates:
     *  Constant: '<S33>/Constant'
     *  Constant: '<S33>/Time constant'
     *  Constant: '<S36>/Constant'
     *  Constant: '<S37>/Constant'
     *  RelationalOperator: '<S36>/Compare'
     *  RelationalOperator: '<S37>/Compare'
     *  Sum: '<S33>/Sum1'
     */
    rtb_LogicalOperator_a = (((real32_T)
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_e -
       MicroMouse_Deploy_B.Probe_g[0]) <= MicroMouse_Deploy_P.Constant_Value_c) &&
      (MicroMouse_Deploy_P.LowPassFilterDiscreteorConti_cz <
       MicroMouse_Deploy_P.CompareToConstant_const_o));

    /* Gain: '<S13>/K' incorporates:
     *  DataStoreRead: '<S10>/Data Store Read'
     *  DataTypeConversion: '<S10>/Cast To Single'
     *  Gain: '<S10>/Gain'
     */
    rtb_K_n = MicroMouse_Deploy_P.Gain_Gain_g * (real32_T)TOF_Distance[2] *
      MicroMouse_Deploy_P.LowPassFilterDiscreteorConti_cj;

    /* DiscreteIntegrator: '<S39>/Integrator' */
    if (MicroMouse_Deploy_DW.Integrator_IC_LOADING_a != 0) {
      MicroMouse_Deploy_DW.Integrator_DSTATE_i = rtb_K_n;
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
      MicroMouse_Deploy_DW.Integrator_DSTATE_i = rtb_K_n;
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

    /* Saturate: '<S39>/Saturation' incorporates:
     *  DiscreteIntegrator: '<S39>/Integrator'
     */
    if (MicroMouse_Deploy_DW.Integrator_DSTATE_i >
        MicroMouse_Deploy_P.Saturation_UpperSat_c) {
      /* Saturate: '<S39>/Saturation' */
      rtb_Saturation_g = MicroMouse_Deploy_P.Saturation_UpperSat_c;
    } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_i <
               MicroMouse_Deploy_P.Saturation_LowerSat_o) {
      /* Saturate: '<S39>/Saturation' */
      rtb_Saturation_g = MicroMouse_Deploy_P.Saturation_LowerSat_o;
    } else {
      /* Saturate: '<S39>/Saturation' */
      rtb_Saturation_g = MicroMouse_Deploy_DW.Integrator_DSTATE_i;
    }

    /* End of Saturate: '<S39>/Saturation' */

    /* ComposeString: '<S3>/Compose String' incorporates:
     *  Saturate: '<S25>/Saturation'
     *  Saturate: '<S32>/Saturation'
     *  Saturate: '<S39>/Saturation'
     */
    snprintf(&rtb_ComposeString_0[0], 256U, "L%1.2f F%1.2f R%1.2f",
             rtb_Saturation, rtb_Saturation_e, rtb_Saturation_g);

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
     *  Constant: '<S3>/Constant1'
     *  DataStoreWrite: '<S6>/Data Store Write'
     *  RelationalOperator: '<S3>/GreaterThan'
     *  RelationalOperator: '<S3>/GreaterThan1'
     *  RelationalOperator: '<S3>/GreaterThan2'
     */
    LED[0] = (uint8_T)(rtb_Saturation <= MicroMouse_Deploy_P.Constant1_Value);
    LED[1] = (uint8_T)(rtb_Saturation_e <= MicroMouse_Deploy_P.Constant1_Value);
    LED[2] = (uint8_T)(rtb_Saturation_g <= MicroMouse_Deploy_P.Constant1_Value);

    /* DataTypeConversion: '<S9>/Cast To Boolean' incorporates:
     *  Constant: '<S3>/Constant'
     *  DataStoreWrite: '<S9>/Data Store Write'
     */
    rtb_Product1 = fmod(floor(MicroMouse_Deploy_P.Constant_Value_l), 256.0);
    STATE = (uint8_T)(rtb_Product1 < 0.0 ? (int32_T)(uint8_T)-(int8_T)(uint8_T)
                      -rtb_Product1 : (int32_T)(uint8_T)rtb_Product1);

    /* MinMax: '<S19>/Max' incorporates:
     *  Constant: '<S19>/Time constant'
     */
    rtb_Max = (real32_T)fmax(MicroMouse_Deploy_B.Probe[0],
      MicroMouse_Deploy_P.LowPassFilterDiscreteorContinuo);

    /* Product: '<S11>/1//T' incorporates:
     *  Fcn: '<S19>/Avoid Divide by Zero'
     *  Sum: '<S11>/Sum1'
     */
    rtb_uT = 1.0F / ((real32_T)(rtb_Max == 0.0F) * 2.22044605e-16F + rtb_Max) *
      (rtb_K - rtb_Saturation);

    /* MinMax: '<S26>/Max' incorporates:
     *  Constant: '<S26>/Time constant'
     */
    rtb_Max = (real32_T)fmax(MicroMouse_Deploy_B.Probe_h[0],
      MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_p);

    /* Product: '<S12>/1//T' incorporates:
     *  Fcn: '<S26>/Avoid Divide by Zero'
     *  Sum: '<S12>/Sum1'
     */
    rtb_uT_p = 1.0F / ((real32_T)(rtb_Max == 0.0F) * 2.22044605e-16F + rtb_Max) *
      (rtb_K_i - rtb_Saturation_e);

    /* MinMax: '<S33>/Max' incorporates:
     *  Constant: '<S33>/Time constant'
     */
    rtb_Max = (real32_T)fmax(MicroMouse_Deploy_B.Probe_g[0],
      MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_e);

    /* Product: '<S13>/1//T' incorporates:
     *  Fcn: '<S33>/Avoid Divide by Zero'
     *  Sum: '<S13>/Sum1'
     */
    rtb_uT_e = 1.0F / ((real32_T)(rtb_Max == 0.0F) * 2.22044605e-16F + rtb_Max) *
      (rtb_K_n - rtb_Saturation_g);

    /* UnitDelay: '<S98>/Unit Delay1' */
    MicroMouse_Deploy_B.UnitDelay1 = MicroMouse_Deploy_DW.UnitDelay1_DSTATE;

    /* Gain: '<S98>/tau' incorporates:
     *  Constant: '<S3>/Constant2'
     */
    rtb_p_p = MicroMouse_Deploy_P.tau_Gain * MicroMouse_Deploy_P.Constant2_Value;

    /* Product: '<S100>/Product2' */
    rtb_q = rtb_p_p * rtb_p_p;

    /* Bias: '<S100>/Bias' */
    rtb_r = rtb_q + MicroMouse_Deploy_P.Bias_Bias;

    /* Gain: '<S100>/Gain' */
    rtb_p_p *= MicroMouse_Deploy_P.Gain_Gain_i;

    /* Sum: '<S100>/Add1' */
    rtb_d = rtb_r + rtb_p_p;

    /* Product: '<S100>/Product3' */
    MicroMouse_Deploy_B.Product3 = rtb_q / rtb_d;

    /* UnitDelay: '<S99>/Unit Delay1' */
    MicroMouse_Deploy_B.UnitDelay1_h = MicroMouse_Deploy_DW.UnitDelay1_DSTATE_a;

    /* Gain: '<S99>/tau' incorporates:
     *  Constant: '<S3>/Constant2'
     */
    rtb_p = MicroMouse_Deploy_P.tau_Gain_m * MicroMouse_Deploy_P.Constant2_Value;

    /* Product: '<S101>/Product2' */
    rtb_q_m = rtb_p * rtb_p;

    /* Bias: '<S101>/Bias' */
    rtb_r_l = rtb_q_m + MicroMouse_Deploy_P.Bias_Bias_l;

    /* Gain: '<S101>/Gain' */
    rtb_p *= MicroMouse_Deploy_P.Gain_Gain_d;

    /* Sum: '<S101>/Add1' */
    rtb_d_n = rtb_r_l + rtb_p;

    /* Product: '<S101>/Product3' */
    MicroMouse_Deploy_B.Product3_b = rtb_q_m / rtb_d_n;
  }

  /* Gain: '<S73>/Integral Gain' */
  MicroMouse_Deploy_B.IntegralGain = MicroMouse_Deploy_P.PIDController_I *
    rtb_Sum_l;

  /* Sum: '<S98>/Sum' incorporates:
   *  DataStoreRead: '<S5>/Data Store Read1'
   *  DataTypeConversion: '<S3>/Data Type Conversion'
   */
  rtb_Product2_d = IMU_Gyro[2] + MicroMouse_Deploy_B.UnitDelay1;

  /* Product: '<S98>/Product1' */
  Sum = rtb_Product2_d * MicroMouse_Deploy_B.Product3;

  /* Sum: '<S99>/Sum' */
  rtb_Sum_l = Sum + MicroMouse_Deploy_B.UnitDelay1_h;

  /* Product: '<S99>/Product1' */
  rtb_Product1 = rtb_Sum_l * MicroMouse_Deploy_B.Product3_b;

  /* MATLABSystem: '<S18>/Median Filter' */
  if (MicroMouse_Deploy_DW.obj.pMID.isInitialized != 1) {
    MicroMouse_Deploy_DW.obj.pMID.isInitialized = 1;
    MicroMouse_Deploy_DW.obj.pMID.isSetupComplete = true;
    MicroMou_MedianFilter_resetImpl(&MicroMouse_Deploy_DW.obj.pMID);
  }

  vprev = MicroMouse_Deploy_DW.obj.pMID.pBuf[(int32_T)
    MicroMouse_Deploy_DW.obj.pMID.pIdx - 1];
  MicroMouse_Deploy_DW.obj.pMID.pBuf[(int32_T)MicroMouse_Deploy_DW.obj.pMID.pIdx
    - 1] = rtb_Product1;
  p = MicroMouse_Deploy_DW.obj.pMID.pPos[(int32_T)
    MicroMouse_Deploy_DW.obj.pMID.pIdx - 1];
  MicroMouse_Deploy_DW.obj.pMID.pIdx++;
  if (MicroMouse_Deploy_DW.obj.pMID.pWinLen + 1.0 ==
      MicroMouse_Deploy_DW.obj.pMID.pIdx) {
    MicroMouse_Deploy_DW.obj.pMID.pIdx = 1.0;
  }

  if (p > MicroMouse_Deploy_DW.obj.pMID.pMidHeap) {
    if (vprev < rtb_Product1) {
      Mic_MedianFilter_trickleDownMin(&MicroMouse_Deploy_DW.obj.pMID, (p -
        MicroMouse_Deploy_DW.obj.pMID.pMidHeap) * 2.0);
    } else {
      vprev = p - MicroMouse_Deploy_DW.obj.pMID.pMidHeap;
      exitg1 = false;
      while ((!exitg1) && (vprev > 0.0)) {
        p = vprev / 2.0;
        ind1 = vprev + MicroMouse_Deploy_DW.obj.pMID.pMidHeap;
        ind2 = trunc(p) + MicroMouse_Deploy_DW.obj.pMID.pMidHeap;
        rtb_Product1 = MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind1 - 1];
        tmp_0 = MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind2 - 1];
        if (MicroMouse_Deploy_DW.obj.pMID.pBuf[(int32_T)rtb_Product1 - 1] >=
            MicroMouse_Deploy_DW.obj.pMID.pBuf[(int32_T)tmp_0 - 1]) {
          exitg1 = true;
        } else {
          MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind1 - 1] = tmp_0;
          MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind2 - 1] = rtb_Product1;
          MicroMouse_Deploy_DW.obj.pMID.pPos[(int32_T)
            MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind1 - 1] - 1] = ind1;
          MicroMouse_Deploy_DW.obj.pMID.pPos[(int32_T)
            MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind2 - 1] - 1] = ind2;
          vprev = trunc(p);
        }
      }

      if (vprev == 0.0) {
        Mic_MedianFilter_trickleDownMax(&MicroMouse_Deploy_DW.obj.pMID, -1.0);
      }
    }
  } else if (p < MicroMouse_Deploy_DW.obj.pMID.pMidHeap) {
    if (rtb_Product1 < vprev) {
      Mic_MedianFilter_trickleDownMax(&MicroMouse_Deploy_DW.obj.pMID, (p -
        MicroMouse_Deploy_DW.obj.pMID.pMidHeap) * 2.0);
    } else {
      vprev = p - MicroMouse_Deploy_DW.obj.pMID.pMidHeap;
      exitg1 = false;
      while ((!exitg1) && (vprev < 0.0)) {
        p = vprev / 2.0;
        ind1 = trunc(p) + MicroMouse_Deploy_DW.obj.pMID.pMidHeap;
        ind2 = vprev + MicroMouse_Deploy_DW.obj.pMID.pMidHeap;
        rtb_Product1 = MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind1 - 1];
        tmp_0 = MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind2 - 1];
        if (MicroMouse_Deploy_DW.obj.pMID.pBuf[(int32_T)rtb_Product1 - 1] >=
            MicroMouse_Deploy_DW.obj.pMID.pBuf[(int32_T)tmp_0 - 1]) {
          exitg1 = true;
        } else {
          MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind1 - 1] = tmp_0;
          MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind2 - 1] = rtb_Product1;
          MicroMouse_Deploy_DW.obj.pMID.pPos[(int32_T)
            MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind1 - 1] - 1] = ind1;
          MicroMouse_Deploy_DW.obj.pMID.pPos[(int32_T)
            MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)ind2 - 1] - 1] = ind2;
          vprev = trunc(p);
        }
      }

      if (vprev == 0.0) {
        Mic_MedianFilter_trickleDownMin(&MicroMouse_Deploy_DW.obj.pMID, 1.0);
      }
    }
  } else {
    if (MicroMouse_Deploy_DW.obj.pMID.pMaxHeapLength != 0.0) {
      Mic_MedianFilter_trickleDownMax(&MicroMouse_Deploy_DW.obj.pMID, -1.0);
    }

    if (MicroMouse_Deploy_DW.obj.pMID.pMinHeapLength > 0.0) {
      Mic_MedianFilter_trickleDownMin(&MicroMouse_Deploy_DW.obj.pMID, 1.0);
    }
  }

  rtb_Product1 = MicroMouse_Deploy_DW.obj.pMID.pBuf[(int32_T)
    MicroMouse_Deploy_DW.obj.pMID.pHeap[(int32_T)
    MicroMouse_Deploy_DW.obj.pMID.pMidHeap - 1] - 1];

  /* DeadZone: '<S18>/Dead Zone' incorporates:
   *  MATLABSystem: '<S18>/Median Filter'
   */
  if (rtb_Product1 > MicroMouse_Deploy_P.DeadZone_End) {
    /* DeadZone: '<S18>/Dead Zone' */
    MicroMouse_Deploy_B.DeadZone = rtb_Product1 -
      MicroMouse_Deploy_P.DeadZone_End;
  } else if (rtb_Product1 >= MicroMouse_Deploy_P.DeadZone_Start) {
    /* DeadZone: '<S18>/Dead Zone' */
    MicroMouse_Deploy_B.DeadZone = 0.0;
  } else {
    /* DeadZone: '<S18>/Dead Zone' */
    MicroMouse_Deploy_B.DeadZone = rtb_Product1 -
      MicroMouse_Deploy_P.DeadZone_Start;
  }

  /* End of DeadZone: '<S18>/Dead Zone' */
  if (tmp) {
    /* Product: '<S100>/Product4' incorporates:
     *  Bias: '<S100>/Bias1'
     *  UnaryMinus: '<S100>/Unary Minus'
     */
    MicroMouse_Deploy_B.Product4 = (-rtb_q + MicroMouse_Deploy_P.Bias1_Bias) /
      rtb_d;

    /* Product: '<S100>/Product5' incorporates:
     *  Sum: '<S100>/Add3'
     */
    MicroMouse_Deploy_B.Product5 = 1.0 / rtb_d * (rtb_r - rtb_p_p);

    /* UnitDelay: '<S98>/Unit Delay2' */
    MicroMouse_Deploy_B.UnitDelay2 = MicroMouse_Deploy_DW.UnitDelay2_DSTATE;

    /* Product: '<S101>/Product4' incorporates:
     *  Bias: '<S101>/Bias1'
     *  UnaryMinus: '<S101>/Unary Minus'
     */
    MicroMouse_Deploy_B.Product4_l = (-rtb_q_m +
      MicroMouse_Deploy_P.Bias1_Bias_d) / rtb_d_n;

    /* Product: '<S101>/Product5' incorporates:
     *  Sum: '<S101>/Add3'
     */
    MicroMouse_Deploy_B.Product5_a = 1.0 / rtb_d_n * (rtb_r_l - rtb_p);

    /* UnitDelay: '<S99>/Unit Delay2' */
    MicroMouse_Deploy_B.UnitDelay2_i = MicroMouse_Deploy_DW.UnitDelay2_DSTATE_o;
  }

  /* Sum: '<S98>/Sum1' incorporates:
   *  DataStoreRead: '<S5>/Data Store Read1'
   *  DataTypeConversion: '<S3>/Data Type Conversion'
   *  Gain: '<S98>/Gain1'
   *  Product: '<S98>/Product5'
   *  Sum: '<S98>/Sum2'
   */
  MicroMouse_Deploy_B.Sum1 = (rtb_Product2_d * MicroMouse_Deploy_B.Product4 +
    IMU_Gyro[2]) * MicroMouse_Deploy_P.Gain1_Gain_f +
    MicroMouse_Deploy_B.UnitDelay2;

  /* Sum: '<S98>/Sum3' incorporates:
   *  DataStoreRead: '<S5>/Data Store Read1'
   *  DataTypeConversion: '<S3>/Data Type Conversion'
   *  Product: '<S98>/Product2'
   */
  MicroMouse_Deploy_B.Sum3 = IMU_Gyro[2] - rtb_Product2_d *
    MicroMouse_Deploy_B.Product5;

  /* Sum: '<S99>/Sum1' incorporates:
   *  Gain: '<S99>/Gain1'
   *  Product: '<S99>/Product5'
   *  Sum: '<S99>/Sum2'
   */
  MicroMouse_Deploy_B.Sum1_k = (rtb_Sum_l * MicroMouse_Deploy_B.Product4_l + Sum)
    * MicroMouse_Deploy_P.Gain1_Gain_e + MicroMouse_Deploy_B.UnitDelay2_i;

  /* Sum: '<S99>/Sum3' incorporates:
   *  Product: '<S99>/Product2'
   */
  MicroMouse_Deploy_B.Sum3_n = Sum - rtb_Sum_l * MicroMouse_Deploy_B.Product5_a;
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
          /* Update for UnitDelay: '<S95>/Delay Input1'
           *
           * Block description for '<S95>/Delay Input1':
           *
           *  Store in Global RAM
           */
          MicroMouse_Deploy_DW.DelayInput1_DSTATE = rtb_Compare;

          /* Update for DiscreteIntegrator: '<S25>/Integrator' */
          MicroMouse_Deploy_DW.Integrator_IC_LOADING = 0U;
          MicroMouse_Deploy_DW.Integrator_DSTATE +=
            MicroMouse_Deploy_P.Integrator_gainval * rtb_uT;
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

          /* End of Update for DiscreteIntegrator: '<S25>/Integrator' */

          /* Update for DiscreteIntegrator: '<S32>/Integrator' */
          MicroMouse_Deploy_DW.Integrator_IC_LOADING_k = 0U;
          MicroMouse_Deploy_DW.Integrator_DSTATE_m +=
            MicroMouse_Deploy_P.Integrator_gainval_n * rtb_uT_p;
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

          /* End of Update for DiscreteIntegrator: '<S32>/Integrator' */

          /* Update for DiscreteIntegrator: '<S39>/Integrator' */
          MicroMouse_Deploy_DW.Integrator_IC_LOADING_a = 0U;
          MicroMouse_Deploy_DW.Integrator_DSTATE_i +=
            MicroMouse_Deploy_P.Integrator_gainval_k * rtb_uT_e;
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

          /* End of Update for DiscreteIntegrator: '<S39>/Integrator' */

          /* Update for UnitDelay: '<S98>/Unit Delay1' */
          MicroMouse_Deploy_DW.UnitDelay1_DSTATE = MicroMouse_Deploy_B.Sum1;

          /* Update for UnitDelay: '<S99>/Unit Delay1' */
          MicroMouse_Deploy_DW.UnitDelay1_DSTATE_a = MicroMouse_Deploy_B.Sum1_k;

          /* Update for UnitDelay: '<S98>/Unit Delay2' */
          MicroMouse_Deploy_DW.UnitDelay2_DSTATE = MicroMouse_Deploy_B.Sum3;

          /* Update for UnitDelay: '<S99>/Unit Delay2' */
          MicroMouse_Deploy_DW.UnitDelay2_DSTATE_o = MicroMouse_Deploy_B.Sum3_n;
        }
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

  /* Derivatives for Integrator: '<S14>/Integrator' */
  _rtXdot->Integrator_CSTATE = MicroMouse_Deploy_B.DeadZone;

  /* Derivatives for Integrator: '<S76>/Integrator' */
  _rtXdot->Integrator_CSTATE_p = MicroMouse_Deploy_B.IntegralGain;

  /* Derivatives for Integrator: '<S71>/Filter' */
  _rtXdot->Filter_CSTATE = MicroMouse_Deploy_B.FilterCoefficient;
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

    /* Start for DataStoreMemory: '<S1>/Data Store Memory' */
    MOTOR_LS = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_oa;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory1' */
    MOTOR_RS = MicroMouse_Deploy_P.DataStoreMemory1_InitialValu_p2;

    /* Start for Probe: '<S19>/Probe' */
    MicroMouse_Deploy_B.Probe[0] = 0.01F;
    MicroMouse_Deploy_B.Probe[1] = 0.0F;

    /* Start for Probe: '<S26>/Probe' */
    MicroMouse_Deploy_B.Probe_h[0] = 0.01F;
    MicroMouse_Deploy_B.Probe_h[1] = 0.0F;

    /* Start for Probe: '<S33>/Probe' */
    MicroMouse_Deploy_B.Probe_g[0] = 0.01F;
    MicroMouse_Deploy_B.Probe_g[1] = 0.0F;
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

      /* InitializeConditions for Delay: '<S15>/Delay' */
      MicroMouse_Deploy_DW.Delay_DSTATE =
        MicroMouse_Deploy_P.Delay_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S95>/Delay Input1'
       *
       * Block description for '<S95>/Delay Input1':
       *
       *  Store in Global RAM
       */
      MicroMouse_Deploy_DW.DelayInput1_DSTATE =
        MicroMouse_Deploy_P.DetectRisePositive_vinit;

      /* InitializeConditions for Integrator: '<S14>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE = MicroMouse_Deploy_P.Integrator_IC;

      /* InitializeConditions for Integrator: '<S76>/Integrator' */
      MicroMouse_Deploy_X.Integrator_CSTATE_p =
        MicroMouse_Deploy_P.PIDController_InitialConditio_l;

      /* InitializeConditions for Integrator: '<S71>/Filter' */
      MicroMouse_Deploy_X.Filter_CSTATE =
        MicroMouse_Deploy_P.PIDController_InitialConditionF;

      /* InitializeConditions for DiscreteIntegrator: '<S25>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING = 1U;

      /* InitializeConditions for DiscreteIntegrator: '<S32>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING_k = 1U;

      /* InitializeConditions for DiscreteIntegrator: '<S39>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING_a = 1U;

      /* InitializeConditions for UnitDelay: '<S98>/Unit Delay1' */
      MicroMouse_Deploy_DW.UnitDelay1_DSTATE =
        MicroMouse_Deploy_P.UnitDelay1_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S99>/Unit Delay1' */
      MicroMouse_Deploy_DW.UnitDelay1_DSTATE_a =
        MicroMouse_Deploy_P.UnitDelay1_InitialCondition_p;

      /* InitializeConditions for UnitDelay: '<S98>/Unit Delay2' */
      MicroMouse_Deploy_DW.UnitDelay2_DSTATE =
        MicroMouse_Deploy_P.UnitDelay2_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S99>/Unit Delay2' */
      MicroMouse_Deploy_DW.UnitDelay2_DSTATE_o =
        MicroMouse_Deploy_P.UnitDelay2_InitialCondition_n;

      /* SystemInitialize for Chart: '<S3>/distance_control' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c3_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c3_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* SystemInitialize for Chart: '<S14>/turn_adjus' */
      MicroMouse_Deploy_DW.bitsForTID1.is_active_c1_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID1.is_c1_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;

      /* Start for MATLABSystem: '<S18>/Median Filter' */
      MicroMouse_Deploy_DW.obj.matlabCodegenIsDeleted = false;
      MicroMouse_Deploy_DW.obj.isInitialized = 1;
      MicroMouse_Deploy_DW.obj.NumChannels = 1;
      MicroMouse_Deploy_DW.obj.pMID.isInitialized = 0;
      MicroMouse_Deploy_DW.obj.isSetupComplete = true;
    }
  }

  {
  }
}

/* Model terminate function */
void MicroMouse_Deploy_terminate(void)
{
  /* Terminate for MATLABSystem: '<S18>/Median Filter' */
  if (!MicroMouse_Deploy_DW.obj.matlabCodegenIsDeleted) {
    MicroMouse_Deploy_DW.obj.matlabCodegenIsDeleted = true;
    if ((MicroMouse_Deploy_DW.obj.isInitialized == 1) &&
        MicroMouse_Deploy_DW.obj.isSetupComplete) {
      MicroMouse_Deploy_DW.obj.NumChannels = -1;
      if (MicroMouse_Deploy_DW.obj.pMID.isInitialized == 1) {
        MicroMouse_Deploy_DW.obj.pMID.isInitialized = 2;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S18>/Median Filter' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
