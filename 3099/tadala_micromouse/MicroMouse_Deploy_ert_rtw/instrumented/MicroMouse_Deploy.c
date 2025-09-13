/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy.c
 *
 * Code generated for Simulink model 'MicroMouse_Deploy'.
 *
 * Model version                  : 5.5
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sat Sep 13 18:27:11 2025
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
#include "rtwtypes.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "MicroMouse_Deploy_private.h"
#include "code_profiling_utility_functions.h"

/* Named constants for Chart: '<S3>/distance_control' */
#define MicroMouse_D_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define MicroMouse_Deploy_IN_Run       ((uint8_T)1U)
#define MicroMouse_Deploy_IN_Stop      ((uint8_T)2U)

/* Block signals (default storage) */
B_MicroMouse_Deploy_T MicroMouse_Deploy_B;

/* Block states (default storage) */
DW_MicroMouse_Deploy_T MicroMouse_Deploy_DW;

/* Real-time model */
static RT_MODEL_MicroMouse_Deploy_T MicroMouse_Deploy_M_;
RT_MODEL_MicroMouse_Deploy_T *const MicroMouse_Deploy_M = &MicroMouse_Deploy_M_;

/* Model step function */
void MicroMouse_Deploy_step(void)
{
  real_T tmp;
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
  uint8_T rtb_Compare;
  boolean_T rtb_LogicalOperator;
  boolean_T rtb_LogicalOperator_d;
  boolean_T rtb_LogicalOperator_k;

  /* RelationalOperator: '<S40>/Compare' incorporates:
   *  Constant: '<S38>/Constant'
   *  Constant: '<S40>/Constant'
   *  DataStoreRead: '<S4>/Data Store Read4'
   *  DataTypeConversion: '<S4>/Cast To Single'
   *  Gain: '<S4>/Gain2'
   *  RelationalOperator: '<S38>/Compare'
   */
  rtb_Compare = (uint8_T)((MicroMouse_Deploy_P.Gain2_Gain * (real32_T)
    V_PHOTO_MOT_RS >= MicroMouse_Deploy_P.CompareToConstant_const_l) > (int32_T)
    MicroMouse_Deploy_P.Constant_Value_d);

  /* Sum: '<S14>/Add' incorporates:
   *  Delay: '<S14>/Delay'
   *  RelationalOperator: '<S39>/FixPt Relational Operator'
   *  UnitDelay: '<S39>/Delay Input1'
   *
   * Block description for '<S39>/Delay Input1':
   *
   *  Store in Global RAM
   */
  MicroMouse_Deploy_DW.Delay_DSTATE = (uint8_T)((uint32_T)(rtb_Compare >
    MicroMouse_Deploy_DW.DelayInput1_DSTATE) + MicroMouse_Deploy_DW.Delay_DSTATE);

  /* Chart: '<S3>/distance_control' incorporates:
   *  Constant: '<S16>/Circumference'
   *  Constant: '<S3>/ref'
   *  Delay: '<S14>/Delay'
   *  Gain: '<S16>/Tick_per_rev'
   *  Product: '<S16>/Product'
   */
  if (MicroMouse_Deploy_DW.bitsForTID0.is_active_c3_MicroMouse_Deploy == 0) {
    MicroMouse_Deploy_DW.bitsForTID0.is_active_c3_MicroMouse_Deploy = 1U;
    MicroMouse_Deploy_DW.bitsForTID0.is_c3_MicroMouse_Deploy =
      MicroMouse_Deploy_IN_Run;
  } else if (MicroMouse_Deploy_DW.bitsForTID0.is_c3_MicroMouse_Deploy ==
             MicroMouse_Deploy_IN_Run) {
    if ((real_T)(MicroMouse_Deploy_P.Tick_per_rev_Gain *
                 MicroMouse_Deploy_DW.Delay_DSTATE) * 0.0009765625 *
        MicroMouse_Deploy_P.Circumference_Value >= MicroMouse_Deploy_P.ref_Value)
    {
      MicroMouse_Deploy_DW.bitsForTID0.is_c3_MicroMouse_Deploy =
        MicroMouse_Deploy_IN_Stop;
    } else {
      MicroMouse_Deploy_B.ang_vel = 60.0;
    }
  } else {
    /* case IN_Stop: */
    MicroMouse_Deploy_B.ang_vel = 0.0;
  }

  /* End of Chart: '<S3>/distance_control' */

  /* DataTypeConversion: '<S1>/Cast1' incorporates:
   *  DataStoreWrite: '<S1>/Data Store Write1'
   *  Gain: '<Root>/Gain1'
   *  Sum: '<S3>/Sum'
   */
  tmp = fmod(floor(MicroMouse_Deploy_P.Gain1_Gain * MicroMouse_Deploy_B.ang_vel),
             256.0);
  MOTOR_RS = (int8_T)(tmp < 0.0 ? (int32_T)(int8_T)-(int8_T)(uint8_T)-tmp :
                      (int32_T)(int8_T)(uint8_T)tmp);

  /* DataTypeConversion: '<S1>/Cast' incorporates:
   *  DataStoreWrite: '<S1>/Data Store Write'
   *  Gain: '<Root>/Gain'
   *  Gain: '<S3>/Gain'
   *  Sum: '<S3>/Sum1'
   */
  tmp = fmod(floor(MicroMouse_Deploy_P.Gain_Gain * MicroMouse_Deploy_B.ang_vel *
                   MicroMouse_Deploy_P.Gain_Gain_c), 256.0);
  MOTOR_LS = (int8_T)(tmp < 0.0 ? (int32_T)(int8_T)-(int8_T)(uint8_T)-tmp :
                      (int32_T)(int8_T)(uint8_T)tmp);

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

  /* Logic: '<S17>/Logical Operator' incorporates:
   *  Constant: '<S17>/Constant'
   *  Constant: '<S17>/Time constant'
   *  Constant: '<S20>/Constant'
   *  Constant: '<S21>/Constant'
   *  RelationalOperator: '<S20>/Compare'
   *  RelationalOperator: '<S21>/Compare'
   *  Sum: '<S17>/Sum1'
   */
  rtb_LogicalOperator = (((real32_T)
    (MicroMouse_Deploy_P.LowPassFilterDiscreteorContinuo -
     MicroMouse_Deploy_B.Probe[0]) <= MicroMouse_Deploy_P.Constant_Value_l) &&
    (MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_k <
     MicroMouse_Deploy_P.CompareToConstant_const));

  /* Gain: '<S11>/K' incorporates:
   *  DataStoreRead: '<S10>/Data Store Read'
   *  DataTypeConversion: '<S10>/Cast To Single'
   *  Gain: '<S10>/Gain'
   */
  rtb_K = MicroMouse_Deploy_P.Gain_Gain_g * (real32_T)TOF_Distance[0] *
    MicroMouse_Deploy_P.LowPassFilterDiscreteorConti_pu;

  /* DiscreteIntegrator: '<S23>/Integrator' */
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

  if (rtb_LogicalOperator || (MicroMouse_Deploy_DW.Integrator_PrevResetState !=
       0)) {
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

  /* Saturate: '<S23>/Saturation' incorporates:
   *  DiscreteIntegrator: '<S23>/Integrator'
   */
  if (MicroMouse_Deploy_DW.Integrator_DSTATE >
      MicroMouse_Deploy_P.Saturation_UpperSat) {
    rtb_Saturation = MicroMouse_Deploy_P.Saturation_UpperSat;
  } else if (MicroMouse_Deploy_DW.Integrator_DSTATE <
             MicroMouse_Deploy_P.Saturation_LowerSat) {
    rtb_Saturation = MicroMouse_Deploy_P.Saturation_LowerSat;
  } else {
    rtb_Saturation = MicroMouse_Deploy_DW.Integrator_DSTATE;
  }

  /* End of Saturate: '<S23>/Saturation' */

  /* Logic: '<S24>/Logical Operator' incorporates:
   *  Constant: '<S24>/Constant'
   *  Constant: '<S24>/Time constant'
   *  Constant: '<S27>/Constant'
   *  Constant: '<S28>/Constant'
   *  RelationalOperator: '<S27>/Compare'
   *  RelationalOperator: '<S28>/Compare'
   *  Sum: '<S24>/Sum1'
   */
  rtb_LogicalOperator_d = (((real32_T)
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

  /* DiscreteIntegrator: '<S30>/Integrator' */
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

  if (rtb_LogicalOperator_d || (MicroMouse_Deploy_DW.Integrator_PrevResetState_a
       != 0)) {
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

  /* Saturate: '<S30>/Saturation' incorporates:
   *  DiscreteIntegrator: '<S30>/Integrator'
   */
  if (MicroMouse_Deploy_DW.Integrator_DSTATE_m >
      MicroMouse_Deploy_P.Saturation_UpperSat_f) {
    rtb_Saturation_e = MicroMouse_Deploy_P.Saturation_UpperSat_f;
  } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_m <
             MicroMouse_Deploy_P.Saturation_LowerSat_b) {
    rtb_Saturation_e = MicroMouse_Deploy_P.Saturation_LowerSat_b;
  } else {
    rtb_Saturation_e = MicroMouse_Deploy_DW.Integrator_DSTATE_m;
  }

  /* End of Saturate: '<S30>/Saturation' */

  /* Logic: '<S31>/Logical Operator' incorporates:
   *  Constant: '<S31>/Constant'
   *  Constant: '<S31>/Time constant'
   *  Constant: '<S34>/Constant'
   *  Constant: '<S35>/Constant'
   *  RelationalOperator: '<S34>/Compare'
   *  RelationalOperator: '<S35>/Compare'
   *  Sum: '<S31>/Sum1'
   */
  rtb_LogicalOperator_k = (((real32_T)
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

  /* DiscreteIntegrator: '<S37>/Integrator' */
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

  if (rtb_LogicalOperator_k || (MicroMouse_Deploy_DW.Integrator_PrevResetState_j
       != 0)) {
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

  /* Saturate: '<S37>/Saturation' incorporates:
   *  DiscreteIntegrator: '<S37>/Integrator'
   */
  if (MicroMouse_Deploy_DW.Integrator_DSTATE_i >
      MicroMouse_Deploy_P.Saturation_UpperSat_c) {
    rtb_Saturation_g = MicroMouse_Deploy_P.Saturation_UpperSat_c;
  } else if (MicroMouse_Deploy_DW.Integrator_DSTATE_i <
             MicroMouse_Deploy_P.Saturation_LowerSat_o) {
    rtb_Saturation_g = MicroMouse_Deploy_P.Saturation_LowerSat_o;
  } else {
    rtb_Saturation_g = MicroMouse_Deploy_DW.Integrator_DSTATE_i;
  }

  /* End of Saturate: '<S37>/Saturation' */

  /* ComposeString: '<S3>/Compose String' incorporates:
   *  Saturate: '<S23>/Saturation'
   *  Saturate: '<S30>/Saturation'
   *  Saturate: '<S37>/Saturation'
   */
  snprintf(&rtb_ComposeString_0[0], 256U, "L%1.2f F%1.2f R%1.2f", rtb_Saturation,
           rtb_Saturation_e, rtb_Saturation_g);

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
  snprintf(&rtb_ComposeString1_0[0], 256U, "X%2.1f  Y%2.1f  Z%2.1f", IMU_Accel[0],
           IMU_Accel[1], IMU_Accel[2]);

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
  tmp = fmod(floor(MicroMouse_Deploy_P.Constant_Value), 256.0);
  STATE = (uint8_T)(tmp < 0.0 ? (int32_T)(uint8_T)-(int8_T)(uint8_T)-tmp :
                    (int32_T)(uint8_T)tmp);

  /* MinMax: '<S17>/Max' incorporates:
   *  Constant: '<S17>/Time constant'
   */
  rtb_Max = (real32_T)fmax(MicroMouse_Deploy_B.Probe[0],
    MicroMouse_Deploy_P.LowPassFilterDiscreteorContinuo);

  /* Product: '<S11>/1//T' incorporates:
   *  Fcn: '<S17>/Avoid Divide by Zero'
   *  Sum: '<S11>/Sum1'
   */
  rtb_K = 1.0F / ((real32_T)(rtb_Max == 0.0F) * 2.22044605e-16F + rtb_Max) *
    (rtb_K - rtb_Saturation);

  /* MinMax: '<S24>/Max' incorporates:
   *  Constant: '<S24>/Time constant'
   */
  rtb_Max = (real32_T)fmax(MicroMouse_Deploy_B.Probe_h[0],
    MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_p);

  /* Product: '<S12>/1//T' incorporates:
   *  Fcn: '<S24>/Avoid Divide by Zero'
   *  Sum: '<S12>/Sum1'
   */
  rtb_K_i = 1.0F / ((real32_T)(rtb_Max == 0.0F) * 2.22044605e-16F + rtb_Max) *
    (rtb_K_i - rtb_Saturation_e);

  /* MinMax: '<S31>/Max' incorporates:
   *  Constant: '<S31>/Time constant'
   */
  rtb_Max = (real32_T)fmax(MicroMouse_Deploy_B.Probe_g[0],
    MicroMouse_Deploy_P.LowPassFilterDiscreteorContin_e);

  /* Product: '<S13>/1//T' incorporates:
   *  Fcn: '<S31>/Avoid Divide by Zero'
   *  Sum: '<S13>/Sum1'
   */
  rtb_K_n = 1.0F / ((real32_T)(rtb_Max == 0.0F) * 2.22044605e-16F + rtb_Max) *
    (rtb_K_n - rtb_Saturation_g);

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

    /* Update for UnitDelay: '<S39>/Delay Input1'
     *
     * Block description for '<S39>/Delay Input1':
     *
     *  Store in Global RAM
     */
    MicroMouse_Deploy_DW.DelayInput1_DSTATE = rtb_Compare;

    /* Update for DiscreteIntegrator: '<S23>/Integrator' */
    MicroMouse_Deploy_DW.Integrator_IC_LOADING = 0U;
    MicroMouse_Deploy_DW.Integrator_DSTATE +=
      MicroMouse_Deploy_P.Integrator_gainval * rtb_K;
    if (MicroMouse_Deploy_DW.Integrator_DSTATE >
        MicroMouse_Deploy_P.Integrator_UpperSat) {
      MicroMouse_Deploy_DW.Integrator_DSTATE =
        MicroMouse_Deploy_P.Integrator_UpperSat;
    } else if (MicroMouse_Deploy_DW.Integrator_DSTATE <
               MicroMouse_Deploy_P.Integrator_LowerSat) {
      MicroMouse_Deploy_DW.Integrator_DSTATE =
        MicroMouse_Deploy_P.Integrator_LowerSat;
    }

    MicroMouse_Deploy_DW.Integrator_PrevResetState = (int8_T)rtb_LogicalOperator;

    /* End of Update for DiscreteIntegrator: '<S23>/Integrator' */

    /* Update for DiscreteIntegrator: '<S30>/Integrator' */
    MicroMouse_Deploy_DW.Integrator_IC_LOADING_k = 0U;
    MicroMouse_Deploy_DW.Integrator_DSTATE_m +=
      MicroMouse_Deploy_P.Integrator_gainval_n * rtb_K_i;
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
      rtb_LogicalOperator_d;

    /* End of Update for DiscreteIntegrator: '<S30>/Integrator' */

    /* Update for DiscreteIntegrator: '<S37>/Integrator' */
    MicroMouse_Deploy_DW.Integrator_IC_LOADING_a = 0U;
    MicroMouse_Deploy_DW.Integrator_DSTATE_i +=
      MicroMouse_Deploy_P.Integrator_gainval_k * rtb_K_n;
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
      rtb_LogicalOperator_k;

    /* End of Update for DiscreteIntegrator: '<S37>/Integrator' */
  }

  {
  }
}

/* Model initialize function */
void MicroMouse_Deploy_initialize(void)
{
  {
    int32_T i;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory' */
    MOTOR_LS = MicroMouse_Deploy_P.DataStoreMemory_InitialValue_oa;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory1' */
    MOTOR_RS = MicroMouse_Deploy_P.DataStoreMemory1_InitialValu_p2;

    /* Start for Probe: '<S17>/Probe' */
    MicroMouse_Deploy_B.Probe[0] = 0.01F;
    MicroMouse_Deploy_B.Probe[1] = 0.0F;

    /* Start for Probe: '<S24>/Probe' */
    MicroMouse_Deploy_B.Probe_h[0] = 0.01F;
    MicroMouse_Deploy_B.Probe_h[1] = 0.0F;

    /* Start for Probe: '<S31>/Probe' */
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

      /* InitializeConditions for Delay: '<S14>/Delay' */
      MicroMouse_Deploy_DW.Delay_DSTATE =
        MicroMouse_Deploy_P.Delay_InitialCondition;

      /* InitializeConditions for UnitDelay: '<S39>/Delay Input1'
       *
       * Block description for '<S39>/Delay Input1':
       *
       *  Store in Global RAM
       */
      MicroMouse_Deploy_DW.DelayInput1_DSTATE =
        MicroMouse_Deploy_P.DetectRisePositive_vinit;

      /* InitializeConditions for DiscreteIntegrator: '<S23>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING = 1U;

      /* InitializeConditions for DiscreteIntegrator: '<S30>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING_k = 1U;

      /* InitializeConditions for DiscreteIntegrator: '<S37>/Integrator' */
      MicroMouse_Deploy_DW.Integrator_IC_LOADING_a = 1U;

      /* SystemInitialize for Chart: '<S3>/distance_control' */
      MicroMouse_Deploy_DW.bitsForTID0.is_active_c3_MicroMouse_Deploy = 0U;
      MicroMouse_Deploy_DW.bitsForTID0.is_c3_MicroMouse_Deploy =
        MicroMouse_D_IN_NO_ACTIVE_CHILD;
    }
  }

  {
  }
}

/* Model terminate function */
void MicroMouse_Deploy_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
