/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy_private.h
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

#ifndef MicroMouse_Deploy_private_h_
#define MicroMouse_Deploy_private_h_
#include "rtwtypes.h"
#include "MicroMouse_Deploy_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */

/* Imported (extern) states */
extern real32_T IMU_Accel[3];          /* '<S5>/Data Store Memory' */
extern real32_T IMU_Gyro[3];           /* '<S5>/Data Store Memory1' */
extern real32_T IMU_Temp;              /* '<S5>/Data Store Memory2' */
extern uint32_T TOF_Distance[3];       /* '<S10>/Data Store Memory' */
extern uint32_T TOF_Status[3];         /* '<S10>/Data Store Memory3' */
extern int16_T Vbattery;               /* '<S8>/Data Store Memory' */
extern int16_T Vshunt;                 /* '<S8>/Data Store Memory1' */
extern int16_T Current;                /* '<S8>/Data Store Memory2' */
extern int16_T Power;                  /* '<S8>/Data Store Memory3' */
extern uint16_T V_BATT;                /* '<S4>/Data Store Memory' */
extern uint16_T V_PHOTO_DOWN_LS;       /* '<S4>/Data Store Memory1' */
extern uint16_T V_PHOTO_DOWN_RS;       /* '<S4>/Data Store Memory2' */
extern uint16_T V_PHOTO_MOT_LS;        /* '<S4>/Data Store Memory3' */
extern uint16_T V_PHOTO_MOT_RS;        /* '<S4>/Data Store Memory4' */
extern int16_T TOF_Ambient[3];         /* '<S10>/Data Store Memory1' */
extern int16_T TOF_Signal[3];          /* '<S10>/Data Store Memory2' */
extern int8_T MOTOR_LS;                /* '<S1>/Data Store Memory' */
extern int8_T MOTOR_RS;                /* '<S1>/Data Store Memory1' */
extern uint8_T oled_string1[18];       /* '<S2>/Data Store Memory' */
extern uint8_T oled_string2[18];       /* '<S2>/Data Store Memory1' */
extern uint8_T oled_string3[18];       /* '<S2>/Data Store Memory2' */
extern uint8_T oled_string4[18];       /* '<S2>/Data Store Memory3' */
extern uint8_T oled_string5[18];       /* '<S2>/Data Store Memory4' */
extern uint8_T LED[3];                 /* '<S6>/Data Store Memory' */
extern uint8_T STATE;                  /* '<S9>/Data Store Memory' */
extern uint8_T SW[2];                  /* '<S7>/Data Store Memory' */
extern uint8_T batteryLife;            /* '<S8>/Data Store Memory4' */

/* Exported functions */
extern void mdlDerivatives_c8_MicroMouse_Deploy(void);
extern void mdlDerivatives_c6_MicroMouse_Deploy(void);
extern void mdlDerivatives_c10_MicroMouse_Deploy(void);

/* private model entry point functions */
extern void MicroMouse_Deploy_derivatives(void);

#endif                                 /* MicroMouse_Deploy_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
