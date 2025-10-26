/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouse_Deploy_types.h
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

#ifndef MicroMouse_Deploy_types_h_
#define MicroMouse_Deploy_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_KH1GJakkX2IHMZSxCcfvtG
#define struct_tag_KH1GJakkX2IHMZSxCcfvtG

struct tag_KH1GJakkX2IHMZSxCcfvtG
{
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T pWinLen;
  real_T pBuf[5];
  real_T pHeap[5];
  real_T pMidHeap;
  real_T pIdx;
  real_T pPos[5];
  real_T pMinHeapLength;
  real_T pMaxHeapLength;
};

#endif                                 /* struct_tag_KH1GJakkX2IHMZSxCcfvtG */

#ifndef typedef_e_dsp_internal_codegen_Median_T
#define typedef_e_dsp_internal_codegen_Median_T

typedef struct tag_KH1GJakkX2IHMZSxCcfvtG e_dsp_internal_codegen_Median_T;

#endif                             /* typedef_e_dsp_internal_codegen_Median_T */

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_MicroMouse_Deploy_T
#define typedef_cell_wrap_MicroMouse_Deploy_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_MicroMouse_Deploy_T;

#endif                               /* typedef_cell_wrap_MicroMouse_Deploy_T */

#ifndef struct_tag_K0FadUkDjNPyvxQGEfW0ZC
#define struct_tag_K0FadUkDjNPyvxQGEfW0ZC

struct tag_K0FadUkDjNPyvxQGEfW0ZC
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  cell_wrap_MicroMouse_Deploy_T inputVarSize;
  int32_T NumChannels;
  e_dsp_internal_codegen_Median_T pMID;
};

#endif                                 /* struct_tag_K0FadUkDjNPyvxQGEfW0ZC */

#ifndef typedef_dsp_simulink_MedianFilter_Mic_T
#define typedef_dsp_simulink_MedianFilter_Mic_T

typedef struct tag_K0FadUkDjNPyvxQGEfW0ZC dsp_simulink_MedianFilter_Mic_T;

#endif                             /* typedef_dsp_simulink_MedianFilter_Mic_T */

#ifndef struct_tag_0ydU7cpoEUmekgMPJZntVG
#define struct_tag_0ydU7cpoEUmekgMPJZntVG

struct tag_0ydU7cpoEUmekgMPJZntVG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T TunablePropsChanged;
  cell_wrap_MicroMouse_Deploy_T inputVarSize;
  int32_T NumChannels;
  int32_T FrameLength;
  real32_T pCumSum;
  real32_T pCumSumRev[299];
  real32_T pCumRevIndex;
  real32_T pModValueRev;
};

#endif                                 /* struct_tag_0ydU7cpoEUmekgMPJZntVG */

#ifndef typedef_dsp_simulink_MovingAverage_Mi_T
#define typedef_dsp_simulink_MovingAverage_Mi_T

typedef struct tag_0ydU7cpoEUmekgMPJZntVG dsp_simulink_MovingAverage_Mi_T;

#endif                             /* typedef_dsp_simulink_MovingAverage_Mi_T */

/* Parameters (default storage) */
typedef struct P_MicroMouse_Deploy_T_ P_MicroMouse_Deploy_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_MicroMouse_Deploy_T RT_MODEL_MicroMouse_Deploy_T;

#endif                                 /* MicroMouse_Deploy_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
