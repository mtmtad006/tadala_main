#ifndef mm_autograde_M2_h_
#define mm_autograde_M2_h_
#ifndef mm_autograde_M2_COMMON_INCLUDES_
#define mm_autograde_M2_COMMON_INCLUDES_
#include <stdlib.h>
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging_simtarget.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "dt_info.h"
#include "ext_work.h"
#include "blas.h"
#include "libmwippgeotrans.h"
#endif
#include "mm_autograde_M2_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include <stddef.h>
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#define MODEL_NAME mm_autograde_M2
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (90) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (10)   
#elif NCSTATES != 10
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T mche153be2 ; } lksteqb2kb ; typedef struct { int32_T
jjh5jiiwwz ; uint32_T o2n3z045kg ; uint8_T d14swqzgdy ; } m5mimjyas2 ;
typedef struct { real_T mbm0sbapgr ; real_T eczzet5oy5 ; } eyt0fft2h4 ;
typedef struct { int32_T gpq1yop1nj ; uint32_T h1velika3t ; uint8_T
fl0i5tniw0 ; } in13gckeft ; typedef struct { uint8_T inputImage [ 126075 ] ;
uint8_T b_B [ 121203 ] ; oranel4bzt xwc [ 3 ] ; real_T xwa [ 8946 ] ; uint8_T
inputImage_mbvzarwird [ 42025 ] ; uint8_T outputImageTemp_data [ 40401 ] ;
real_T xwh [ 4473 ] ; real_T mpv [ 4473 ] ; real_T b_b [ 2982 ] ; real_T mpvm
[ 1491 ] ; int16_T tmp_data [ 4473 ] ; int16_T tmp_data_cl54gopm0x [ 4473 ] ;
boolean_T ii [ 4473 ] ; uint8_T mpvm_kkiq3xxxve [ 4473 ] ; real_T k0yntn5iai
; real_T oyebtlo0yf ; real_T lvavfjzz5q ; real_T llw2dpuhqu ; real_T
m0rkmcd5js ; real_T ojwunabl10 ; real_T n0opddonv2 [ 3 ] ; real_T osvcphwaz1
[ 3 ] ; real_T cq2zdm4bqp [ 2 ] ; real_T gd41vhwken [ 3 ] ; real_T jo4whguwls
; real_T itucdj1ewz ; real_T nvkrz3mxgj [ 2 ] ; real_T l2snlaj1nc ; real_T
idij42i2gw ; real_T dcpasfbtt4 [ 2 ] ; real_T hergh2yap5 ; real_T n3zs4v3t2b
; real_T iqpya5s3hr ; real_T mvr2suztle [ 2 ] ; real_T bg0bf32bte ; real_T
l2ldfzarhu ; real_T p01dvdbqoi ; real_T i1mjatxjiw ; real_T a1of3lhpsj ;
real_T didojtsufe ; real_T omofh2yahr ; real_T bunjhnt5tq ; real_T gwobx1gfe4
; real_T ozsdm2k5rd ; real_T j00txr32sr ; real_T l1kjxwjqqg ; real_T
dbebchkcdj ; real_T aiil34kpb4 ; real_T fnjiupqbnh ; real_T mdqf3z5ief ;
real_T ay1hzknmrn ; real_T lznx0xw0r2 ; real_T bcxqw2seh3 ; real_T njpg11ct4x
; real_T ahfo3sk05y ; real_T f3slecfhi1 ; real_T ez2cgqi4sl ; real_T
on1trmxucp ; real_T nyluu4k4k5 ; real_T hvrs41ksah ; real_T oi4ncurpey ;
real_T b5m2srzd22 ; real_T ofnpkggoia [ 3 ] ; real_T dvbjweik0a ; real_T
a3gdd2mdkb ; real_T ocfwx3wy2w ; real_T ktwu2bvg2d ; uint8_T ob0lu21rke ;
uint8_T kiqq2yiad0 ; uint8_T awba5oiga4 ; uint8_T lcf41nfjxv ; uint8_T
ghfyexmhos ; uint8_T hni21roo01 ; uint8_T ol2rqhhab3 ; uint8_T csw3govdsp ;
uint8_T jk11yckv5c ; uint8_T ma1i1rk4i3 ; boolean_T lhyjn2urf1 ; boolean_T
p4hc2zjm4g ; boolean_T d4qqji2cg3 ; boolean_T berhq5rp5u ; boolean_T
o5f5tw0bhw ; boolean_T ajqs00jqrg ; boolean_T dqjphyzewl ; boolean_T
oozws1hbtg ; boolean_T f1tnuwwv30 ; boolean_T l3xv3oeegq ; boolean_T
dhnpill5yj ; boolean_T heezp0izai ; eyt0fft2h4 lmzy5tlewm ; eyt0fft2h4
mckzz5wvl4 ; lksteqb2kb gg14xtouxc ; lksteqb2kb ovb0n2j4hj ; } B ; typedef
struct { real_T a5mppswd4g ; real_T lmyjrqqm0q ; real_T cbu1x43z5o ; real_T
dkwoq1zn2q ; real_T g4l4sstjgw ; real_T ix0bqloo3w ; real_T pfodavbtqb ;
real_T cgcpzoopc3 ; real_T gyc2homrea ; real_T gj0annlnte ; real_T idqmwdd5yb
; real_T buzua3mg0c ; real_T njlzb3ymr4 ; real_T lrjjgww0fr ; real_T
hvtlz3ytk4 ; real_T pwkg1i0clx ; real_T ebtr03oego ; real_T ihcygfgz50 ;
real_T fgjijjwqop ; real_T k0wn542ln0 ; real_T fxvkp0bdm5 ; real_T ou2xmqri5p
; real_T j4tfuxf0r4 ; real_T a32ulgyecd ; real_T keebpc1wve ; real_T
bvrt110pwk ; real_T fi0ngahxjt ; real_T ok2ij41o2v ; real_T hs3ilvk2e4 ;
real_T kt54bbuvjk ; real_T ddopguiexo ; real_T nxxmufiqog ; real_T l010k3tsbk
; real_T ijbaarr4nz ; real_T o00jirvkiw ; real_T hdavbqdih1 ; real_T
kw1wkwwjxk ; real_T mcj31hnz02 ; real_T ar1zrakxb3 ; struct { void *
LoggedData [ 2 ] ; } cfivhtuao0 ; struct { void * LoggedData ; } bb4jgoabcn ;
struct { void * LoggedData ; } g5vm0pxz20 ; struct { void * LoggedData ; }
iro3gppa3o ; struct { void * LoggedData ; } ltnd253qsk ; struct { void *
LoggedData ; } patxlot3wj ; struct { void * LoggedData ; } gdwkbvj3os ;
struct { void * LoggedData ; } ikpzcg3ezr ; struct { void * LoggedData ; }
a0wn5dv0oh ; struct { void * LoggedData ; } n5hawsku0t ; struct { void *
LoggedData ; } f0eloybr32 ; struct { void * LoggedData [ 2 ] ; } iike22gcof ;
struct { void * LoggedData ; } m0npouxp1d ; struct { void * LoggedData ; }
ga3kjdz0ce ; struct { void * LoggedData ; } pr0y2sredb ; struct { void *
LoggedData [ 2 ] ; } b0gq1idaar ; struct { void * LoggedData ; } dh1m5auff4 ;
struct { void * LoggedData ; } kep231mdda ; struct { void * AQHandles ; }
hzap5lnx0x ; struct { void * LoggedData [ 5 ] ; } py1hl0uwoj ; int32_T
ectirthedq ; int32_T gfxrso0ugv ; int32_T nnwjuiot3s ; int32_T bd3pmtlbxa ;
int32_T igyxtuvsol ; int32_T kja2yab2jc ; int32_T nssubxjuqy ; int32_T
ou0eypazjv ; uint32_T d0bfvwdil1 ; uint32_T gkom5dg04h ; uint32_T oqmdwhzvqd
; uint8_T klyc1nbon0 ; uint8_T faxliy3dxh ; uint8_T hc5a2ng0u1 ; uint8_T
a2i0vc0nyr ; boolean_T m2kqvebxm1 ; int8_T ozgxdva2cb ; int8_T l32ey2ufcp ;
int8_T jlhwx40y1w ; int8_T doubgfhhla ; int8_T dfi5eeqqke ; uint8_T
gjcro2b4mx ; uint8_T hynvkpd32l ; uint8_T g2u1e0iaht ; uint8_T kufypb5p4u ;
uint8_T csj4ylidjq ; uint8_T dt2xtcrnce ; boolean_T kdcruf4nch ; boolean_T
ev1smh1ixm ; boolean_T i0x4wmxvc4 ; boolean_T a4r3kbjevf ; boolean_T
jyntvb5nm4 ; boolean_T kego0fkvyv ; boolean_T dehjsdofsv ; in13gckeft
lmzy5tlewm ; in13gckeft mckzz5wvl4 ; m5mimjyas2 gg14xtouxc ; m5mimjyas2
ovb0n2j4hj ; } DW ; typedef struct { real_T lmzt2junna ; real_T gyxhpgpfzx ;
real_T es0f4w5bgy ; real_T bu3m45n5du ; real_T g2oapnb13e ; real_T nzagyewug1
; real_T podyqrebun ; real_T mftes1armz ; real_T lslvrji2jk ; real_T
kn1ub3n4ea ; } X ; typedef struct { real_T lmzt2junna ; real_T gyxhpgpfzx ;
real_T es0f4w5bgy ; real_T bu3m45n5du ; real_T g2oapnb13e ; real_T nzagyewug1
; real_T podyqrebun ; real_T mftes1armz ; real_T lslvrji2jk ; real_T
kn1ub3n4ea ; } XDot ; typedef struct { boolean_T lmzt2junna ; boolean_T
gyxhpgpfzx ; boolean_T es0f4w5bgy ; boolean_T bu3m45n5du ; boolean_T
g2oapnb13e ; boolean_T nzagyewug1 ; boolean_T podyqrebun ; boolean_T
mftes1armz ; boolean_T lslvrji2jk ; boolean_T kn1ub3n4ea ; } XDis ; typedef
struct { rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { ssbus
simstruct ; real_T PIDController_D ; real_T PIDController_D_aybtd55nno ;
real_T PIDController_InitialConditionForFilter ; real_T
PIDController_InitialConditionForFilter_mxzkvf05nt ; real_T
LowPassFilterDiscreteorContinuous_K ; real_T
LowPassFilterDiscreteorContinuous2_K ; real_T
LowPassFilterDiscreteorContinuous1_K ; real_T PIDController_N ; real_T
PIDController_N_mlojjm5ri3 ; real_T PIDController_P ; real_T
PIDController_P_iq3bdsbgdi ; real_T LowPassFilterDiscreteorContinuous_T ;
real_T LowPassFilterDiscreteorContinuous2_T ; real_T
LowPassFilterDiscreteorContinuous1_T ; real_T CompareToConstant_const ;
real_T CompareToConstant_const_ktpuubd0d5 ; real_T
CompareToConstant_const_g0sgrhol0k ; real_T
CompareToConstant_const_coiiynw3vz ; real_T
CompareToConstant_const_jgfq3drtzc ; real_T mmdistancesensors_debug ; real_T
LowPassFilterDiscreteorContinuous_init_option ; real_T
LowPassFilterDiscreteorContinuous1_init_option ; real_T
LowPassFilterDiscreteorContinuous2_init_option ; real_T mmrobotpose_theta0 ;
real_T mmwheelencoders_ticsperrev ; real_T mmsimplerobot_x0 ; real_T
mmsimplerobot_y0 ; boolean_T DetectFallNonpositive_vinit ; uint8_T
DetectRisePositive_vinit ; uint8_T DetectRisePositive_vinit_lngnpu3jyt ;
real_T Out1_Y0 ; real_T Constant_Value ; real_T Constant_Value_aig0pcsgad ;
real_T Constant_Value_lqgksk13vx ; real_T Integrator_gainval ; real_T
Integrator_UpperSat ; real_T Integrator_LowerSat ; real_T Saturation_UpperSat
; real_T Saturation_LowerSat ; real_T Constant1_Value ; real_T
Integrator_gainval_oofl1tlpir ; real_T Integrator_UpperSat_hfuo5t0da2 ;
real_T Integrator_LowerSat_ovbjd4im3c ; real_T Saturation_UpperSat_khw5hppl1l
; real_T Saturation_LowerSat_f1bwbpmoly ; real_T Integrator1_IC ; real_T
Integrator_IC ; real_T Gain1_Gain ; real_T Integrator_gainval_keeib2hddd ;
real_T Integrator_UpperSat_oshaxlda54 ; real_T Integrator_LowerSat_kruqrcyoju
; real_T Saturation_UpperSat_c1owngef0q ; real_T
Saturation_LowerSat_llldnyvgbu ; real_T Constant2_Value ; real_T
Integrator_IC_jmrokuovbr ; real_T Gain_Gain ; real_T Gain1_Gain_kyjkekkkxt ;
real_T Integrator_IC_o4vb0f3qhf ; real_T Gain_Gain_lisflq1rpf ; real_T
Gain1_Gain_ha3bnp5l4j ; real_T Integrator1_IC_pd3mj3kddd ; real_T
Gain_Gain_dlrejf44zs ; real_T Gain1_Gain_bcmg3d0bsc ; real_T
Constant_Value_e2dunptcm0 ; real_T Constant_Value_b2pjk4fc2b ; real_T
Circumference_Value ; real_T Circumference_Value_caan2y1dzp ; real_T
Motor_Left1_Value ; real_T Motor_Right1_Value ; real_T
Constant_Value_e5jcotvvny ; real_T Constant1_Value_ibpbochjmd [ 3 ] ; real_T
Constant_Value_lr4ms5o4l0 ; real_T Constant1_Value_ddcl5gh2ak ; real_T
Constant2_Value_n2l4lxleww ; void * OLED_STRING1_String ; void *
OLED_STRING2_String ; boolean_T Constant_Value_hfe5oy52it ; boolean_T
Constant_Value_leeph2khp1 ; boolean_T Constant_Value_hfn3nbsevr ; uint8_T
Tick_per_rev_Gain ; uint8_T Tick_per_rev_Gain_hhz0nuj2bt ; uint8_T
Delay_InitialCondition ; uint8_T Delay_InitialCondition_h3211qidyt ; } ;
extern const char_T * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X
rtX ; extern DW rtDW ; extern P rtP ; extern mxArray *
mr_mm_autograde_M2_GetDWork ( ) ; extern void mr_mm_autograde_M2_SetDWork ( const mxArray * ssDW ) ; extern mxArray * mr_mm_autograde_M2_GetSimStateDisallowedBlocks ( ) ; extern const rtwCAPI_ModelMappingStaticInfo * mm_autograde_M2_GetCAPIStaticMap ( void ) ; extern SimStruct * const rtS ; extern DataMapInfo * rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs ( int_T tid ) ; void MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) ;
#endif
