#ifndef mm_autograde_M2_types_h_
#define mm_autograde_M2_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_ssbus_
#define DEFINED_TYPEDEF_FOR_ssbus_
typedef struct { real_T m ; real_T I ; real_T L ; real_T d ; real_T R ;
real_T m_c ; real_T I_w ; real_T La ; real_T Ra ; real_T Kb ; real_T N ;
real_T Kt ; uint8_T robot_img [ 121203 ] ; uint8_T robot_imgalpha [ 40401 ] ;
uint8_T sl_padding0 [ 4 ] ; real_T robot_imgxyd [ 201 ] ; real_T robot_rad ;
real_T Rimu [ 9 ] ; real_T timu [ 3 ] ; real_T toflpose [ 3 ] ; real_T
toffpose [ 3 ] ; real_T tofrpose [ 3 ] ; real_T tofposes [ 9 ] ; real_T
tofspts [ 1491 ] ; uint8_T mapim [ 240400 ] ; uint8_T mapimfud [ 240400 ] ;
real_T mapres ; real32_T mapdt [ 240400 ] ; real_T wencr ; } ssbus ;
#endif
#ifndef struct_tag_SLyPVOxmSxU6ohhzgEeWtH
#define struct_tag_SLyPVOxmSxU6ohhzgEeWtH
struct tag_SLyPVOxmSxU6ohhzgEeWtH { real_T f1 [ 2 ] ; } ;
#endif
#ifndef typedef_ljkdvujd3s
#define typedef_ljkdvujd3s
typedef struct tag_SLyPVOxmSxU6ohhzgEeWtH ljkdvujd3s ;
#endif
#ifndef struct_tag_belDyztK4QiH6lXsypfI0C
#define struct_tag_belDyztK4QiH6lXsypfI0C
struct tag_belDyztK4QiH6lXsypfI0C { real_T f1 [ 2982 ] ; } ;
#endif
#ifndef typedef_oranel4bzt
#define typedef_oranel4bzt
typedef struct tag_belDyztK4QiH6lXsypfI0C oranel4bzt ;
#endif
#ifndef struct_tag_XAN0yEPIVmgGTBGGitkNoG
#define struct_tag_XAN0yEPIVmgGTBGGitkNoG
struct tag_XAN0yEPIVmgGTBGGitkNoG { real_T XWorldLimits [ 2 ] ; real_T
YWorldLimits [ 2 ] ; real_T ImageSizeAlias [ 2 ] ; boolean_T
ForcePixelExtentToOne ; } ;
#endif
#ifndef typedef_dt1241vwrn
#define typedef_dt1241vwrn
typedef struct tag_XAN0yEPIVmgGTBGGitkNoG dt1241vwrn ;
#endif
#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T
struct emxArray_uint8_T { uint8_T * data ; int32_T * size ; int32_T
allocatedSize ; int32_T numDimensions ; boolean_T canFreeData ; } ;
#endif
#ifndef typedef_mgau0tx4td
#define typedef_mgau0tx4td
typedef struct emxArray_uint8_T mgau0tx4td ;
#endif
#ifndef SS_UINT64
#define SS_UINT64 23
#endif
#ifndef SS_INT64
#define SS_INT64 24
#endif
typedef struct P_ P ;
#endif
