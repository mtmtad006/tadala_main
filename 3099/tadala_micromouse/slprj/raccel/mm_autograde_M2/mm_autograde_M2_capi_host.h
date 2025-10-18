#ifndef mm_autograde_M2_cap_host_h__
#define mm_autograde_M2_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap_simtarget.h"
typedef struct { rtwCAPI_ModelMappingInfo mmi ; }
mm_autograde_M2_host_DataMapInfo_T ;
#ifdef __cplusplus
extern "C" {
#endif
void mm_autograde_M2_host_InitializeDataMapInfo ( mm_autograde_M2_host_DataMapInfo_T * dataMap , const char * path ) ;
#ifdef __cplusplus
}
#endif
#endif
#endif
