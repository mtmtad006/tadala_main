#include "mm_autograde_M2_capi_host.h"
static mm_autograde_M2_host_DataMapInfo_T root;
static int initialized = 0;
__declspec( dllexport ) rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        mm_autograde_M2_host_InitializeDataMapInfo(&(root), "mm_autograde_M2");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction(){return(getRootMappingInfo());}
