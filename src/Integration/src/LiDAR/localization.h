#include "Global/global.h"
#include "kernel.cuh"
#include <faiss/IndexFlat.h>

void LidarLocalization(HMC_LIDAR_DATA_t *pst_LidarData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, OMNI_POINT_t *pst_OmniPoint, ICP_MAP_t *pst_ICPMap);

void LoadOmniMap();

void OmniFeatureExtraction(HMC_LIDAR_DATA_t *pst_LidarData, OMNI_POINT_t *pst_OmniPoint);

void OmniCoarseLocalization(HMC_LIDAR_DATA_t *pst_LidarData, OMNI_POINT_t *pst_OmniPoint);

void OmniLoadMap(OMNI_POINT_t *pst_OmniPoint);

void OmniSaveMap(OMNI_FEATURE_t *pst_OmniFeature, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, OMNI_POINT_t *pst_OmniPoint);

void OmniPositionUpdate(HMC_LIDAR_DATA_t *pst_LidarData, OMNI_POINT_t* pst_OmniPoint);

void GetVehicleHeaidngUsingLane(HMC_LIDAR_DATA_t *pst_LidarData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, PLANNING_DATA_t *pst_PlanningData);

void GetICPPoint(HMC_LIDAR_DATA_t *pst_LidarData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData);

void ICPRecordMap(HMC_LIDAR_DATA_t *pst_LidarData, ICP_MAP_t *pst_ICPMap);

void ICPSaveMap(HMC_LIDAR_DATA_t *pst_LidarData, ICP_MAP_t *pst_ICPMap);

void ICPLoadMap(ICP_MAP_t *pst_ICPMap);

void ICPUpdateRecordMap(ICP_POINT_CLOUD_t *pst_Map, ICP_POINT_CLOUD_t *pst_New);

void ICPAddMap(ICP_POINT_CLOUD_t *pst_Map, ICP_POINT_CLOUD_t *pst_New);
