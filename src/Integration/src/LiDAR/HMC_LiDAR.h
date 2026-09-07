#include "Global/global.h"
#include "kernel.cuh"
#include <queue>
#include "localization.h"

using namespace std;


void HMC_LIDARProcessing(RAW_LIDAR_DATA_t *pst_RawData, HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, OMNI_POINT_t *pst_OmniPoint, ICP_MAP_t *pst_ICPMap);

void HMC_LidarGroundFilteringOMP(HMC_LIDAR_DATA_t *pst_LidarPointData, int32_t s32_LIDARMode);

void HMC_Clustering(HMC_LIDAR_DATA_t* pst_LidarPointData);

void HMC_MapBoundaryFilterCluster(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData);

void HMC_CheckMapBoundary(PATH_t *pst_Boundary, float32_t f32_GlobalX, float32_t f32_GlobalY, int32_t &s32_Cross, float32_t &f32_MinDistance);

void HMC_MapBoundaryFilterLocal(HMC_LIDAR_DATA_t *pst_LidarData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, PLANNING_DATA_t *pst_PlanningData, int32_t s32_LIDARMode);

void HMC_ObjectTracking(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData);

void HMC_SearchNearObject(HMC_LIDAR_DATA_t *pst_LidarData);

void HMC_DeleteTrackingObject(HMC_LIDAR_DATA_t *pst_LidarData, int32_t s32_Idx);

void HMC_SearchNewObject(HMC_LIDAR_DATA_t *pst_LidarData);

void HMC_SaveTrackingObject(HMC_TRACKING_t *pst_Tracking, HMC_CLUSTER_t *pst_Cluster, HMC_CLUSTER_t *pst_PrevCluster);

void HMC_CopyClusterObject(HMC_LIDAR_DATA_t *pst_LidarData);

void HMC_InitializeTrackingObject(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData);

void HMC_SetGlobalPosition(HMC_TRACKING_t *pst_Tracking, PLANNING_DATA_t *pst_PlanningData, int32_t s32_Idx);

void HMC_PredictState(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData);

void HMC_UpdateMeasurement(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData);

void HMC_SetMeasurementVelocity(HMC_TRACKING_t *pst_Tracking, int32_t s32_Idx, VectorXf &st_Z);

void HMC_CombineSplitObjects(HMC_LIDAR_DATA_t *pst_LidarData);

void HMC_MapBoundaryFilterTracking(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData);

void HMC_DetermineDynamicOrStatic(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData);

void VehicleICP(HMC_LIDAR_DATA_t *pst_LidarData);