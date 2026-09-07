#ifndef TESTPLANNING_H
#define TESTPLANNING_H

#include "Global/global.h"
#include <Eigen/Dense>

void TEST_PlanningProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                     DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                     CAMERA_DATA_t *pst_CameraData,
                                     PLANNING_DATA_t *pst_PlanningData,
                                     CAN_DATA_t *pst_CANData);

void TEST_PitStopProcessing(PLANNING_DATA_t *pst_PlanningData, SPLINE2D_PARAM_t **pst_Spline2DParam, SPLINE_PATH_t **pst_SplinePath);

void TEST_FindNearIdx(PLANNING_DATA_t *pst_PlanningData);
void TEST_FindBankFlag(PLANNING_DATA_t *pst_PlanningData);
void TEST_FlagProcessing(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
void TEST_RestartZoneCheck(PLANNING_DATA_t *pst_PlanningData);

void TEST_EgoVehicleDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CAN_DATA_t *pst_CANData, HMC_LIDAR_DATA_t *pst_LidarData);
void TEST_ClusteredDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, HMC_LIDAR_DATA_t *pst_LidarData, CAN_DATA_t *pst_CANData);

void TEST_FrenetPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void TEST_CollisionCheckPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void TEST_FrenetPathGeneration(PLANNING_DATA_t* pst_PlanningData);

void TEST_ProcessFrenetPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_FrenetPath, int32_t s32_FrenetPathNum);
void TEST_ProcessCollisionCheckPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_CollisionCheckPath, int32_t s32_PathNum);
void TEST_CalcFrenetToGlobalPath(PLANNING_DATA_t *pst_PlanningData);

void TEST_FindAvailableFrenetLane(PLANNING_DATA_t *pst_PlanningData);
void TEST_FindAvailableGridLane(PLANNING_DATA_t *pst_PlanningData);

void TEST_StateProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                  CAMERA_DATA_t *pst_CameraData,
                                  PLANNING_DATA_t *pst_PlanningData,
                                  CAN_DATA_t *pst_CANData);

void TEST_StraightZonePlanning(PLANNING_DATA_t *pst_PlanningData);

int32_t TEST_SelectFrenetBestPath(PLANNING_DATA_t *pst_PlanningData, int32_t s32_TargetLane);
int32_t TEST_SelectOverTakingPath(PLANNING_DATA_t *pst_PlanningData);

void TEST_AccProcessing(PLANNING_DATA_t *pst_PlanningData);
void TEST_InitAccMode(PLANNING_DATA_t *pst_PlanningData);
int32_t TEST_SelectTargetAccIdx(PLANNING_DATA_t *pst_PlanningData);
bool TEST_IsAccAvailable(PLANNING_DATA_t *pst_PlanningData);

void TEST_TargetLaneProcessing(PLANNING_DATA_t *pst_PlanningData);

void TEST_FinalPath(PLANNING_DATA_t *pst_PlanningData);

#endif
