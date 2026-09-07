#ifndef TESTDRIVE1_H
#define TESTDRIVE1_H

#include "Global/global.h"
#include <Eigen/Dense>

void TESTDRIVE1_PlanningProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                     DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                     CAMERA_DATA_t *pst_CameraData,
                                     PLANNING_DATA_t *pst_PlanningData,
                                     CAN_DATA_t *pst_CANData);

void TESTDRIVE1_PitStopProcessing(PLANNING_DATA_t *pst_PlanningData, SPLINE2D_PARAM_t **pst_Spline2DParam, SPLINE_PATH_t **pst_SplinePath);

void TESTDRIVE1_FindNearIdx(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE1_FindBankFlag(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE1_FlagProcessing(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
void TESTDRIVE1_RestartZoneCheck(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE1_EgoVehicleDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CAN_DATA_t *pst_CANData, HMC_LIDAR_DATA_t *pst_LidarData);
void TESTDRIVE1_ClusteredDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, HMC_LIDAR_DATA_t *pst_LidarData, CAN_DATA_t *pst_CANData);

void TESTDRIVE1_FrenetPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void TESTDRIVE1_CollisionCheckPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void TESTDRIVE1_FrenetPathGeneration(PLANNING_DATA_t* pst_PlanningData);

void TESTDRIVE1_ProcessFrenetPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_FrenetPath, int32_t s32_FrenetPathNum);
void TESTDRIVE1_ProcessCollisionCheckPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_CollisionCheckPath, int32_t s32_PathNum);
void TESTDRIVE1_CalcFrenetToGlobalPath(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE1_FindAvailableFrenetLane(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE1_FindAvailableGridLane(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE1_StateProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                  CAMERA_DATA_t *pst_CameraData,
                                  PLANNING_DATA_t *pst_PlanningData,
                                  CAN_DATA_t *pst_CANData);

void TESTDRIVE1_StraightZonePlanning(PLANNING_DATA_t *pst_PlanningData);

int32_t TESTDRIVE1_SelectFrenetBestPath(PLANNING_DATA_t *pst_PlanningData, int32_t s32_TargetLane);
int32_t TESTDRIVE1_SelectOverTakingPath(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE1_AccProcessing(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE1_InitAccMode(PLANNING_DATA_t *pst_PlanningData);
int32_t TESTDRIVE1_SelectTargetAccIdx(PLANNING_DATA_t *pst_PlanningData);
bool TESTDRIVE1_IsAccAvailable(PLANNING_DATA_t *pst_PlanningData);
bool TESTDRIVE1_IsOverTakingSuccess(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE1_TargetLaneProcessing(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE1_VelocityPlanning(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE1_FinalPath(PLANNING_DATA_t *pst_PlanningData);

#endif
