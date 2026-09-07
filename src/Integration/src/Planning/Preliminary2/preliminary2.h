#ifndef PRELIMINARY2_H
#define PRELIMINARY2_H

#include "Global/global.h"
#include <Eigen/Dense>

void PRE2_PlanningProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                     DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                     CAMERA_DATA_t *pst_CameraData,
                                     PLANNING_DATA_t *pst_PlanningData,
                                     CAN_DATA_t *pst_CANData);

void PRE2_PitStopProcessing(PLANNING_DATA_t *pst_PlanningData, SPLINE2D_PARAM_t **pst_Spline2DParam, SPLINE_PATH_t **pst_SplinePath);

void PRE2_FindNearIdx(PLANNING_DATA_t *pst_PlanningData);
void PRE2_FindBankFlag(PLANNING_DATA_t *pst_PlanningData);
void PRE2_FlagProcessing(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
void PRE2_RestartZoneCheck(PLANNING_DATA_t *pst_PlanningData);
void PRE2_VelocityPlanning(PLANNING_DATA_t *pst_PlanningData);

void PRE2_EgoVehicleDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CAN_DATA_t *pst_CANData, HMC_LIDAR_DATA_t *pst_LidarData);
void PRE2_ClusteredDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, HMC_LIDAR_DATA_t *pst_LidarData, CAN_DATA_t *pst_CANData);

void PRE2_FrenetPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void PRE2_CollisionCheckPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void PRE2_ControlPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void PRE2_FrenetPathGeneration(PLANNING_DATA_t* pst_PlanningData);

void PRE2_ProcessFrenetPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_FrenetPath, int32_t s32_FrenetPathNum);
void PRE2_ProcessCollisionCheckPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_CollisionCheckPath, int32_t s32_PathNum);
void PRE2_ProcessControlPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_FrenetPath, int32_t s32_FrenetPathNum);
void PRE2_CalcFrenetToGlobalPath(PLANNING_DATA_t *pst_PlanningData);

void PRE2_FindAvailableFrenetLane(PLANNING_DATA_t *pst_PlanningData);
void PRE2_FindAvailableGridLane(PLANNING_DATA_t *pst_PlanningData);

void PRE2_StateProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                  CAMERA_DATA_t *pst_CameraData,
                                  PLANNING_DATA_t *pst_PlanningData,
                                  CAN_DATA_t *pst_CANData);
void PRE2_StraightZonePlanning(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
void PRE2_BankZonePlanning(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);


int32_t PRE2_SelectFrenetBestPath(PLANNING_DATA_t *pst_PlanningData, int32_t s32_TargetLane);
int32_t PRE2_SelectOverTakingPath(PLANNING_DATA_t *pst_PlanningData);


void PRE2_AccProcessing(PLANNING_DATA_t *pst_PlanningData);
void PRE2_InitAccMode(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
int32_t PRE2_SelectTargetAccIdx(PLANNING_DATA_t *pst_PlanningData);
bool PRE2_IsAccAvailable(PLANNING_DATA_t *pst_PlanningData);

bool PRE2_IsLeftObject(PLANNING_DATA_t *pst_PlanningData, int32_t s32_CurrentLane);
bool PRE2_IsRightObject(PLANNING_DATA_t *pst_PlanningData, int32_t s32_CurrentLane);
bool PRE2_IsOverTakingAvailable(PLANNING_DATA_t *pst_PlanningData);
bool PRE2_IsAccTargetLow(PLANNING_DATA_t *pst_PlanningData);
bool PRE2_IsWidePlace(PLANNING_DATA_t *pst_PlanningData);
bool PRE2_IsOverTakingStillSafe(PLANNING_DATA_t *pst_PlanningData);
bool PRE2_IsOverTakingSuccess(PLANNING_DATA_t *pst_PlanningData);

void PRE2_TargetLaneProcessing(PLANNING_DATA_t *pst_PlanningData);

void PRE2_FinalPath(PLANNING_DATA_t *pst_PlanningData);

#endif
