#ifndef TESTDRIVE2_H
#define TESTDRIVE2_H

#include "Global/global.h"
#include <Eigen/Dense>

void TESTDRIVE2_PlanningProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                     DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                     CAMERA_DATA_t *pst_CameraData,
                                     PLANNING_DATA_t *pst_PlanningData,
                                     CAN_DATA_t *pst_CANData);

void TESTDRIVE2_PitStopProcessing(PLANNING_DATA_t *pst_PlanningData, SPLINE2D_PARAM_t **pst_Spline2DParam, SPLINE_PATH_t **pst_SplinePath);

void TESTDRIVE2_FindNearIdx(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE2_FindBankFlag(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE2_FlagProcessing(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
void TESTDRIVE2_RestartZoneCheck(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE2_VelocityPlanning(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE2_EgoVehicleDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CAN_DATA_t *pst_CANData, HMC_LIDAR_DATA_t *pst_LidarData);
void TESTDRIVE2_ClusteredDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, HMC_LIDAR_DATA_t *pst_LidarData, CAN_DATA_t *pst_CANData);

void TESTDRIVE2_FrenetPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void TESTDRIVE2_CollisionCheckPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void TESTDRIVE2_ControlPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues);
void TESTDRIVE2_FrenetPathGeneration(PLANNING_DATA_t* pst_PlanningData);

void TESTDRIVE2_ProcessFrenetPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_FrenetPath, int32_t s32_FrenetPathNum);
void TESTDRIVE2_ProcessCollisionCheckPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_CollisionCheckPath, int32_t s32_PathNum);
void TESTDRIVE2_ProcessControlPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_FrenetPath, int32_t s32_FrenetPathNum);
void TESTDRIVE2_CalcFrenetToGlobalPath(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE2_FindAvailableFrenetLane(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE2_FindAvailableGridLane(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE2_StateProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                  CAMERA_DATA_t *pst_CameraData,
                                  PLANNING_DATA_t *pst_PlanningData,
                                  CAN_DATA_t *pst_CANData);
void TESTDRIVE2_StraightZonePlanning(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
void TESTDRIVE2_BankZonePlanning(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);


int32_t TESTDRIVE2_SelectFrenetBestPath(PLANNING_DATA_t *pst_PlanningData, int32_t s32_TargetLane);
int32_t TESTDRIVE2_SelectOverTakingPath(PLANNING_DATA_t *pst_PlanningData);


void TESTDRIVE2_AccProcessing(PLANNING_DATA_t *pst_PlanningData);
void TESTDRIVE2_InitAccMode(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
int32_t TESTDRIVE2_SelectTargetAccIdx(PLANNING_DATA_t *pst_PlanningData);
bool TESTDRIVE2_IsAccAvailable(PLANNING_DATA_t *pst_PlanningData);

bool TESTDRIVE2_IsLeftObject(PLANNING_DATA_t *pst_PlanningData, int32_t s32_CurrentLane);
bool TESTDRIVE2_IsRightObject(PLANNING_DATA_t *pst_PlanningData, int32_t s32_CurrentLane);
bool TESTDRIVE2_IsOverTakingAvailable(PLANNING_DATA_t *pst_PlanningData);
bool TESTDRIVE2_IsAccTargetLow(PLANNING_DATA_t *pst_PlanningData);
bool TESTDRIVE2_IsWidePlace(PLANNING_DATA_t *pst_PlanningData);
bool TESTDRIVE2_IsOverTakingStillSafe(PLANNING_DATA_t *pst_PlanningData);
bool TESTDRIVE2_IsOverTakingSuccess(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE2_TargetLaneProcessing(PLANNING_DATA_t *pst_PlanningData);

void TESTDRIVE2_FinalPath(PLANNING_DATA_t *pst_PlanningData);

#endif
