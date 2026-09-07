#ifndef FINAL_H
#define FINAL_H

#include "Global/global.h"
#include <Eigen/Dense>

void FINAL_PlanningProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                 DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                 CAMERA_DATA_t *pst_CameraData,
                                 PLANNING_DATA_t *pst_PlanningData,
                                 CAN_DATA_t *pst_CANData);

void FINAL_PitStopProcessing(PLANNING_DATA_t *pst_PlanningData, SPLINE2D_PARAM_t **pst_Spline2DParam, SPLINE_PATH_t **pst_SplinePath);

void FINAL_FlagProcessing(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);

void FINAL_RestartZoneCheck(PLANNING_DATA_t *pst_PlanningData);

void FINAL_EgoVehicleDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CAN_DATA_t *pst_CANData, HMC_LIDAR_DATA_t *pst_LidarData);

void FINAL_ClusteredDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, HMC_LIDAR_DATA_t *pst_LidarData, CAN_DATA_t *pst_CANData);

void FINAL_FrenetPathGeneration(PLANNING_DATA_t *pst_PlanningData);

void FINAL_CalcFrenetToGlobalPath(PLANNING_DATA_t *pst_PlanningData);

void FINAL_FindAvailableGridLane(PLANNING_DATA_t *pst_PlanningData);

void FINAL_FindAvailableFrenetLane(PLANNING_DATA_t *pst_PlanningData);

void FINAL_StateProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                  CAMERA_DATA_t *pst_CameraData,
                                  PLANNING_DATA_t *pst_PlanningData,
                                  CAN_DATA_t *pst_CANData);

void FINAL_StraightZonePlanning(PLANNING_DATA_t *pst_PlanningData);

void FINAL_BankZonePlanning(PLANNING_DATA_t *pst_PlanningData);

int32_t FINAL_SelectFrenetBestPath(PLANNING_DATA_t *pst_PlanningData, int32_t s32_TargetLane);

int32_t FINAL_SelectOverTakingPath(PLANNING_DATA_t *pst_PlanningData);

int32_t FINAL_SelectTargetAccIdx(PLANNING_DATA_t *pst_PlanningData);

bool FINAL_IsAccAvailable(PLANNING_DATA_t *pst_PlanningData);

void FINAL_AccProcessing(PLANNING_DATA_t *pst_PlanningData);

void FINAL_InitAccMode(PLANNING_DATA_t *pst_PlanningData);

bool FINAL_IsOverTakingAvailable(PLANNING_DATA_t *pst_PlanningData);

bool FINAL_IsAccTargetLow(PLANNING_DATA_t *pst_PlanningData);

bool FINAL_IsWidePlace(PLANNING_DATA_t *pst_PlanningData);

bool FINAL_IsOverTakingStillSafe(PLANNING_DATA_t *pst_PlanningData);

bool FINAL_IsOverTakingSuccess(PLANNING_DATA_t *pst_PlanningData);

void FINAL_ReturnToLane(PLANNING_DATA_t *pst_PlanningData);

bool FINAL_IsLaneReturnComplete(PLANNING_DATA_t *pst_PlanningData);

bool FINAL_IsEmergencySituation(PLANNING_DATA_t *pst_PlanningData);

void FINAL_TargetLaneProcessing(PLANNING_DATA_t *pst_PlanningData);

void FINAL_FinalPath(PLANNING_DATA_t *pst_PlanningData);


#endif
