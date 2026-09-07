#ifndef CONTROL_H
#define CONTROL_H

#include "Global/global.h"


void ControlProcessing(PLANNING_DATA_t *pst_PlanningData,
					   DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
					   CONTROL_DATA_t *pst_ControlData,
					   CAN_DATA_t *pst_CANData,
					   CAMERA_DATA_t *pst_CameraData);

// Data processing
void ControlSetup(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void StateProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void V2XControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void FlagControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData);
void BankProcess(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData);
void CurvatureControl(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData);
void GG_Diagram(DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void SpeedProfile(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);

// Control Method
void LateralControl(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData, CAMERA_DATA_t *pst_CameraData);
void LongitudinalControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);

// Lateral Control
void AdvancedStanleyMethod(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void HybridMethod(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void LKAS(CAMERA_DATA_t *pst_CameraData, CONTROL_DATA_t *pst_ControlData, PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
void PIDControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData);
void AdaptiveCruiseControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);

// Longitudinal MPC
tuple<casadi::MX, casadi::MX> GetSpeedModelMatrix(CONTROL_DATA_t *pst_ControlData);
void InitLongitudinalMPC(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void LongitudinalMPC(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void SpeedModel(CONTROL_DATA_t *pst_ControlData, float32_t f32_Throttle, float32_t f32_PreVelocity_m_s);
void PredictSpeed(CONTROL_DATA_t *pst_ControlData);

// Lateral MPC
tuple<casadi::MX, casadi::MX> GetLinearModelMatrix(CONTROL_DATA_t *pst_ControlData);
void InitLateralMPC(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void LateralMPC(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData);
void GetRefTrajectory(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData);
void VehicleDynamics(CONTROL_DATA_t *pst_ControlData, float32_t delta);
void PredictMotion(CONTROL_DATA_t *pst_ControlData);

#endif
