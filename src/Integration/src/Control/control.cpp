#include "control.h"

extern pthread_mutex_t st_ControlToCAN;
extern int32_t s32_CanMode;

void ControlProcessing(PLANNING_DATA_t *pst_PlanningData,
					   DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
					   CONTROL_DATA_t *pst_ControlData,
					   CAN_DATA_t *pst_CANData, CAMERA_DATA_t *pst_CameraData)
{
	StateProcessing(pst_PlanningData, pst_DeadReckoningData, pst_ControlData, pst_CANData);
	ControlSetup(pst_PlanningData, pst_DeadReckoningData, pst_ControlData, pst_CANData);
	LateralControl(pst_PlanningData, pst_DeadReckoningData, pst_ControlData, pst_CANData, pst_CameraData);
	LongitudinalControl(pst_PlanningData, pst_ControlData, pst_CANData);
}

void ControlSetup(PLANNING_DATA_t *pst_PlanningData,
				  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
				  CONTROL_DATA_t *pst_ControlData,
				  CAN_DATA_t *pst_CANData)
{
	GG_Diagram(pst_DeadReckoningData, pst_ControlData, pst_CANData);
	V2XControl(pst_PlanningData, pst_ControlData, pst_CANData);
	SpeedProfile(pst_PlanningData, pst_DeadReckoningData, pst_ControlData, pst_CANData);
	FlagControl(pst_PlanningData, pst_ControlData);
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Control Setup ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void StateProcessing(PLANNING_DATA_t *pst_PlanningData,
					 DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
					 CONTROL_DATA_t *pst_ControlData,
					 CAN_DATA_t *pst_CANData)
{
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;
	IMU_DATA_t *pst_IMU = &pst_DeadReckoningData->st_IMU;
	VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

	pst_OriginState->f32_CX = pst_EgoVehicleData->f32_X_global - sin(pst_EgoVehicleData->f32_Yaw_rad_NED) * 0.069f;
	pst_OriginState->f32_CY = pst_EgoVehicleData->f32_Y_global - cos(pst_EgoVehicleData->f32_Yaw_rad_NED) * 0.069f;
	pst_OriginState->f32_VelocityX_m_s = pst_EgoVehicleData->f32_VelocityX_m_s_local;
	pst_OriginState->f32_VelocityY_m_s = pst_EgoVehicleData->f32_VelocityY_m_s_local;
	pst_OriginState->f32_Yaw_rad_ENU = pst_EgoVehicleData->f32_Yaw_rad_ENU;
	pst_OriginState->f32_Yaw_rad_NED = pst_EgoVehicleData->f32_Yaw_rad_NED;
	pst_OriginState->f32_Yawrate_rad_s = deg2rad(pst_CANData->f32_YAW_RATE);
}

void V2XControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData)
{
	PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;

	if (pst_ControlData->s32_V2XMode == 4)
		return;

	if (pst_Planner->u8_GoSignal || pst_Planner->u8_SlowOffSignal)
	{
		pst_ControlData->s32_V2XMode = 1;
	}
	else if (pst_Planner->u8_StopSignal)
	{
		pst_ControlData->s32_V2XMode = 0;
	}
	else if (pst_Planner->u8_SlowOnSignal)
	{
		pst_ControlData->s32_V2XMode = 2;
	}

	// PitStop Finish
	if (pst_Planner->u8_PitStopMode && InRange(c_ARRIVE_IDX - 2, c_ARRIVE_IDX + 100, pst_PlanningData->st_ReferenceSELine2_global.s32_NearIdx))
	{
		pst_ControlData->s32_V2XMode = 0;
		pst_ControlData->b_PitStopFlag = false;
		pst_Planner->u8_PitStopMode = false;
	}
}

void LateralControl(PLANNING_DATA_t *pst_PlanningData,
					DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
					CONTROL_DATA_t *pst_ControlData,
					CAN_DATA_t *pst_CANData,
					CAMERA_DATA_t *pst_CameraData)
{
	switch (pst_ControlData->s32_ControlMethod)
	{
	case 0:
		AdvancedStanleyMethod(pst_PlanningData, pst_DeadReckoningData, pst_ControlData, pst_CANData);
		break;
	case 1:
		LKAS(pst_CameraData, pst_ControlData, pst_PlanningData, pst_CANData);
		break;
	case 2:
		LateralMPC(pst_PlanningData, pst_DeadReckoningData, pst_ControlData, pst_CANData);
		break;
	case 3: // Test
		HybridMethod(pst_PlanningData, pst_DeadReckoningData, pst_ControlData, pst_CANData);
		break;
	}
}

void LongitudinalControl(PLANNING_DATA_t *pst_PlanningData,
						 CONTROL_DATA_t *pst_ControlData,
						 CAN_DATA_t *pst_CANData)
{
	switch (pst_PlanningData->st_Planner.b_AccMode)
	{
	case 0: // Tracking
		PIDControl(pst_PlanningData, pst_ControlData);
		// LongitudinalMPC(pst_PlanningData, pst_ControlData, pst_CANData);
		break;
	case 1: // ACC
		AdaptiveCruiseControl(pst_PlanningData, pst_ControlData, pst_CANData);
		break;
	}
}

void FlagControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData)
{
	PATH_t *pst_ReferenceSELine2_global = &pst_PlanningData->st_ReferenceSELine2_global;
	PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
	VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;

	if (pst_ControlData->s32_V2XMode == 4)
		return;

	// Pit Stop doing
	float32_t f32_RefDistance = 0;
	float32_t f32_Time = 0;
	float32_t f32_ThresDist = 0;
	if (pst_Planner->u8_PitStopMode)
	{
		if (pst_ControlData->b_PitStopFlag == false && InRange(0, c_REFERENCE_IDX, pst_ReferenceSELine2_global->s32_NearIdx))
		{
			pst_ControlData->f32_FinalDistance = getDistance2d(pst_ReferenceSELine2_global->arf32_X[c_REFERENCE_IDX],
																pst_ReferenceSELine2_global->arf32_Y[c_REFERENCE_IDX],
																pst_EgoVehicleData->f32_X_global,
																pst_EgoVehicleData->f32_Y_global);
			pst_ControlData->f32_FinalVelocity_kph = pst_EgoVehicleData->f32_Velocity_m_s_global;
			
			f32_Time = (pst_ControlData->f32_FinalVelocity_kph - 5.f) / (0.4f * 9.81f); 	
			f32_ThresDist = 1.25f * (pst_ControlData->f32_FinalVelocity_kph * f32_Time - 0.5f * 0.4f * 9.81f * powf(f32_Time, 2));  
			pst_ControlData->b_PitStopFlag = InRange(0.0f, f32_ThresDist, pst_ControlData->f32_FinalDistance);
		}
		else if (pst_ControlData->b_PitStopFlag == true && InRange(0, c_REFERENCE_IDX, pst_ReferenceSELine2_global->s32_NearIdx))
		{
			pst_ControlData->f32_TargetSpeed_kph = 5.f;
			GetModData(10.0f, pst_Param->f32_MAX_SPEED_kph, pst_ControlData->f32_TargetSpeed_kph);
			pst_ControlData->f32_TargetSpeed_m_s = pst_ControlData->f32_TargetSpeed_kph / 3.6f;
		}
		else if (InRange(c_REFERENCE_IDX, c_ARRIVE_IDX, pst_ReferenceSELine2_global->s32_NearIdx))
		{
			pst_ControlData->f32_TargetSpeed_kph = 5;
			pst_ControlData->f32_TargetSpeed_m_s = 5 / 3.6;
		}
	}

	// V2X Mode Set
	switch (pst_ControlData->s32_V2XMode)
	{
	case 0: // Stop
		if (!pst_Planner->b_Bank)
		{
			pst_ControlData->f32_TargetSpeed_kph = -30;
			pst_ControlData->f32_TargetSpeed_m_s = -30 / 3.6;

			// // AEB
			// if (pst_EgoVehicleData->f32_Velocity_m_s_global >= 60.f / 3.6f, pst_CANData->u8_EPS_Control_Status == 2)
			// {
			// 	pst_CANData->u8_AEB_En = 1;
			// }
			// if (pst_EgoVehicleData->f32_Velocity_m_s_global <= 0.1f, pst_CANData->u8_EPS_Control_Status == 2)
			// {
			// 	pst_CANData->u8_AEB_En = 0;
			// }
		}
		break;
	case 1: // Steady State
		break;
	case 2: // Slow On
		if (!pst_Planner->b_Bank)
		{
			pst_ControlData->f32_TargetSpeed_kph = 5;
			pst_ControlData->f32_TargetSpeed_m_s = 5 / 3.6;
		}
		break;
	}
}

void SpeedProfile(PLANNING_DATA_t *pst_PlanningData,
				  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
				  CONTROL_DATA_t *pst_ControlData,
				  CAN_DATA_t *pst_CANData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;

	BankProcess(pst_PlanningData, pst_ControlData);
	CurvatureControl(pst_PlanningData, pst_DeadReckoningData, pst_ControlData);

	static bool b_StateChanged = 0;

	// Temporary Speed Control
	switch (pst_PlanningData->s32_SpeedSignal)
	{
	case 0: // Default (Off)
		break;
	case 1: // On
		b_StateChanged = 0;
		pst_ControlData->f32_TargetSpeed_kph = pst_PlanningData->f32_TargetSpeed_kph;
		pst_ControlData->f32_TargetSpeed_m_s = pst_ControlData->f32_TargetSpeed_kph / 3.6;
		break;
	case 2: // Go to Before Speed
		if (b_StateChanged == 0)
		{
			YAML::Node st_Config = YAML::LoadFile("ControlParam.yaml");
			pst_ControlData->f32_TargetSpeed_kph = st_Config["TargetSpeed"].as<float32_t>();
			pst_ControlData->f32_TargetSpeed_m_s = pst_ControlData->f32_TargetSpeed_kph / 3.6;
			b_StateChanged = 1;
		}
		break;
	}

	GetModData(pst_Param->f32_MIN_SPEED_kph, pst_Param->f32_MAX_SPEED_kph, pst_ControlData->f32_TargetSpeed_kph);
	pst_ControlData->f32_TargetSpeed_m_s = pst_ControlData->f32_TargetSpeed_kph / 3.6;	
}

void BankProcess(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData)
{
	VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
	int32_t s32_NearIdx = pst_PlanningData->st_ReferenceLine2_global.s32_NearIdx;
	int32_t s32_CurrentLane = pst_PlanningData->st_EgoVehicleData.s32_CurrentLane;

	// Bank In Judgement
	if (pst_ControlData->b_PreBank == false &&
		pst_Planner->b_Bank == true &&
		pst_ControlData->s32_SpeedProfileMode == 1)
	{
		pst_ControlData->b_BankInControl = true;
	}
	pst_ControlData->b_PreBank = pst_Planner->b_Bank;

	// Bank In
	if (pst_ControlData->b_BankInControl)
	{
		pst_ControlData->s32_SpeedProfileMode = 0;
		pst_ControlData->f32_TargetSpeed_kph = pst_Param->arf32_bankSpeed[s32_CurrentLane / 6];
		pst_ControlData->f32_TargetSpeed_m_s = pst_ControlData->f32_TargetSpeed_kph / 3.6;

		if (InRange(0.f, pst_ControlData->f32_TargetSpeed_m_s + 0.5f, pst_EgoVehicleData->f32_VelocityX_m_s_local))
		{
			pst_ControlData->s32_SpeedProfileMode = 1;
			pst_ControlData->b_BankInControl = false;
		}
	}

	// bank out
	if (InRange(2016, 2240, s32_NearIdx) && pst_ControlData->b_BankOutControl == false)
	{
		pst_ControlData->f32_TargetSpeed_kph = pst_Param->arf32_bankOutSpeed[s32_CurrentLane / 6];
		pst_ControlData->f32_TargetSpeed_m_s = pst_ControlData->f32_TargetSpeed_kph / 3.6;
		pst_ControlData->s32_SpeedProfileMode = 0;

		if(InRange(0.f, pst_ControlData->f32_TargetSpeed_m_s + 0.5f, pst_EgoVehicleData->f32_VelocityX_m_s_local))
		{
			pst_ControlData->b_BankOutControl = true;
			pst_ControlData->s32_SpeedProfileMode = 1;
		}
	}
	else if (!InRange(2016, 2240, s32_NearIdx) && pst_Planner->b_Bank == false)
	{
		pst_ControlData->b_BankOutControl = false;
	}
}

void CurvatureControl(PLANNING_DATA_t *pst_PlanningData,
					  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
					  CONTROL_DATA_t *pst_ControlData)
{
	if(!pst_ControlData->s32_SpeedProfileMode)
		return;
	
	PATH_t *pst_FinalPath = &pst_PlanningData->st_FinalPath_global;
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	IMU_DATA_t *pst_IMUData = &pst_DeadReckoningData->st_IMU;

	int32_t s32_CurrentLane = pst_PlanningData->st_EgoVehicleData.s32_CurrentLane;
	int32_t s32_NearIdx = pst_PlanningData->st_ReferenceLine2_global.s32_NearIdx;
	float32_t f32_Ratio = pst_Param->arf32_SlipRatio[s32_CurrentLane / 6];
	float32_t f32_Roll = pst_IMUData->f32_Roll_rad;
	GetModData(0.0, 180.0, f32_Roll);

	float32_t f32_Radius = 0;
	float32_t f32_TargetSpeed_kph = 0;

	if (!InRange(c_FIRSTBANKSTART_IDX, c_SECONDBANKEND_IDX, s32_NearIdx))
	{
		f32_Ratio = 0.2f;
	}

	GetRadius(pst_FinalPath->arf32_X[0], pst_FinalPath->arf32_Y[0],
				pst_FinalPath->arf32_X[pst_FinalPath->s32_Num / 2], pst_FinalPath->arf32_Y[pst_FinalPath->s32_Num / 2],
				pst_FinalPath->arf32_X[pst_FinalPath->s32_Num - 1], pst_FinalPath->arf32_Y[pst_FinalPath->s32_Num - 1], f32_Radius);
	f32_TargetSpeed_kph = sqrtf(f32_Radius * 127.14 * (abs(tanf(f32_Roll)) + f32_Ratio));

	GetModData(pst_Param->f32_MIN_SPEED_kph, pst_Param->f32_MAX_SPEED_kph, f32_TargetSpeed_kph);
	pst_ControlData->f32_TargetSpeed_kph = f32_TargetSpeed_kph;
	pst_ControlData->f32_TargetSpeed_m_s = f32_TargetSpeed_kph / 3.6;
}

void GG_Diagram(DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData)
{
	IMU_DATA_t *pst_IMU = &pst_DeadReckoningData->st_IMU;
	GG_Diagram_t *pst_GG_Diagram = &pst_ControlData->st_GG_Diagram;

	// Get GG Diagram
	for (int32_t i = 19; i > 0; i--)
	{
		pst_GG_Diagram->arf32_AccelX[i] = pst_GG_Diagram->arf32_AccelX[i - 1];
		pst_GG_Diagram->arf32_AccelY[i] = pst_GG_Diagram->arf32_AccelY[i - 1];
	}

	pst_GG_Diagram->arf32_AccelX[0] = pst_CANData->f32_Long_ACCEL / 9.81;
	pst_GG_Diagram->arf32_AccelY[0] = pst_CANData->f32_LAT_ACCEL / 9.81;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Control Method ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void AdvancedStanleyMethod(PLANNING_DATA_t *pst_PlanningData,
						   DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
						   CONTROL_DATA_t *pst_ControlData,
						   CAN_DATA_t *pst_CANData)
{
	PATH_t *pst_FinalPath = &pst_PlanningData->st_FinalPath_global;
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	Stanley_t *pst_Stanley = &pst_ControlData->st_Stanley;
	IMU_DATA_t *pst_IMU = &pst_DeadReckoningData->st_IMU;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;
	
	int32_t s32_NearIdx = pst_PlanningData->st_ReferenceLine2_global.s32_NearIdx;
	float32_t f32_TargetSteer_rad = 0;
	float32_t f32_LD = 0;
	float32_t f32_k = 0;

	if (InRange(0.f, 100.f, pst_OriginState->f32_VelocityX_m_s * 3.6f))
	{
		f32_k = pst_Stanley->f32_lowk;
	}
	else
	{
		f32_k = pst_Stanley->f32_highk;
	}

	if (InRange(c_FIRSTBANKSTART_IDX, c_SECONDBANKEND_IDX, s32_NearIdx))
	{
		f32_LD = 1.1f;
		if(InRange(c_FIRSTBANKEND_IDX - 100, c_FIRSTBANKEND_IDX, s32_NearIdx))
		{
			f32_LD = 0.9f;
		}
	}
	else
	{
		f32_LD = 1.2f;
	}

	// LD Set
	pst_Stanley->f32_LD = pst_OriginState->f32_VelocityX_m_s * 0.36 * f32_LD;
	if (pst_PlanningData->st_Planner.b_Bank &&
		pst_PlanningData->st_EgoVehicleData.s32_CurrentLane <= 5 &&
		abs(pst_IMU->f32_Roll_deg) >= 5.f)
	{
		pst_Stanley->f32_LD = pst_OriginState->f32_VelocityX_m_s * 0.36 * 0.8;
	}
	GetModData(4.0, 20, pst_Stanley->f32_LD);

	// Position of Front Axle
	pst_Stanley->f32_FX = pst_OriginState->f32_CX + sin(pst_OriginState->f32_Yaw_rad_NED) * pst_Param->f32_LF;
	pst_Stanley->f32_FY = pst_OriginState->f32_CY + cos(pst_OriginState->f32_Yaw_rad_NED) * pst_Param->f32_LF;
	CalcNearestDistIdx(pst_Stanley->f32_FX, pst_Stanley->f32_FY, pst_FinalPath->arf32_X, pst_FinalPath->arf32_Y,
					   pst_FinalPath->s32_Num, pst_Stanley->f32_FrontDist, pst_Stanley->s32_FrontNear);

	// Get LD Index
	for (int32_t i = pst_Stanley->s32_FrontNear; i < pst_Stanley->s32_FrontNear + 100; i++)
	{
		int32_t s32_Idx = i;
		if (s32_Idx > pst_FinalPath->s32_Num - 1)
		{
			s32_Idx = pst_FinalPath->s32_Num - 1;
		}
		float32_t tmp_dist = getDistance2d(pst_FinalPath->arf32_X[s32_Idx], pst_FinalPath->arf32_Y[s32_Idx], pst_Stanley->f32_FX, pst_Stanley->f32_FY);
		if (tmp_dist > pst_Stanley->f32_LD)
		{
			pst_Stanley->s32_LDlen = s32_Idx - pst_Stanley->s32_FrontNear;
			break;
		}
	}

	// Path Vector
	float32_t arf32_Path_Vec[2] = {pst_Stanley->f32_FX - pst_FinalPath->arf32_X[pst_Stanley->s32_FrontNear],
								   pst_Stanley->f32_FY - pst_FinalPath->arf32_Y[pst_Stanley->s32_FrontNear]};

	// Front Axle Vector
	float32_t arf32_Axle_Vec[2] = {sin(pst_OriginState->f32_Yaw_rad_NED),
								   cos(pst_OriginState->f32_Yaw_rad_NED)};

	// Get Error
	pst_Stanley->f32_Error = arf32_Path_Vec[1] * arf32_Axle_Vec[0] - arf32_Path_Vec[0] * arf32_Axle_Vec[1];

	// Get Reference Data
	pst_Stanley->f32_ReferenceYaw = pst_FinalPath->arf32_Yaw_rad_NED[pst_Stanley->s32_FrontNear + pst_Stanley->s32_LDlen];
	pst_Stanley->f32_speed = pst_OriginState->f32_VelocityX_m_s;
	GetModData(1.0, pst_Param->f32_MAX_SPEED_m_s, pst_Stanley->f32_speed);

	// Get Target Steer
	f32_TargetSteer_rad = (pst_Stanley->f32_ke * pi2pi(pst_Stanley->f32_ReferenceYaw - pst_OriginState->f32_Yaw_rad_NED) +
						   atan2f(f32_k * pst_Stanley->f32_Error, pst_Stanley->f32_speed)) /
						  pst_Stanley->f32_Factor;

	// Filter
	OverShootFilter(pst_Param->f32_MAX_DSTEER_rad_s * pst_Param->f32_DT, pst_Stanley->f32_PreSteer_rad, f32_TargetSteer_rad);
	GetModData(-pst_Param->f32_MAX_STEER_rad, pst_Param->f32_MAX_STEER_rad, f32_TargetSteer_rad);

	// Update PreSteer
	pst_Stanley->f32_PreSteer_rad = f32_TargetSteer_rad;

	// Control to CAN
	pthread_mutex_lock(&st_ControlToCAN);
	pst_ControlData->f32_TargetSteer_rad = f32_TargetSteer_rad;
	pst_ControlData->f32_TargetSteer_deg = rad2deg(f32_TargetSteer_rad);
	pthread_mutex_unlock(&st_ControlToCAN);
}

void HybridMethod(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData)
{
	PATH_t *pst_FinalPath = &pst_PlanningData->st_FinalPath_global;
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	Stanley_t *pst_Stanley = &pst_ControlData->st_Stanley;
	IMU_DATA_t *pst_IMU = &pst_DeadReckoningData->st_IMU;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	// Target Steer
	float32_t f32_TargetSteer_rad = 0;

	// LD Set
	pst_Stanley->f32_LD = pst_OriginState->f32_VelocityX_m_s * 0.36 * 1.05;
	if (pst_PlanningData->st_Planner.b_Bank &&
		pst_PlanningData->st_EgoVehicleData.s32_CurrentLane <= 5 &&
		abs(pst_IMU->f32_Roll_deg) >= 5.f)
	{
		pst_Stanley->f32_LD = pst_OriginState->f32_VelocityX_m_s * 0.36 * 0.8;
	}
	GetModData(4.0, 20, pst_Stanley->f32_LD);

	// Position of Front Axle
	pst_Stanley->f32_FX = pst_OriginState->f32_CX + sin(pst_OriginState->f32_Yaw_rad_NED) * (pst_Param->f32_LF + pst_Stanley->f32_LD);
	pst_Stanley->f32_FY = pst_OriginState->f32_CY + cos(pst_OriginState->f32_Yaw_rad_NED) * (pst_Param->f32_LF + pst_Stanley->f32_LD);
	CalcNearestDistIdx(pst_Stanley->f32_FX, pst_Stanley->f32_FY, pst_FinalPath->arf32_X, pst_FinalPath->arf32_Y,
					   pst_FinalPath->s32_Num, pst_Stanley->f32_FrontDist, pst_Stanley->s32_FrontNear);

	// Path Vector
	float32_t arf32_Path_Vec[2] = {pst_Stanley->f32_FX - pst_FinalPath->arf32_X[pst_Stanley->s32_FrontNear],
								   pst_Stanley->f32_FY - pst_FinalPath->arf32_Y[pst_Stanley->s32_FrontNear]};

	// Front Axle Vector
	float32_t arf32_Axle_Vec[2] = {sin(pst_OriginState->f32_Yaw_rad_NED),
								   cos(pst_OriginState->f32_Yaw_rad_NED)};

	// Get Error
	pst_Stanley->f32_Error = arf32_Path_Vec[1] * arf32_Axle_Vec[0] - arf32_Path_Vec[0] * arf32_Axle_Vec[1];

	// Get Reference Data
	pst_Stanley->f32_ReferenceYaw = pst_FinalPath->arf32_Yaw_rad_NED[pst_Stanley->s32_FrontNear];
	pst_Stanley->f32_speed = pst_OriginState->f32_VelocityX_m_s;
	GetModData(1.0, pst_Param->f32_MAX_SPEED_m_s, pst_Stanley->f32_speed);

	// Get Target Steer
	f32_TargetSteer_rad = (pst_Stanley->f32_ke * pi2pi(pst_Stanley->f32_ReferenceYaw - pst_OriginState->f32_Yaw_rad_NED) +
						   atan2f(pst_Stanley->f32_lowk * pst_Stanley->f32_Error, pst_Stanley->f32_speed));
	//    /pst_Stanley->f32_Factor;

	// Filter
	OverShootFilter(pst_Param->f32_MAX_DSTEER_rad_s * pst_Param->f32_DT, pst_Stanley->f32_PreSteer_rad, f32_TargetSteer_rad);
	GetModData(-pst_Param->f32_MAX_STEER_rad, pst_Param->f32_MAX_STEER_rad, f32_TargetSteer_rad);

	// Update PreSteer
	pst_Stanley->f32_PreSteer_rad = f32_TargetSteer_rad;

	// Control to CAN
	pthread_mutex_lock(&st_ControlToCAN);
	pst_ControlData->f32_TargetSteer_rad = f32_TargetSteer_rad;
	pst_ControlData->f32_TargetSteer_deg = rad2deg(f32_TargetSteer_rad);
	pthread_mutex_unlock(&st_ControlToCAN);
}

void LKAS(CAMERA_DATA_t *pst_CameraData, CONTROL_DATA_t *pst_ControlData, PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LKAS_t *pst_LKAS = &pst_ControlData->st_LKAS;
	PID_t *pst_PID = &pst_ControlData->st_PID_LKAS;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	float32_t f32_TargetSteer_rad = 0;
	float32_t f32_MaxDsteer = pst_LKAS->f32_MAX_DSTEER_rad_s * 0.05;
	float32_t f32_speed = pst_OriginState->f32_VelocityX_m_s;
	GetModData(3.0, pst_Param->f32_MAX_SPEED_m_s, f32_speed);

	// Error Function
	pst_LKAS->f32_HeadingError = atanf((float32_t)(436 - pst_CameraData->s32_WayPntX) / (float32_t)(400 - pst_CameraData->s32_WayPntY)); // 450, 400
	pst_LKAS->f32_PathError = (pst_CameraData->f32_DistLeft - pst_CameraData->f32_DistRight) / (pst_CameraData->f32_DistLeft + pst_CameraData->f32_DistRight);

	// Steering Angle
	f32_TargetSteer_rad = pst_LKAS->f32_ke * pst_LKAS->f32_HeadingError + atan2(pst_LKAS->f32_k * pst_LKAS->f32_PathError, f32_speed);

	// Propotional Control
	pst_PID->f32_Error = f32_TargetSteer_rad - deg2rad(pst_CANData->f32_StrAng / 12.9);
	pst_PID->f32_InputP = pst_PID->f32_KP * pst_PID->f32_Error;
	f32_TargetSteer_rad = pst_PID->f32_InputP;

	// Filter
	GetModData(-pst_Param->f32_MAX_STEER_rad, pst_Param->f32_MAX_STEER_rad, f32_TargetSteer_rad);
	OverShootFilter(f32_MaxDsteer, pst_LKAS->f32_PreSteerAngle_rad, f32_TargetSteer_rad);
	pst_LKAS->f32_PreSteerAngle_rad = f32_TargetSteer_rad;

	// Control to Can
	pthread_mutex_lock(&st_ControlToCAN);
	pst_ControlData->f32_TargetSteer_rad = -f32_TargetSteer_rad;
	pst_ControlData->f32_TargetSteer_deg = -rad2deg(f32_TargetSteer_rad);
	pthread_mutex_unlock(&st_ControlToCAN);
}

void PIDControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	PID_t *pst_PID = &pst_ControlData->st_PID_Throttle;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	float32_t f32_TargetThrottle = 0;

	// Get Error
	pst_PID->f32_Error = pst_ControlData->f32_TargetSpeed_m_s - pst_OriginState->f32_VelocityX_m_s;
	// Calculation Input
	pst_PID->f32_InputP = pst_PID->f32_KP * pst_PID->f32_Error;
	pst_PID->f32_InputD = pst_PID->f32_KD * (pst_PID->f32_Error - pst_PID->f32_PreError) / pst_Param->f32_DT;
	f32_TargetThrottle = (pst_PID->f32_InputP + pst_PID->f32_InputD);

	if (f32_TargetThrottle >= 0)
	{
		f32_TargetThrottle = f32_TargetThrottle / 2.0f;
	}
	else if (f32_TargetThrottle < 0)
	{
		f32_TargetThrottle = f32_TargetThrottle / 4.f;
	}

	// Update PreError
	pst_PID->f32_PreError = pst_PID->f32_Error;

	pthread_mutex_lock(&st_ControlToCAN);
	pst_ControlData->f32_TargetThrottle = f32_TargetThrottle;
	GetModData(-4.0, 2.0, pst_ControlData->f32_TargetThrottle);
	pthread_mutex_unlock(&st_ControlToCAN);
}

// void AdaptiveCruiseControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData)
// {

// 	// 기존 코드
// 	VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;
// 	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
// 	PID_t *pst_PID = &pst_ControlData->st_PID_ACC;
// 	ACC_t *pst_ACC = &pst_ControlData->st_ACC;
// 	State_t *pst_OriginState = &pst_ControlData->st_OriginState;
// 	PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;

// 	if (pst_ControlData->s32_V2XMode != 1)
// 		return;

// 	int32_t s32_TargetAccIdx = pst_PlanningData->s32_AccObjIdx;

// 	// Relative Distance and Relative Velocity
// 	pst_ACC->f32_SafetyDist = pst_PlanningData->arst_ObjectData[s32_TargetAccIdx].f32_SafetyDistance;
// 	pst_ACC->f32_RelDist = pst_PlanningData->arst_ObjectData[s32_TargetAccIdx].f32_RelativeDistance;
// 	pst_ACC->f32_RelVel = pst_PlanningData->arst_ObjectData[s32_TargetAccIdx].f32_RelativeVelocity;

// 	// Error Calculation
// 	pst_ACC->f32_ErrorDistance = (pst_ACC->f32_RelDist - pst_ACC->f32_SafetyDist) / pst_ACC->f32_MaxDistance;
// 	pst_ACC->f32_ErrorVelocity = pst_ACC->f32_RelVel / pst_ACC->f32_MaxVelocity;
// 	pst_PID->f32_Error = pst_ACC->f32_ErrorDistance + pst_ACC->f32_ErrorVelocity;

// 	// Proportial
// 	pst_PID->f32_InputP = pst_PID->f32_KP * pst_PID->f32_Error;

// 	// Differential
// 	pst_PID->f32_InputD = pst_PID->f32_KD * (pst_PID->f32_Error - pst_PID->f32_PreError) / pst_Param->f32_DT;
// 	pst_PID->f32_PreError = pst_PID->f32_Error;

// 	// Control to CAN
// 	pthread_mutex_lock(&st_ControlToCAN);
// 	pst_ControlData->f32_TargetThrottle = pst_PID->f32_InputP + pst_PID->f32_InputD;

// 	if (pst_OriginState->f32_VelocityX_m_s > pst_Param->f32_MAX_SPEED_m_s)
// 	{
// 		pst_ControlData->f32_TargetThrottle = -4;
// 	}
// 	GetModData(-4.0, 2.0, pst_ControlData->f32_TargetThrottle);

// 	if (pst_PlanningData->arst_ObjectData[s32_TargetAccIdx].b_AEB_Mode)
// 	{
// 		pst_ControlData->f32_TargetThrottle = -4;
// 	}

// 	if (pst_PlanningData->arst_ObjectData[s32_TargetAccIdx].b_ABS_Mode
// 		&& pst_CANData->u8_EPS_Control_Status == 2)
// 	{
// 		pst_CANData->u8_AEB_En = 1;
// 	}
// 	else
// 	{
// 		pst_CANData->u8_AEB_En = 0;
// 	}

// 	// if (!pst_PlanningData->st_Planner.s32_LabCount)
// 	// {
// 	// 	if (20.f < pst_PlanningData->st_EgoVehicleData.f32_Velocity_kph_global && pst_PlanningData->st_EgoVehicleData.f32_Velocity_kph_global < 25.f)
// 	// 	{
// 	// 		GetModData(0.0, 0.5, pst_ControlData->f32_TargetThrottle);
// 	// 	}
// 	// 	else if (25.f <= pst_PlanningData->st_EgoVehicleData.f32_Velocity_kph_global)
// 	// 	{
// 	// 		pst_ControlData->f32_TargetThrottle = 0.f;
// 	// 	}
// 	// }

// 	// if (pst_ControlData->s32_V2XMode == 0)
// 	// {
// 	// 	pst_ControlData->f32_TargetThrottle = -4;
// 	// }

// 	pthread_mutex_unlock(&st_ControlToCAN);
// }


//Sangwoo
void AdaptiveCruiseControl(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData, CAN_DATA_t *pst_CANData)
{

	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	PID_t *pst_PID = &pst_ControlData->st_PID_Throttle;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	float32_t f32_TargetThrottle = 0;
	float32_t f32_TargetSpeed_m_s = pst_PlanningData->arst_ObjectData[pst_PlanningData->s32_AccObjIdx].f32_TargetVelocity_m_s;

	// Get Error
	if (pst_PlanningData->st_Planner.s32_LabCount ==0)
	{
		if (f32_TargetSpeed_m_s > 25.f)
		{
			f32_TargetSpeed_m_s = 25.f;
		}
	}

	pst_PID->f32_Error = f32_TargetSpeed_m_s - pst_OriginState->f32_VelocityX_m_s;

	// Calculation Input
	pst_PID->f32_InputP = pst_PID->f32_KP * pst_PID->f32_Error;
	pst_PID->f32_InputD = pst_PID->f32_KD * (pst_PID->f32_Error - pst_PID->f32_PreError) / pst_Param->f32_DT;
	f32_TargetThrottle = (pst_PID->f32_InputP + pst_PID->f32_InputD);

	if (f32_TargetThrottle >= 0)
	{
		f32_TargetThrottle = f32_TargetThrottle / 2.0f;
	}
	else if (f32_TargetThrottle < 0)
	{
		f32_TargetThrottle = f32_TargetThrottle / 4.f;
	}

	if (pst_PlanningData->arst_ObjectData[pst_PlanningData->s32_AccObjIdx].b_ABS_Mode && pst_CANData->u8_EPS_Control_Status == 2)
	{
		pst_CANData->u8_AEB_En = 1;
	}
	else
	{
		pst_CANData->u8_AEB_En = 0;
	}
	pst_PID->f32_PreError = pst_PID->f32_Error;

	pthread_mutex_lock(&st_ControlToCAN);
	pst_ControlData->f32_TargetThrottle = f32_TargetThrottle;
	GetModData(-4.0, 2.0, pst_ControlData->f32_TargetThrottle);
	pthread_mutex_unlock(&st_ControlToCAN);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Longitudinal MPC Processing /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

tuple<casadi::MX, casadi::MX> GetSpeedModelMatrix(CONTROL_DATA_t *pst_ControlData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LONG_MPC_t *pst_LongMPC = &pst_ControlData->st_LongMPC;

	// State Matrix
	casadi::MX A = casadi::MX::zeros(2, 2);
	A(0, 0) = 1.0;
	A(0, 1) = pst_Param->f32_Sample_DT;
	A(1, 1) = 1.0;

	// Input Matrix
	casadi::MX B = casadi::MX::zeros(2, 1);
	B(0) = 0.5 * pow(pst_Param->f32_Sample_DT, 2);
	B(1) = pst_Param->f32_Sample_DT;

	return make_tuple(A, B);
}

void SpeedModel(CONTROL_DATA_t *pst_ControlData, float32_t f32_Throttle, float32_t f32_PreVelocity_m_s)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LONG_MPC_t *pst_LongMPC = &pst_ControlData->st_LongMPC;

	// Throttle Limit
	GetModData(-4.f, 2.f, f32_Throttle);

	// Speed Model
	pst_LongMPC->f32_Predict_Velocity_m_s = f32_PreVelocity_m_s + f32_Throttle * pst_Param->f32_Sample_DT;
	pst_LongMPC->f32_Predict_X = pst_LongMPC->f32_Predict_X + pst_LongMPC->f32_Predict_Velocity_m_s * pst_Param->f32_Sample_DT + 0.5 * f32_Throttle * pow(pst_Param->f32_Sample_DT, 2);

	// Speed Limit
	GetModData(pst_Param->f32_MIN_SPEED_m_s, pst_Param->f32_MAX_SPEED_m_s, pst_LongMPC->f32_Predict_Velocity_m_s);
}

void PredictSpeed(CONTROL_DATA_t *pst_ControlData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LONG_MPC_t *pst_LongMPC = &pst_ControlData->st_LongMPC;

	// Predicted State Start
	pst_LongMPC->arf32_Predict_X[0] = pst_LongMPC->f32_X;
	pst_LongMPC->arf32_Predict_Velocity[0] = pst_LongMPC->f32_Velocity_m_s;

	// Predicted State Update
	for (int32_t i = 1; i < pst_Param->u32_T + 1; i++)
	{
		SpeedModel(pst_ControlData, pst_LongMPC->arf32_Sol_Throttle[i - 1], pst_LongMPC->arf32_Predict_Velocity[i - 1]);
		pst_LongMPC->arf32_Predict_X[i] = pst_LongMPC->f32_X;
		pst_LongMPC->arf32_Predict_Velocity[i] = pst_LongMPC->f32_Velocity_m_s;
	}
}

void InitLongitudinalMPC(PLANNING_DATA_t *pst_PlanningData,
						 CONTROL_DATA_t *pst_ControlData,
						 CAN_DATA_t *pst_CANData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LONG_MPC_t *pst_LongMPC = &pst_ControlData->st_LongMPC;

	float64_t f64_Travel = 0.0;

	// Set Initial State
	pst_LongMPC->f32_X = 0.f;
	pst_LongMPC->f32_Velocity_m_s = pst_PlanningData->st_EgoVehicleData.f32_Velocity_m_s_global;

	// Set Reference State
	pst_LongMPC->MX_refState = casadi::MX::zeros(3, pst_Param->u32_T + 1);
	pst_LongMPC->MX_refState(0, 0) = pst_LongMPC->f32_X;
	pst_LongMPC->MX_refState(1, 0) = pst_ControlData->f32_TargetSpeed_m_s;

	for (int32_t i = 1; i < pst_Param->u32_T + 1; i++)
	{
		f64_Travel += abs(pst_ControlData->f32_TargetSpeed_m_s) * pst_Param->f32_Sample_DT;
		pst_LongMPC->MX_refState(0, i) = f64_Travel;
		pst_LongMPC->MX_refState(1, i) = pst_ControlData->f32_TargetSpeed_m_s;
	}
}

void LongitudinalMPC(PLANNING_DATA_t *pst_PlanningData,
					 CONTROL_DATA_t *pst_ControlData,
					 CAN_DATA_t *pst_CANData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LONG_MPC_t *pst_LongMPC = &pst_ControlData->st_LongMPC;

	// Exception handling
	if (pst_PlanningData->st_EgoVehicleData.f32_Velocity_m_s_global <= 0 && s32_CanMode == 0)
		return;
	if (pst_PlanningData->st_EgoVehicleData.f32_Velocity_m_s_global > pst_Param->f32_MAX_SPEED_m_s)
	{
		PIDControl(pst_PlanningData, pst_ControlData);
		return;
	}
	if (pst_ControlData->f32_TargetSpeed_m_s <= 0)
	{
		pthread_mutex_lock(&st_ControlToCAN);
		pst_ControlData->f32_TargetThrottle = -4.f;
		pthread_mutex_unlock(&st_ControlToCAN);
		return;
	}

	// Set Up Optimization
	InitLongitudinalMPC(pst_PlanningData, pst_ControlData, pst_CANData);

	// Init Optimization
	casadi::Opti opti = casadi::Opti();
	casadi::MX MX_X = opti.variable(2, pst_Param->u32_T + 1); // State variables (X, Velocity)
	casadi::MX MX_u = opti.variable(1, pst_Param->u32_T);
	casadi::MX cost = 0.0;
	casadi::MX A, B;

	// Weight Matrix
	pst_LongMPC->arf32_Qf[0] = pst_LongMPC->arf32_Q[0];
	pst_LongMPC->arf32_Qf[1] = pst_LongMPC->arf32_Q[1];

	// Predict Speed
	PredictSpeed(pst_ControlData);

	// Get Speed Model Matrix
	tie(A, B) = GetSpeedModelMatrix(pst_ControlData);

	for (int i = 0; i < pst_Param->u32_T; i++)
	{
		// constraints
		opti.subject_to(MX_X(casadi::Slice(), i + 1) == mtimes(A, MX_X(casadi::Slice(), i)) + mtimes(B, MX_u(casadi::Slice(), i)));
		opti.subject_to(MX_X(1, i) <= pst_Param->f32_MAX_SPEED_m_s);
		opti.subject_to(MX_X(1, i) >= pst_Param->f32_MIN_SPEED_m_s);
		opti.subject_to(MX_u(0, i) <= 2.0);
		opti.subject_to(MX_u(0, i) >= -4.0);

		if (i != 0)
		{
			// Cost Reference State
			cost += pst_LongMPC->arf32_Q[0] * mtimes((MX_X(0, i) - pst_LongMPC->MX_refState(0, i)).T(), MX_X(0, i) - pst_LongMPC->MX_refState(0, i));
			cost += pst_LongMPC->arf32_Q[1] * mtimes((MX_X(1, i) - pst_LongMPC->MX_refState(1, i)).T(), MX_X(1, i) - pst_LongMPC->MX_refState(1, i));
		}

		// Cost Control Input
		cost += pst_LongMPC->f32_R * mtimes(MX_u(0, i).T(), MX_u(0, i));

		// Cost Control Input Derivative
		if (i < pst_Param->u32_T - 1)
		{
			cost += pst_LongMPC->f32_Rd * mtimes((MX_u(0, i + 1) - MX_u(0, i)).T(), MX_u(0, i + 1) - MX_u(0, i));
		}
	}

	// Cost Final State
	cost += pst_LongMPC->arf32_Qf[0] * mtimes((MX_X(0, pst_Param->u32_T) - pst_LongMPC->MX_refState(0, pst_Param->u32_T)).T(), MX_X(0, pst_Param->u32_T) - pst_LongMPC->MX_refState(0, pst_Param->u32_T));
	cost += pst_LongMPC->arf32_Qf[1] * mtimes((MX_X(1, pst_Param->u32_T) - pst_LongMPC->MX_refState(1, pst_Param->u32_T)).T(), MX_X(1, pst_Param->u32_T) - pst_LongMPC->MX_refState(1, pst_Param->u32_T));

	// initial state
	opti.subject_to(MX_X(0, 0) == 0);
	opti.subject_to(MX_X(1, 0) == pst_LongMPC->f32_Velocity_m_s);

	// Minimize the cost
	opti.minimize(cost);

	// Plugin options
	casadi::Dict plugin_options;
	plugin_options["print_time"] = false;
	plugin_options["error_on_fail"] = false;

	// Solver options
	casadi::Dict solver_options;
	solver_options["print_level"] = 0;

	// Define the solver
	opti.solver("ipopt", plugin_options, solver_options);

	// Solve the optimization problem
	casadi::OptiSol sol = opti.solve();

	// get solver status
	if (opti.return_status() != "Solve_Succeeded")
	{
		printf("Fail to find the solution\n");
		for (int32_t i = 0; i < pst_Param->u32_T + 1; i++)
		{
			pst_LongMPC->arf32_Sol_X[i] = 0;
			pst_LongMPC->arf32_Sol_Velocity[i] = 0;
			pst_LongMPC->arf32_Sol_Throttle[i] = 0;
		}
		PIDControl(pst_PlanningData, pst_ControlData);
	}
	else
	{
		// printf("Solver status: %s", opti.return_status().c_str());
		for (int32_t i = 0; i < pst_Param->u32_T + 1; i++)
		{
			pst_LongMPC->arf32_Sol_X[i] = float64_t(sol.value(MX_X(0, i)));
			pst_LongMPC->arf32_Sol_Velocity[i] = float64_t(sol.value(MX_X(1, i)));

			if (i < pst_Param->u32_T)
			{
				pst_LongMPC->arf32_Sol_Throttle[i] = float64_t(sol.value(MX_u(0, i)));
			}
		}
	}

	// Control to CAN
	pthread_mutex_lock(&st_ControlToCAN);
	pst_ControlData->f32_TargetThrottle = pst_LongMPC->arf32_Sol_Throttle[0];
	pthread_mutex_unlock(&st_ControlToCAN);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Lateral MPC Processing ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void InitLateralMPC(PLANNING_DATA_t *pst_PlanningData,
					DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
					CONTROL_DATA_t *pst_ControlData,
					CAN_DATA_t *pst_CANData)
{
	LAT_MPC_t *pst_LatMPC = &pst_ControlData->st_LatMPC;
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	// // Set Origin State
	pst_LatMPC->st_CurrentState.f32_CX = pst_OriginState->f32_CX;
	pst_LatMPC->st_CurrentState.f32_CY = pst_OriginState->f32_CY;
	pst_LatMPC->st_CurrentState.f32_VelocityX_m_s = pst_OriginState->f32_VelocityX_m_s;
	pst_LatMPC->st_CurrentState.f32_VelocityY_m_s = pst_OriginState->f32_VelocityY_m_s;
	pst_LatMPC->st_CurrentState.f32_Yaw_rad_ENU = pst_OriginState->f32_Yaw_rad_ENU;
	pst_LatMPC->st_CurrentState.f32_Yawrate_rad_s = pst_OriginState->f32_Yawrate_rad_s;

	// Get Reference Trajectory
	GetRefTrajectory(pst_PlanningData, pst_ControlData);
}

void GetRefTrajectory(PLANNING_DATA_t *pst_PlanningData, CONTROL_DATA_t *pst_ControlData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LAT_MPC_t *pst_LatMPC = &pst_ControlData->st_LatMPC;
	PATH_t *pst_FinalPath = &pst_PlanningData->st_FinalPath_global;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	State_t st_RefState = {0};
	float32_t f32_Travel = 0;
	float32_t f32_Dist = 0;
	float32_t f32_Radius = 0;
	int32_t s32_NearIdx = 0;

	pst_LatMPC->MX_refState = casadi::MX::zeros(4, pst_Param->u32_T + 1);

	for (int32_t i = 0; i < pst_Param->u32_T + 1; i++)
	{
		st_RefState.f32_CX = pst_OriginState->f32_CX + sin(pst_OriginState->f32_Yaw_rad_NED) * f32_Travel;
		st_RefState.f32_CY = pst_OriginState->f32_CY + cos(pst_OriginState->f32_Yaw_rad_NED) * f32_Travel;
		f32_Travel += pst_ControlData->f32_TargetSpeed_m_s * pst_Param->f32_Sample_DT;

		CalcNearestDistIdx(st_RefState.f32_CX, st_RefState.f32_CY, pst_FinalPath->arf32_X, pst_FinalPath->arf32_Y,
						   pst_FinalPath->s32_Num, f32_Dist, s32_NearIdx);

		float32_t arf32_Path_Vec[2] = {st_RefState.f32_CX - pst_FinalPath->arf32_X[s32_NearIdx],
									   st_RefState.f32_CY - pst_FinalPath->arf32_Y[s32_NearIdx]};
		float32_t arf32_Axle_Vec[2] = {sin(pst_OriginState->f32_Yaw_rad_NED),
									   cos(pst_OriginState->f32_Yaw_rad_NED)};

		GetRadius(pst_FinalPath->arf32_X[s32_NearIdx], pst_FinalPath->arf32_Y[s32_NearIdx],
				  pst_FinalPath->arf32_X[s32_NearIdx + 1], pst_FinalPath->arf32_Y[s32_NearIdx + 1],
				  pst_FinalPath->arf32_X[s32_NearIdx + 2], pst_FinalPath->arf32_Y[s32_NearIdx + 2], f32_Radius);

		// Reference Data
		pst_LatMPC->arf32_Ref_Y[i] = -(arf32_Path_Vec[1] * arf32_Axle_Vec[0] - arf32_Path_Vec[0] * arf32_Axle_Vec[1]);
		pst_LatMPC->arf32_Ref_Yaw_rad[i] = pi2pi(pst_FinalPath->arf32_Yaw_rad_ENU[s32_NearIdx] - pst_OriginState->f32_Yaw_rad_ENU);
		pst_LatMPC->arf32_Ref_VY_m_s[i] = pst_ControlData->f32_TargetSpeed_m_s * pst_LatMPC->arf32_Ref_Yaw_rad[i];
		pst_LatMPC->arf32_Ref_Yawrate_rad_s[i] = pst_ControlData->f32_TargetSpeed_m_s / f32_Radius;

		// Reference Matrix
		pst_LatMPC->MX_refState(0, i) = pst_LatMPC->arf32_Ref_Y[i];
		pst_LatMPC->MX_refState(1, i) = pst_LatMPC->arf32_Ref_VY_m_s[i];
		pst_LatMPC->MX_refState(2, i) = pst_LatMPC->arf32_Ref_Yaw_rad[i];
		pst_LatMPC->MX_refState(3, i) = pst_LatMPC->arf32_Ref_Yawrate_rad_s[i];
	}
}

std::tuple<casadi::MX, casadi::MX> GetLinearModelMatrix(CONTROL_DATA_t *pst_ControlData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LAT_MPC_t *pst_LatMPC = &pst_ControlData->st_LatMPC;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	float32_t f32_MatA_y1 = 1 - (pst_Param->f32_CF + pst_Param->f32_CR) / (pst_Param->f32_Mass * pst_OriginState->f32_VelocityX_m_s) * pst_Param->f32_Sample_DT;
	float32_t f32_MatA_y2 = pst_Param->f32_Sample_DT * ((pst_Param->f32_CR * pst_Param->f32_LR - pst_Param->f32_CF * pst_Param->f32_LF) / (pst_Param->f32_Mass * pst_OriginState->f32_VelocityX_m_s) - pst_OriginState->f32_VelocityX_m_s);
	float32_t f32_MatB_y = pst_Param->f32_CF / pst_Param->f32_Mass * pst_Param->f32_Sample_DT;
	float32_t f32_MatA_psi1 = pst_Param->f32_Sample_DT * (pst_Param->f32_CR * pst_Param->f32_LR - pst_Param->f32_CF * pst_Param->f32_LF) / (pst_Param->f32_IZ * pst_OriginState->f32_VelocityX_m_s);
	float32_t f32_MatA_psi2 = 1 - pst_Param->f32_Sample_DT * (pst_Param->f32_CR * pow(pst_Param->f32_LR, 2) + pst_Param->f32_CF * pow(pst_Param->f32_LF, 2)) / (pst_Param->f32_IZ * pst_OriginState->f32_VelocityX_m_s);
	float32_t f32_MatB_psi = pst_Param->f32_CF / pst_Param->f32_IZ * pst_Param->f32_LF * pst_Param->f32_Sample_DT;

	casadi::MX A = casadi::MX::zeros(4, 4);
	A(0, 0) = 1.0;
	A(0, 1) = pst_Param->f32_Sample_DT;
	A(1, 1) = f32_MatA_y1;
	A(1, 3) = f32_MatA_y2;
	A(2, 2) = 1.0;
	A(2, 3) = pst_Param->f32_Sample_DT;
	A(3, 1) = f32_MatA_psi1;
	A(3, 3) = f32_MatA_psi2;

	casadi::MX B = casadi::MX::zeros(4, 1);
	B(1, 0) = f32_MatB_y;
	B(3, 0) = f32_MatB_psi;

	return std::make_tuple(A, B);
}

void VehicleDynamics(CONTROL_DATA_t *pst_ControlData, float32_t f32_delta)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LAT_MPC_t *pst_LatMPC = &pst_ControlData->st_LatMPC;
	State_t *pst_PredictState = &pst_LatMPC->st_PredictState;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	// Input Limit
	GetModData(-pst_Param->f32_MAX_STEER_rad, pst_Param->f32_MAX_STEER_rad, f32_delta);

	// Linear Model Matrix
	float32_t f32_MatA_y1 = 1 - (pst_Param->f32_CF + pst_Param->f32_CR) / (pst_Param->f32_Mass * pst_OriginState->f32_VelocityX_m_s) * pst_Param->f32_Sample_DT;
	float32_t f32_MatA_y2 = pst_Param->f32_Sample_DT * ((pst_Param->f32_CR * pst_Param->f32_LR - pst_Param->f32_CF * pst_Param->f32_LF) / (pst_Param->f32_Mass * pst_OriginState->f32_VelocityX_m_s) - pst_OriginState->f32_VelocityX_m_s);
	float32_t f32_MatB_y = pst_Param->f32_CF / pst_Param->f32_Mass * pst_Param->f32_Sample_DT;
	float32_t f32_MatA_psi1 = pst_Param->f32_Sample_DT * (pst_Param->f32_CR * pst_Param->f32_LR - pst_Param->f32_CF * pst_Param->f32_LF) / (pst_Param->f32_IZ * pst_OriginState->f32_VelocityX_m_s);
	float32_t f32_MatA_psi2 = 1 - pst_Param->f32_Sample_DT * (pst_Param->f32_CR * pow(pst_Param->f32_LR, 2) + pst_Param->f32_CF * pow(pst_Param->f32_LF, 2)) / (pst_Param->f32_IZ * pst_OriginState->f32_VelocityX_m_s);
	float32_t f32_MatB_psi = pst_Param->f32_CF / pst_Param->f32_IZ * pst_Param->f32_LF * pst_Param->f32_Sample_DT;

	// Update State
	pst_PredictState->f32_CY = pst_PredictState->f32_CY + pst_PredictState->f32_VelocityY_m_s * pst_Param->f32_Sample_DT;
	pst_PredictState->f32_Yaw_rad_ENU = pst_PredictState->f32_Yaw_rad_ENU + pst_PredictState->f32_Yawrate_rad_s * pst_Param->f32_Sample_DT;

	pst_PredictState->f32_VelocityY_m_s = pst_PredictState->f32_VelocityY_m_s * f32_MatA_y1 +
										  pst_PredictState->f32_Yawrate_rad_s * f32_MatA_y2 +
										  f32_delta * f32_MatB_y;
	pst_PredictState->f32_Yawrate_rad_s = pst_PredictState->f32_VelocityY_m_s * f32_MatA_psi1 +
										  pst_PredictState->f32_Yawrate_rad_s * f32_MatA_psi2 +
										  f32_delta * f32_MatB_psi;
}

void PredictMotion(CONTROL_DATA_t *pst_ControlData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LAT_MPC_t *pst_LatMPC = &pst_ControlData->st_LatMPC;
	State_t *pst_OriginState = &pst_ControlData->st_OriginState;

	// Set initial state for MPC
	pst_LatMPC->st_PredictState.f32_CY = 0;
	pst_LatMPC->st_PredictState.f32_VelocityY_m_s = pst_OriginState->f32_VelocityY_m_s;
	pst_LatMPC->st_PredictState.f32_Yaw_rad_ENU = 0;
	pst_LatMPC->st_PredictState.f32_Yawrate_rad_s = pst_OriginState->f32_Yawrate_rad_s;

	// Predicted Data First
	pst_LatMPC->arf32_Predict_Y[0] = pst_LatMPC->st_PredictState.f32_CY;
	pst_LatMPC->arf32_Predict_VY_m_s[0] = pst_LatMPC->st_PredictState.f32_VelocityY_m_s;
	pst_LatMPC->arf32_Predict_Yaw_rad[0] = pst_LatMPC->st_PredictState.f32_Yaw_rad_ENU;
	pst_LatMPC->arf32_Predict_Yawrate_rad_s[0] = pst_LatMPC->st_PredictState.f32_Yawrate_rad_s;

	// Predicted Data
	for (int32_t i = 1; i < pst_Param->u32_T + 1; i++)
	{
		VehicleDynamics(pst_ControlData, pst_LatMPC->arf32_SolSteer[i - 1]);
		pst_LatMPC->arf32_Predict_Y[i] = pst_LatMPC->st_PredictState.f32_CY;
		pst_LatMPC->arf32_Predict_VY_m_s[i] = pst_LatMPC->st_PredictState.f32_VelocityY_m_s;
		pst_LatMPC->arf32_Predict_Yaw_rad[i] = pst_LatMPC->st_PredictState.f32_Yaw_rad_ENU;
		pst_LatMPC->arf32_Predict_Yawrate_rad_s[i] = pst_LatMPC->st_PredictState.f32_Yawrate_rad_s;
	}
}

void SystemID()
{
	// Init Optimization
	// casadi::Opti opti = casadi::Opti();
	// casadi::MX delta = opti.variable(1);
	// casadi::MX yawrate = opti.variable(1);

	// // Define the cost function
	// casadi::MX cost = 0.0;

	// // Define the yawrate equation as a constraint
	// opti.subject_to(yawrate == pst_PlanningData->st_EgoVehicleData.f32_Velocity_m_s_global *
	//                       cos(atan(pst_Param->f32_LR * tan(delta) / pst_Param->f32_WB)) /
	//                       pst_Param->f32_WB * tan(delta));

	// // Define the cost function as the squared difference between desired and actual yaw rate
	// cost = pow(yawrate - deg2rad(pst_CANData->f32_YAW_RATE) , 2);

	// // Minimize the cost
	// opti.minimize(cost);

	// // Plugin options
	// casadi::Dict plugin_options;
	// plugin_options["print_time"] = false;
	// plugin_options["error_on_fail"] = false;

	// // Solver options
	// casadi::Dict solver_options;
	// solver_options["print_level"] = 0;

	// // Define the solver
	// opti.solver("ipopt", plugin_options, solver_options);

	// // Solve the optimization problem
	// casadi::OptiSol sol = opti.solve();

	// float64_t f64_delta = float64_t(sol.value(delta));
	// f64_delta = rad2deg(f64_delta);
	// // Output the solution

	// 	// Dynamic
	// float32_t f32_Steer_rad = deg2rad(pst_CANData->f32_StrAng / 12.9);
	// float32_t f32_FrontWheelSpeed = (pst_CANData->f32_WHEEL_SPD_FL + pst_CANData->f32_WHEEL_SPD_FR) / 2.f;
	// float32_t f32_RearWheelSpeed = (pst_CANData->f32_WHEEL_SPD_RL + pst_CANData->f32_WHEEL_SPD_RR) / 2.f;
	// float32_t f32_beta = atanf(pst_Param->f32_LR * tan(f32_Steer_rad) / pst_Param->f32_WB);

	// float32_t f32_FrontForceX = pst_Param->f32_Mass * pst_Param->f32_LR / pst_Param->f32_WB *
	// 							  (pst_CANData->f32_Long_ACCEL * cosf(f32_Steer_rad) + pst_CANData->f32_LAT_ACCEL * sinf(f32_Steer_rad));

	// float32_t f32_FrontForceY = pst_Param->f32_Mass * pst_Param->f32_LR / pst_Param->f32_WB *
	// 							  (pst_CANData->f32_Long_ACCEL * sinf(f32_Steer_rad) - pst_CANData->f32_LAT_ACCEL * cosf(f32_Steer_rad));

	// cout << "Slep Angle : " << f64_delta - pst_CANData->f32_StrAng / 12.9 << endl;
	// cout << "Force :  " << f32_FrontForceY << endl;

	// // Open a file in write mode.
	// ofstream outfile;
	// outfile.open("output.txt", ios::app); // Open file in append mode to keep previous logs

	// // Write the results to the file instead of cout
	// outfile << f32_FrontForceY << " " << f64_delta - pst_CANData->f32_StrAng / 12.9 << endl;

	// // Close the file
	// outfile.close();
}

void LateralMPC(PLANNING_DATA_t *pst_PlanningData,
				DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
				CONTROL_DATA_t *pst_ControlData,
				CAN_DATA_t *pst_CANData)
{
	CONTROL_PARAM_t *pst_Param = &pst_ControlData->st_Param;
	LAT_MPC_t *pst_LatMPC = &pst_ControlData->st_LatMPC;

	if (pst_PlanningData->st_EgoVehicleData.f32_Velocity_m_s_global <= 0 && s32_CanMode == 0)
		return;

	// Set initial state
	InitLateralMPC(pst_PlanningData, pst_DeadReckoningData, pst_ControlData, pst_CANData);

	// Init Optimization
	casadi::Opti opti = casadi::Opti();
	casadi::MX A, B;
	casadi::MX cost = 0.0;

	// Variables
	casadi::MX x = opti.variable(4, pst_Param->u32_T + 1); // State variables (Y, VY, Yaw, Yawrate)
	casadi::MX u = opti.variable(1, pst_Param->u32_T);	   // Control variables (steer)

	// predict state
	// PredictMotion(pst_ControlData);

	// // Optimization Problem Setting
	// for (int32_t i = 0; i < pst_Param->u32_T; i++)
	// {
	// 	// system dynamics
	// 	std::tie(A, B) = GetLinearModelMatrix(pst_LatMPC->arf32_PredictV[i],
	// 									      pst_LatMPC->arf32_PredictYaw[i] - pst_ControlData->f32_PreYaw,
	// 									      0, pst_ControlData);

	// 	// constraints
	// 	opti.subject_to(x(casadi::Slice(), i + 1) == mtimes(A, x(casadi::Slice(), i)) + mtimes(B, u(casadi::Slice(), i)));
	// 	opti.subject_to(u(0, i) <= pst_Param->f32_MAX_STEER_rad);
	// 	opti.subject_to(u(0, i) >= -pst_Param->f32_MAX_STEER_rad);

	// 	// cost function
	// 	if (i != 0)
	// 	{
	// 		cost += arf32_Q[0] * mtimes((x(0, i) - pst_LatMPC->MX_refState(0, i)).T(), x(0, i) - pst_LatMPC->MX_refState(0, i));
	// 		cost += arf32_Q[1] * mtimes((x(1, i) - pst_LatMPC->MX_refState(1, i)).T(), x(1, i) - pst_LatMPC->MX_refState(1, i));
	// 		cost += arf32_Q[2] * mtimes((x(2, i) - pst_LatMPC->MX_refState(2, i)).T(), x(2, i) - pst_LatMPC->MX_refState(2, i));
	// 		cost += arf32_Q[3] * mtimes((x(3, i) - pst_LatMPC->MX_refState(3, i)).T(), x(3, i) - pst_LatMPC->MX_refState(3, i));
	// 	}

	// 	cost += arf32_R[0] * mtimes(u(0, i).T(), u(0, i));
	// 	cost += arf32_R[1] * mtimes(u(1, i).T(), u(1, i));

	// 	// Differentiation Cost & Constraints
	// 	if (i < pst_Param->u32_T - 1)
	// 	{
	// 		opti.subject_to(fabs(u(1, i + 1) - u(1, i)) <= pst_Param->f32_MAX_DSTEER_rad_s * pst_Param->f32_DT); // Delta Steer Rate
	// 		cost += 1.5 * mtimes((x(3, i + 1) - x(3, i)).T(), x(3, i + 1) - x(3, i));
	// 		cost += arf32_Rd[0] * mtimes((u(0, i + 1) - u(0, i)).T(), u(0, i + 1) - u(0, i));
	// 		cost += arf32_Rd[1] * mtimes((u(1, i + 1) - u(1, i)).T(), u(1, i + 1) - u(1, i));
	// 	}
	// }

	// // Final cost
	// for (int32_t i = 0; i < pst_Param->u32_NX; i++)
	// {
	// 	cost += arf32_Qf[i] * mtimes((x(i, pst_Param->u32_T) - pst_LatMPC->MX_refState(i, pst_Param->u32_T)).T(),
	// 								 x(i, pst_Param->u32_T) - pst_LatMPC->MX_refState(i, pst_Param->u32_T));
	// }
	// // initial state
	// opti.subject_to(x(0, 0) == 0);
	// opti.subject_to(x(1, 0) == 0);
	// opti.subject_to(x(2, 0) == pst_LatMPC->st_OriginState.f32_Velocity_m_s);
	// opti.subject_to(x(3, 0) == 0);

	// // minimize cost
	// opti.minimize(cost);

	// // plugin options
	// casadi::Dict plugin_options;
	// plugin_options["print_time"] = false;
	// plugin_options["error_on_fail"] = false;

	// // solver options
	// casadi::Dict solver_options;
	// solver_options["print_level"] = 0;

	// // solver
	// opti.solver("ipopt", plugin_options, solver_options);
	// casadi::OptiSol sol = opti.solve();

	// // get solver status
	// if (opti.return_status() != "Solve_Succeeded")
	// {
	// 	printf("Fail to find the solution\n");
	// 	for (int32_t i = 0; i < pst_Param->u32_T; i++)
	// 	{
	// 		pst_LatMPC->arf32_SolX[i] = 0;
	// 		pst_LatMPC->arf32_SolY[i] = 0;
	// 		pst_LatMPC->arf32_SolV[i] = 0;
	// 		pst_LatMPC->arf32_SolYaw[i] = 0;
	// 		pst_LatMPC->arf32_SolAccel[i] = 0;
	// 		pst_LatMPC->arf32_SolSteer[i] = 0;
	// 	}
	// }
	// else
	// {
	// 	printf("Solver status: %s", opti.return_status().c_str());

	// 	// Predicted Trajectory & Control
	// 	for (int32_t i = 0; i < pst_Param->u32_T; i++)
	// 	{
	// 		pst_LatMPC->arf32_SolX[i] = float64_t(sol.value(x(0, i)));
	// 		pst_LatMPC->arf32_SolY[i] = float64_t(sol.value(x(1, i)));
	// 		pst_LatMPC->arf32_SolV[i] = float64_t(sol.value(x(2, i)));
	// 		pst_LatMPC->arf32_SolYaw[i] = float64_t(sol.value(x(3, i)));

	// 		if (i < pst_Param->u32_T)
	// 		{
	// 			pst_LatMPC->arf32_SolAccel[i] = float64_t(sol.value(u(0, i)));
	// 			pst_LatMPC->arf32_SolSteer[i] = float64_t(sol.value(u(1, i)));
	// 		}
	// 	}
	// }

	// // Update pre yaw
	// pst_ControlData->f32_PreYaw = pst_LatMPC->st_OriginState.f32_Yaw_rad;

	// pthread_mutex_lock(&st_ControlToCAN);
	// // pst_ControlData->f32_TargetSteer_rad = f32_TargetSteer_rad / 2.5;
	// // pst_ControlData->f32_TargetSteer_deg = rad2deg(f32_TargetSteer_rad) / 2.5;
	// pthread_mutex_unlock(&st_ControlToCAN);
}
