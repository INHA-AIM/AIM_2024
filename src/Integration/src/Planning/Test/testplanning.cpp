#include "testplanning.h"

extern std::string s_MapName;
extern VEHICLE_STATE_t st_VehicleState;

// ACC Test, Lane Change Test, Path Tracking Test

void TEST_PlanningProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                 DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                 CAMERA_DATA_t *pst_CameraData,
                                 PLANNING_DATA_t *pst_PlanningData,
                                 CAN_DATA_t *pst_CANData)
{
    TEST_FindNearIdx(pst_PlanningData);
    TEST_FindBankFlag(pst_PlanningData);
    TEST_FlagProcessing(pst_PlanningData, pst_CANData);    // Go, Stop, Slow on, Slow off, Pit Stop에 대한 Flag Planning 수행
    TEST_RestartZoneCheck(pst_PlanningData);               // PitStop에서 나갈 때, 차량이 진입하는지?

    TEST_EgoVehicleDataProcessing(pst_PlanningData, pst_DeadReckoningData, pst_CANData, pst_LidarData);
    TEST_ClusteredDataProcessing(pst_PlanningData, pst_DeadReckoningData, pst_LidarData, pst_CANData);

    TEST_FrenetPathGeneration(pst_PlanningData);            // 총 9개의 Frenet Path 생성
    TEST_CalcFrenetToGlobalPath(pst_PlanningData);          // Frenet Path 를 Global ENU 좌표로 변환

    TEST_FindAvailableFrenetLane(pst_PlanningData);               // 충돌하지 않는 프레넷 경로 상 여유라인을 찾아주는 함수
    TEST_FindAvailableGridLane(pst_PlanningData);

    TEST_StateProcessing(pst_LidarData, pst_DeadReckoningData, pst_CameraData, pst_PlanningData, pst_CANData);
    TEST_FinalPath(pst_PlanningData);                       // Final Path는 Control에 넘겨줘야 하는 최종 Path
}


void TEST_PitStopProcessing(PLANNING_DATA_t *pst_PlanningData, SPLINE2D_PARAM_t **pst_Spline2DParam, SPLINE_PATH_t **pst_SplinePath)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;

    if (pst_Planner->u8_PitStopMode || (pst_Planner->s32_LabCount == 0  && pst_PlanningData->st_ReferenceSELine2_global.s32_NearIdx < 520))
    {
        *pst_Spline2DParam = &pst_PlanningData->st_PisStopSpline2DParam;
        *pst_SplinePath = &pst_PlanningData->st_PitStopSplinePath_global;
    }
    else
    {
        *pst_Spline2DParam = &pst_PlanningData->st_Spline2DParam;
        *pst_SplinePath = &pst_PlanningData->st_SplinePath_global;
    }
}


void TEST_FindNearIdx(PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_NearIdx;
    float32_t f32_NearDist;

    CalcNearestDistIdx(pst_PlanningData->st_EgoVehicleData.f32_X_global, pst_PlanningData->st_EgoVehicleData.f32_Y_global
                      ,pst_PlanningData->st_ReferenceLine2_global.arf32_X, pst_PlanningData->st_ReferenceLine2_global.arf32_Y
                      ,pst_PlanningData->st_ReferenceLine2_global.s32_Num, f32_NearDist, s32_NearIdx);
                       pst_PlanningData->st_ReferenceLine2_global.s32_NearIdx = s32_NearIdx;

    CalcNearestDistIdx(pst_PlanningData->st_EgoVehicleData.f32_X_global, pst_PlanningData->st_EgoVehicleData.f32_Y_global
                      ,pst_PlanningData->st_ReferenceSELine2_global.arf32_X, pst_PlanningData->st_ReferenceSELine2_global.arf32_Y
                      ,pst_PlanningData->st_ReferenceSELine2_global.s32_Num, f32_NearDist, s32_NearIdx);
                       pst_PlanningData->st_ReferenceSELine2_global.s32_NearIdx = s32_NearIdx;
}

void TEST_FindBankFlag(PLANNING_DATA_t *pst_PlanningData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;

    int32_t s32_NearIdx = pst_PlanningData->st_ReferenceLine2_global.s32_NearIdx;

    if (c_FIRSTBANKSTART_IDX < s32_NearIdx && s32_NearIdx <  c_FIRSTBANKEND_IDX)
    {
        pst_Planner->b_Bank = true;
    }
    else if(c_SECONDBANKSTART_IDX < s32_NearIdx && s32_NearIdx <  c_SECONDBANKEND_IDX)
    {
        pst_Planner->b_Bank = true;
    }
    else
    {
        pst_Planner->b_Bank = false;
    }

    if (484 < s32_NearIdx && s32_NearIdx <  756)
    {
        pst_Planner->b_PlanningBank = true;
    }
    else if(1716 < s32_NearIdx && s32_NearIdx <  2016)
    {
        pst_Planner->b_PlanningBank = true;
    }
    else
    {
        pst_Planner->b_PlanningBank = false;
    }
}

void TEST_FlagProcessing(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;

    int32_t s32_NearIdx = pst_PlanningData->st_ReferenceLine2_global.s32_NearIdx;


    pst_Planner->u8_GoSignal = pst_CANData->u8_SIG_GO;
    pst_Planner->u8_StopSignal = pst_CANData->u8_SIG_STOP;
    pst_Planner->u8_SlowOnSignal = pst_CANData->u8_SIG_SLOW_ON;
    pst_Planner->u8_SlowOffSignal = pst_CANData->u8_SIG_SLOW_OFF;
    pst_Planner->u8_PitStopSignal = pst_CANData->u8_SIG_PIT_STOP;

    if(pst_Planner->u8_GoSignal)
    {
        pst_CANData->u8_EPS_En = 1;
        pst_CANData->u8_ACC_En = 1;
    }

    if (pst_Planner->u8_PitStopSignal)
    {
        pst_Planner->u8_PitStopReady = true;
    }
    if (pst_Planner->u8_PitStopReady && !(205 < s32_NearIdx && s32_NearIdx < 519))
    {
        pst_Planner->u8_PitStopMode = true;
    }
}


// PitStop에서 나갈 때 차량 들어오는 거 확인!
void TEST_RestartZoneCheck(PLANNING_DATA_t *pst_PlanningData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
    VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;

    uint64_t u64_CurrentTime = getMillisecond();
    int32_t s32_I;

    if (pst_Planner->s32_LabCount == 0)
    {
        int32_t s32_NearIdx = pst_PlanningData->st_ReferenceSELine2_global.s32_NearIdx;

        if (448 < s32_NearIdx && s32_NearIdx < 507)
        {
            for (s32_I = 0;s32_I < pst_PlanningData->s32_ObjectNum;s32_I++)
            {
                if (-50 < pst_ObjectData[s32_I].f32_DS && pst_ObjectData[s32_I].f32_DS < 0)
                {
                    if (u64_CurrentTime >= pst_Planner->u64_RestartStopEndTime)
                    {
                        pst_Planner->b_RestartStop = true;
                        pst_Planner->u64_RestartStopEndTime = u64_CurrentTime + 3000;
                    }
                    break;
                }
            }
        }
    }

    if (u64_CurrentTime >= pst_Planner->u64_RestartStopEndTime)
    {
        pst_Planner->b_RestartStop = false;
    }
}



// 자기 차량에 대한 정보 갱신 X, Y, Z, VX, VY, AX, AY, YAW, S, D, Current Lane 등
void TEST_EgoVehicleDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CAN_DATA_t *pst_CANData, HMC_LIDAR_DATA_t *pst_LidarData)
{
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    IMU_DATA_t *pst_IMU = &pst_DeadReckoningData->st_IMU;
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
    SPLINE_PATH_t *pst_SplinePath;
    SPLINE2D_PARAM_t *pst_Spline2DParam;
    TEST_PitStopProcessing(pst_PlanningData, &pst_Spline2DParam, &pst_SplinePath);

    pst_PlanningData->st_Planner.s32_LabCount = pst_DeadReckoningData->st_GPS.s32_LabCount;

    float32_t f32_MinDistance;
    float32_t f32_DX, f32_DY, f32_AngleError;
    float32_t f32_NearDist;
    int32_t s32_NearIdx = 0;
    int32_t s32_TargetNear;
    int32_t s32_GPSMode = 0;

    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");
    s32_GPSMode = st_Config["GPS_Mode"].as<int32_t>();
    if(s32_GPSMode == c_PARSING_MORAI)
    {
        lla2enu(pst_DeadReckoningData->st_GPS.f64_Latitude,
                    pst_DeadReckoningData->st_GPS.f64_Longitude,
                    0,
                    pst_EgoVehicleData->f32_X_global,
                    pst_EgoVehicleData->f32_Y_global,
                    pst_EgoVehicleData->f32_Z_global);

            pst_EgoVehicleData->f32_Yaw_rad_ENU = pst_CANData->f32_Heading_rad;
            pst_EgoVehicleData->f32_Yaw_rad_NED = axisRotate(pst_EgoVehicleData->f32_Yaw_rad_ENU);

            pst_EgoVehicleData->f32_Velocity_m_s_global = pst_CANData->f32_Speed_m_s;
            pst_EgoVehicleData->f32_VelocityX_m_s_global = pst_EgoVehicleData->f32_Velocity_m_s_global * cosf(pst_EgoVehicleData->f32_Yaw_rad_ENU);
            pst_EgoVehicleData->f32_VelocityY_m_s_global = pst_EgoVehicleData->f32_Velocity_m_s_global * sinf(pst_EgoVehicleData->f32_Yaw_rad_ENU);

            pst_EgoVehicleData->f32_Velocity_kph_global = ms2kph(pst_EgoVehicleData->f32_Velocity_m_s_global);
            pst_EgoVehicleData->f32_VelocityX_kph_global = ms2kph(pst_EgoVehicleData->f32_VelocityX_m_s_global);
            pst_EgoVehicleData->f32_VelocityY_kph_global = ms2kph(pst_EgoVehicleData->f32_VelocityY_m_s_global);

            pst_EgoVehicleData->f32_VelocityX_m_s_local = pst_CANData->f32_Speed_m_s;
            pst_EgoVehicleData->f32_VelocityY_m_s_local = 0.f;
            pst_EgoVehicleData->f32_Velocity_m_s_local = pst_CANData->f32_Speed_m_s;

            pst_EgoVehicleData->f32_Velocity_kph_local = ms2kph(pst_EgoVehicleData->f32_Velocity_m_s_local);
            pst_EgoVehicleData->f32_VelocityX_kph_local = ms2kph(pst_EgoVehicleData->f32_VelocityX_m_s_local);
            pst_EgoVehicleData->f32_VelocityY_kph_local = ms2kph(pst_EgoVehicleData->f32_VelocityY_m_s_local);

            pst_EgoVehicleData->f32_AccelX = pst_CANData->f32_AccelX;
            pst_EgoVehicleData->f32_AccelY = pst_CANData->f32_AccelY;
    }
    else if(s32_GPSMode == c_PARSING_AVANTE)
    {
        pst_EgoVehicleData->f32_X_global = pst_LidarData->f32_PredictX;
        pst_EgoVehicleData->f32_Y_global = pst_LidarData->f32_PredictY;
        pst_EgoVehicleData->f32_Z_global = pst_LidarData->f32_PredictZ;

        pst_EgoVehicleData->f32_Yaw_rad_NED = deg2rad(pst_LidarData->f32_PredictYaw_deg);
        pst_EgoVehicleData->f32_Yaw_rad_ENU = axisRotate(deg2rad(pst_LidarData->f32_PredictYaw_deg));

        pst_EgoVehicleData->f32_Velocity_m_s_global = pst_CANData->u8_VS / 3.6f;
        pst_EgoVehicleData->f32_VelocityX_m_s_global = pst_EgoVehicleData->f32_Velocity_m_s_global * cosf(pst_EgoVehicleData->f32_Yaw_rad_ENU);
        pst_EgoVehicleData->f32_VelocityY_m_s_global = pst_EgoVehicleData->f32_Velocity_m_s_global * sinf(pst_EgoVehicleData->f32_Yaw_rad_ENU);

        pst_EgoVehicleData->f32_Velocity_kph_global = ms2kph(pst_EgoVehicleData->f32_Velocity_m_s_global);
        pst_EgoVehicleData->f32_VelocityX_kph_global = ms2kph(pst_EgoVehicleData->f32_VelocityX_m_s_global);
        pst_EgoVehicleData->f32_VelocityY_kph_global = ms2kph(pst_EgoVehicleData->f32_VelocityY_m_s_global);

        pst_EgoVehicleData->f32_VelocityX_m_s_local = pst_DeadReckoningData->st_IMU.arf32_VelocityXYZ_m_s[0];
        pst_EgoVehicleData->f32_VelocityY_m_s_local = pst_DeadReckoningData->st_IMU.arf32_VelocityXYZ_m_s[1];
        pst_EgoVehicleData->f32_Velocity_m_s_local = getDistance2d(0.f, 0.f, pst_EgoVehicleData->f32_VelocityX_m_s_local, pst_EgoVehicleData->f32_VelocityY_m_s_local);

        pst_EgoVehicleData->f32_Velocity_kph_local = ms2kph(pst_EgoVehicleData->f32_Velocity_m_s_local);
        pst_EgoVehicleData->f32_VelocityX_kph_local = ms2kph(pst_EgoVehicleData->f32_VelocityX_m_s_local);
        pst_EgoVehicleData->f32_VelocityY_kph_local = ms2kph(pst_EgoVehicleData->f32_VelocityY_m_s_local);

        pst_EgoVehicleData->f32_AccelX = pst_CANData->f32_Long_ACCEL;
        pst_EgoVehicleData->f32_AccelY = pst_CANData->f32_LAT_ACCEL;
    }
    // Spline Calculation
    CalcNearestDistIdx(pst_EgoVehicleData->f32_X_global, pst_EgoVehicleData->f32_Y_global,
                       pst_SplinePath->arf32_X, pst_SplinePath->arf32_Y, pst_SplinePath->s32_Num,
                       f32_NearDist, s32_NearIdx);
    pst_SplinePath->s32_NearestIndex = s32_NearIdx;
    f32_MinDistance = f32_NearDist;
    f32_DX = pst_SplinePath->arf32_X[s32_NearIdx] - pst_EgoVehicleData->f32_X_global;
    f32_DY = pst_SplinePath->arf32_Y[s32_NearIdx] - pst_EgoVehicleData->f32_Y_global;
    f32_AngleError = pi2pi(pst_SplinePath->arf32_Yaw_rad_ENU[s32_NearIdx] - atan2f(f32_DY, f32_DX));
    pst_FrenetParam->f32_CurV = pst_EgoVehicleData->f32_Velocity_m_s_global;
    pst_FrenetParam->f32_CurA = getDistance2d(0.f, 0.f, pst_IMU->f32_AccelX, pst_IMU->f32_AccelY);
    pst_FrenetParam->f32_CurS0 = pst_SplinePath->arf32_S[s32_NearIdx];
    pst_FrenetParam->f32_CurS1 = 0.f;
    pst_FrenetParam->f32_CurS2 = 0.f;
    if (f32_AngleError < 0.f)
    {
        pst_FrenetParam->f32_CurD0 = -1.f * f32_MinDistance;
    }
    else
    {
        pst_FrenetParam->f32_CurD0 = f32_MinDistance;
    }
    pst_FrenetParam->f32_CurD1 = 0.f;
    pst_FrenetParam->f32_CurD2 = 0.f;
    pst_EgoVehicleData->f32_S = pst_FrenetParam->f32_CurS0;
    pst_EgoVehicleData->f32_D = pst_FrenetParam->f32_CurD0;

    // Path Gap
    s32_NearIdx = pst_PlanningData->st_ReferenceLine2_global.s32_NearIdx;
    s32_TargetNear = (int32_t)pst_FrenetParam->f32_TargetSpeed_m_s / 2;

    string MapName = "Kiapi";

    if (s_MapName == MapName)
    {
        pst_PlanningData->f32_PathGapList[0] = pst_PlanningData->st_LineGap1.arf32_D[(s32_NearIdx + s32_TargetNear) % pst_PlanningData->st_LineGap1.s32_Num];
        pst_PlanningData->f32_PathGapList[1] = 0.0f;
        pst_PlanningData->f32_PathGapList[2] = pst_PlanningData->st_LineGap3.arf32_D[(s32_NearIdx + s32_TargetNear) % pst_PlanningData->st_LineGap3.s32_Num];
    }
    else
    {
        pst_PlanningData->f32_PathGapList[0] = 3.5f;
        pst_PlanningData->f32_PathGapList[1] = 0.0f;
        pst_PlanningData->f32_PathGapList[2] = -3.5f;
    }

    float32_t f32_Line1Dist = pst_PlanningData->st_BoundaryGap1.arf32_D[s32_NearIdx];
    float32_t f32_Line2Dist = pst_PlanningData->st_BoundaryGap2.arf32_D[s32_NearIdx];
    float32_t f32_Line3Dist = pst_PlanningData->st_BoundaryGap3.arf32_D[s32_NearIdx];

    float32_t s32_Start = f32_Line2Dist / 2 + f32_Line1Dist / 2;
    float32_t S32_End = -f32_Line2Dist / 2 - f32_Line3Dist / 2;

    // 총 17개의 구간으로 나누기 위한 단일 구간의 폭 계산
    float32_t f32_Segment = (s32_Start - S32_End) / 17.0f;

    // 현재 위치에 따른 Lane 계산
    if (pst_FrenetParam->f32_CurD0 < S32_End)
    {
        pst_EgoVehicleData->s32_CurrentLane = 17;
    }
    else if (pst_FrenetParam->f32_CurD0 > s32_Start)
    {
        pst_EgoVehicleData->s32_CurrentLane = 1;
    }
    else
    {
        float32_t f32_Tmp = pst_FrenetParam->f32_CurD0 - S32_End;
        pst_EgoVehicleData->s32_CurrentLane = 17 - (int)(f32_Tmp / f32_Segment);
    }

    CalcVertex(pst_EgoVehicleData->f32_X_global,
               pst_EgoVehicleData->f32_Y_global,
               pst_EgoVehicleData->f32_MaxX,
               pst_EgoVehicleData->f32_MaxY,
               pst_EgoVehicleData->f32_MinX,
               pst_EgoVehicleData->f32_MinY,
               pst_EgoVehicleData->f32_Yaw_rad_ENU,
               pst_EgoVehicleData->f32_Vertex ,
               0);

}



// Clustered Vehicle X, Y, Z, VX, VY, YAW, S, D, Current Lane, Time To Collision, Safety Distance, 자기 차량과의 Relative Lane 갱신
void TEST_ClusteredDataProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, HMC_LIDAR_DATA_t *pst_LidarData, CAN_DATA_t *pst_CANData)
{
    VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;
    VEHICLE_DATA_t *pst_CarData = pst_PlanningData->arst_CarData;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    SPLINE_PATH_t *pst_SplinePath = &pst_PlanningData->st_SplinePath_global;
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;

    uint16_t s32_I;
    int32_t s32_J = 0;


    for (s32_I = 0;s32_I < pst_LidarData->s32_TrackingNum;s32_I++)
    {
        HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

        pst_ObjectData[s32_I].s32_VehicleIdx = s32_I;
        pst_ObjectData[s32_I].f32_X_global = pst_Tracking[s32_I].f32_GlobalX;
        pst_ObjectData[s32_I].f32_Y_global = pst_Tracking[s32_I].f32_GlobalY;

        pst_ObjectData[s32_I].f32_X_local = pst_Tracking[s32_I].f32_X;
        pst_ObjectData[s32_I].f32_Y_local = pst_Tracking[s32_I].f32_Y;

        pst_ObjectData[s32_I].f32_VelocityX_m_s_local = pst_Tracking[s32_I].f32_VelocityX_m_s;
        pst_ObjectData[s32_I].f32_VelocityY_m_s_local = pst_Tracking[s32_I].f32_VelocityY_m_s;
        pst_ObjectData[s32_I].f32_Velocity_m_s_local = getDistance2d(0.f ,0.f ,
            pst_Tracking[s32_I].f32_VelocityX_m_s, pst_Tracking[s32_I].f32_VelocityY_m_s);
        if(pst_ObjectData[s32_I].f32_VelocityX_m_s_local < 0.0f)
        {
            pst_ObjectData[s32_I].f32_Velocity_m_s_local = -pst_ObjectData[s32_I].f32_Velocity_m_s_local;
        }

        pst_ObjectData[s32_I].f32_VelocityX_kph_local = ms2kph(pst_ObjectData[s32_I].f32_VelocityX_m_s_local);
        pst_ObjectData[s32_I].f32_VelocityY_kph_local = ms2kph(pst_ObjectData[s32_I].f32_VelocityY_m_s_local);
        pst_ObjectData[s32_I].f32_Velocity_kph_local = ms2kph(pst_ObjectData[s32_I].f32_Velocity_m_s_local);

        pst_ObjectData[s32_I].f32_VelocityX_m_s_global = pst_Tracking->f32_AbsoluteVelocityX_m_s;
        pst_ObjectData[s32_I].f32_VelocityY_m_s_global = pst_Tracking->f32_AbsoluteVelocityY_m_s;
        pst_ObjectData[s32_I].f32_Velocity_m_s_global = getDistance2d(0.f, 0.f,
                                                                   pst_CarData[s32_J].f32_VelocityX_m_s_global, pst_CarData[s32_J].f32_VelocityY_m_s_global);

        pst_ObjectData[s32_I].f32_VelocityX_kph_global = ms2kph(pst_ObjectData[s32_I].f32_VelocityX_m_s_global);
        pst_ObjectData[s32_I].f32_VelocityY_kph_global = ms2kph(pst_ObjectData[s32_I].f32_VelocityY_m_s_global);
        pst_ObjectData[s32_I].f32_Velocity_kph_global = ms2kph(pst_ObjectData[s32_I].f32_Velocity_m_s_global);

        pst_ObjectData[s32_I].f32_Yaw_rad_ENU = pst_Tracking[s32_I].f32_Yaw_rad_ENU;
        pst_ObjectData[s32_I].f32_Yaw_rad_NED = axisRotate(pst_Tracking[s32_I].f32_Yaw_rad_ENU);

        pst_ObjectData[s32_I].f32_MaxX = pst_Tracking[s32_I].f32_X + (pst_Tracking[s32_I].f32_Length / 2.f);
        pst_ObjectData[s32_I].f32_MinX = pst_Tracking[s32_I].f32_X - (pst_Tracking[s32_I].f32_Length / 2.f);
        pst_ObjectData[s32_I].f32_MaxY = pst_Tracking[s32_I].f32_Y + (pst_Tracking[s32_I].f32_Width / 2.f);
        pst_ObjectData[s32_I].f32_MinY = pst_Tracking[s32_I].f32_Y - (pst_Tracking[s32_I].f32_Width / 2.f);

        // Spline Line 2 Near Index and Dist Search for Object
        int32_t s32_VehicleNearIdx;
        float32_t f32_VehicleNearDist;
        CalcNearestDistIdx(pst_ObjectData[s32_I].f32_X_global,
                           pst_ObjectData[s32_I].f32_Y_global,
                           pst_SplinePath->arf32_X,
                           pst_SplinePath->arf32_Y,
                           pst_SplinePath->s32_Num,
                           f32_VehicleNearDist,
                           s32_VehicleNearIdx);

        CalcVertex(pst_ObjectData[s32_I].f32_X_global,
                   pst_ObjectData[s32_I].f32_Y_global,
                   (pst_ObjectData[s32_I].f32_MaxX - pst_ObjectData[s32_I].f32_X_local),
                   (pst_ObjectData[s32_I].f32_MaxY - pst_ObjectData[s32_I].f32_Y_local),
                   (pst_ObjectData[s32_I].f32_MinX - pst_ObjectData[s32_I].f32_X_local),
                   (pst_ObjectData[s32_I].f32_MinY - pst_ObjectData[s32_I].f32_Y_local),
                   pst_SplinePath->arf32_Yaw_rad_ENU[s32_VehicleNearIdx],
                   pst_ObjectData[s32_I].f32_Vertex);


        // Is Object Left or Right about Spline Path
        float32_t f32_DX = pst_SplinePath->arf32_X[s32_VehicleNearIdx] - pst_ObjectData[s32_I].f32_X_global;
        float32_t f32_DY = pst_SplinePath->arf32_Y[s32_VehicleNearIdx] - pst_ObjectData[s32_I].f32_Y_global;
        float32_t f32_AngleError = pi2pi(pst_SplinePath->arf32_Yaw_rad_ENU[s32_VehicleNearIdx] - atan2f(f32_DY, f32_DX));
        if (f32_AngleError < 0.f)
        {
            pst_ObjectData[s32_I].f32_D = -1.f * f32_VehicleNearDist;
        }
        else
        {
            pst_ObjectData[s32_I].f32_D = f32_VehicleNearDist;
        }
        pst_ObjectData[s32_I].f32_S = pst_SplinePath->arf32_S[s32_VehicleNearIdx];

        float32_t f32_AllDistance = pst_SplinePath->f32_LastS;
        float32_t f32_Dist = pst_ObjectData[s32_I].f32_S - pst_EgoVehicleData->f32_S;
        if(f32_Dist > f32_AllDistance/2)
        {
            f32_Dist -= f32_AllDistance;
        }
        else if(f32_Dist < -f32_AllDistance/2)
        {
            f32_Dist += f32_AllDistance;
        }
        pst_ObjectData[s32_I].f32_DS = f32_Dist;  // Object S - Ego S
        pst_ObjectData[s32_I].f32_DD = pst_ObjectData[s32_I].f32_D - pst_EgoVehicleData->f32_D;

        if ( -100.f < pst_ObjectData[s32_I].f32_DS && pst_ObjectData[s32_I].f32_DS < 100.f)
        {
            if (pst_ObjectData[s32_I].f32_D < -1.8f)
            {
                pst_ObjectData[s32_I].s32_CurrentLane = 3;
            }
            else if (-1.8f <= pst_ObjectData[s32_I].f32_D && pst_ObjectData[s32_I].f32_D <= 1.8f)
            {
                pst_ObjectData[s32_I].s32_CurrentLane = 2;
            }
            else if (1.8f < pst_ObjectData[s32_I].f32_D)
            {
                pst_ObjectData[s32_I].s32_CurrentLane = 1;
            }
        }

        if(pst_Tracking[s32_I].u8_StateFlag == c_HMC_OBJECT_DYNAMIC)
        {
            memcpy(&pst_CarData[s32_J], &pst_ObjectData[s32_I], sizeof(VEHICLE_DATA_t));
            s32_J++;
        }
    }
    pst_PlanningData->s32_ObjectNum = s32_I;
    pst_PlanningData->s32_CarNum = s32_J;

}

void TEST_FrenetPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues)
{
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global;
    SPLINE_PATH_t *pst_SplinePath;
    SPLINE2D_PARAM_t *pst_Spline2DParam;
    TEST_PitStopProcessing(pst_PlanningData, &pst_Spline2DParam, &pst_SplinePath);

    float32_t f32_S;
    QUINTIC_t st_Quantic;
    QUARTIC_t st_Quartic;

    int32_t s32_PathNum = pst_FrenetParam->s32_PathNum;
    int32_t s32_Idx;
    int32_t s32_I = 0;
    int32_t s32_Num = 0;
    int32_t s32_FrenetPathNum = 0;

    pst_FrenetParam->f32_TargetSpeed_m_s = 32.f;
    float32_t f32_T = 0.f;
    float32_t f32_V = pst_FrenetParam->f32_TargetSpeed_m_s - pst_FrenetParam->f32_SpeedSamplingTime * pst_FrenetParam->s32_SampleNum;
    float32_t f32_MaxVelocity = pst_FrenetParam->f32_TargetSpeed_m_s + pst_FrenetParam->f32_SpeedSamplingTime * pst_FrenetParam->s32_SampleNum;
    float32_t f32_Di;
    float32_t f32_Ti;

    float32_t f32_Jp;
    float32_t f32_Js;
    float32_t f32_ds;

    for (s32_Idx = 0; s32_Idx < s32_PathNum; s32_Idx++)
    {
        f32_Di = f32_PathGapList[s32_Idx];

        for (f32_Ti = pst_FrenetParam->f32_MinT; f32_Ti <= pst_FrenetParam->f32_MaxT; f32_Ti += 1.f)
        {
            st_Quantic.Init(pst_FrenetParam->f32_CurD0,
                pst_FrenetParam->f32_CurD1,
                pst_FrenetParam->f32_CurD2, f32_Di, 0.0, 0.0, f32_Ti);

            FRENET_PATH_t st_TempFrenetPath;
            s32_Num = 0;
            f32_T = 0.f;

            while (f32_T < f32_Ti)
            {
                st_TempFrenetPath.arf32_T[s32_Num] = f32_T;
                st_TempFrenetPath.arf32_D0[s32_Num] = st_Quantic.CalcPoint(f32_T);
                st_TempFrenetPath.arf32_D1[s32_Num] = st_Quantic.CalcFirstDerivative(f32_T);
                st_TempFrenetPath.arf32_D2[s32_Num] = st_Quantic.CalcSecondDerivative(f32_T);
                st_TempFrenetPath.arf32_D3[s32_Num] = st_Quantic.CalcThirdDerivative(f32_T);

                f32_T += pst_FrenetParam->f32_DT;
                s32_Num++;
            }

            f32_V = pst_FrenetParam->f32_TargetSpeed_m_s - pst_FrenetParam->f32_SpeedSamplingTime * pst_FrenetParam->s32_SampleNum;
            float32_t f32_LastS = pst_SplinePath->arf32_S[pst_SplinePath->s32_Num - 1];

            while (f32_V <= f32_MaxVelocity)
            {
                st_Quartic.Init(pst_FrenetParam->f32_CurS0,
                    pst_FrenetParam->f32_CurS1,
                    0.0f, f32_V, 10.0f, f32_Ti);

                memcpy(&pst_FrenetPath[s32_FrenetPathNum], &st_TempFrenetPath, sizeof(FRENET_PATH_t));

                for (s32_I = 0; s32_I < s32_Num; s32_I++)
                {
                    f32_T = st_TempFrenetPath.arf32_T[s32_I];
                    pst_FrenetPath[s32_FrenetPathNum].arf32_S0[s32_I] = st_Quartic.CalcPoint(f32_T);
                    pst_FrenetPath[s32_FrenetPathNum].arf32_S1[s32_I] = st_Quartic.CalcFirstDerivative(f32_T);
                    pst_FrenetPath[s32_FrenetPathNum].arf32_S2[s32_I] = st_Quartic.CalcSecondDerivative(f32_T);
                    pst_FrenetPath[s32_FrenetPathNum].arf32_S3[s32_I] = st_Quartic.CalcThirdDerivative(f32_T);
                }

                f32_Jp = powf(pst_FrenetPath[s32_FrenetPathNum].arf32_D3[s32_Num - 1], 2.f);
                f32_Js = powf(pst_FrenetPath[s32_FrenetPathNum].arf32_S3[s32_Num - 1], 2.f);
                f32_ds = powf(pst_FrenetParam->f32_TargetSpeed_m_s - pst_FrenetPath[s32_FrenetPathNum].arf32_S1[s32_Num - 1], 2.f);

                pst_FrenetPath[s32_FrenetPathNum].s32_CurrentLane = s32_CurrentLaneValues[s32_Idx];
                pst_FrenetPath[s32_FrenetPathNum].f32_CostD = pst_FrenetParam->f32_KJ * f32_Jp +
                    pst_FrenetParam->f32_KT * f32_Ti +
                    pst_FrenetParam->f32_KD * powf(pst_FrenetPath[s32_FrenetPathNum].arf32_D0[s32_Num - 1]
                  - f32_PathGapList[pst_PlanningData->st_Planner.s32_TargetLane - 1], 2.f);

                pst_FrenetPath[s32_FrenetPathNum].f32_CostV = pst_FrenetParam->f32_KJ * f32_Js +
                    pst_FrenetParam->f32_KT * f32_Ti +
                    pst_FrenetParam->f32_KD * f32_ds;

                pst_FrenetPath[s32_FrenetPathNum].f32_CostF = pst_FrenetParam->f32_KLat * pst_FrenetPath[s32_FrenetPathNum].f32_CostD +
                    pst_FrenetParam->f32_KLon * pst_FrenetPath[s32_FrenetPathNum].f32_CostV;

                pst_FrenetPath[s32_FrenetPathNum].s32_Num = s32_Num;

                f32_V += pst_FrenetParam->f32_SpeedSamplingTime;
                s32_FrenetPathNum++;
            }
        }
    }
    pst_PlanningData->s32_FrenetPathNum = s32_FrenetPathNum;
}

void TEST_CollisionCheckPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues)
{
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    FRENET_PATH_t *pst_CollisionCheckPath = pst_PlanningData->arst_CollisonCheckPath_global;
    SPLINE_PATH_t *pst_SplinePath;
    SPLINE2D_PARAM_t *pst_Spline2DParam;
    TEST_PitStopProcessing(pst_PlanningData, &pst_Spline2DParam, &pst_SplinePath);

    QUINTIC_t st_Quantic;
    QUARTIC_t st_Quartic;

    int32_t s32_PathNum = pst_FrenetParam->s32_PathNum;
    int32_t s32_Idx;
    int32_t s32_Num = 0;
    int32_t s32_FrenetPathNum = 0;
    float32_t f32_Di;
    float32_t f32_Ti = 5.0f;
    float32_t f32_T = 0.f;

    for (s32_Idx = 0; s32_Idx < s32_PathNum; s32_Idx++)
    {
        f32_Di = f32_PathGapList[s32_Idx];

        st_Quantic.Init(f32_Di,
            pst_FrenetParam->f32_CurD1,
            pst_FrenetParam->f32_CurD2, f32_Di, 0.0, 0.0, f32_Ti);

        FRENET_PATH_t st_TempCollisionPath;
        s32_Num = 0;
        f32_T = 0.f;

        while (f32_T < f32_Ti)
        {
            st_TempCollisionPath.arf32_T[s32_Num] = f32_T;
            st_TempCollisionPath.arf32_D0[s32_Num] = st_Quantic.CalcPoint(f32_T);
            f32_T += pst_FrenetParam->f32_DT;
            s32_Num++;
        }

        st_Quartic.Init(pst_FrenetParam->f32_CurS0 - 7.f,
            pst_FrenetParam->f32_CurS1,
            0.0f, 2.7f, 0.0f, f32_Ti);

        memcpy(&pst_CollisionCheckPath[s32_FrenetPathNum], &st_TempCollisionPath, sizeof(FRENET_PATH_t));

        for (int32_t s32_I = 0; s32_I < s32_Num; s32_I++)
        {
            f32_T = st_TempCollisionPath.arf32_T[s32_I];
            pst_CollisionCheckPath[s32_FrenetPathNum].arf32_S0[s32_I] = st_Quartic.CalcPoint(f32_T);
        }

        pst_CollisionCheckPath[s32_FrenetPathNum].s32_Num = s32_Num;
        pst_CollisionCheckPath[s32_FrenetPathNum].s32_CurrentLane = s32_CurrentLaneValues[s32_Idx];

        s32_FrenetPathNum++;
    }
}

void TEST_ControlPath(PLANNING_DATA_t *pst_PlanningData, float32_t *f32_PathGapList, int32_t *s32_CurrentLaneValues)
{
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    FRENET_PATH_t *pst_ControlPath = pst_PlanningData->arst_ControlPath_global;
    SPLINE_PATH_t *pst_SplinePath;
    SPLINE2D_PARAM_t *pst_Spline2DParam;
    TEST_PitStopProcessing(pst_PlanningData, &pst_Spline2DParam, &pst_SplinePath);

    float32_t f32_S;
    QUINTIC_t st_Quantic;
    QUARTIC_t st_Quartic;

    int32_t s32_PathNum = pst_FrenetParam->s32_PathNum;
    int32_t s32_Idx;
    int32_t s32_I = 0;
    int32_t s32_Num = 0;
    int32_t s32_FrenetPathNum = 0;

    pst_FrenetParam->f32_TargetSpeed_m_s = pst_PlanningData->st_EgoVehicleData.f32_Velocity_m_s_global * 1.8f + 5.f;

    float32_t f32_T = 0.f;
    float32_t f32_V = pst_FrenetParam->f32_TargetSpeed_m_s - pst_FrenetParam->f32_SpeedSamplingTime * pst_FrenetParam->s32_SampleNum;
    float32_t f32_MaxVelocity = pst_FrenetParam->f32_TargetSpeed_m_s + pst_FrenetParam->f32_SpeedSamplingTime * pst_FrenetParam->s32_SampleNum;
    float32_t f32_Di;
    float32_t f32_Ti;

    float32_t f32_Jp;
    float32_t f32_Js;
    float32_t f32_ds;

    for (s32_Idx = 0; s32_Idx < s32_PathNum; s32_Idx++)
    {
        f32_Di = f32_PathGapList[s32_Idx];

        for (f32_Ti = pst_FrenetParam->f32_MinT; f32_Ti <= pst_FrenetParam->f32_MaxT; f32_Ti += 1.f)
        {
            st_Quantic.Init(pst_FrenetParam->f32_CurD0,
                pst_FrenetParam->f32_CurD1,
                pst_FrenetParam->f32_CurD2, f32_Di, 0.0, 0.0, f32_Ti);

            FRENET_PATH_t st_TempFrenetPath;
            s32_Num = 0;
            f32_T = 0.f;

            while (f32_T < f32_Ti)
            {
                st_TempFrenetPath.arf32_T[s32_Num] = f32_T;
                st_TempFrenetPath.arf32_D0[s32_Num] = st_Quantic.CalcPoint(f32_T);
                st_TempFrenetPath.arf32_D1[s32_Num] = st_Quantic.CalcFirstDerivative(f32_T);
                st_TempFrenetPath.arf32_D2[s32_Num] = st_Quantic.CalcSecondDerivative(f32_T);
                st_TempFrenetPath.arf32_D3[s32_Num] = st_Quantic.CalcThirdDerivative(f32_T);

                f32_T += pst_FrenetParam->f32_DT;
                s32_Num++;
            }

            f32_V = pst_FrenetParam->f32_TargetSpeed_m_s - pst_FrenetParam->f32_SpeedSamplingTime * pst_FrenetParam->s32_SampleNum;
            float32_t f32_LastS = pst_SplinePath->arf32_S[pst_SplinePath->s32_Num - 1];

            while (f32_V <= f32_MaxVelocity)
            {
                st_Quartic.Init(pst_FrenetParam->f32_CurS0,
                    pst_FrenetParam->f32_CurS1,
                    0.0f, f32_V, 0.0f, f32_Ti);

                memcpy(&pst_ControlPath[s32_FrenetPathNum], &st_TempFrenetPath, sizeof(FRENET_PATH_t));

                for (s32_I = 0; s32_I < s32_Num; s32_I++)
                {
                    f32_T = st_TempFrenetPath.arf32_T[s32_I];
                    pst_ControlPath[s32_FrenetPathNum].arf32_S0[s32_I] = st_Quartic.CalcPoint(f32_T);
                    pst_ControlPath[s32_FrenetPathNum].arf32_S1[s32_I] = st_Quartic.CalcFirstDerivative(f32_T);
                    pst_ControlPath[s32_FrenetPathNum].arf32_S2[s32_I] = st_Quartic.CalcSecondDerivative(f32_T);
                    pst_ControlPath[s32_FrenetPathNum].arf32_S3[s32_I] = st_Quartic.CalcThirdDerivative(f32_T);
                }

                f32_Jp = powf(pst_ControlPath[s32_FrenetPathNum].arf32_D3[s32_Num - 1], 2.f);
                f32_Js = powf(pst_ControlPath[s32_FrenetPathNum].arf32_S3[s32_Num - 1], 2.f);
                f32_ds = powf(pst_FrenetParam->f32_TargetSpeed_m_s - pst_ControlPath[s32_FrenetPathNum].arf32_S1[s32_Num - 1], 2.f);

                pst_ControlPath[s32_FrenetPathNum].s32_CurrentLane = s32_CurrentLaneValues[s32_Idx];
                pst_ControlPath[s32_FrenetPathNum].f32_CostD = pst_FrenetParam->f32_KJ * f32_Jp +
                    pst_FrenetParam->f32_KT * f32_Ti +
                    pst_FrenetParam->f32_KD * powf(pst_ControlPath[s32_FrenetPathNum].arf32_D0[s32_Num - 1]
                  - f32_PathGapList[pst_PlanningData->st_Planner.s32_TargetLane - 1], 2.f);

                pst_ControlPath[s32_FrenetPathNum].f32_CostV = pst_FrenetParam->f32_KJ * f32_Js +
                    pst_FrenetParam->f32_KT * f32_Ti +
                    pst_FrenetParam->f32_KD * f32_ds;

                pst_ControlPath[s32_FrenetPathNum].f32_CostF = pst_FrenetParam->f32_KLat * pst_ControlPath[s32_FrenetPathNum].f32_CostD +
                    pst_FrenetParam->f32_KLon * pst_ControlPath[s32_FrenetPathNum].f32_CostV;

                pst_ControlPath[s32_FrenetPathNum].s32_Num = s32_Num;

                f32_V += pst_FrenetParam->f32_SpeedSamplingTime;
                s32_FrenetPathNum++;
            }
        }
    }
    pst_PlanningData->s32_FrenetPathNum = s32_FrenetPathNum;
}

void TEST_FrenetPathGeneration(PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_PathNum = pst_PlanningData->st_FrenetParam.s32_PathNum;

    int32_t s32_CurrentLaneValues[s32_PathNum] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
    float32_t f32_PathGapList[s32_PathNum] =
    {
        pst_PlanningData->f32_PathGapList[0] * 1.000f,
        pst_PlanningData->f32_PathGapList[0] * 0.875f,
        pst_PlanningData->f32_PathGapList[0] * 0.750f,
        pst_PlanningData->f32_PathGapList[0] * 0.625f,
        pst_PlanningData->f32_PathGapList[0] * 0.500f,
        pst_PlanningData->f32_PathGapList[0] * 0.375f,
        pst_PlanningData->f32_PathGapList[0] * 0.250f,
        pst_PlanningData->f32_PathGapList[0] * 0.125f,
        pst_PlanningData->f32_PathGapList[1] * 0.000f,
        pst_PlanningData->f32_PathGapList[2] * 0.125f,
        pst_PlanningData->f32_PathGapList[2] * 0.250f,
        pst_PlanningData->f32_PathGapList[2] * 0.375f,
        pst_PlanningData->f32_PathGapList[2] * 0.500f,
        pst_PlanningData->f32_PathGapList[2] * 0.625f,
        pst_PlanningData->f32_PathGapList[2] * 0.750f,
        pst_PlanningData->f32_PathGapList[2] * 0.875f,
        pst_PlanningData->f32_PathGapList[2] * 1.000f
    };

    TEST_FrenetPath(pst_PlanningData, f32_PathGapList, s32_CurrentLaneValues);
    TEST_CollisionCheckPath(pst_PlanningData, f32_PathGapList, s32_CurrentLaneValues);
    TEST_ControlPath(pst_PlanningData, f32_PathGapList, s32_CurrentLaneValues);
}


// Frenet Path {s, d} converts Cartesian {x, y}

void TEST_ProcessFrenetPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_FrenetPath, int32_t s32_FrenetPathNum)
{
    int32_t s32_I = 0;
    int32_t s32_J = 0;
    float32_t f32_X, f32_Y, f32_Yaw_rad, f32_D0, f32_DX, f32_DY, f32_DS;

    for (s32_I = 0; s32_I < s32_FrenetPathNum; s32_I++)
    {
        for (s32_J = 0; s32_J < pst_FrenetPath[s32_I].s32_Num; s32_J++)
        {
            float32_t f32_S = pst_FrenetPath[s32_I].arf32_S0[s32_J];
            if (f32_S > pst_SplinePath->f32_LastS)
            {
                f32_S = f32_S - pst_SplinePath->f32_LastS;
            }
            else if (pst_SplinePath->f32_LastBeforeS <= f32_S && f32_S <= pst_SplinePath->f32_LastS)
            {
                f32_S = pst_SplinePath->f32_LastBeforeS;
            }

            PointArray2D Point = pst_Spline2DParam->calc_position(f32_S);
            f32_X = Point[0];
            f32_Y = Point[1];

            f32_Yaw_rad = pst_Spline2DParam->calc_yaw(f32_S);
            f32_D0 = pst_FrenetPath[s32_I].arf32_D0[s32_J];
            pst_FrenetPath[s32_I].arf32_X[s32_J] = f32_X + f32_D0 * cosf(f32_Yaw_rad + M_PI / 2.0);
            pst_FrenetPath[s32_I].arf32_Y[s32_J] = f32_Y + f32_D0 * sinf(f32_Yaw_rad + M_PI / 2.0);

            if (s32_J > 0)
            {
                f32_DX = pst_FrenetPath[s32_I].arf32_X[s32_J] - pst_FrenetPath[s32_I].arf32_X[s32_J - 1];
                f32_DY = pst_FrenetPath[s32_I].arf32_Y[s32_J] - pst_FrenetPath[s32_I].arf32_Y[s32_J - 1];
                f32_Yaw_rad = atan2f(f32_DY, f32_DX);
                f32_DS = getDistance2d(0.f, 0.f, f32_DX, f32_DY);
                pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J - 1] = f32_Yaw_rad;
                pst_FrenetPath[s32_I].arf32_Yaw_rad_NED[s32_J - 1] = axisRotate(pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J - 1]);
                pst_FrenetPath[s32_I].arf32_DS[s32_J - 1] = f32_DS;
            }
        }
        pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[pst_FrenetPath[s32_I].s32_Num - 1] = f32_Yaw_rad;
        pst_FrenetPath[s32_I].arf32_Yaw_rad_NED[pst_FrenetPath[s32_I].s32_Num - 1] = axisRotate(pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[pst_FrenetPath[s32_I].s32_Num - 1]);
        pst_FrenetPath[s32_I].arf32_DS[pst_FrenetPath[s32_I].s32_Num - 1] = f32_DS;

        for (s32_J = 0; s32_J < pst_FrenetPath[s32_I].s32_Num - 1; s32_J++)
        {
            pst_FrenetPath[s32_I].arf32_C[s32_J] = fabsf((pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J + 1] -
                pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J])) /
            pst_FrenetPath[s32_I].arf32_DS[s32_J];
        }
    }
}

void TEST_ProcessCollisionCheckPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_CollisionCheckPath, int32_t s32_PathNum)
{
    int32_t s32_I = 0;
    int32_t s32_J = 0;
    float32_t f32_X, f32_Y, f32_Yaw_rad, f32_D0, f32_DX, f32_DY, f32_DS;

    for (s32_I = 0; s32_I < s32_PathNum; s32_I++)
    {
        for (s32_J = 0; s32_J < pst_CollisionCheckPath[s32_I].s32_Num; s32_J++)
        {
            float32_t f32_S = pst_CollisionCheckPath[s32_I].arf32_S0[s32_J];
            if (f32_S > pst_SplinePath->f32_LastS)
            {
                f32_S = f32_S - pst_SplinePath->f32_LastS;
            }
            else if (pst_SplinePath->f32_LastBeforeS <= f32_S && f32_S <= pst_SplinePath->f32_LastS)
            {
                f32_S = pst_SplinePath->f32_LastBeforeS;
            }

            PointArray2D Point = pst_Spline2DParam->calc_position(f32_S);
            f32_X = Point[0];
            f32_Y = Point[1];

            f32_Yaw_rad = pst_Spline2DParam->calc_yaw(f32_S);
            f32_D0 = pst_CollisionCheckPath[s32_I].arf32_D0[s32_J];
            pst_CollisionCheckPath[s32_I].arf32_X[s32_J] = f32_X + f32_D0 * cosf(f32_Yaw_rad + M_PI / 2.0);
            pst_CollisionCheckPath[s32_I].arf32_Y[s32_J] = f32_Y + f32_D0 * sinf(f32_Yaw_rad + M_PI / 2.0);
            if (s32_J > 0)
            {
                f32_DX = pst_CollisionCheckPath[s32_I].arf32_X[s32_J] - pst_CollisionCheckPath[s32_I].arf32_X[s32_J - 1];
                f32_DY = pst_CollisionCheckPath[s32_I].arf32_Y[s32_J] - pst_CollisionCheckPath[s32_I].arf32_Y[s32_J - 1];
                f32_Yaw_rad = atan2f(f32_DY, f32_DX);
                f32_DS = getDistance2d(0.f, 0.f, f32_DX, f32_DY);
                pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_ENU[s32_J - 1] = f32_Yaw_rad;
                pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_NED[s32_J - 1] = axisRotate(pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_ENU[s32_J - 1]);
                pst_CollisionCheckPath[s32_I].arf32_DS[s32_J - 1] = f32_DS;
            }
        }
        pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_ENU[pst_CollisionCheckPath[s32_I].s32_Num - 1] = f32_Yaw_rad;
        pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_NED[pst_CollisionCheckPath[s32_I].s32_Num - 1] = axisRotate(pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_ENU[pst_CollisionCheckPath[s32_I].s32_Num - 1]);
        pst_CollisionCheckPath[s32_I].arf32_DS[pst_CollisionCheckPath[s32_I].s32_Num - 1] = f32_DS;

        for (s32_J = 0; s32_J < pst_CollisionCheckPath[s32_I].s32_Num - 1; s32_J++)
        {
            pst_CollisionCheckPath[s32_I].arf32_C[s32_J] = fabsf((pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_ENU[s32_J + 1] -
                pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_ENU[s32_J])) /
            pst_CollisionCheckPath[s32_I].arf32_DS[s32_J];
        }
    }
}

void TEST_ProcessControlPath(SPLINE_PATH_t *pst_SplinePath, SPLINE2D_PARAM_t *pst_Spline2DParam, FRENET_PATH_t *pst_FrenetPath, int32_t s32_FrenetPathNum)
{
    int32_t s32_I = 0;
    int32_t s32_J = 0;
    float32_t f32_X, f32_Y, f32_Yaw_rad, f32_D0, f32_DX, f32_DY, f32_DS;

    for (s32_I = 0; s32_I < s32_FrenetPathNum; s32_I++)
    {
        for (s32_J = 0; s32_J < pst_FrenetPath[s32_I].s32_Num; s32_J++)
        {
            float32_t f32_S = pst_FrenetPath[s32_I].arf32_S0[s32_J];
            if (f32_S > pst_SplinePath->f32_LastS)
            {
                f32_S = f32_S - pst_SplinePath->f32_LastS;
            }
            else if (pst_SplinePath->f32_LastBeforeS <= f32_S && f32_S <= pst_SplinePath->f32_LastS)
            {
                f32_S = pst_SplinePath->f32_LastBeforeS;
            }

            PointArray2D Point = pst_Spline2DParam->calc_position(f32_S);
            f32_X = Point[0];
            f32_Y = Point[1];

            f32_Yaw_rad = pst_Spline2DParam->calc_yaw(f32_S);
            f32_D0 = pst_FrenetPath[s32_I].arf32_D0[s32_J];
            pst_FrenetPath[s32_I].arf32_X[s32_J] = f32_X + f32_D0 * cosf(f32_Yaw_rad + M_PI / 2.0);
            pst_FrenetPath[s32_I].arf32_Y[s32_J] = f32_Y + f32_D0 * sinf(f32_Yaw_rad + M_PI / 2.0);

            if (s32_J > 0)
            {
                f32_DX = pst_FrenetPath[s32_I].arf32_X[s32_J] - pst_FrenetPath[s32_I].arf32_X[s32_J - 1];
                f32_DY = pst_FrenetPath[s32_I].arf32_Y[s32_J] - pst_FrenetPath[s32_I].arf32_Y[s32_J - 1];
                f32_Yaw_rad = atan2f(f32_DY, f32_DX);
                f32_DS = getDistance2d(0.f, 0.f, f32_DX, f32_DY);
                pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J - 1] = f32_Yaw_rad;
                pst_FrenetPath[s32_I].arf32_Yaw_rad_NED[s32_J - 1] = axisRotate(pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J - 1]);
                pst_FrenetPath[s32_I].arf32_DS[s32_J - 1] = f32_DS;
            }
        }
        pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[pst_FrenetPath[s32_I].s32_Num - 1] = f32_Yaw_rad;
        pst_FrenetPath[s32_I].arf32_Yaw_rad_NED[pst_FrenetPath[s32_I].s32_Num - 1] = axisRotate(pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[pst_FrenetPath[s32_I].s32_Num - 1]);
        pst_FrenetPath[s32_I].arf32_DS[pst_FrenetPath[s32_I].s32_Num - 1] = f32_DS;

        for (s32_J = 0; s32_J < pst_FrenetPath[s32_I].s32_Num - 1; s32_J++)
        {
            pst_FrenetPath[s32_I].arf32_C[s32_J] = fabsf((pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J + 1] -
                pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J])) /
            pst_FrenetPath[s32_I].arf32_DS[s32_J];
        }
    }
}

void TEST_CalcFrenetToGlobalPath(PLANNING_DATA_t *pst_PlanningData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global;
    FRENET_PATH_t *pst_CollisionCheckPath = pst_PlanningData->arst_CollisonCheckPath_global;
    FRENET_PATH_t *pst_ControlPath = pst_PlanningData->arst_ControlPath_global;
    SPLINE_PATH_t *pst_SplinePath;
    SPLINE2D_PARAM_t *pst_Spline2DParam;

    TEST_PitStopProcessing(pst_PlanningData, &pst_Spline2DParam, &pst_SplinePath);
    TEST_ProcessFrenetPath(pst_SplinePath, pst_Spline2DParam, pst_FrenetPath, pst_PlanningData->s32_FrenetPathNum);
    TEST_ProcessCollisionCheckPath(pst_SplinePath, pst_Spline2DParam, pst_CollisionCheckPath, pst_FrenetParam->s32_PathNum);
    TEST_ProcessControlPath(pst_SplinePath, pst_Spline2DParam, pst_ControlPath, pst_FrenetParam->s32_PathNum);
}

void TEST_FindAvailableGridLane(PLANNING_DATA_t *pst_PlanningData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global;
    FRENET_PATH_t *pst_CollisionCheckPath = pst_PlanningData->arst_CollisonCheckPath_global;
    VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    int32_t s32_PathNum = pst_FrenetParam->s32_PathNum;
    int32_t s32_AvailPathCnt = 0;
    int32_t s32_I, s32_J, s32_K;

    float32_t f32_Ratio = pst_FrenetParam->f32_Ratio;

    for (s32_I = 0; s32_I < s32_PathNum; s32_I++)
    {
        pst_Planner->ars32_AvailGridPath[s32_I] = 0;
    }
    for(s32_I = 0; s32_I < s32_PathNum; s32_I++)
    {
        int32_t s32_M = 0;
        bool b_Collision = false;

        for(s32_J = 0; s32_J < pst_CollisionCheckPath[s32_I].s32_Num; s32_J++)
        {
            float32_t f32_Vertex[4][2];
            CalcVertex(pst_CollisionCheckPath[s32_I].arf32_X[s32_J],
                       pst_CollisionCheckPath[s32_I].arf32_Y[s32_J],
                       pst_EgoVehicleData->f32_MaxX * 1.f,
                       pst_EgoVehicleData->f32_MaxY * 1.f * f32_Ratio,
                       pst_EgoVehicleData->f32_MinX * 1.f,
                       pst_EgoVehicleData->f32_MinY * 1.f * f32_Ratio,
                       pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_ENU[s32_J], f32_Vertex,
                       0);

            for (s32_K = 0; s32_K < pst_PlanningData->s32_ObjectNum; s32_K++)
            {
                if (CalcSAT(pst_ObjectData[s32_K].f32_Vertex, f32_Vertex))
                {
                    pst_CollisionCheckPath[s32_I].b_IsCollision = true;
                    pst_CollisionCheckPath[s32_I].ars32_Collision[s32_M] = s32_J;
                    s32_M = s32_M + 1;
                    b_Collision = true;
                }
            }
        }
        pst_CollisionCheckPath[s32_I].s32_CollisionNum = s32_M;

        if (!b_Collision)
        {
            pst_Planner->ars32_AvailGridPath[s32_I] = 1;
            s32_AvailPathCnt++;
        }
    }
}

void TEST_FindAvailableFrenetLane(PLANNING_DATA_t *pst_PlanningData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global;
    FRENET_PATH_t *pst_CollisionCheckPath = pst_PlanningData->arst_CollisonCheckPath_global;
    VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    int32_t s32_PathNum = pst_FrenetParam->s32_PathNum;
    int32_t s32_AvailPathCnt = 0;
    int32_t s32_I, s32_J, s32_K;

    float32_t f32_Ratio = pst_FrenetParam->f32_Ratio;

    for (s32_I = 0; s32_I < s32_PathNum; s32_I++)
    {
        pst_Planner->ars32_AvailFrenetPath[s32_I] = 0;
    }

    for(s32_I = 0; s32_I < s32_PathNum; s32_I++)
    {
        int32_t s32_M = 0;
        bool b_Collision = false;

        for(s32_J = 20; s32_J < pst_FrenetPath[s32_I].s32_Num; s32_J++)
        {
            float32_t f32_Vertex[4][2];
            CalcVertex(pst_FrenetPath[s32_I].arf32_X[s32_J],
                       pst_FrenetPath[s32_I].arf32_Y[s32_J],
                       pst_EgoVehicleData->f32_MaxX * 1.f,
                       pst_EgoVehicleData->f32_MaxY * 1.f * f32_Ratio,
                       pst_EgoVehicleData->f32_MinX * 1.f,
                       pst_EgoVehicleData->f32_MinY * 1.f * f32_Ratio,
                       pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J], f32_Vertex,
                       0);

            for (s32_K = 0; s32_K < pst_PlanningData->s32_ObjectNum; s32_K++)
            {
                if (CalcSAT(pst_ObjectData[s32_K].f32_Vertex, f32_Vertex))
                {
                    pst_FrenetPath[s32_I].b_IsCollision = true;
                    pst_FrenetPath[s32_I].ars32_Collision[s32_M] = s32_J;
                    s32_M = s32_M + 1;
                    b_Collision = true;
                }
            }
        }
        pst_FrenetPath[s32_I].s32_CollisionNum = s32_M;

        if (!b_Collision)
        {
            pst_Planner->ars32_AvailFrenetPath[s32_I] = 1;
            s32_AvailPathCnt++;
        }
    }
}


void TEST_StateProcessing(HMC_LIDAR_DATA_t *pst_LidarData,
                                  DEAD_RECKONING_DATA_t *pst_DeadReckoningData,
                                  CAMERA_DATA_t *pst_CameraData,
                                  PLANNING_DATA_t *pst_PlanningData,
                                  CAN_DATA_t *pst_CANData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
    TEST_StraightZonePlanning(pst_PlanningData);
}


void TEST_StraightZonePlanning(PLANNING_DATA_t *pst_PlanningData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;

    bool b_InvadeLane = TEST_IsAccAvailable(pst_PlanningData); // 현재 차선에 차량 침범 여부 판단
    TEST_TargetLaneProcessing(pst_PlanningData);

    switch (st_VehicleState)
    {
        case STATE_IDLE:
            if (b_InvadeLane)
            {
                st_VehicleState = STATE_IDLE_ACC_MODE;
            }
            break;

        case STATE_IDLE_ACC_MODE:
            if (b_InvadeLane)
            {
                // ACC 모드 유지
                TEST_AccProcessing(pst_PlanningData);
                pst_Planner->b_AccMode = true;
            }
            else
            {
                // ACC 모드 종료
                TEST_InitAccMode(pst_PlanningData);
                pst_Planner->b_AccMode = false;
                st_VehicleState = STATE_IDLE;
            }
            break;  

        case STATE_OVERTAKING:
            break;

        case STATE_STATIC:
            break;

        case STATE_EMERGENCY:
            break;
    }
}


// Best Path 선택

int32_t TEST_SelectFrenetBestPath(PLANNING_DATA_t *pst_PlanningData, int32_t s32_TargetLane)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global;
    PATH_t *pst_FinalPath = &pst_PlanningData->st_FinalPath_global;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    VEHICLE_DATA_t *pst_CarData = pst_PlanningData->arst_CarData;

    int32_t s32_I;
    float32_t f32_MinCost = 999999999.f;
    int32_t s32_Result = -1;

    for(s32_I = 0; s32_I < pst_PlanningData->s32_FrenetPathNum; s32_I++)
    {
        if(pst_FrenetPath[s32_I].s32_CurrentLane != s32_TargetLane)
        {
            continue;
        }
        if(pst_FrenetPath[s32_I].f32_CostF < f32_MinCost)
        {
            f32_MinCost = pst_FrenetPath[s32_I].f32_CostF;
            s32_Result = s32_I;
        }
    }
    return s32_Result;
}


int32_t TEST_SelectOverTakingPath(PLANNING_DATA_t *pst_PlanningData)
{
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global;
    PATH_t *pst_FinalPath = &pst_PlanningData->st_FinalPath_global;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    VEHICLE_DATA_t *pst_CarData = pst_PlanningData->arst_CarData;
    FRENET_PATH_t *pst_CollisionCheckPath = pst_PlanningData->arst_CollisonCheckPath_global;

    int32_t s32_I;
    float32_t f32_MinCost = 999999999.f;
    int32_t s32_Result = -1;

    for(s32_I = 0; s32_I < pst_PlanningData->s32_FrenetPathNum; s32_I++)
    {
        if(!pst_Planner->ars32_AvailFrenetPath[s32_I] || !pst_Planner->ars32_AvailGridPath[s32_I])
        {
            continue;
        }
        if(pst_FrenetPath[s32_I].f32_CostF < f32_MinCost)
        {
            f32_MinCost = pst_FrenetPath[s32_I].f32_CostF;
            s32_Result = s32_I;
        }
    }
    return s32_Result;
}



int32_t TEST_SelectTargetAccIdx(PLANNING_DATA_t *pst_PlanningData)
{
    VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    ACCCHECK_t *pst_AccCheck = &pst_PlanningData->st_AccCheck;

    int32_t s32_TargetAccIdx = -1;
    float32_t f32_MinDistance = 999999999.f;

    for (int32_t s32_Count = 0; s32_Count < pst_AccCheck->s32_Num; s32_Count++)
    {
        int32_t s32_I = pst_AccCheck->ars32_ProbAccObjIdx[s32_Count];
        
        if (pst_ObjectData[s32_I].f32_DS < f32_MinDistance)
        {
            f32_MinDistance = pst_ObjectData[s32_I].f32_DS;
            s32_TargetAccIdx = s32_I;
        }
    }

    return s32_TargetAccIdx;
}


bool TEST_IsAccAvailable(PLANNING_DATA_t *pst_PlanningData)
{
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global + pst_PlanningData->s32_BestPath;
    ACCCHECK_t *pst_AccCheck = &pst_PlanningData->st_AccCheck;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;

    int32_t s32_I, s32_J, s32_K, s32_M = 0;

    memset(pst_AccCheck, 0, sizeof(ACCCHECK_t));

    for (s32_I = 0;s32_I < pst_FrenetPath->s32_Num;s32_I++)
    {
        float32_t f32_Vertex[4][2];
        CalcVertex(pst_FrenetPath->arf32_X[s32_I],
                   pst_FrenetPath->arf32_Y[s32_I],
                   pst_EgoVehicleData->f32_MaxX * 1.f,
                   pst_EgoVehicleData->f32_MaxY * 1.f * 1.7f,
                   pst_EgoVehicleData->f32_MinX * 1.f,
                   pst_EgoVehicleData->f32_MinY * 1.f * 1.7f,
                   pst_FrenetPath->arf32_Yaw_rad_ENU[s32_I], f32_Vertex,
                   0);

        for (s32_J = 0;s32_J < pst_PlanningData->s32_ObjectNum;s32_J++)
        {
            if (CalcSAT(pst_ObjectData[s32_J].f32_Vertex, f32_Vertex))
            {
                bool b_IsAlreadyExist = false;
                for (s32_K = 0;s32_K < s32_M;s32_K++)
                {
                    if (pst_AccCheck->ars32_ProbAccObjIdx[s32_K] == s32_J)
                    {
                        b_IsAlreadyExist = true;
                        break;
                    }
                }

                if (!b_IsAlreadyExist)
                {
                    pst_AccCheck->ars32_ProbAccObjIdx[s32_M] = s32_J;
                    s32_M++;
                }
            }
        }
    }

    pst_AccCheck->s32_Num = s32_M;
    return (s32_M > 0);
}


void TEST_AccProcessing(PLANNING_DATA_t *pst_PlanningData)
{
    VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    ACCCHECK_t *pst_AccCheck = &pst_PlanningData->st_AccCheck;

    float32_t f32_AccelMax = 5.f;
    float32_t f32_EgoParam = pst_FrenetParam->f32_EgoParam;
    float32_t f32_RelativeParam = pst_FrenetParam->f32_RelativeParam;
    float32_t f32_MinMarginParam = pst_FrenetParam->f32_MinMarginParam;
    float32_t f32_MaxBrakeThreshold = pst_FrenetParam->f32_MaxBrakeThreshold;

    int32_t s32_TargetAccIdx = TEST_SelectTargetAccIdx(pst_PlanningData);
    pst_PlanningData->s32_AccObjIdx = s32_TargetAccIdx;

    pst_ObjectData[s32_TargetAccIdx].f32_RelativeDistance = pst_ObjectData[s32_TargetAccIdx].f32_DS;
    pst_ObjectData[s32_TargetAccIdx].f32_RelativeVelocity = pst_ObjectData[s32_TargetAccIdx].f32_Velocity_m_s_local;
    pst_ObjectData[s32_TargetAccIdx].f32_SafetyDistance = (pst_EgoVehicleData->f32_Velocity_m_s_global) * f32_EgoParam
                                                         + f32_RelativeParam * powf(pst_EgoVehicleData->f32_Velocity_m_s_global - pst_ObjectData[s32_TargetAccIdx].f32_Velocity_m_s_global, 2) / (2.f * f32_AccelMax)
                                                         + f32_MinMarginParam;
    pst_ObjectData[s32_TargetAccIdx].f32_AbsDistance = 0.f;
    if(pst_ObjectData[s32_TargetAccIdx].f32_RelativeVelocity < 0.f)
    {
       pst_ObjectData[s32_TargetAccIdx].f32_AbsDistance = 0.0529f * pow(pst_ObjectData[s32_TargetAccIdx].f32_RelativeVelocity, 2)
                                                        + 0.1861f * fabsf(pst_ObjectData[s32_TargetAccIdx].f32_RelativeVelocity)
                                                        - 0.034f  + 10.f;
    }
    bool b_AccMode = true, b_AebMode = false, b_AbsMode = false;
    
    if (pst_ObjectData[s32_TargetAccIdx].f32_SafetyDistance > pst_ObjectData[s32_TargetAccIdx].f32_RelativeDistance)
    {
        if( f32_MaxBrakeThreshold < pst_EgoVehicleData->f32_Velocity_m_s_global - pst_ObjectData[s32_TargetAccIdx].f32_Velocity_m_s_global)
        {
            b_AebMode = true;
            pst_ObjectData[s32_TargetAccIdx].b_AEB_Mode = b_AebMode;
        }
    }
    pst_ObjectData[s32_TargetAccIdx].b_ACC_Mode = b_AccMode;
    pst_ObjectData[s32_TargetAccIdx].b_AEB_Mode = b_AebMode;
    // pst_ObjectData[s32_TargetAccIdx].b_ABS_Mode = b_AbsMode;
}



void TEST_InitAccMode(PLANNING_DATA_t *pst_PlanningData)
{
    pst_PlanningData->s32_AccObjIdx = -1;
    pst_PlanningData->arst_ObjectData[pst_PlanningData->s32_AccObjIdx].b_ACC_Mode = false;
    pst_PlanningData->arst_ObjectData[pst_PlanningData->s32_AccObjIdx].b_AEB_Mode = false;
    pst_PlanningData->arst_ObjectData[pst_PlanningData->s32_AccObjIdx].b_ABS_Mode = false;
}

void TEST_TargetLaneProcessing(PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_I;

    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;

    pst_PlanningData->s32_BestPath = TEST_SelectFrenetBestPath(pst_PlanningData, pst_Planner->s32_TargetLane);
}

void TEST_FinalPath(PLANNING_DATA_t *pst_PlanningData)
{
    PATH_t *pst_FinalPath = &pst_PlanningData->st_FinalPath_global;
    FRENET_PATH_t *pst_ControlPath = pst_PlanningData->arst_ControlPath_global;
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    int32_t s32_I;

    float32_t f32_NearDist = 0.0f;
    int32_t s32_NearIdx = 0;

    memset(pst_FinalPath, 0, sizeof(PATH_t));
    for (s32_I = 0; s32_I < pst_FrenetPath[pst_PlanningData->s32_BestPath].s32_Num; s32_I++)
    {
        pst_FinalPath->arf32_X[s32_I] = pst_ControlPath[pst_PlanningData->s32_BestPath].arf32_X[s32_I];
        pst_FinalPath->arf32_Y[s32_I] = pst_ControlPath[pst_PlanningData->s32_BestPath].arf32_Y[s32_I];
        pst_FinalPath->arf32_Yaw_rad_ENU[s32_I] = pst_ControlPath[pst_PlanningData->s32_BestPath].arf32_Yaw_rad_ENU[s32_I];
        pst_FinalPath->arf32_Yaw_rad_NED[s32_I] = pst_ControlPath[pst_PlanningData->s32_BestPath].arf32_Yaw_rad_NED[s32_I];
    }
    pst_FinalPath->s32_Num = s32_I;

    CalcNearestDistIdx(pst_EgoVehicleData->f32_X_global, pst_EgoVehicleData->f32_Y_global, pst_FinalPath->arf32_X, pst_FinalPath->arf32_Y, pst_FinalPath->s32_Num, f32_NearDist, s32_NearIdx);
    pst_FinalPath->s32_NearIdx = s32_NearIdx;
    pst_FinalPath->f32_NearDist = f32_NearDist;
}
