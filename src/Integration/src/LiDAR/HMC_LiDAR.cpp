#include "HMC_LiDAR.h"
#include <omp.h>

extern std::string s_LIDARName;
extern SENSOR_DATA_t st_SensorData;
extern LIDAR_PARAM_t st_LidarParam;
extern int32_t s32_CanMode;

float32_t f32_HMC_StartTime_s = getMillisecond() / 1000.f;
float32_t f32_HMC_EndTime_s;
float32_t f32_HMC_DeltaTime_s;

float32_t f32_HMC_CurrYaw_rad;
float32_t f32_HMC_DeltaYaw_rad;
float32_t f32_HMC_PrevYaw_rad;

int32_t s32_TrackingNumBackup = 0;
HMC_TRACKING_t arst_TrackingBackup[c_TOTAL_TRACKING_NUM];
int32_t s32_PrevClusterNum = 0;
HMC_CLUSTER_t arst_PrevCluster[c_HMC_MAX_CLUSTER_NUM];

void HMC_LIDARProcessing(RAW_LIDAR_DATA_t *pst_RawData, HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, OMNI_POINT_t *pst_OmniPoint, ICP_MAP_t *pst_ICPMap)
{
    float32_t f32_Roll;
    float32_t f32_Pitch;
    int32_t s32_LIDARMode;

    memcpy(&f32_Roll, &pst_DeadReckoningData->st_IMU.f32_Roll_deg, sizeof(float32_t));
    memcpy(&f32_Pitch, &pst_DeadReckoningData->st_IMU.f32_Pitch_deg, sizeof(float32_t));

    if(s_LIDARName == "OS2")
    {
        s32_LIDARMode = c_HMC_MODE_OS2;
    }
    else
    {
        s32_LIDARMode = c_HMC_MODE_VLS;
    }
    
    CUDA_LidarPreprocessing(pst_RawData->arc_Buffer + 12, pst_LidarData, f32_Roll, f32_Pitch, s32_LIDARMode);

    LidarLocalization(pst_LidarData, pst_DeadReckoningData, pst_OmniPoint, pst_ICPMap);

    HMC_Clustering(pst_LidarData);

    // CUDA_MapBoundaryFilter() 안전성 확인 필요
    // HMC_MapBoundaryFilterCluster(pst_LidarData, pst_PlanningData);
    CUDA_MapBoundaryFilter(pst_LidarData, &pst_PlanningData->st_BoundaryInOut, &pst_PlanningData->st_BoundaryIn2,
                           st_LidarParam.f32_MapDistanceOffset, &pst_PlanningData->st_EgoVehicleData, 0);

    pst_LidarData->u64_Time_ms = pst_RawData->u64_Timestamp;

    HMC_ObjectTracking(pst_LidarData, pst_PlanningData);
}




void HMC_LidarGroundFilteringOMP(HMC_LIDAR_DATA_t *pst_LidarPointData, int32_t s32_LIDARMode)
{
    int32_t s32_Index = 0;
    int32_t s32_PointNum = pst_LidarPointData->s32_PointNum;

    #pragma omp parallel for num_threads(1024)
    for (s32_Index = 0;s32_Index < pst_LidarPointData->s32_PointNum;s32_Index += 128)
    {
        int32_t s32_I = 0;
        int32_t s32_J = 0;
        int32_t ars32_Buffer[128] = {0};
        int32_t s32_BufferNum = 0;
        int32_t s32_CurrentIndex = 0;
        int32_t s32_NextIndex = 0;
        uint8_t u8_PrevFlag = 0;

        float32_t f32_X0, f32_Y0, f32_Z0;
        float32_t f32_X1, f32_Y1, f32_Z1;
        float32_t f32_X2, f32_Y2, f32_Z2;
        float32_t f32_DistX, f32_DistY;
        float32_t f32_Dist = 0.f;
        float32_t f32_DistXY;
        float32_t f32_DistZ;
        float32_t f32_SumZ;
        float32_t f32_Inclination;

        HMC_POINT_t* pst_CurPoint;
        HMC_POINT_t* pst_NextPoint;

        while (s32_I < 128)
        {
            // INVALID는 넘어가면서 128 넘어가지 않도록 방지
            while (pst_LidarPointData->arst_Point[s32_Index + s32_I].u8_Flag != c_POINT_GROUND && s32_I < 128)
            {
                s32_I += 1;
            }

            // 127ch 점이면 break
            if (s32_I >= 127)
            {
                break;
            }
            // 그 이하의 Channel이면 현재 Index 받기
            else
            {
                s32_CurrentIndex = s32_Index + s32_I;
            }

            pst_CurPoint = &pst_LidarPointData->arst_Point[s32_CurrentIndex];

            f32_X1 = pst_CurPoint->f32_X;
            f32_Y1 = pst_CurPoint->f32_Y;
            f32_Z1 = pst_CurPoint->f32_Z;
            s32_BufferNum = 0;
            f32_SumZ = 0.f;

            for (s32_J = s32_I + 1;s32_J < 128;s32_J++)
            {
                s32_NextIndex = s32_Index + s32_J;
                pst_NextPoint = &pst_LidarPointData->arst_Point[s32_NextIndex];

                // 다음 점이 INVALID면 넘어간다.
                if (pst_NextPoint->u8_Flag != c_POINT_GROUND)
                {
                    continue;
                }

                f32_X2 = pst_NextPoint->f32_X;
                f32_Y2 = pst_NextPoint->f32_Y;
                f32_Z2 = pst_NextPoint->f32_Z;

                f32_DistX = f32_X2 - f32_X1;
                f32_DistY = f32_Y2 - f32_Y1;

                f32_DistXY = sqrtf((f32_DistX * f32_DistX) + (f32_DistY * f32_DistY));
                f32_DistZ = f32_Z2 - f32_Z1;

                f32_Inclination = f32_DistZ / f32_DistXY;

                // 다음 점과의 거리가 0.5 이하(섹터 별로 지면제거 가능), 현재 점 보다 다음 점의 높이가 더 높고 현재점~다음점의 높이차가 0.1 이상
                // 이때는 Buffer에 그 다음 점들의 Index를 모두 담고 높이차를 누적합한다.

                if(s32_LIDARMode == c_HMC_MODE_OS2)
                {
                    if (
                        (f32_DistXY < 0.5f) &&
                        (f32_DistZ > 0.1f) &&
                        (f32_Inclination > 1.2f) // Bank Removal Rule
                    )
                    {
                        ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                        f32_SumZ += f32_DistZ;
                    }
                    else if(pst_CurPoint->f32_Distance >= pst_NextPoint->f32_Distance)
                    {
                        ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                        f32_SumZ += f32_DistZ;
                    }
                    else if(
                        pst_CurPoint->f32_Distance > 60.f &&
                        f32_DistXY < 1.5f &&
                        (f32_Inclination > 1.f)
                    )
                    {
                        ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                        f32_SumZ += f32_DistZ;
                    }
                }
                else
                {
                    if (
                        (f32_DistXY < 0.5f) &&
                        (f32_Inclination > 1.2f) // Bank Removal Rule
                    )
                    {
                        ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                        f32_SumZ += f32_DistZ;
                    }
                    else if(
                        pst_CurPoint->f32_Distance > 60.f &&
                        f32_DistXY < 1.5f &&
                        (f32_Inclination > 1.f)
                    )
                    {
                        ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                        f32_SumZ += f32_DistZ;
                    }
                }
            }

            // Buffer 내 점의 갯수가 3 이상이고 높이차의 누적합이 0.3 이상이면 그 점들은 모두 살린다.
            if ((s32_BufferNum >= 3) && (f32_SumZ > 0.3f))
            {
                for (s32_J = 0;s32_J < s32_BufferNum;s32_J++)
                {
                    pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_Flag = c_POINT_VALID;
                    pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_GroundFilteringFlag = 1;
                }
            }
            // 현재 점까지의 거리가 60m 이상, Buffer 내 점의 갯수가 2 이상, 누적합이 0.2 이상이면 그 점들은 모두 살린다.
            else if ((pst_CurPoint->f32_Distance > 60.f) && (s32_BufferNum >= 2) && (f32_SumZ > 0.2f))
            {
                for (s32_J = 0;s32_J < s32_BufferNum;s32_J++)
                {
                    pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_Flag = c_POINT_VALID;
                    pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_GroundFilteringFlag = 2;
                }
            }
            s32_I += 1;
        }
    }
}



void BFS(HMC_VOXEL_t* pst_Voxel, int32_t s32_VoxelIndex, int32_t s32_ClusterID, float32_t f32_Threshold, HMC_CLUSTER_t* pst_Cluster)
{
    int32_t s32_I = 0;
    int32_t s32_J = 0;
    int32_t s32_K = 0;
    int32_t s32_DR = 0;
    int32_t s32_DAzi = 0;
    int32_t s32_DZ = 0;
    int32_t s32_Cur = 0;
    int32_t s32_Next = 0;
    float32_t f32_DIstance = 0.f;
    int32_t ars32_MoveR[] = { -c_HMC_VOXEL_SIZE_AZIMUTH * c_HMC_VOXEL_SIZE_Z, 0, c_HMC_VOXEL_SIZE_AZIMUTH * c_HMC_VOXEL_SIZE_Z };
    int32_t ars32_MoveAzi[] = { -c_HMC_VOXEL_SIZE_Z, 0, c_HMC_VOXEL_SIZE_Z };
    int32_t ars32_MoveZ[] = { -1, 0, 1 };
    int32_t s32_Count = 0;

    queue<int32_t> q;
    q.push(s32_VoxelIndex);

    HMC_VOXEL_t* st_Voxel;

    //uint8_t aru8_Visited[c_VOXEL_SIZE] = { 0 };

    pst_Voxel[s32_VoxelIndex].s32_ClusterID = s32_ClusterID;

    while (q.size())
    {
        s32_Cur = q.front();
        q.pop();

        for (int32_t s32_I = 0; s32_I < 3; s32_I++)
        {
            s32_DR = ars32_MoveR[s32_I];

            for (int32_t s32_J = 0; s32_J < 3; s32_J++)
            {
                s32_DAzi = ars32_MoveAzi[s32_J];

                for (int32_t s32_K = 0; s32_K < 3; s32_K++)
                {
                    s32_DZ = ars32_MoveZ[s32_K];

                    s32_Next = s32_Cur + s32_DAzi + s32_DR + s32_DZ;

                    if (s32_Next < 0 || s32_Next >= c_HMC_VOXEL_SIZE || s32_Next == s32_Cur)
                    {
                        continue;
                    }

                    //if (aru8_Visited[s32_Next] >= 26)
                    //{
                    //  continue;
                    //}

                    //aru8_Visited[s32_Next] += 1;

                    if (pst_Voxel[s32_Next].b_HasPoint == false)
                    {
                        continue;
                    }

                    if (pst_Voxel[s32_Next].s32_ClusterID != -1)
                    {
                        continue;
                    }

                    //f32_DIstance = GetDistanceBetweenVoxel26(pst_Voxel, s32_Cur, s32_Next, s32_I, s32_J, s32_K);

                    //if (f32_DIstance < f32_Threshold)
                    //{
                    //  pst_Voxel[s32_Next].s32_ClusterID = s32_ClusterID;
                    //  q.push(s32_Next);
                    //}

                    //if (CheckNearesetVoxel26(pst_Voxel, s32_Cur, s32_Next, f32_Threshold))
                    //{
                    pst_Voxel[s32_Next].s32_ClusterID = s32_ClusterID;
                    pst_Cluster[s32_ClusterID].s32_VoxelNum += 1;
                    q.push(s32_Next);
                    s32_Count++;
                    if(s32_Count == 70) return;
                    //}
                }
            }
        }
    }
}


void HMC_Clustering(HMC_LIDAR_DATA_t* pst_LidarPointData)
{
    int32_t s32_I = 0;
    int32_t s32_J = 0;
    int32_t s32_ClusterID = 0;
    int32_t s32_ClusterAngle = 0;

    HMC_VOXEL_t* pst_Voxel = pst_LidarPointData->arst_Voxel;
    HMC_CLUSTER_t* pst_Cluster = pst_LidarPointData->arst_Cluster;

    float32_t f32_X = 0.f;
    float32_t f32_Y = 0.f;
    float32_t f32_Z = 0.f;
    float32_t f32_AzimuthDistance = 0;

    float32_t f32_CenterX = 0.f;
    float32_t f32_CenterY = 0.f;

    float32_t f32_Distance = 0.f;
    float32_t f32_Azimuth_deg = 0.f;
    float32_t f32_Length = 0.f;

    memset(pst_LidarPointData->arst_Cluster, 0, sizeof(HMC_CLUSTER_t) * c_HMC_MAX_CLUSTER_NUM);

    for (s32_I = 0; s32_I < c_HMC_MAX_CLUSTER_NUM; s32_I++)
    {
        pst_LidarPointData->arst_Cluster[s32_I].f32_X1 = -9999.f;
        pst_LidarPointData->arst_Cluster[s32_I].f32_Y1 = -9999.f;
        pst_LidarPointData->arst_Cluster[s32_I].f32_Z1 = -9999.f;
        pst_LidarPointData->arst_Cluster[s32_I].f32_X2 = 9999.f;
        pst_LidarPointData->arst_Cluster[s32_I].f32_Y2 = 9999.f;
        pst_LidarPointData->arst_Cluster[s32_I].f32_Z2 = 9999.f;
    }


    for (s32_I = 0; s32_I < c_HMC_VOXEL_SIZE_AZIMUTH * c_HMC_VOXEL_SIZE_R * c_HMC_VOXEL_SIZE_Z; s32_I++)
    {
        if (pst_LidarPointData->arst_Voxel[s32_I].b_HasPoint == false)
        {
            continue;
        }

        if (pst_LidarPointData->arst_Voxel[s32_I].s32_ClusterID != -1)
        {
            continue;
        }

        BFS(pst_LidarPointData->arst_Voxel, s32_I, s32_ClusterID, 1.f, pst_LidarPointData->arst_Cluster);
        s32_ClusterID += 1;
    }

    pst_LidarPointData->s32_ClusterNum = s32_ClusterID;

    for (s32_I = 0; s32_I < pst_LidarPointData->s32_PointNum; s32_I++)
    {
        if (pst_LidarPointData->arst_Point[s32_I].u8_Flag != c_POINT_VALID)
        {
            continue;
        }

        if (pst_LidarPointData->arst_Point[s32_I].s32_VoxelIndex == -1)
        {
            continue;
        }


        s32_ClusterID = pst_Voxel[pst_LidarPointData->arst_Point[s32_I].s32_VoxelIndex].s32_ClusterID;

        if (s32_ClusterID == -1)
        {
            continue;
        }

        f32_X = pst_LidarPointData->arst_Point[s32_I].f32_X;
        f32_Y = pst_LidarPointData->arst_Point[s32_I].f32_Y;
        f32_Z = pst_LidarPointData->arst_Point[s32_I].f32_Z;
        f32_Distance = pst_LidarPointData->arst_Point[s32_I].f32_Distance;
        f32_Azimuth_deg = pst_LidarPointData->arst_Point[s32_I].f32_Azimuth_deg;;

        pst_Cluster[s32_ClusterID].f32_X1 = fmaxf(f32_X, pst_Cluster[s32_ClusterID].f32_X1);
        pst_Cluster[s32_ClusterID].f32_X2 = fminf(f32_X, pst_Cluster[s32_ClusterID].f32_X2);

        pst_Cluster[s32_ClusterID].f32_Y1 = fmaxf(f32_Y, pst_Cluster[s32_ClusterID].f32_Y1);
        pst_Cluster[s32_ClusterID].f32_Y2 = fminf(f32_Y, pst_Cluster[s32_ClusterID].f32_Y2);

        pst_Cluster[s32_ClusterID].f32_Z1 = fmaxf(f32_Z, pst_Cluster[s32_ClusterID].f32_Z1);
        pst_Cluster[s32_ClusterID].f32_Z2 = fminf(f32_Z, pst_Cluster[s32_ClusterID].f32_Z2);
        
        if (pst_Cluster[s32_ClusterID].s32_PointNum < c_HMC_CLUSTER_MAX_POINT_NUM)
        {
            pst_Cluster[s32_ClusterID].ars32_PointIndex[pst_Cluster[s32_ClusterID].s32_PointNum] = s32_I;

            pst_Cluster[s32_ClusterID].s32_PointNum += 1;
        }

        // s32_J = (int32_t)roundf(fmodf(f32_Azimuth_deg + 360.f, 360.f)) % 360;

        // if (pst_Cluster[s32_ClusterID].arb_Flag[s32_J] == false)
        // {
        //     pst_Cluster[s32_ClusterID].arf32_X[s32_J] = f32_X;
        //     pst_Cluster[s32_ClusterID].arf32_Y[s32_J] = f32_Y;
        //     pst_Cluster[s32_ClusterID].arf32_R[s32_J] = f32_Distance;
        //     pst_Cluster[s32_ClusterID].arb_Flag[s32_J] = true;
        // }
        // else if (pst_Cluster[s32_ClusterID].arf32_R[s32_J] > f32_Distance)
        // {
        //     pst_Cluster[s32_ClusterID].arf32_X[s32_J] = f32_X;
        //     pst_Cluster[s32_ClusterID].arf32_Y[s32_J] = f32_Y;
        //     pst_Cluster[s32_ClusterID].arf32_R[s32_J] = f32_Distance;
        // }
    }

    for (s32_I = 0; s32_I < pst_LidarPointData->s32_ClusterNum; s32_I++)
    {
        pst_Cluster = pst_LidarPointData->arst_Cluster;
        pst_Cluster[s32_I].b_TrackingFlag = 1;


        // if (fabsf(pst_Cluster[s32_I].f32_CenterX) < 1.f)
        // {
        //     ;
        // }
        // else if (pst_Cluster[s32_I].f32_CenterX > 0.f)
        // {
        //     pst_Cluster[s32_I].f32_CenterX = pst_Cluster[s32_I].f32_X2;
        // }
        // else
        // {
        //     pst_Cluster[s32_I].f32_CenterX = pst_Cluster[s32_I].f32_X1;
        // }

        pst_Cluster[s32_I].f32_CenterX = (pst_Cluster[s32_I].f32_X1 + pst_Cluster[s32_I].f32_X2) / 2.f;
        pst_Cluster[s32_I].f32_CenterY = (pst_Cluster[s32_I].f32_Y1 + pst_Cluster[s32_I].f32_Y2) / 2.f;
        pst_Cluster[s32_I].f32_CenterZ = (pst_Cluster[s32_I].f32_Z1 + pst_Cluster[s32_I].f32_Z2) / 2.f;

        pst_Cluster[s32_I].f32_Length = getDistance2d(pst_Cluster[s32_I].f32_X1, pst_Cluster[s32_I].f32_Y1, pst_Cluster[s32_I].f32_X2, pst_Cluster[s32_I].f32_Y2);
        pst_Cluster[s32_I].f32_Size = fabsf(pst_Cluster[s32_I].f32_X1 - pst_Cluster[s32_I].f32_X2) *
                                      fabsf(pst_Cluster[s32_I].f32_Y1 - pst_Cluster[s32_I].f32_Y2) *
                                      fabsf(pst_Cluster[s32_I].f32_Z1 - pst_Cluster[s32_I].f32_Z2);
        // printf("cluster size : %f, cluster length : %f\n", pst_Cluster[s32_I].f32_Size, pst_Cluster[s32_I].f32_Length);
        if (pst_Cluster[s32_I].s32_PointNum < st_LidarParam.s32_MinClusterPointNum ||
            pst_Cluster[s32_I].f32_Length > st_LidarParam.f32_MaxClusterLength ||
            pst_Cluster[s32_I].f32_Size < st_LidarParam.f32_MinClusterSize)
        {
            pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
        }
        else
        {
            pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_VEHICLE;
        }
    }
}


void HMC_MapBoundaryFilterCluster(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_I;
    int32_t s32_Cross;

    float32_t f32_ClusterGlobalX, f32_ClusterGlobalY;
    float32_t f32_MinDistance;

    HMC_CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    PATH_t *pst_RefBoundary = &pst_PlanningData->st_BoundaryInOut;
    PATH_t *pst_PitBoundary = &pst_PlanningData->st_BoundaryIn2;

    for(s32_I = 0; s32_I < pst_LidarData->s32_ClusterNum; s32_I++)
    {
        getGlobalCoord(pst_EgoVehicleData->f32_X_global, pst_EgoVehicleData->f32_Y_global, pst_EgoVehicleData->f32_Yaw_rad_ENU,
                       pst_Cluster[s32_I].f32_CenterX, pst_Cluster[s32_I].f32_CenterY, f32_ClusterGlobalX, f32_ClusterGlobalY);

        f32_MinDistance = FLT_MAX;
        s32_Cross = 0;

        HMC_CheckMapBoundary(pst_RefBoundary, f32_ClusterGlobalX, f32_ClusterGlobalY, s32_Cross, f32_MinDistance);

        if(s32_Cross % 2 == 0)
        {
            pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
        }
        else if(f32_MinDistance < st_LidarParam.f32_MapDistanceOffset)
        {
            pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
        }
        else
        {
            s32_Cross = 0;

            HMC_CheckMapBoundary(pst_PitBoundary, f32_ClusterGlobalX, f32_ClusterGlobalY, s32_Cross, f32_MinDistance);

            if(s32_Cross % 2 != 0)
            {
                pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
            }
            else if(f32_MinDistance < st_LidarParam.f32_MapDistanceOffset)
            {
                pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
            }
        }
    }
}


void HMC_CheckMapBoundary(PATH_t *pst_Boundary, float32_t f32_GlobalX, float32_t f32_GlobalY, int32_t &s32_Cross, float32_t &f32_MinDistance)
{
    int32_t s32_I;

    float32_t f32_StartX, f32_StartY, f32_EndX, f32_EndY;
    float32_t f32_IntersectX;
    float32_t f32_UnitProjection, f32_Distance, f32_ProjectionX, f32_ProjectionY;

    POINT_t st_LineVector, st_PointVector;

    for(s32_I = 0; s32_I < pst_Boundary->s32_Num; s32_I++)
    {
        f32_StartX = pst_Boundary->arf32_X[s32_I];
        f32_StartY = pst_Boundary->arf32_Y[s32_I];
        f32_EndX = pst_Boundary->arf32_X[(s32_I + 1) % pst_Boundary->s32_Num];
        f32_EndY = pst_Boundary->arf32_Y[(s32_I + 1) % pst_Boundary->s32_Num];

        if((f32_StartY > f32_GlobalY) != (f32_EndY > f32_GlobalY))
        {
            f32_IntersectX = (f32_EndX - f32_StartX) * (f32_GlobalY - f32_StartY) / (f32_EndY - f32_StartY) + f32_StartX;

            if(f32_GlobalX < f32_IntersectX)
            {
                s32_Cross++;
            }
        }

        st_LineVector.f32_X = f32_EndX - f32_StartX;
        st_LineVector.f32_Y = f32_EndY - f32_StartY;
        st_PointVector.f32_X = f32_GlobalX - f32_StartX;
        st_PointVector.f32_Y = f32_GlobalY - f32_StartY;

        f32_UnitProjection = CalcDotProduct(st_PointVector, st_LineVector) / CalcDotProduct(st_LineVector, st_LineVector);

        if(f32_UnitProjection < 0.f)
        {
            f32_Distance = getDistance2d(f32_StartX, f32_StartY, f32_GlobalX, f32_GlobalY);
        }
        else if(f32_UnitProjection > 1.f)
        {
            f32_Distance = getDistance2d(f32_EndX, f32_EndY, f32_GlobalX, f32_GlobalY);
        }
        else
        {
            f32_ProjectionX = f32_StartX + f32_UnitProjection * st_LineVector.f32_X;
            f32_ProjectionY = f32_StartY + f32_UnitProjection * st_LineVector.f32_Y;

            f32_Distance = getDistance2d(f32_ProjectionX, f32_ProjectionY, f32_GlobalX, f32_GlobalY);
        }

        if(f32_Distance < f32_MinDistance)
        {
            f32_MinDistance = f32_Distance;
        }
    }
}


void HMC_MapBoundaryFilterLocal(HMC_LIDAR_DATA_t *pst_LidarData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, PLANNING_DATA_t *pst_PlanningData, int32_t s32_LIDARMode)
{
    int32_t s32_I, s32_J, s32_K;
    float32_t f32_LocalX, f32_LocalY;
    float32_t f32_X, f32_Y, f32_Z;
    float32_t f32_E, f32_N, f32_U;
    float32_t f32_Yaw_deg;

    int32_t s32_LeftNearIdx1, s32_LeftNearIdx2, s32_RightNearIdx1, s32_RightNearIdx2;
    int32_t s32_NormalLeftNearIdx, s32_NormalRightNearIdx, s32_JokerLeftNearIdx, s32_JokerRightNearIdx, s32_ClusterIdx;
    float32_t f32_Dist, f32_Dist1, f32_Dist2, f32_Dist3, f32_Dist4; 
    float32_t f32_NormalLeftMinDist, f32_NormalRightMinDist, f32_JokerLeftMinDist, f32_JokerRightMinDist;
    float32_t f32_LeftMinDist1, f32_LeftMinDist2, f32_RightMinDist1, f32_RightMinDist2;

    std::string s_Line;

    vector<pair<float32_t, float32_t> > st_BoundJokerGlobalLeft;
    vector<pair<float32_t, float32_t> > st_BoundJokerGlobalRight;
    
    vector<pair<float32_t, float32_t> > st_BoundLeft;
    vector<pair<float32_t, float32_t> > st_BoundRight;

    lla2enu(pst_DeadReckoningData->st_GPS.f64_Latitude, 
            pst_DeadReckoningData->st_GPS.f64_Longitude,
            pst_DeadReckoningData->st_GPS.f64_Altitude, 
            f32_E, f32_N, f32_U);

    PATH_t *pst_Boundary[] = {
                                &pst_PlanningData->st_Boundary1_global,
                                &pst_PlanningData->st_Boundary4_global
                              };

    std::ifstream st_JokerLeftBoundaryFile("src/Integration/map/Kiapi/Boundary/Boundary5_OnlySideway.txt");
    std::ifstream st_JokerRightBoundaryFile("src/Integration/map/Kiapi/Boundary/Boundary6_OnlySideway.txt");

    if(s32_LIDARMode == c_HMC_MODE_OS2)
        f32_Yaw_deg = -pst_LidarData->f32_PredictYaw_deg + 90.f;
    else
        f32_Yaw_deg = pst_DeadReckoningData->st_IMU.f32_Yaw_deg;

    while (std::getline(st_JokerLeftBoundaryFile, s_Line))
    {
        std::istringstream st_Stream(s_Line);
        st_Stream >> f32_X >> f32_Y;
        st_BoundJokerGlobalLeft.push_back(make_pair(f32_X, f32_Y));
    }
    st_JokerLeftBoundaryFile.close();

    while (std::getline(st_JokerRightBoundaryFile, s_Line))
    {
        std::istringstream st_Stream(s_Line);
        st_Stream >> f32_X >> f32_Y;
        st_BoundJokerGlobalRight.push_back(make_pair(f32_X, f32_Y));
    }
    st_JokerRightBoundaryFile.close();

    // Get Left Min Dist to Joker Lane
    f32_JokerLeftMinDist = 999999999.f;

    for(s32_I = 0; s32_I < st_BoundJokerGlobalLeft.size(); s32_I++)
    {
        f32_Dist = getDistance2d(f32_E, f32_N, st_BoundJokerGlobalLeft[s32_I].first, st_BoundJokerGlobalLeft[s32_I].second);
        
        if(f32_JokerLeftMinDist > f32_Dist)
        {
            f32_JokerLeftMinDist = f32_Dist; 
            s32_JokerLeftNearIdx = s32_I;
        }
    }

    // Get Right Min Dist to Joker Lane 
    f32_JokerRightMinDist = 999999999.f;

    for(s32_I = 0; s32_I < st_BoundJokerGlobalRight.size(); s32_I++)
    {
        f32_Dist = getDistance2d(f32_E, f32_N, st_BoundJokerGlobalRight[s32_I].first, st_BoundJokerGlobalRight[s32_I].second);
        
        if(f32_JokerRightMinDist > f32_Dist)
        {
            f32_JokerRightMinDist = f32_Dist; 
            s32_JokerRightNearIdx = s32_I;
        }
    }

    CalcNearestDistIdx(f32_E, f32_N, 
                        pst_Boundary[0]->arf32_X,
                        pst_Boundary[0]->arf32_Y,
                        pst_Boundary[0]->s32_Num,
                        f32_NormalLeftMinDist, s32_NormalLeftNearIdx);

    CalcNearestDistIdx(f32_E, f32_N, 
                        pst_Boundary[1]->arf32_X,
                        pst_Boundary[1]->arf32_Y,
                        pst_Boundary[1]->s32_Num,
                        f32_NormalRightMinDist, s32_NormalRightNearIdx);

    if(f32_NormalLeftMinDist < f32_JokerLeftMinDist)
    {
        for (int s32_K : {0, 1})
        {
            for (s32_I = -100; s32_I <= 100; s32_I += 1)
            {
                if(s32_K == 0)
                {
                    s32_J = (s32_NormalLeftNearIdx + s32_I + pst_Boundary[s32_K]->s32_Num) % pst_Boundary[s32_K]->s32_Num;
                }
                else
                {
                    s32_J = (s32_NormalRightNearIdx + s32_I + pst_Boundary[s32_K]->s32_Num) % pst_Boundary[s32_K]->s32_Num;
                }

                f32_X = pst_Boundary[s32_K]->arf32_X[s32_J]; 
                f32_Y = pst_Boundary[s32_K]->arf32_Y[s32_J];

                getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg), f32_X, f32_Y, f32_LocalX, f32_LocalY);

                if (s32_K == 0)
                {
                    st_BoundLeft.push_back(make_pair(f32_LocalX, f32_LocalY));
                }
                else
                {
                    st_BoundRight.push_back(make_pair(f32_LocalX, f32_LocalY));
                }
            }
        }
    }
    else
    {
        for (s32_I = -100; s32_I <= 100; s32_I += 1)
        {
            s32_J = (s32_JokerLeftNearIdx + s32_I + st_BoundJokerGlobalLeft.size()) % st_BoundJokerGlobalLeft.size();

            f32_X = st_BoundJokerGlobalLeft[s32_J].first;
            f32_Y = st_BoundJokerGlobalLeft[s32_J].second; 

            getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg), f32_X, f32_Y, f32_LocalX, f32_LocalY);
            st_BoundLeft.push_back(make_pair(f32_LocalX, f32_LocalY + 3.f));
        }

        for (s32_I = -100; s32_I <= 100; s32_I += 1)
        {
            s32_J = (s32_JokerRightNearIdx + s32_I + st_BoundJokerGlobalRight.size()) % st_BoundJokerGlobalRight.size();

            f32_X = st_BoundJokerGlobalRight[s32_J].first;
            f32_Y = st_BoundJokerGlobalRight[s32_J].second; 

            getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg), f32_X, f32_Y, f32_LocalX, f32_LocalY);
            st_BoundRight.push_back(make_pair(f32_LocalX, f32_LocalY - 3.f));
        }

        if(s32_JokerLeftNearIdx > 300)
        {
            for (int s32_K : {0, 1})
            {
                for (s32_I = -100; s32_I <= 100; s32_I += 1)
                {
                    if(s32_K == 0)
                    {
                        s32_J = (s32_NormalLeftNearIdx + s32_I + pst_Boundary[s32_K]->s32_Num) % pst_Boundary[s32_K]->s32_Num;
                    }
                    else
                    {
                        s32_J = (s32_NormalRightNearIdx + s32_I + pst_Boundary[s32_K]->s32_Num) % pst_Boundary[s32_K]->s32_Num;
                    }

                    f32_X = pst_Boundary[s32_K]->arf32_X[s32_J]; 
                    f32_Y = pst_Boundary[s32_K]->arf32_Y[s32_J];

                    getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg), f32_X, f32_Y, f32_LocalX, f32_LocalY);

                    if (s32_K == 0)
                    {
                        st_BoundLeft.push_back(make_pair(f32_LocalX, f32_LocalY));
                    }
                    else
                    {
                        st_BoundRight.push_back(make_pair(f32_LocalX, f32_LocalY));
                    }
                }
            }
        }
    }


    for (s32_ClusterIdx = 0; s32_ClusterIdx < pst_LidarData->s32_ClusterNum; s32_ClusterIdx++)
    {
        HMC_CLUSTER_t *pst_Cluster = &pst_LidarData->arst_Cluster[s32_ClusterIdx];

        f32_LeftMinDist1 = FLT_MAX;
        f32_LeftMinDist2 = FLT_MAX;
        f32_RightMinDist1 = FLT_MAX;
        f32_RightMinDist2 = FLT_MAX;

        // 가장 가까운 LeftRoadBoundary 점 찾기
        for (s32_J = 0; s32_J < st_BoundLeft.size(); s32_J++)
        {
            f32_Dist1 = getDistance2d(pst_Cluster->f32_X1, pst_Cluster->f32_Y1, st_BoundLeft[s32_J].first, st_BoundLeft[s32_J].second);
            f32_Dist2 = getDistance2d(pst_Cluster->f32_X2, pst_Cluster->f32_Y1, st_BoundLeft[s32_J].first, st_BoundLeft[s32_J].second);
            
            if (f32_Dist1 < f32_LeftMinDist1)
            {
                f32_LeftMinDist1 = f32_Dist1;
                s32_LeftNearIdx1 = s32_J;
            }

            if (f32_Dist2 < f32_LeftMinDist2)
            {
                f32_LeftMinDist2 = f32_Dist2;
                s32_LeftNearIdx2 = s32_J;
            }
        }

        // 가장 가까운 RightRoadBoundary 점 찾기
        for (s32_K = 0; s32_K < st_BoundRight.size(); s32_K++)
        {
            f32_Dist3 = getDistance2d(pst_Cluster->f32_X1, pst_Cluster->f32_Y2, st_BoundRight[s32_K].first, st_BoundRight[s32_K].second);
            f32_Dist4 = getDistance2d(pst_Cluster->f32_X2, pst_Cluster->f32_Y2, st_BoundRight[s32_K].first, st_BoundRight[s32_K].second);

            if (f32_Dist3 < f32_RightMinDist1)
            {
                f32_RightMinDist1 = f32_Dist3;
                s32_RightNearIdx1 = s32_K;
            }
            
            if (f32_Dist4 < f32_RightMinDist2)
            {
                f32_RightMinDist2 = f32_Dist4;
                s32_RightNearIdx2 = s32_K;
            }
        }

        if (
            (   
                (
                    pst_Cluster->f32_Y1 > (st_BoundLeft[s32_LeftNearIdx1].second - 0.3f) ||
                    pst_Cluster->f32_Y1 > (st_BoundLeft[s32_LeftNearIdx2].second - 0.3f) ||
                    pst_Cluster->f32_Y2 < (st_BoundRight[s32_RightNearIdx1].second + 0.3f) ||
                    pst_Cluster->f32_Y2 < (st_BoundRight[s32_RightNearIdx2].second + 0.3f)
                )
            )
        )
        {
            pst_Cluster->u8_Flag = c_HMC_CLUSTER_NONE;
        }
    }
}


void HMC_ObjectTracking(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
{
    memcpy(pst_LidarData->arst_Tracking, arst_TrackingBackup, sizeof(HMC_TRACKING_t) * c_TOTAL_TRACKING_NUM);
    pst_LidarData->s32_TrackingNum = s32_TrackingNumBackup;

    if(s32_CanMode == 0)
    {
        f32_HMC_EndTime_s = pst_LidarData->u64_Time_ms / 1000.f;
        f32_HMC_DeltaTime_s = f32_HMC_EndTime_s - f32_HMC_StartTime_s;
        f32_HMC_StartTime_s = pst_LidarData->u64_Time_ms / 1000.f;
    }
    else
    {
        f32_HMC_EndTime_s = getMillisecond() / 1000.f;
        f32_HMC_DeltaTime_s = f32_HMC_EndTime_s - f32_HMC_StartTime_s;
        f32_HMC_StartTime_s = getMillisecond() / 1000.f;
    }

    f32_HMC_CurrYaw_rad = pst_PlanningData->st_EgoVehicleData.f32_Yaw_rad_ENU;
    f32_HMC_DeltaYaw_rad = f32_HMC_CurrYaw_rad - f32_HMC_PrevYaw_rad;
    f32_HMC_PrevYaw_rad = f32_HMC_CurrYaw_rad;

    HMC_SearchNearObject(pst_LidarData);

    HMC_SearchNewObject(pst_LidarData);

    HMC_InitializeTrackingObject(pst_LidarData, pst_PlanningData);

    HMC_MapBoundaryFilterTracking(pst_LidarData, pst_PlanningData);

    HMC_CombineSplitObjects(pst_LidarData);

    // VehicleICP(pst_LidarData);

    HMC_PredictState(pst_LidarData, pst_PlanningData);

    HMC_UpdateMeasurement(pst_LidarData, pst_PlanningData);

    HMC_DetermineDynamicOrStatic(pst_LidarData, pst_PlanningData);

    memcpy(arst_TrackingBackup, pst_LidarData->arst_Tracking, sizeof(HMC_TRACKING_t) * c_TOTAL_TRACKING_NUM);
    s32_TrackingNumBackup = pst_LidarData->s32_TrackingNum;


    for(int32_t s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
    {
        // printf("-----------------\n");
        // printf("idx : %d\n", s32_I);
        // printf("X : %f\n", pst_LidarData->arst_Tracking[s32_I].f32_X);
        // printf("Y : %f\n", pst_LidarData->arst_Tracking[s32_I].f32_Y);
        // printf("Length : %f\n", pst_LidarData->arst_Tracking[s32_I].f32_Length);
        // printf("Width : %f\n", pst_LidarData->arst_Tracking[s32_I].f32_Width);
        // printf("VX (kph) : %f\n", pst_LidarData->arst_Tracking[s32_I].f32_VelocityX_m_s * 3.6f);
        // printf("VY (kph) : %f\n", pst_LidarData->arst_Tracking[s32_I].f32_VelocityY_m_s * 3.6f);
        // printf("GlobalX : %f\n", pst_LidarData->arst_Tracking[s32_I].f32_GlobalX);
        // printf("GlobalY : %f\n", pst_LidarData->arst_Tracking[s32_I].f32_GlobalY);
        // printf("Flag : %d\n", pst_LidarData->arst_Tracking[s32_I].u8_StateFlag);
    }
}



void HMC_SearchNearObject(HMC_LIDAR_DATA_t *pst_LidarData)
{
    int32_t s32_I, s32_J;
    int32_t s32_BestObjIdx;

    float32_t f32_Distance;
    float32_t f32_MinDistance;

    HMC_CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

    for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
    {
        f32_MinDistance = FLT_MAX;

        for(s32_J = 0; s32_J < pst_LidarData->s32_ClusterNum; s32_J++)
        {
            if (pst_Cluster[s32_J].u8_Flag == c_HMC_CLUSTER_NONE)
            {
                continue;
            }

            if(pst_Cluster[s32_J].b_TrackingFlag == true)
            {
                f32_Distance = getDistance2d(pst_Tracking[s32_I].f32_X, pst_Tracking[s32_I].f32_Y, 
                                             pst_Cluster[s32_J].f32_CenterX, pst_Cluster[s32_J].f32_CenterY);

                if(f32_Distance < f32_MinDistance && f32_Distance < st_LidarParam.f32_MaxNearObjectDistance)
                {
                    f32_MinDistance = f32_Distance;
                    s32_BestObjIdx = s32_J;
                    pst_Tracking[s32_I].b_UpdateFlag = true;
                }
            }
        }

        if(pst_Tracking[s32_I].b_UpdateFlag == true)
        {
            pst_Tracking[s32_I].f32_ClusterX = pst_Cluster[s32_BestObjIdx].f32_CenterX;
            pst_Tracking[s32_I].f32_ClusterY = pst_Cluster[s32_BestObjIdx].f32_CenterY;
            pst_Tracking[s32_I].f32_Z = pst_Cluster[s32_BestObjIdx].f32_CenterZ;
            pst_Tracking[s32_I].f32_Length = pst_Cluster[s32_BestObjIdx].f32_X1 - pst_Cluster[s32_BestObjIdx].f32_X2;
            pst_Tracking[s32_I].f32_Width = pst_Cluster[s32_BestObjIdx].f32_Y1 - pst_Cluster[s32_BestObjIdx].f32_Y2;
            pst_Tracking[s32_I].f32_Height = pst_Cluster[s32_BestObjIdx].f32_Z1 - pst_Cluster[s32_BestObjIdx].f32_Z2;
            pst_Tracking[s32_I].f32_MaxLength = fmaxf(pst_Tracking[s32_I].f32_Length, pst_Tracking[s32_I].f32_MaxLength);
            pst_Tracking[s32_I].f32_MaxWidth = fmaxf(pst_Tracking[s32_I].f32_Width, pst_Tracking[s32_I].f32_MaxWidth);
            pst_Tracking[s32_I].f32_MaxHeight = fmaxf(pst_Tracking[s32_I].f32_Height, pst_Tracking[s32_I].f32_MaxHeight);
            memcpy(pst_Tracking[s32_I].ars32_PointIndex, pst_Cluster[s32_BestObjIdx].ars32_PointIndex, sizeof(int32_t) * pst_Cluster[s32_BestObjIdx].s32_PointNum);
            pst_Tracking[s32_I].s32_PointNum = pst_Cluster[s32_BestObjIdx].s32_PointNum;
            pst_Cluster[s32_BestObjIdx].b_TrackingFlag = false;

            for(s32_J = 0; s32_J < pst_LidarData->s32_ClusterNum; s32_J++)
            {
                if (pst_Cluster[s32_J].u8_Flag == c_HMC_CLUSTER_NONE)
                {
                    continue;
                }
                
                if(pst_Cluster[s32_J].b_TrackingFlag == true)
                {
                    f32_Distance = getDistance2d(pst_Cluster[s32_BestObjIdx].f32_CenterX, pst_Cluster[s32_BestObjIdx].f32_CenterY, pst_Cluster[s32_J].f32_CenterX, pst_Cluster[s32_J].f32_CenterY);

                    if(f32_Distance < st_LidarParam.f32_MaxNearNeighborDistance)
                    {
                        pst_Cluster[s32_J].b_TrackingFlag = false;
                    }
                }
            }
        }
        else
        {
            pst_Tracking[s32_I].u32_EraseCnt++;

            if(pst_Tracking[s32_I].u32_EraseCnt > st_LidarParam.u32_MaxEraseCnt)
            {
                HMC_DeleteTrackingObject(pst_LidarData, s32_I);
                s32_I--;
            }
        }
    }
}


void HMC_DeleteTrackingObject(HMC_LIDAR_DATA_t *pst_LidarData, int32_t s32_Idx)
{
    int32_t s32_I;

    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

    for(s32_I = s32_Idx; s32_I < pst_LidarData->s32_TrackingNum - 1; s32_I++)
    {
        // pst_Tracking[s32_I] = pst_Tracking[s32_I + 1];
        memcpy(&pst_Tracking[s32_I], &pst_Tracking[s32_I + 1], sizeof(HMC_TRACKING_t));

    }

    pst_LidarData->s32_TrackingNum--;
}


void HMC_SearchNewObject(HMC_LIDAR_DATA_t *pst_LidarData)
{
    int32_t s32_I, s32_J;
    int32_t s32_NewObjIdx, s32_PrevNewObjIdx;
    
    float32_t f32_Distance;
    float32_t f32_MinDistance;

    bool b_SaveCluObj;

    int32_t s32_NewTrackingNum = 0;
    HMC_CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

    for(s32_I = 0; s32_I < pst_LidarData->s32_ClusterNum; s32_I++)
    {
        if (pst_Cluster[s32_I].u8_Flag == c_HMC_CLUSTER_NONE)
        {
            continue;
        }

        if(pst_Cluster[s32_I].b_TrackingFlag == true)
        {
            f32_MinDistance = FLT_MAX;
            s32_NewObjIdx = -1;

            for(s32_J = 0; s32_J < s32_PrevClusterNum; s32_J++)
            {
                if(arst_PrevCluster[s32_J].u8_Flag == c_HMC_CLUSTER_NONE)
                {
                    continue;
                }

                if(arst_PrevCluster[s32_J].b_TrackingFlag == true)
                {
                    f32_Distance = getDistance2d(pst_Cluster[s32_I].f32_CenterX, pst_Cluster[s32_I].f32_CenterY, 
                                                 arst_PrevCluster[s32_J].f32_CenterX, arst_PrevCluster[s32_J].f32_CenterY);

                    if(f32_Distance < f32_MinDistance && f32_Distance < st_LidarParam.f32_MaxNewObjectDistance)
                    {
                        f32_MinDistance = f32_Distance;
                        s32_NewObjIdx = s32_I;
                        s32_PrevNewObjIdx = s32_J;
                    }
                }
            }


            if (s32_NewObjIdx != -1)
            {
                pst_Cluster[s32_NewObjIdx].b_TrackingFlag = false;
                HMC_SaveTrackingObject(&pst_Tracking[pst_LidarData->s32_TrackingNum], &pst_Cluster[s32_NewObjIdx], &arst_PrevCluster[s32_PrevNewObjIdx]);
                pst_LidarData->s32_TrackingNum += 1;
            }
        }
    }

    HMC_CopyClusterObject(pst_LidarData);
}


void HMC_SaveTrackingObject(HMC_TRACKING_t *pst_Tracking, HMC_CLUSTER_t *pst_Cluster, HMC_CLUSTER_t *pst_PrevCluster)
{
    pst_Tracking->f32_X = pst_Cluster->f32_CenterX;
    pst_Tracking->f32_Y = pst_Cluster->f32_CenterY;
    pst_Tracking->f32_Z = pst_Cluster->f32_CenterZ;

    pst_Tracking->f32_VelocityX_m_s = (pst_Cluster->f32_CenterX - pst_PrevCluster->f32_CenterX) / f32_HMC_DeltaTime_s;
    pst_Tracking->f32_VelocityY_m_s = (pst_Cluster->f32_CenterY - pst_PrevCluster->f32_CenterY) / f32_HMC_DeltaTime_s;

    pst_Tracking->f32_ClusterX = pst_Cluster->f32_CenterX;
    pst_Tracking->f32_ClusterY = pst_Cluster->f32_CenterY;
    pst_Tracking->f32_PrevClusterX = pst_PrevCluster->f32_CenterX;
    pst_Tracking->f32_PrevClusterY = pst_PrevCluster->f32_CenterY;

    pst_Tracking->f32_Length = pst_Cluster->f32_X1 - pst_Cluster->f32_X2;
    pst_Tracking->f32_Width = pst_Cluster->f32_Y1 - pst_Cluster->f32_Y2;
    pst_Tracking->f32_Height = pst_Cluster->f32_Z1 - pst_Cluster->f32_Z2;
    pst_Tracking->f32_PrevLength = pst_PrevCluster->f32_X1 - pst_PrevCluster->f32_X2;
    pst_Tracking->f32_PrevWidth = pst_PrevCluster->f32_Y1 - pst_PrevCluster->f32_Y2;
    pst_Tracking->f32_PrevHeight = pst_PrevCluster->f32_Z1 - pst_PrevCluster->f32_Z2;
    pst_Tracking->f32_MaxLength = fmaxf(pst_Tracking->f32_Length, pst_Tracking->f32_PrevLength);
    pst_Tracking->f32_MaxWidth = fmaxf(pst_Tracking->f32_Width, pst_Tracking->f32_PrevWidth);
    pst_Tracking->f32_MaxHeight = fmaxf(pst_Tracking->f32_Height, pst_Tracking->f32_PrevHeight);

    memcpy(pst_Tracking->ars32_PointIndex, pst_Cluster->ars32_PointIndex, sizeof(int32_t) * pst_Cluster->s32_PointNum);
    pst_Tracking->s32_PointNum = pst_Cluster->s32_PointNum;
    pst_Tracking->b_InitlzFlag = true;
}


void HMC_CopyClusterObject(HMC_LIDAR_DATA_t *pst_LidarData)
{
    int32_t s32_I;

    HMC_CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;

    for(s32_I = 0; s32_I < pst_LidarData->s32_ClusterNum; s32_I++)
    {
        arst_PrevCluster[s32_I] = pst_Cluster[s32_I];
    }

    s32_PrevClusterNum = pst_LidarData->s32_ClusterNum;
}


void HMC_InitializeTrackingObject(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_I;

    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
    {
        if(pst_Tracking[s32_I].b_InitlzFlag == true)
        {
            pst_Tracking[s32_I].arf32_X[0] = pst_Tracking[s32_I].f32_X;
            pst_Tracking[s32_I].arf32_X[1] = pst_Tracking[s32_I].f32_Y;
            pst_Tracking[s32_I].arf32_X[2] = pst_Tracking[s32_I].f32_VelocityX_m_s;
            pst_Tracking[s32_I].arf32_X[3] = pst_Tracking[s32_I].f32_VelocityY_m_s;

            pst_Tracking[s32_I].arf32_P[0] = st_LidarParam.f32_InitialP0;
            pst_Tracking[s32_I].arf32_P[5] = st_LidarParam.f32_InitialP5;
            pst_Tracking[s32_I].arf32_P[10] = st_LidarParam.f32_InitialP10;
            pst_Tracking[s32_I].arf32_P[15] = st_LidarParam.f32_InitialP15;

            pst_Tracking[s32_I].arf32_Q[0] = st_LidarParam.f32_InitialQ0;
            pst_Tracking[s32_I].arf32_Q[5] = st_LidarParam.f32_InitialQ5;
            pst_Tracking[s32_I].arf32_Q[10] = st_LidarParam.f32_InitialQ10;
            pst_Tracking[s32_I].arf32_Q[15] = st_LidarParam.f32_InitialQ15;

            pst_Tracking[s32_I].arf32_R[0] = st_LidarParam.f32_InitialR0;
            pst_Tracking[s32_I].arf32_R[5] = st_LidarParam.f32_InitialR5;
            pst_Tracking[s32_I].arf32_R[10] = st_LidarParam.f32_InitialR10;
            pst_Tracking[s32_I].arf32_R[15] = st_LidarParam.f32_InitialR15;

            HMC_SetGlobalPosition(pst_Tracking, pst_PlanningData, s32_I);

            pst_Tracking[s32_I].f32_PrevGlobalX = pst_Tracking[s32_I].f32_GlobalX;
            pst_Tracking[s32_I].f32_PrevGlobalY = pst_Tracking[s32_I].f32_GlobalY;
        }
    }
}


void HMC_SetGlobalPosition(HMC_TRACKING_t *pst_Tracking, PLANNING_DATA_t *pst_PlanningData, int32_t s32_Idx)
{
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    pst_Tracking[s32_Idx].f32_GlobalX = cos(pst_EgoVehicleData->f32_Yaw_rad_ENU) * pst_Tracking[s32_Idx].arf32_X[0] 
                                        - sin(pst_EgoVehicleData->f32_Yaw_rad_ENU) * pst_Tracking[s32_Idx].arf32_X[1];
    pst_Tracking[s32_Idx].f32_GlobalY = sin(pst_EgoVehicleData->f32_Yaw_rad_ENU) * pst_Tracking[s32_Idx].arf32_X[0]
                                        + cos(pst_EgoVehicleData->f32_Yaw_rad_ENU) * pst_Tracking[s32_Idx].arf32_X[1];

    pst_Tracking[s32_Idx].f32_GlobalX += pst_EgoVehicleData->f32_X_global;
    pst_Tracking[s32_Idx].f32_GlobalY += pst_EgoVehicleData->f32_Y_global;
}


void HMC_PredictState(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_I;

    float32_t f32_Distance;
    float32_t f32_Velocity_m_s;

    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    MatrixXf st_Rotation = MatrixXf(4, 4);
    MatrixXf st_A = MatrixXf(4, 4);

    for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
    {
        if(pst_Tracking[s32_I].b_InitlzFlag == false)
        {
            st_Rotation << cos(-f32_HMC_DeltaYaw_rad), -sin(-f32_HMC_DeltaYaw_rad), 0, 0,
                           sin(-f32_HMC_DeltaYaw_rad),  cos(-f32_HMC_DeltaYaw_rad), 0, 0,
                           0, 0, cos(-f32_HMC_DeltaYaw_rad), -sin(-f32_HMC_DeltaYaw_rad),
                           0, 0, sin(-f32_HMC_DeltaYaw_rad),  cos(-f32_HMC_DeltaYaw_rad);

            memcpy(pst_Tracking[s32_I].arf32_PrevX, pst_Tracking[s32_I].arf32_X, sizeof(pst_Tracking[s32_I].arf32_X));

            pst_Tracking[s32_I].f32_PrevGlobalX = pst_Tracking[s32_I].f32_GlobalX;
            pst_Tracking[s32_I].f32_PrevGlobalY = pst_Tracking[s32_I].f32_GlobalY;

            st_A << 1, 0, f32_HMC_DeltaTime_s, 0,
                    0, 1, 0, f32_HMC_DeltaTime_s,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

            Eigen::Map<Vector4f> st_X(pst_Tracking[s32_I].arf32_X);
            Eigen::Map<Matrix4f> st_P(pst_Tracking[s32_I].arf32_P);
            Eigen::Map<Matrix4f> st_Q(pst_Tracking[s32_I].arf32_Q);

            st_X = st_Rotation * st_X;     

            st_X = st_A * st_X;
  
            st_P = st_A * st_P * st_A.transpose() + st_Q;

            pst_Tracking[s32_I].f32_X = st_X(0);
            pst_Tracking[s32_I].f32_Y = st_X(1);
            pst_Tracking[s32_I].f32_VelocityX_m_s = st_X(2);
            pst_Tracking[s32_I].f32_VelocityY_m_s = st_X(3);

            Eigen::Map<Eigen::Vector4f>(pst_Tracking[s32_I].arf32_X) = st_X;
            Eigen::Map<Eigen::Matrix4f>(pst_Tracking[s32_I].arf32_P) = st_P;
            Eigen::Map<Eigen::Matrix4f>(pst_Tracking[s32_I].arf32_Q) = st_Q;

            HMC_SetGlobalPosition(pst_Tracking, pst_PlanningData, s32_I);

            f32_Distance = getDistance2d(pst_Tracking[s32_I].arf32_X[0], pst_Tracking[s32_I].arf32_X[1], pst_Tracking[s32_I].arf32_PrevX[0], pst_Tracking[s32_I].arf32_PrevX[1]);
            f32_Velocity_m_s = getDistance2d(pst_Tracking[s32_I].arf32_X[2], pst_Tracking[s32_I].arf32_X[3], 0.f, 0.f);

            if(f32_Distance > st_LidarParam.f32_MaxObjectDistance)
            {
                HMC_DeleteTrackingObject(pst_LidarData, s32_I);
                s32_I--;
            }
            else if(f32_Velocity_m_s * 3.6f > st_LidarParam.f32_MaxObjectVelocity_kph)
            {
                HMC_DeleteTrackingObject(pst_LidarData, s32_I);
                s32_I--;
            }
        }
    }
}


void HMC_UpdateMeasurement(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_I;

    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

    VectorXf st_Z = VectorXf(4);
    MatrixXf st_H = Matrix4f::Identity();
    for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
    {
        if(pst_Tracking[s32_I].b_UpdateFlag == true && pst_Tracking[s32_I].b_InitlzFlag == false)
        {
            if(s32_CanMode == 0)
            {
                pst_Tracking[s32_I].f32_EndTime_s = pst_LidarData->u64_Time_ms / 1000.f;
                pst_Tracking[s32_I].f32_DeltaTime_s = pst_Tracking[s32_I].f32_EndTime_s - pst_Tracking[s32_I].f32_StartTime_s;
                pst_Tracking[s32_I].f32_StartTime_s = pst_LidarData->u64_Time_ms / 1000.f;
            }
            else
            {
                pst_Tracking[s32_I].f32_EndTime_s = getMillisecond() / 1000.f;
                pst_Tracking[s32_I].f32_DeltaTime_s = pst_Tracking[s32_I].f32_EndTime_s - pst_Tracking[s32_I].f32_StartTime_s;
                pst_Tracking[s32_I].f32_StartTime_s = getMillisecond() / 1000.f;
            }

            st_Z(0) = pst_Tracking[s32_I].f32_ClusterX;
            st_Z(1) = pst_Tracking[s32_I].f32_ClusterY;
            st_Z(2) = (pst_Tracking[s32_I].f32_ClusterX - pst_Tracking[s32_I].f32_PrevClusterX) / pst_Tracking[s32_I].f32_DeltaTime_s;
            st_Z(3) = (pst_Tracking[s32_I].f32_ClusterY - pst_Tracking[s32_I].f32_PrevClusterY) / pst_Tracking[s32_I].f32_DeltaTime_s;

            // HMC_SetMeasurementVelocity(pst_Tracking, s32_I, st_Z);

            Eigen::Map<Vector4f> st_X(pst_Tracking[s32_I].arf32_X);
            Eigen::Map<Matrix4f> st_K(pst_Tracking[s32_I].arf32_K);
            Eigen::Map<Matrix4f> st_P(pst_Tracking[s32_I].arf32_P);
            Eigen::Map<Matrix4f> st_R(pst_Tracking[s32_I].arf32_R);

            st_K = st_P * st_H.transpose() * (st_H * st_P * st_H.transpose() + st_R).inverse();

            st_P = st_P - st_K * st_H * st_P;

            st_X = st_X + st_K * (st_Z - st_H * st_X);

            pst_Tracking[s32_I].f32_X = st_X(0);
            pst_Tracking[s32_I].f32_Y = st_X(1);
            pst_Tracking[s32_I].f32_VelocityX_m_s = st_X(2);
            pst_Tracking[s32_I].f32_VelocityY_m_s = st_X(3);

            Eigen::Map<Eigen::Vector4f>(pst_Tracking[s32_I].arf32_X) = st_X;
            Eigen::Map<Eigen::Matrix4f>(pst_Tracking[s32_I].arf32_K) = st_K;
            Eigen::Map<Eigen::Matrix4f>(pst_Tracking[s32_I].arf32_P) = st_P;
            Eigen::Map<Eigen::Matrix4f>(pst_Tracking[s32_I].arf32_R) = st_R;

            pst_Tracking[s32_I].f32_PrevClusterX = pst_Tracking[s32_I].f32_ClusterX;
            pst_Tracking[s32_I].f32_PrevClusterY = pst_Tracking[s32_I].f32_ClusterY;
            pst_Tracking[s32_I].f32_PrevLength = pst_Tracking[s32_I].f32_Length;
            pst_Tracking[s32_I].f32_PrevWidth = pst_Tracking[s32_I].f32_Width;
            pst_Tracking[s32_I].f32_PrevHeight = pst_Tracking[s32_I].f32_Height;

            HMC_SetGlobalPosition(pst_Tracking, pst_PlanningData, s32_I);

            pst_Tracking[s32_I].u32_EraseCnt = 0;
        }
        else
        {
            pst_Tracking[s32_I].b_InitlzFlag = false; // for new object
            pst_Tracking[s32_I].f32_StartTime_s = getMillisecond() / 1000.f;
        }

        pst_Tracking[s32_I].b_UpdateFlag = false;
    }
}


void HMC_SetMeasurementVelocity(HMC_TRACKING_t *pst_Tracking, int32_t s32_Idx, VectorXf &st_Z)
{
    POINT_t arst_CurrentObject[4] = {};
    POINT_t arst_PrevObject[4] = {};

    float32_t f32_MaxX = pst_Tracking[s32_Idx].f32_X + (pst_Tracking[s32_Idx].f32_Length / 2.f);
    float32_t f32_MaxY = pst_Tracking[s32_Idx].f32_Y + (pst_Tracking[s32_Idx].f32_Width / 2.f);
    float32_t f32_MinX = pst_Tracking[s32_Idx].f32_X - (pst_Tracking[s32_Idx].f32_Length / 2.f);
    float32_t f32_MinY = pst_Tracking[s32_Idx].f32_Y - (pst_Tracking[s32_Idx].f32_Width / 2.f);
    float32_t f32_PrevMaxX = pst_Tracking[s32_Idx].f32_X + (pst_Tracking[s32_Idx].f32_PrevLength / 2.f);
    float32_t f32_PrevMaxY = pst_Tracking[s32_Idx].f32_Y + (pst_Tracking[s32_Idx].f32_PrevWidth / 2.f);
    float32_t f32_PrevMinX = pst_Tracking[s32_Idx].f32_X - (pst_Tracking[s32_Idx].f32_PrevLength / 2.f);
    float32_t f32_PrevMinY = pst_Tracking[s32_Idx].f32_Y - (pst_Tracking[s32_Idx].f32_PrevWidth / 2.f);

    arst_CurrentObject[0].f32_X = f32_MaxX;
    arst_CurrentObject[0].f32_Y = f32_MaxY;
    arst_CurrentObject[1].f32_X = f32_MaxX;
    arst_CurrentObject[1].f32_Y = f32_MinY;
    arst_CurrentObject[2].f32_X = f32_MinX;
    arst_CurrentObject[2].f32_Y = f32_MaxY;
    arst_CurrentObject[3].f32_X = f32_MinX;
    arst_CurrentObject[3].f32_Y = f32_MinY;

    arst_PrevObject[0].f32_X = f32_PrevMaxX;
    arst_PrevObject[0].f32_Y = f32_PrevMaxY;
    arst_PrevObject[1].f32_X = f32_PrevMaxX;
    arst_PrevObject[1].f32_Y = f32_PrevMinY;
    arst_PrevObject[2].f32_X = f32_PrevMinX;
    arst_PrevObject[2].f32_Y = f32_PrevMaxY;
    arst_PrevObject[3].f32_X = f32_PrevMinX;
    arst_PrevObject[3].f32_Y = f32_PrevMinY;

    int32_t s32_J;
    int32_t s32_PointIdx;

    float32_t f32_Distance;
    float32_t f32_MinDistance = FLT_MAX;

    for(s32_J = 0; s32_J < 4; s32_J++)
    {
        f32_Distance = getDistance2d(0.f, 0.f, arst_CurrentObject[s32_J].f32_X, arst_CurrentObject[s32_J].f32_Y);

        if(f32_Distance < f32_MinDistance)
        {
            f32_MinDistance = f32_Distance;
            s32_PointIdx = s32_J;
        }
    }

    float32_t f32_OffsetX = arst_CurrentObject[s32_PointIdx].f32_X - arst_PrevObject[s32_PointIdx].f32_X;
    float32_t f32_OffsetY = arst_CurrentObject[s32_PointIdx].f32_Y - arst_PrevObject[s32_PointIdx].f32_Y;

    float32_t f32_DeltaL = pst_Tracking[s32_Idx].f32_ClusterX - (pst_Tracking[s32_Idx].f32_PrevClusterX + f32_OffsetX);
    float32_t f32_DeltaW = pst_Tracking[s32_Idx].f32_ClusterY - (pst_Tracking[s32_Idx].f32_PrevClusterY + f32_OffsetY);

    st_Z(0) = pst_Tracking[s32_Idx].f32_ClusterX;
    st_Z(1) = pst_Tracking[s32_Idx].f32_ClusterY;
    st_Z(2) = ((pst_Tracking[s32_Idx].f32_ClusterX - f32_DeltaL) - pst_Tracking[s32_Idx].f32_PrevClusterX) / pst_Tracking[s32_Idx].f32_DeltaTime_s;
    st_Z(3) = ((pst_Tracking[s32_Idx].f32_ClusterY - f32_DeltaW) - pst_Tracking[s32_Idx].f32_PrevClusterY) / pst_Tracking[s32_Idx].f32_DeltaTime_s;
}


void HMC_CombineSplitObjects(HMC_LIDAR_DATA_t *pst_LidarData)
{
    int32_t s32_I, s32_J;

    float32_t f32_SizeI, f32_SizeJ;
    float32_t f32_MaxX1, f32_MaxY1, f32_MinX1, f32_MinY1;
    float32_t f32_MaxX2, f32_MaxY2, f32_MinX2, f32_MinY2;
    float32_t f32_InterMaxX, f32_InterMinX, f32_InterMaxY, f32_InterMinY;
    float32_t f32_IntersctionArea;
    float32_t f32_IoU;

    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

    for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum - 1; s32_I++)
    {
        f32_SizeI = pst_Tracking[s32_I].f32_Length * pst_Tracking[s32_I].f32_Width;

        for(s32_J = s32_I + 1; s32_J < pst_LidarData->s32_TrackingNum; s32_J++)
        {
            f32_SizeJ = pst_Tracking[s32_J].f32_Length * pst_Tracking[s32_J].f32_Width;

            if(f32_SizeI < f32_SizeJ)
            {
                continue;
            }

            f32_MaxX1 = pst_Tracking[s32_I].f32_X + (pst_Tracking[s32_I].f32_Length / 2.f);
            f32_MaxY1 = pst_Tracking[s32_I].f32_Y + (pst_Tracking[s32_I].f32_Width / 2.f);
            f32_MinX1 = pst_Tracking[s32_I].f32_X - (pst_Tracking[s32_I].f32_Length / 2.f);
            f32_MinY1 = pst_Tracking[s32_I].f32_Y - (pst_Tracking[s32_I].f32_Width / 2.f);
            f32_MaxX2 = pst_Tracking[s32_J].f32_X + (pst_Tracking[s32_J].f32_Length / 2.f);
            f32_MaxY2 = pst_Tracking[s32_J].f32_Y + (pst_Tracking[s32_J].f32_Width / 2.f);
            f32_MinX2 = pst_Tracking[s32_J].f32_X - (pst_Tracking[s32_J].f32_Length / 2.f);
            f32_MinY2 = pst_Tracking[s32_J].f32_Y - (pst_Tracking[s32_J].f32_Width / 2.f);

            f32_InterMaxX = fminf(f32_MaxX1, f32_MaxX2);
            f32_InterMaxY = fminf(f32_MaxY1, f32_MaxY2);
            f32_InterMinX = fmaxf(f32_MinX1, f32_MinX2);
            f32_InterMinY = fmaxf(f32_MinY1, f32_MinY2);

            if(f32_InterMinX < f32_InterMaxX && f32_InterMinY < f32_InterMaxY)
            {
                f32_IntersctionArea = (f32_InterMaxX - f32_InterMinX) * (f32_InterMaxY - f32_InterMinY);
            }
            else
            {
                f32_IntersctionArea = 0;
            }

            f32_IoU = f32_IntersctionArea / f32_SizeJ;

            if(f32_IoU > st_LidarParam.f32_MinIoU)
            {
                HMC_DeleteTrackingObject(pst_LidarData, s32_J);
                s32_J--;
            }
        }
    }
}


void HMC_MapBoundaryFilterTracking(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_I;
    int32_t s32_Cross;

    float32_t f32_TrackingGlobalX, f32_TrackingGlobalY;
    float32_t f32_MinDistance;

    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    PATH_t *pst_RefBoundary = &pst_PlanningData->st_BoundaryInOut;
    PATH_t *pst_PitBoundary = &pst_PlanningData->st_BoundaryIn2;

    for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
    {
        getGlobalCoord(pst_EgoVehicleData->f32_X_global, pst_EgoVehicleData->f32_Y_global, pst_EgoVehicleData->f32_Yaw_rad_ENU,
                       pst_Tracking[s32_I].f32_X, pst_Tracking[s32_I].f32_Y, f32_TrackingGlobalX, f32_TrackingGlobalY);

        f32_MinDistance = FLT_MAX;
        s32_Cross = 0;

        HMC_CheckMapBoundary(pst_RefBoundary, f32_TrackingGlobalX, f32_TrackingGlobalY, s32_Cross, f32_MinDistance);

        if(s32_Cross % 2 == 0)
        {
            HMC_DeleteTrackingObject(pst_LidarData, s32_I);
            s32_I--;
        }
        else if(f32_MinDistance < st_LidarParam.f32_MapDistanceOffset)
        {
            HMC_DeleteTrackingObject(pst_LidarData, s32_I);
            s32_I--;
        }
        else
        {
            s32_Cross = 0;

            HMC_CheckMapBoundary(pst_PitBoundary, f32_TrackingGlobalX, f32_TrackingGlobalY, s32_Cross, f32_MinDistance);

            if(s32_Cross % 2 != 0)
            {
                HMC_DeleteTrackingObject(pst_LidarData, s32_I);
                s32_I--;
            }
            else if(f32_MinDistance < st_LidarParam.f32_MapDistanceOffset)
            {
                HMC_DeleteTrackingObject(pst_LidarData, s32_I);
                s32_I--;
            }
        }
    }
}


void HMC_DetermineDynamicOrStatic(HMC_LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_I;

    float32_t f32_AbsoluteVelocityX_m_s, f32_AbsoluteVelocityY_m_s, f32_AbsoluteSpeed_m_s;

    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

    for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
    {
        if(isnan(pst_Tracking[s32_I].f32_X) || isnan(pst_Tracking[s32_I].f32_Y))
        {
            HMC_DeleteTrackingObject(pst_LidarData, s32_I);
            s32_I--;
            continue;
        }

        pst_Tracking[s32_I].f32_Yaw_rad_ENU = atan2(pst_Tracking[s32_I].f32_GlobalY - pst_Tracking[s32_I].f32_PrevGlobalY,
                                                    pst_Tracking[s32_I].f32_GlobalX - pst_Tracking[s32_I].f32_PrevGlobalX);

        f32_AbsoluteVelocityX_m_s = pst_Tracking[s32_I].f32_VelocityX_m_s + pst_EgoVehicleData->f32_VelocityX_m_s_local;
        f32_AbsoluteVelocityY_m_s = pst_Tracking[s32_I].f32_VelocityY_m_s - pst_EgoVehicleData->f32_VelocityY_m_s_local
                                    * log2f(abs(pst_Tracking[s32_I].f32_X)) * (pst_Tracking[s32_I].f32_X / abs(pst_Tracking[s32_I].f32_X));

        f32_AbsoluteSpeed_m_s = getDistance2d(f32_AbsoluteVelocityX_m_s, f32_AbsoluteVelocityY_m_s, 0.f, 0.f);

        pst_Tracking[s32_I].f32_AbsoluteVelocityX_m_s = f32_AbsoluteVelocityX_m_s;
        pst_Tracking[s32_I].f32_AbsoluteVelocityY_m_s = f32_AbsoluteVelocityY_m_s;

        // printf("IDX : %d\n", s32_I);
        // printf("Object Position : %f, %f\n", pst_Tracking[s32_I].f32_X, pst_Tracking[s32_I].f32_Y);
        // printf("RelativeVelocity : %f, %f\n", pst_Tracking[s32_I].f32_VelocityX_m_s, pst_Tracking[s32_I].f32_VelocityY_m_s);
        // printf("EgoVelocity : %f, %f\n", pst_EgoVehicleData->f32_VelocityX_m_s_local, pst_EgoVehicleData->f32_VelocityY_m_s_local);
        // printf("AbsoluteVelocity : %f, %f\n\n", f32_AbsoluteVelocityX_m_s, f32_AbsoluteVelocityY_m_s);

        if(f32_AbsoluteSpeed_m_s * 3.6 > st_LidarParam.f32_MaxStaticVelocity_kph)
        {
            pst_Tracking[s32_I].u8_StateFlag = c_HMC_OBJECT_DYNAMIC;
            pst_Tracking[s32_I].f32_Yaw_rad = atan2(f32_AbsoluteVelocityY_m_s, f32_AbsoluteVelocityX_m_s);
        }
        else
        {
            pst_Tracking[s32_I].u8_StateFlag = c_HMC_OBJECT_STATIC;
            pst_Tracking[s32_I].f32_Yaw_rad = 0;
        }
    }
}


void VehicleICP(HMC_LIDAR_DATA_t *pst_LidarData)
{
    int32_t s32_I, s32_J;
    int32_t s32_PointIdx;
    int32_t s32_PointNum;

    float32_t f32_Score;
    float32_t f32_X, f32_Y;

    ICP_POINT_CLOUD_t st_ObjectPointCloud;
    HMC_TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;
    HMC_POINT_t *pst_Point = pst_LidarData->arst_Point;

    for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
    {
        if(pst_Tracking[s32_I].b_UpdateFlag == false || pst_Tracking[s32_I].b_InitlzFlag == true)
        {
            continue;
        }

        st_ObjectPointCloud.f32_X = pst_Tracking[s32_I].f32_ClusterX;
        st_ObjectPointCloud.f32_Y = pst_Tracking[s32_I].f32_ClusterY;
        st_ObjectPointCloud.f32_Z = pst_Tracking[s32_I].f32_Z;
        st_ObjectPointCloud.f32_Yaw_deg = atan2(pst_Tracking[s32_I].f32_AbsoluteVelocityY_m_s, pst_Tracking[s32_I].f32_AbsoluteVelocityX_m_s); // rad

        s32_PointNum = 0;

        for(s32_J = 0; s32_J < pst_Tracking[s32_I].s32_PointNum; s32_J++)
        {
            s32_PointIdx = pst_Tracking[s32_I].ars32_PointIndex[s32_J];

            st_ObjectPointCloud.arf32_X[s32_PointNum] = pst_Point[s32_PointIdx].f32_X;
            st_ObjectPointCloud.arf32_Y[s32_PointNum] = pst_Point[s32_PointIdx].f32_Y;
            st_ObjectPointCloud.arf32_Z[s32_PointNum] = pst_Point[s32_PointIdx].f32_Z;

            s32_PointNum++;
        }

        st_ObjectPointCloud.s32_PointNum = s32_PointNum;

        f32_Score = CUDA_VehicleICP(&st_ObjectPointCloud, &st_LidarParam.st_VehiclePointCloud, 50);
        // printf("f32_Score : %f\n", f32_Score);
        pst_Tracking[s32_I].f32_Score = f32_Score;

        if(f32_Score < 0.15f)
        {
            f32_X = cos(st_ObjectPointCloud.f32_Yaw_deg) * st_ObjectPointCloud.f32_X - sin(st_ObjectPointCloud.f32_Yaw_deg) * st_ObjectPointCloud.f32_Y;
            f32_Y = sin(st_ObjectPointCloud.f32_Yaw_deg) * st_ObjectPointCloud.f32_X + cos(st_ObjectPointCloud.f32_Yaw_deg) * st_ObjectPointCloud.f32_Y;

            pst_Tracking[s32_I].f32_ClusterX -= f32_X;
            pst_Tracking[s32_I].f32_ClusterY -= f32_Y;
            pst_Tracking[s32_I].f32_Z -= st_ObjectPointCloud.f32_Z;
            pst_Tracking[s32_I].f32_Length = 4.6f;
            pst_Tracking[s32_I].f32_Width = 2.f;
            pst_Tracking[s32_I].f32_Height = 1.5f;
            pst_Tracking[s32_I].f32_Yaw_rad = st_ObjectPointCloud.f32_Yaw_deg; // rad
        }
    }
}
