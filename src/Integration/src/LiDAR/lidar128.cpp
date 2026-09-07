// #include "lidar128.h"
// #include <vector>
// #include "localization.h"

// extern SENSOR_DATA_t st_SensorData;
// float32_t f32_StartTime_s = getMillisecond() / 1000.f;
// float32_t f32_EndTime_s;
// float32_t f32_DeltaTime_s;

// void LIDARProcessingVelodyne128(RAW_LIDAR_DATA_t *pst_RawData, LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, OMNI_POINT_t *pst_OmniPoint)
// {
//     InitLidarDataStructure(pst_LidarData);

//     ParsingVelodyne128(pst_RawData, pst_LidarData);

//     // // Ground Extraction
//     // // ExtractGroundVeloyne128(pst_LidarData);
//     RemoveGroundVelodyne128(pst_LidarData);

//     LidarLocalization(pst_LidarData, pst_DeadReckoningData, pst_OmniPoint);

//     // // // // Voxelization
//     VoxelizePointCloud(pst_LidarData);
    
//     // // // // Clustering
//     ClusterPointCloud(pst_LidarData);
    
//     // // // Classification, LShape
//     // ClassifyPointCloud(pst_LidarData, pst_PlanningData);

//     // // Object Tracking
//     ObjectTracking(pst_LidarData, pst_PlanningData);
// }


// void LIDARProcessingOuster128(RAW_LIDAR_DATA_t *pst_RawData, LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, OMNI_POINT_t *pst_OmniPoint)
// {
//     ParsingOuster128(pst_RawData, pst_LidarData);

//     // Ground Extraction
//     // ExtractGroundOuster128(pst_LidarData);

//     // Voxelization
//     // VoxelizePointCloud(pst_LidarData);
    
//     // Clustering
//     // ClusterPointCloud(pst_LidarData);

//     // L-Shape

//     // Object Tracking..
//     ObjectTracking(pst_LidarData, pst_PlanningData);
// }

// void InitLidarDataStructure(LIDAR_DATA_t *pst_LidarData)
// {
//     memset(pst_LidarData->arst_Point, 0, sizeof(POINT_t) * c_TOTAL_POINT_NUM);
//     memset(pst_LidarData->arst_RoadPoint, 0, sizeof(POINT_t) * 22230);
//     // memset(&pst_LidarData->st_Voxel, 0, sizeof(VOXEL_t));
//     // memset(pst_LidarData->arst_Cluster, 0, sizeof(CLUSTER_t) * c_TOTAL_CLUSTER_NUM);
//     // memset(pst_LidarData->arst_PrevCluster, 0, sizeof(CLUSTER_t) * c_TOTAL_CLUSTER_NUM);
//     memset(&pst_LidarData->st_OmniFeature, 0, sizeof(OMNI_FEATURE_t));

//     pst_LidarData->s32_PointNum = 0;
//     pst_LidarData->s32_RoadPointNum = 0;
//     pst_LidarData->s32_ClusterNum = 0;
//     pst_LidarData->s32_PrevClusterNum = 0;
//     pst_LidarData->s32_TrackingNum = 0;
    
//     pst_LidarData->f32_LiDARRoll_deg = 0;
//     pst_LidarData->f32_LiDARPitch_deg = 0;
//     pst_LidarData->f32_LiDARYaw_deg = 0;

//     pst_LidarData->f32_LiDARRoll_rad = 0;
//     pst_LidarData->f32_LiDARPitch_rad = 0;
//     pst_LidarData->f32_LiDARYaw_rad = 0;

//     pst_LidarData->f32_PredictX = 0;
//     pst_LidarData->f32_PredictY = 0;
//     pst_LidarData->f32_PredictYaw_deg = 0;
    
// }




// void ParsingVelodyne128(RAW_LIDAR_DATA_t *pst_RawData, LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I, s32_J, s32_K;
//     int32_t s32_PointNum = 0;
//     int32_t s32_PointOffset = 0;
//     int32_t s32_OrderedPointNum = 0;
//     int32_t s32_ChannelIdx;

//     uint8_t u8_Intensity, u8_Channel;
//     float32_t f32_Azimuth;
//     float32_t f32_Azimuth_deg;
//     float32_t f32_Elevation_deg;
//     int32_t s32_AzimuthBufferOffset, s32_DistanceBufferOffset;
//     float32_t f32_Distance;

//     float32_t f32_Omega;
//     float32_t f32_Alpha;
//     float32_t f32_Roll;
//     float32_t f32_Pitch;
//     float32_t f32_Yaw;

//     float32_t f32_X, f32_Y, f32_Z;
//     float32_t f32_RX, f32_RY, f32_RZ;
    
//     uint8_t u8_LaserOrder;
//     int32_t s32_PointNumOffset;
//     memcpy(&f32_Roll, &st_SensorData.st_RawIMU.arc_Buffer[52], sizeof(float));
//     memcpy(&f32_Pitch, &st_SensorData.st_RawIMU.arc_Buffer[56], sizeof(float));
//     memcpy(&f32_Yaw, &st_SensorData.st_RawIMU.arc_Buffer[60], sizeof(float));


//     for (s32_I = 0; s32_I < pst_RawData->s32_Num; s32_I += 1200)
//     {
//         for (s32_J = 0; s32_J < 1200; s32_J += 100)
//         {
//             s32_AzimuthBufferOffset = s32_I + s32_J;
//             f32_Azimuth = (float32_t)(((uint8_t)pst_RawData->arc_Buffer[s32_AzimuthBufferOffset + 3]) << 8 | 
//                         (uint8_t)pst_RawData->arc_Buffer[s32_AzimuthBufferOffset + 2]) * 0.01f;
            
//             for (s32_K = s32_J + 4; s32_K < s32_J + 100; s32_K += 3)
//             {
//                 s32_DistanceBufferOffset = s32_I + s32_K;
//                 f32_Distance = (((uint8_t)pst_RawData->arc_Buffer[s32_DistanceBufferOffset + 1]) << 8 | 
//                                 ((uint8_t)pst_RawData->arc_Buffer[s32_DistanceBufferOffset])) * 0.004f;
//                 u8_Intensity = (uint8_t)pst_RawData->arc_Buffer[s32_DistanceBufferOffset + 2];
                
//                 u8_LaserOrder = (s32_PointNum & 127);

//                 f32_Azimuth_deg = f32_Azimuth - arf32_AzimuthOffset_VLS128[u8_LaserOrder];
//                 if(f32_Azimuth_deg < 0.f) f32_Azimuth_deg += 360.f;
//                 else if(f32_Azimuth_deg > 360.f) f32_Azimuth_deg -= 360.f;
                
//                 f32_Elevation_deg = arf32_ElevationDeg_VLS128[u8_LaserOrder];
//                 u8_Channel = aru8_ChannelMappingTable_VLS128[u8_LaserOrder];
//                 f32_Omega = arf32_ElevationRad_VLS128[u8_Channel] * 0.000001f;

//                 f32_Alpha = f32_Azimuth_deg * 0.01745329251f;   // deg to rad
                
//                 f32_X = f32_Distance * arf32_CosineOmega[u8_Channel] * sinf(f32_Alpha);
//                 f32_Y = f32_Distance * arf32_CosineOmega[u8_Channel] * cosf(f32_Alpha);
//                 f32_Z = f32_Distance * arf32_SineOmega[u8_Channel];

//                 s32_OrderedPointNum = ((s32_PointOffset - ars32_AzimuthIndexOffset_VLS128[u8_LaserOrder] + u8_Channel) + 230400) % 230400;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].f32_X = f32_X - 1.f;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].f32_Y = f32_Y;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].f32_Z = f32_Z - 0.45f;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].f32_Distance = f32_Distance;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].f32_Azimuth_deg = f32_Azimuth_deg;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].f32_Azimuth_rad = f32_Alpha;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].f32_Elevation_deg = f32_Elevation_deg;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].f32_Elevation_rad = f32_Omega;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].u8_Intensity = u8_Intensity;
//                 pst_LidarData->arst_Point[s32_OrderedPointNum].u8_Channel = u8_Channel;

//                 if (f32_Distance < 0.5f || f32_Distance > c_VOXEL_DISTANCE_MAX || f32_Elevation_deg > c_VOXEL_ELEVATION_MAX || f32_Elevation_deg < c_VOXEL_ELEVATION_MIN)
//                 {
//                     pst_LidarData->arst_Point[s32_OrderedPointNum].u8_Flag = c_POINT_INVALID;
//                 }
//                 else
//                 {
//                     pst_LidarData->arst_Point[s32_OrderedPointNum].u8_Flag = c_POINT_VALID;
//                 }

//             ++s32_PointNum;
//             if((s32_PointNum & 127) == 0) s32_PointOffset+=128;
//             }
//         }
//     }

//     pst_LidarData->u64_Timestamp = pst_RawData->u64_Timestamp;
//     pst_LidarData->s32_PointNum = s32_PointNum;

//     float32_t f32_matrix[3][3];
//     rotationMatrix(-f32_Pitch, f32_Roll, 0, f32_matrix);

//     for(s32_I = 0; s32_I < pst_LidarData->s32_PointNum; s32_I += 1)
//     {
//         if(pst_LidarData->arst_Point[s32_I].u8_Flag == c_POINT_VALID)
//         {
//             f32_X = pst_LidarData->arst_Point[s32_I].f32_X;
//             f32_Y = pst_LidarData->arst_Point[s32_I].f32_Y;
//             f32_Z = pst_LidarData->arst_Point[s32_I].f32_Z;

//             f32_RX = f32_matrix[0][0] * f32_X + f32_matrix[0][1] * f32_Y + f32_matrix[0][2] * f32_Z;
//             f32_RY = f32_matrix[1][0] * f32_X + f32_matrix[1][1] * f32_Y + f32_matrix[1][2] * f32_Z;
//             f32_RZ = f32_matrix[2][0] * f32_X + f32_matrix[2][1] * f32_Y + f32_matrix[2][2] * f32_Z;

//             // pst_LidarData->arst_Point[s32_I].f32_Z, -Pitch, Roll, 0, f32_RX, f32_RY, f32_RZ);
//             // printf("%f %f %f\n",f32_RX, f32_RY, f32_RZ);
//             pst_LidarData->arst_Point[s32_I].f32_X = f32_RX + 1.f;
//             pst_LidarData->arst_Point[s32_I].f32_Y = f32_RY;
//             pst_LidarData->arst_Point[s32_I].f32_Z = f32_RZ + 0.45f;

//         }

//     }
// }


// void ParsingOuster128(RAW_LIDAR_DATA_t *pst_RawData, LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I, s32_J, s32_K;
//     int32_t s32_PointNum = 0;
//     int32_t s32_TempPointNum = 0;

//     uint8_t u8_Intensity;
//     float32_t f32_MeasurementId;

//     float32_t f32_Distance;
//     float32_t f32_Encoder_rad;
//     float32_t f32_FinalAzimuth_rad;
//     float32_t f32_FinalAzimuth_deg;
//     float32_t f32_AzimuthOffset_rad;
//     float32_t f32_Phi_rad;
//     float32_t f32_X, f32_Y, f32_Z;
//     float32_t f32_RX, f32_RY, f32_RZ;
    
//     float32_t f32_Roll_deg = 0;
//     float32_t f32_Pitch_deg = 0;
//     float32_t f32_Yaw_deg = 0;

//     float32_t f32_Roll_rad = 0;
//     float32_t f32_Pitch_rad = 0;
//     float32_t f32_Yaw_rad = 0;

//     memcpy(&f32_Roll_deg, pst_RawData->arc_Buffer, sizeof(float32_t));
//     memcpy(&f32_Pitch_deg, pst_RawData->arc_Buffer + 4, sizeof(float32_t));
//     memcpy(&f32_Yaw_deg, pst_RawData->arc_Buffer + 8, sizeof(float32_t));
//     // printf("roll : %f \n", f32_Roll_deg);
//     // printf("pitch : %f \n", f32_Pitch_deg);
//     pst_LidarData->f32_LiDARRoll_deg = f32_Roll_deg;
//     pst_LidarData->f32_LiDARPitch_deg = f32_Pitch_deg;
//     pst_LidarData->f32_LiDARYaw_deg = f32_Yaw_deg;

//     pst_LidarData->f32_LiDARRoll_deg = deg2rad(f32_Roll_deg);
//     pst_LidarData->f32_LiDARPitch_deg = deg2rad(f32_Pitch_deg);
//     pst_LidarData->f32_LiDARYaw_deg = deg2rad(f32_Yaw_deg);

//     POINT_t *pst_Point = pst_LidarData->arst_Point;
//     for (s32_I = 16; s32_I < pst_RawData->s32_Num; s32_I += 24800)
//     {
//         for(s32_J = 32; s32_J < 24800; s32_J += 1548)
//         {
            
//             // if(pst_RawData->arc_Buffer[s32_I + s32_J -2]&0b00000001 == false) continue; //column status

//             f32_MeasurementId = (float32_t)((uint8_t)(pst_RawData->arc_Buffer[s32_I + s32_J +9]) << 8 | (uint8_t)(pst_RawData->arc_Buffer[s32_I + s32_J +8]));
//             f32_Encoder_rad = 2*M_PI*(1 - (f32_MeasurementId / 1024.f));
//             for(s32_K = 0; s32_K < 1536; s32_K += 12)
//             {
//                 f32_Distance = (float32_t)(((uint8_t)pst_RawData->arc_Buffer[s32_I + s32_J+s32_K+14] & 0b00000111) << 16 |
//                                 (uint8_t)pst_RawData->arc_Buffer[s32_I + s32_J+s32_K+13] << 8 | (uint8_t)pst_RawData->arc_Buffer[s32_I + s32_J+s32_K + 12]);

//                 u8_Intensity = pst_RawData->arc_Buffer[s32_I + s32_J+s32_K];

//                 f32_Distance = (f32_Distance - f32_BeamToLidar_OS2)* 0.001f;
//                 f32_Phi_rad = deg2rad(arf32_Elevation_OS2[s32_K/12]);

//                 f32_AzimuthOffset_rad = -deg2rad(arf32_AzimuthOffset_OS2[s32_K/12]);

//                 f32_FinalAzimuth_rad = f32_Encoder_rad + f32_AzimuthOffset_rad;
//                 f32_FinalAzimuth_deg = rad2deg(f32_FinalAzimuth_rad);
//                 printf("azi : %f\n", f32_FinalAzimuth_deg);


//                 if (f32_FinalAzimuth_deg >= 360.f)
//                 {
//                     f32_FinalAzimuth_deg = f32_FinalAzimuth_deg - 360.f;
//                     f32_FinalAzimuth_rad = deg2rad(f32_FinalAzimuth_deg);
//                 }
//                 else if (f32_FinalAzimuth_deg < 0.f)
//                 {
//                     f32_FinalAzimuth_deg = 360.f + f32_FinalAzimuth_deg;
//                     f32_FinalAzimuth_rad = deg2rad(f32_FinalAzimuth_deg);
//                 }
//                 // printf("azi : %f\n", f32_FinalAzimuth_deg);

//                 f32_X =  f32_Distance * cos(f32_FinalAzimuth_rad) * cos(f32_Phi_rad) +
//                          f32_Transform03_OS2 * cos(f32_Encoder_rad) ;


//                 f32_Y =  f32_Distance * sin(f32_FinalAzimuth_rad) * cos(f32_Phi_rad) +
//                          f32_Transform03_OS2 * sin(f32_Encoder_rad);


//                 f32_Z =  f32_Distance * sin(f32_Phi_rad) + f32_Transform23_OS2;


//                 s32_TempPointNum = (s32_PointNum + ars32_IndexArray_OS2[s32_K/12] + 131072) % 131072;

//                 pst_Point[s32_TempPointNum].f32_X = f32_X;
//                 pst_Point[s32_TempPointNum].f32_Y = f32_Y;
//                 pst_Point[s32_TempPointNum].f32_Z = f32_Z;
//                 pst_Point[s32_TempPointNum].f32_Azimuth_deg = f32_FinalAzimuth_deg;
//                 pst_Point[s32_TempPointNum].f32_Azimuth_rad = f32_FinalAzimuth_rad;
//                 pst_Point[s32_TempPointNum].f32_Elevation_deg = rad2deg(f32_Phi_rad);
//                 pst_Point[s32_TempPointNum].f32_Elevation_rad = f32_Phi_rad;
//                 pst_Point[s32_TempPointNum].f32_Distance = f32_Distance;
//                 pst_Point[s32_TempPointNum].u8_Intensity = u8_Intensity;
//                 // printf("azimuth : %f \n", pst_Point[s32_PointNum].f32_Azimuth_deg);
//                 if (f32_Distance < 0.5f)
//                 {
//                     pst_Point[s32_TempPointNum].u8_Flag = c_POINT_INVALID;
//                 }
//                 else
//                 {
//                     pst_Point[s32_TempPointNum].u8_Flag = c_POINT_VALID;
//                 }

                
//             }
//             s32_PointNum += 128;

//         }
//     }
//     pst_LidarData->u64_Timestamp = pst_RawData->u64_Timestamp;
//     pst_LidarData->s32_PointNum = s32_PointNum;

//     float32_t f32_matrix[3][3];
//     rotationMatrix(-f32_Roll_deg, -f32_Pitch_deg, 0, f32_matrix);

//     for(s32_I = 0; s32_I < pst_LidarData->s32_PointNum; s32_I += 1)
//     {
//         if(pst_LidarData->arst_Point[s32_I].u8_Flag == c_POINT_VALID)
//         {
//             f32_X = pst_LidarData->arst_Point[s32_I].f32_X;
//             f32_Y = pst_LidarData->arst_Point[s32_I].f32_Y;
//             f32_Z = pst_LidarData->arst_Point[s32_I].f32_Z;

//             f32_RX = f32_matrix[0][0] * f32_X + f32_matrix[0][1] * f32_Y + f32_matrix[0][2] * f32_Z;
//             f32_RY = f32_matrix[1][0] * f32_X + f32_matrix[1][1] * f32_Y + f32_matrix[1][2] * f32_Z;
//             f32_RZ = f32_matrix[2][0] * f32_X + f32_matrix[2][1] * f32_Y + f32_matrix[2][2] * f32_Z;


//             pst_LidarData->arst_Point[s32_I].f32_X = f32_RX;
//             pst_LidarData->arst_Point[s32_I].f32_Y = f32_RY;
//             pst_LidarData->arst_Point[s32_I].f32_Z = f32_RZ;

//         }
//     }
// }


// void ExtractGroundVeloyne128(LIDAR_DATA_t *pst_LidarData) // for 15 Hz VLS 128
// {
//     int32_t s32_I, s32_J;
//     int32_t s32_FirstIdx, s32_SecondIdx;
//     int32_t s32_Offset, s32_IdxDiff;

//     float32_t f32_DistanceXY, f32_DistanceZ, f32_Slope;

//     POINT_t *pst_Point = pst_LidarData->arst_Point;

//     for(s32_Offset = 0; s32_Offset < pst_LidarData->s32_PointNum; s32_Offset += 128)
//     {
//         for(s32_I = 0; s32_I < 127; s32_I++)
//         {
//             s32_FirstIdx = ars32_PointId[s32_I][0] + s32_Offset;
            
//             if(pst_Point[s32_FirstIdx].u8_Flag == c_POINT_INVALID)
//             {
//                 continue;
//             }

//             for(s32_J = 1; s32_J < 128 - s32_I; s32_J++)
//             {
//                 s32_IdxDiff = ars32_PointId[s32_I + s32_J][1] - ars32_PointId[s32_I][1];

//                 s32_SecondIdx = ars32_PointId[s32_I + s32_J][0] + s32_IdxDiff * 6 * 128 + s32_Offset;

//                 if(s32_SecondIdx < 0)
//                 {
//                     s32_SecondIdx += 153600;
//                 }
//                 else if(s32_SecondIdx > 153599)
//                 {
//                     s32_SecondIdx -= 153600;
//                 }

//                 if(pst_Point[s32_SecondIdx].u8_Flag != c_POINT_INVALID)
//                 {
//                     break;
//                 }
//             }

//             f32_DistanceXY = getDistance2d(pst_Point[s32_FirstIdx].f32_X, pst_Point[s32_FirstIdx].f32_Y, 
//                                            pst_Point[s32_SecondIdx].f32_X, pst_Point[s32_SecondIdx].f32_Y);
//             f32_DistanceZ = abs(pst_Point[s32_FirstIdx].f32_Z - pst_Point[s32_SecondIdx].f32_Z);
//             f32_Slope = atan2(f32_DistanceZ, f32_DistanceXY);

//             if(f32_Slope < 0.35)
//             {
//                 if(pst_Point[s32_FirstIdx].f32_Z < -0.5)
//                 {
//                     pst_Point[s32_FirstIdx].u8_Flag = c_POINT_GROUND;
//                 }

//                 if(pst_Point[s32_SecondIdx].f32_Z < -0.5)
//                 {
//                     pst_Point[s32_SecondIdx].u8_Flag = c_POINT_GROUND;
//                 }
//             }
//             else
//             {
//                 if(pst_Point[s32_FirstIdx].f32_Z > pst_Point[s32_SecondIdx].f32_Z)
//                 {
//                     pst_Point[s32_FirstIdx].u8_Flag = c_POINT_VALID;
//                 }
//                 else
//                 {
//                     pst_Point[s32_SecondIdx].u8_Flag = c_POINT_VALID;
//                 }
//             }
//         }

//         for(s32_I = 0; s32_I < 127; s32_I++)
//         {
//             s32_FirstIdx = ars32_PointId[s32_I][0] + s32_Offset;
            
//             if(pst_Point[s32_FirstIdx].u8_Flag == c_POINT_VALID)
//             {
//                 break;
//             }
//         }

//         for(s32_J = 1; s32_J < 128 - s32_I; s32_J++)
//         {
//             s32_IdxDiff = ars32_PointId[s32_I + s32_J][1] - ars32_PointId[s32_I][1];

//             s32_SecondIdx = ars32_PointId[s32_I + s32_J][0] + s32_IdxDiff * 6 * 128 + s32_Offset;

//             if(s32_SecondIdx < 0)
//             {
//                 s32_SecondIdx += 153600;
//             }
//             else if(s32_SecondIdx > 153599)
//             {
//                 s32_SecondIdx -= 153600;
//             }

//             if(pst_Point[s32_SecondIdx].u8_Flag == c_POINT_GROUND)
//             {
//                 break;
//             }
//         }

//         f32_DistanceXY = getDistance2d(pst_Point[s32_FirstIdx].f32_X, pst_Point[s32_FirstIdx].f32_Y,
//                                        pst_Point[s32_SecondIdx].f32_X, pst_Point[s32_SecondIdx].f32_Y);
//         f32_DistanceZ = abs(pst_Point[s32_FirstIdx].f32_Z - pst_Point[s32_SecondIdx].f32_Z);
//         f32_Slope = atan2(f32_DistanceZ, f32_DistanceXY);

//         if(f32_Slope < 0.35)
//         {
//             if(pst_Point[s32_FirstIdx].f32_Z < -0.5)
//             {
//                 pst_Point[s32_FirstIdx].u8_Flag = c_POINT_GROUND;
//             }
//         }
//     }
// }


// void ExtractGroundOuster128(LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I, s32_J;
//     int32_t s32_FirstIdx, s32_SecondIdx;
//     int32_t s32_Offset, s32_IdxDiff;

//     float32_t f32_DistanceXY, f32_DistanceZ, f32_Slope;

//     POINT_t *pst_Point = pst_LidarData->arst_Point;

//     for(s32_Offset = 0; s32_Offset < pst_LidarData->s32_PointNum; s32_Offset += 128)
//     {
//         for(s32_I = 127; s32_I > 0; s32_I--)
//         {
//             s32_FirstIdx = s32_I + s32_Offset;
            
//             if(pst_Point[s32_FirstIdx].u8_Flag == c_POINT_INVALID)
//             {
//                 continue;
//             }

//             for(s32_J = 1; s32_J < 128 - s32_I; s32_J++)
//             {
//                 s32_IdxDiff = ((s32_I - s32_J) % 4) - (s32_I % 4);

//                 s32_SecondIdx = (s32_I - s32_J) + s32_IdxDiff * 4 * 128 + s32_Offset;

//                 if(s32_SecondIdx < 0)
//                 {
//                     s32_SecondIdx += 131072;
//                 }
//                 else if(s32_SecondIdx > 131071)
//                 {
//                     s32_SecondIdx -= 131072;
//                 }

//                 if(pst_Point[s32_SecondIdx].u8_Flag != c_POINT_INVALID)
//                 {
//                     break;
//                 }
//             }

//             f32_DistanceXY = getDistance2d(pst_Point[s32_FirstIdx].f32_X, pst_Point[s32_FirstIdx].f32_Y,
//                                            pst_Point[s32_SecondIdx].f32_X, pst_Point[s32_SecondIdx].f32_Y);
//             f32_DistanceZ = abs(pst_Point[s32_FirstIdx].f32_Z - pst_Point[s32_SecondIdx].f32_Z);
//             f32_Slope = atan2(f32_DistanceZ, f32_DistanceXY);

//             if(f32_Slope < 0.35)
//             {
//                 if(pst_Point[s32_FirstIdx].f32_Z < -0.5)
//                 {
//                     pst_Point[s32_FirstIdx].u8_Flag = c_POINT_GROUND;
//                 }

//                 if(pst_Point[s32_SecondIdx].f32_Z < -0.5)
//                 {
//                     pst_Point[s32_SecondIdx].u8_Flag = c_POINT_GROUND;
//                 }
//             }
//         }

//         for(s32_I = 127; s32_I > 0; s32_I--)
//         {
//             s32_FirstIdx = s32_I + s32_Offset;
        
//             if(pst_Point[s32_FirstIdx].u8_Flag == c_POINT_INVALID)
//             {
//                 continue;
//             }
//         }

//         for(s32_J = 1; s32_J < 128 - s32_I; s32_J++)
//         {
//             s32_IdxDiff = ((s32_I - s32_J) % 4) - (s32_I % 4);

//             s32_SecondIdx = (s32_I - s32_J) + s32_IdxDiff * 4 * 128 + s32_Offset;

//             if(s32_SecondIdx < 0)
//             {
//                 s32_SecondIdx += 131072;
//             }
//             else if(s32_SecondIdx > 131071)
//             {
//                 s32_SecondIdx -= 131072;
//             }

//             if(pst_Point[s32_SecondIdx].u8_Flag == c_POINT_GROUND)
//             {
//                 break;
//             }
//         }

//         f32_DistanceXY = getDistance2d(pst_Point[s32_FirstIdx].f32_X, pst_Point[s32_FirstIdx].f32_Y,
//                                        pst_Point[s32_SecondIdx].f32_X, pst_Point[s32_SecondIdx].f32_Y);
//         f32_DistanceZ = abs(pst_Point[s32_FirstIdx].f32_Z - pst_Point[s32_SecondIdx].f32_Z);
//         f32_Slope = atan2(f32_DistanceZ, f32_DistanceXY);

//         if(f32_Slope < 0.35)
//         {
//             if(pst_Point[s32_FirstIdx].f32_Z < -0.5)
//             {
//                 pst_Point[s32_FirstIdx].u8_Flag = c_POINT_GROUND;
//             }
//         }
//     }
// }

// void SetRegionOfInterest(POINT_t *pst_Point, int32_t s32_Idx) //by voxel size
// {
//     if(   pst_Point[s32_Idx].f32_Distance >= c_VOXEL_DISTANCE_MAX || pst_Point[s32_Idx].f32_Distance < c_VOXEL_DISTANCE_MIN
//        || pst_Point[s32_Idx].f32_Azimuth_deg >= c_VOXEL_AZIMUTH_MAX || pst_Point[s32_Idx].f32_Azimuth_deg < c_VOXEL_AZIMUTH_MIN
//        || pst_Point[s32_Idx].f32_Elevation_deg >= c_VOXEL_ELEVATION_MAX || pst_Point[s32_Idx].f32_Elevation_deg < c_VOXEL_ELEVATION_MIN
//       )
//     {
//         pst_Point[s32_Idx].u8_Flag = c_POINT_INVALID;
//     }
// }


// void VoxelizePointCloud(LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I;
//     int32_t s32_DistIdx, s32_AzimIdx, s32_ElevIdx;

//     VOXEL_t *pst_Voxel = &pst_LidarData->st_Voxel;
//     POINT_t *pst_Point = pst_LidarData->arst_Point;

//     memset(pst_Voxel->ars32_Grid, 0, sizeof(pst_Voxel->ars32_Grid));

//     for(s32_I = 0; s32_I < pst_LidarData->s32_PointNum; s32_I++)
//     {
//         SetRegionOfInterest(pst_Point, s32_I); 

//         if(pst_Point[s32_I].u8_Flag == c_POINT_INVALID || pst_Point[s32_I].u8_Flag == c_POINT_GROUND)
//         {
//             continue;
//         }      

//         s32_DistIdx = (int32_t)((pst_Point[s32_I].f32_Distance - c_VOXEL_DISTANCE_MIN) / c_DISTANCE_LEAF_SIZE);
//         s32_AzimIdx = (int32_t)((pst_Point[s32_I].f32_Azimuth_deg - c_VOXEL_AZIMUTH_MIN) / c_AZIMUTH_LEAF_SIZE);
//         s32_ElevIdx = (int32_t)((pst_Point[s32_I].f32_Elevation_deg - c_VOXEL_ELEVATION_MIN) / c_ELEVATION_LEAF_SIZE);

//         pst_Point[s32_I].ars32_VoxelIdx[0] = s32_DistIdx;
//         pst_Point[s32_I].ars32_VoxelIdx[1] = s32_AzimIdx;
//         pst_Point[s32_I].ars32_VoxelIdx[2] = s32_ElevIdx;
//         pst_Voxel->ars32_Grid[s32_DistIdx][s32_AzimIdx][s32_ElevIdx]++;
//     }
// }


// void DeleteClusterObject(LIDAR_DATA_t *pst_LidarData, int32_t s32_Idx)
// {
//     int32_t s32_I;

//     CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;

//     for(s32_I = s32_Idx; s32_I < pst_LidarData->s32_ClusterNum - 1; s32_I++)
//     {
//         pst_Cluster[s32_I] = pst_Cluster[s32_I + 1];
//     }

//     pst_LidarData->s32_ClusterNum--;
// }


// // void ClusterPointCloud(LIDAR_DATA_t *pst_LidarData)
// // {
    
// //     int32_t s32_I;
// //     int32_t s32_DistIdx, s32_AzimIdx, s32_ElevIdx;
// //     int32_t s32_D, s32_A, s32_E;
// //     int32_t s32_ClusterIdx;
// //     int32_t s32_NeighborDistIdx, s32_NeighborAzimIdx, s32_NeighborElevIdx;

// //     CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
// //     VOXEL_t *pst_Voxel = &pst_LidarData->st_Voxel;
// //     POINT_t *pst_Point = pst_LidarData->arst_Point;
// //     queue<tuple<int32_t, int32_t, int32_t>> st_IdxQueue;
// //     tuple<int32_t, int32_t, int32_t> st_CurrentIdx;

// //     memset(pst_Voxel->ars16_ClusterIdx, c_UNCLUSTERED, sizeof(pst_Voxel->ars16_ClusterIdx));

// //     for(s32_I = 0; s32_I < pst_LidarData->s32_ClusterNum; s32_I++)
// //     {
// //         pst_Cluster[s32_I].s32_PointNum = 0;
// //         pst_Cluster[s32_I].f32_MaxX = -FLT_MAX;
// //         pst_Cluster[s32_I].f32_MaxY = -FLT_MAX;
// //         pst_Cluster[s32_I].f32_MaxZ = -FLT_MAX;
// //         pst_Cluster[s32_I].f32_MinX =  FLT_MAX;
// //         pst_Cluster[s32_I].f32_MinY =  FLT_MAX;
// //         pst_Cluster[s32_I].f32_MinZ =  FLT_MAX;
// //         pst_Cluster[s32_I].b_State = false;
// //     }

// //     pst_LidarData->s32_ClusterNum = 0;

// //     for(s32_I = 0; s32_I < pst_LidarData->s32_PointNum; s32_I++)
// //     {
// //         if(pst_Point[s32_I].u8_Flag != c_POINT_VALID)
// //         {
// //             continue;
// //         }

// //         s32_DistIdx = pst_Point[s32_I].aru16_VoxelIdx[0];
// //         s32_AzimIdx = pst_Point[s32_I].aru16_VoxelIdx[1];
// //         s32_ElevIdx = pst_Point[s32_I].aru16_VoxelIdx[2];

// //         if(pst_Voxel->ars16_ClusterIdx[s32_DistIdx][s32_AzimIdx][s32_ElevIdx] != c_UNCLUSTERED)
// //         {
// //             continue;
// //         }
        
// //         s32_ClusterIdx = pst_LidarData->s32_ClusterNum;
// //         st_IdxQueue.push({s32_DistIdx, s32_AzimIdx, s32_ElevIdx});

// //         while(!st_IdxQueue.empty())
// //         {
// //             st_CurrentIdx = st_IdxQueue.front();
// //             st_IdxQueue.pop();

// //             for(s32_D = -1; s32_D <= 1; s32_D++)
// //             {
// //                 for(s32_A = -1; s32_A <= 1; s32_A++)
// //                 {
// //                     for(s32_E = -1; s32_E <= 1; s32_E++)
// //                     {
// //                         s32_NeighborDistIdx = get<0>(st_CurrentIdx) + s32_D;
// //                         s32_NeighborAzimIdx = get<1>(st_CurrentIdx) + s32_A;
// //                         s32_NeighborElevIdx = get<2>(st_CurrentIdx) + s32_E;

// //                         if(s32_NeighborAzimIdx >= c_GRID_AZIMUTH_SIZE)
// //                         {
// //                             s32_NeighborAzimIdx -= c_GRID_AZIMUTH_SIZE;
// //                         }
// //                         else if(s32_NeighborAzimIdx < 0)
// //                         {
// //                             s32_NeighborAzimIdx += c_GRID_AZIMUTH_SIZE;
// //                         }

// //                         if(   s32_NeighborDistIdx >= 0 && s32_NeighborDistIdx <= c_GRID_DISTANCE_SIZE
// //                            && s32_NeighborElevIdx >= 0 && s32_NeighborElevIdx <= c_GRID_ELEVATION_SIZE
// //                            && pst_Voxel->aru16_Grid[s32_NeighborDistIdx][s32_NeighborAzimIdx][s32_NeighborElevIdx] != 0
// //                            && pst_Voxel->ars16_ClusterIdx[s32_NeighborDistIdx][s32_NeighborAzimIdx][s32_NeighborElevIdx] == c_UNCLUSTERED
// //                           )
// //                         {
// //                             pst_Voxel->ars16_ClusterIdx[s32_NeighborDistIdx][s32_NeighborAzimIdx][s32_NeighborElevIdx] = s32_ClusterIdx;
// //                             st_IdxQueue.push({s32_NeighborDistIdx, s32_NeighborAzimIdx, s32_NeighborElevIdx});
// //                         }
// //                     }
// //                 }
// //             }
// //         }

// //         pst_LidarData->s32_ClusterNum++;

// //         if(pst_LidarData->s32_ClusterNum == c_TOTAL_CLUSTER_NUM)
// //         {
// //             break;
// //         }
// //     }

// //     for(s32_I = 0; s32_I < pst_LidarData->s32_PointNum; s32_I++)
// //     {
// //         s32_DistIdx = pst_Point[s32_I].aru16_VoxelIdx[0];
// //         s32_AzimIdx = pst_Point[s32_I].aru16_VoxelIdx[1];
// //         s32_ElevIdx = pst_Point[s32_I].aru16_VoxelIdx[2];

// //         s32_ClusterIdx = pst_Voxel->ars16_ClusterIdx[s32_DistIdx][s32_AzimIdx][s32_ElevIdx];

// //         if(s32_ClusterIdx == c_UNCLUSTERED)
// //         {
// //             continue;
// //         }
// //         else if(pst_LidarData->arst_Cluster[s32_ClusterIdx].s32_PointNum == c_CLUSTER_POINT_NUM)
// //         {
// //             continue;
// //         }

// //         pst_Point[s32_I].u8_Intensity = s32_ClusterIdx;

// //         pst_Cluster[s32_ClusterIdx].f32_MaxX = (pst_Cluster[s32_ClusterIdx].f32_MaxX < pst_Point[s32_I].f32_X ? pst_Point[s32_I].f32_X : pst_Cluster[s32_ClusterIdx].f32_MaxX);
// //         pst_Cluster[s32_ClusterIdx].f32_MaxY = (pst_Cluster[s32_ClusterIdx].f32_MaxY < pst_Point[s32_I].f32_Y ? pst_Point[s32_I].f32_Y : pst_Cluster[s32_ClusterIdx].f32_MaxY);
// //         pst_Cluster[s32_ClusterIdx].f32_MaxZ = (pst_Cluster[s32_ClusterIdx].f32_MaxZ < pst_Point[s32_I].f32_Z ? pst_Point[s32_I].f32_Z : pst_Cluster[s32_ClusterIdx].f32_MaxZ);

// //         pst_Cluster[s32_ClusterIdx].f32_MinX = (pst_Cluster[s32_ClusterIdx].f32_MinX > pst_Point[s32_I].f32_X ? pst_Point[s32_I].f32_X : pst_Cluster[s32_ClusterIdx].f32_MinX);
// //         pst_Cluster[s32_ClusterIdx].f32_MinY = (pst_Cluster[s32_ClusterIdx].f32_MinY > pst_Point[s32_I].f32_Y ? pst_Point[s32_I].f32_Y : pst_Cluster[s32_ClusterIdx].f32_MinY);
// //         pst_Cluster[s32_ClusterIdx].f32_MinZ = (pst_Cluster[s32_ClusterIdx].f32_MinZ > pst_Point[s32_I].f32_Z ? pst_Point[s32_I].f32_Z : pst_Cluster[s32_ClusterIdx].f32_MinZ);

// //         pst_Cluster[s32_ClusterIdx].f32_X = (pst_Cluster[s32_ClusterIdx].f32_MaxX + pst_Cluster[s32_ClusterIdx].f32_MinX) / 2;
// //         pst_Cluster[s32_ClusterIdx].f32_Y = (pst_Cluster[s32_ClusterIdx].f32_MaxY + pst_Cluster[s32_ClusterIdx].f32_MinY) / 2;
// //         pst_Cluster[s32_ClusterIdx].f32_Z = (pst_Cluster[s32_ClusterIdx].f32_MaxZ + pst_Cluster[s32_ClusterIdx].f32_MinZ) / 2;

// //         pst_Cluster[s32_ClusterIdx].aru16_PointIdx[pst_Cluster[s32_ClusterIdx].s32_PointNum++] = s32_I;

// //         pst_Cluster[s32_ClusterIdx].b_State = true;
// //     }

// //     // for(s32_I = 0; s32_I < pst_LidarData->s32_ClusterNum; s32_I++)
// //     // {
// //     //     if(pst_Cluster[s32_I].s32_PointNum < 100)
// //     //     {
// //     //         DeleteClusterObject(pst_LidarData, s32_I);
// //     //         s32_I--;
// //     //     }
// //     // }
// // }

// void ClusterPointCloud(LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I;
//     int32_t s32_DistIdx, s32_AzimIdx, s32_ElevIdx;
//     int32_t s32_D, s32_A, s32_E;
//     int32_t s32_ClusterIdx;
//     int32_t s32_NeighborDistIdx, s32_NeighborAzimIdx, s32_NeighborElevIdx;

//     CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
//     VOXEL_t *pst_Voxel = &pst_LidarData->st_Voxel;
//     POINT_t *pst_Point = pst_LidarData->arst_Point;
//     queue<tuple<int32_t, int32_t, int32_t>> st_IdxQueue;
//     tuple<int32_t, int32_t, int32_t> st_CurrentIdx;

//     memset(pst_Voxel->ars32_ClusterIdx, c_UNCLUSTERED, sizeof(pst_Voxel->ars32_ClusterIdx));

//     for(s32_I = 0; s32_I < pst_LidarData->s32_ClusterNum; s32_I++)
//     {
//         pst_Cluster[s32_I].s32_PointNum = 0;
//         pst_Cluster[s32_I].f32_MaxX = -FLT_MAX;
//         pst_Cluster[s32_I].f32_MaxY = -FLT_MAX;
//         pst_Cluster[s32_I].f32_MaxZ = -FLT_MAX;
//         pst_Cluster[s32_I].f32_MinX =  FLT_MAX;
//         pst_Cluster[s32_I].f32_MinY =  FLT_MAX;
//         pst_Cluster[s32_I].f32_MinZ =  FLT_MAX;
//         pst_Cluster[s32_I].b_State = false;
//     }

//     pst_LidarData->s32_ClusterNum = 0;

//     for(s32_I = 0; s32_I < pst_LidarData->s32_PointNum; s32_I++)
//     {
//         if(pst_Point[s32_I].u8_Flag == c_POINT_GROUND || pst_Point[s32_I].u8_Flag == c_POINT_INVALID)
//         {
//             continue;
//         }

//         s32_DistIdx = pst_Point[s32_I].ars32_VoxelIdx[0];
//         s32_AzimIdx = pst_Point[s32_I].ars32_VoxelIdx[1];
//         s32_ElevIdx = pst_Point[s32_I].ars32_VoxelIdx[2];

//         if(pst_Voxel->ars32_ClusterIdx[s32_DistIdx][s32_AzimIdx][s32_ElevIdx] != -1)
//         {
//             continue;
//         }
        
//         s32_ClusterIdx = pst_LidarData->s32_ClusterNum;
//         st_IdxQueue.push({s32_DistIdx, s32_AzimIdx, s32_ElevIdx});

//         while(!st_IdxQueue.empty())
//         {
//             st_CurrentIdx = st_IdxQueue.front();
//             st_IdxQueue.pop();

//             for(s32_D = -1; s32_D <= 1; s32_D++)
//             {
//                 for(s32_A = -1; s32_A <= 1; s32_A++)
//                 {
//                     for(s32_E = -1; s32_E <= 1; s32_E++)
//                     {
//                         s32_NeighborDistIdx = get<0>(st_CurrentIdx) + s32_D;
//                         s32_NeighborAzimIdx = get<1>(st_CurrentIdx) + s32_A;
//                         s32_NeighborElevIdx = get<2>(st_CurrentIdx) + s32_E;

//                         if(s32_NeighborAzimIdx >= c_GRID_AZIMUTH_SIZE)
//                         {
//                             s32_NeighborAzimIdx -= c_GRID_AZIMUTH_SIZE;
//                         }
//                         else if(s32_NeighborAzimIdx < 0)
//                         {
//                             s32_NeighborAzimIdx += c_GRID_AZIMUTH_SIZE;
//                         }

//                         if(   s32_NeighborDistIdx >= 0 && s32_NeighborDistIdx < c_GRID_DISTANCE_SIZE
//                            && s32_NeighborElevIdx >= 0 && s32_NeighborElevIdx < c_GRID_ELEVATION_SIZE
//                            && pst_Voxel->ars32_Grid[s32_NeighborDistIdx][s32_NeighborAzimIdx][s32_NeighborElevIdx] != 0
//                            && pst_Voxel->ars32_ClusterIdx[s32_NeighborDistIdx][s32_NeighborAzimIdx][s32_NeighborElevIdx] == c_UNCLUSTERED
//                           )
//                         {
//                             pst_Voxel->ars32_ClusterIdx[s32_NeighborDistIdx][s32_NeighborAzimIdx][s32_NeighborElevIdx] = s32_ClusterIdx;
//                             st_IdxQueue.push({s32_NeighborDistIdx, s32_NeighborAzimIdx, s32_NeighborElevIdx});
//                         }
//                     }
//                 }
//             }
//         }

//         pst_LidarData->s32_ClusterNum++;

//         if(pst_LidarData->s32_ClusterNum == c_TOTAL_CLUSTER_NUM)
//         {
//             break;
//         }
//     }

//     for(s32_I = 0; s32_I < pst_LidarData->s32_PointNum; s32_I++)
//     {
//         if(pst_Point[s32_I].u8_Flag == c_POINT_GROUND || pst_Point[s32_I].u8_Flag == c_POINT_INVALID)
//         {
//             continue;
//         }

//         s32_DistIdx = pst_Point[s32_I].ars32_VoxelIdx[0];
//         s32_AzimIdx = pst_Point[s32_I].ars32_VoxelIdx[1];
//         s32_ElevIdx = pst_Point[s32_I].ars32_VoxelIdx[2];

//         s32_ClusterIdx = pst_Voxel->ars32_ClusterIdx[s32_DistIdx][s32_AzimIdx][s32_ElevIdx];

//         if(s32_ClusterIdx == c_UNCLUSTERED)
//         {
//             continue;
//         }
//         else if(pst_Cluster[s32_ClusterIdx].s32_PointNum == c_CLUSTER_POINT_NUM)
//         {
//             continue;
//         }

//         pst_Cluster[s32_ClusterIdx].f32_MaxX = (pst_Cluster[s32_ClusterIdx].f32_MaxX < pst_Point[s32_I].f32_X ? pst_Point[s32_I].f32_X : pst_Cluster[s32_ClusterIdx].f32_MaxX);
//         pst_Cluster[s32_ClusterIdx].f32_MaxY = (pst_Cluster[s32_ClusterIdx].f32_MaxY < pst_Point[s32_I].f32_Y ? pst_Point[s32_I].f32_Y : pst_Cluster[s32_ClusterIdx].f32_MaxY);
//         pst_Cluster[s32_ClusterIdx].f32_MaxZ = (pst_Cluster[s32_ClusterIdx].f32_MaxZ < pst_Point[s32_I].f32_Z ? pst_Point[s32_I].f32_Z : pst_Cluster[s32_ClusterIdx].f32_MaxZ);

//         pst_Cluster[s32_ClusterIdx].f32_MinX = (pst_Cluster[s32_ClusterIdx].f32_MinX > pst_Point[s32_I].f32_X ? pst_Point[s32_I].f32_X : pst_Cluster[s32_ClusterIdx].f32_MinX);
//         pst_Cluster[s32_ClusterIdx].f32_MinY = (pst_Cluster[s32_ClusterIdx].f32_MinY > pst_Point[s32_I].f32_Y ? pst_Point[s32_I].f32_Y : pst_Cluster[s32_ClusterIdx].f32_MinY);
//         pst_Cluster[s32_ClusterIdx].f32_MinZ = (pst_Cluster[s32_ClusterIdx].f32_MinZ > pst_Point[s32_I].f32_Z ? pst_Point[s32_I].f32_Z : pst_Cluster[s32_ClusterIdx].f32_MinZ);

//         pst_Cluster[s32_ClusterIdx].f32_X = (pst_Cluster[s32_ClusterIdx].f32_MaxX + pst_Cluster[s32_ClusterIdx].f32_MinX) / 2;
//         pst_Cluster[s32_ClusterIdx].f32_Y = (pst_Cluster[s32_ClusterIdx].f32_MaxY + pst_Cluster[s32_ClusterIdx].f32_MinY) / 2;
//         pst_Cluster[s32_ClusterIdx].f32_Z = (pst_Cluster[s32_ClusterIdx].f32_MaxZ + pst_Cluster[s32_ClusterIdx].f32_MinZ) / 2;

//         pst_Cluster[s32_ClusterIdx].ars32_PointIdx[pst_Cluster->s32_PointNum] = s32_I;
//         pst_Cluster[s32_ClusterIdx].s32_PointNum++;

//         pst_Cluster[s32_ClusterIdx].b_State = true;
//         pst_Cluster[s32_ClusterIdx].b_TrackingFlag = true;
//     }
// }

// // Classifcation Module without Road Boundary Concerns
// // Need to be merged with another Classification Module below

// // void ClassifyPointCloud(LIDAR_DATA_t *pst_LidarData)
// // {
// //     int32_t s32_ClusterIdx, s32_PointIdx, s32_MappedPointIdx, s32_CriteriaIdx;
// //     int32_t s32_PointNum;
// //     float32_t f32_Width, f32_Length, f32_Height, f32_Area, f32_Volume, f32_Density, f32_DistR, f32_DistXY, f32_MinDistXYTop, f32_MinDistXYBottom, f32_MinDistXY;
// //     CLUSTER_t* pst_Cluster = pst_LidarData->arst_Cluster;
// //     POINT_t* pst_Point = pst_LidarData->arst_Point;
// //     uint32_t u32_SubClusterNum = 15;
// //     float32_t arf32_SubWidth[u32_SubClusterNum], arf32_SubLength[u32_SubClusterNum], arf32_SubArea[u32_SubClusterNum];
// //     CLUSTER_t arst_SubCluster[u32_SubClusterNum];
// //     float32_t arf32_ZCriteria[u32_SubClusterNum];
// //     int32_t s32_Count;
// //     float32_t f32_ZWithOffset;
    
// //     for(s32_ClusterIdx = 0; s32_ClusterIdx < pst_LidarData->s32_ClusterNum; s32_ClusterIdx++)
// //     {
// //         s32_Count = 0;
// //         f32_DistR = pst_Cluster[s32_ClusterIdx].f32_Distance = sqrt(pst_Cluster[s32_ClusterIdx].f32_X * pst_Cluster[s32_ClusterIdx].f32_X + pst_Cluster[s32_ClusterIdx].f32_Y * pst_Cluster[s32_ClusterIdx].f32_Y + pst_Cluster[s32_ClusterIdx].f32_Z * pst_Cluster[s32_ClusterIdx].f32_Z);
// //         f32_DistXY = sqrt(pst_Cluster[s32_ClusterIdx].f32_X * pst_Cluster[s32_ClusterIdx].f32_X + pst_Cluster[s32_ClusterIdx].f32_Y * pst_Cluster[s32_ClusterIdx].f32_Y);
// //         f32_Width = pst_Cluster[s32_ClusterIdx].f32_Width = pst_Cluster[s32_ClusterIdx].f32_MaxX - pst_Cluster[s32_ClusterIdx].f32_MinX;
// //         f32_Length = pst_Cluster[s32_ClusterIdx].f32_Length = pst_Cluster[s32_ClusterIdx].f32_MaxY - pst_Cluster[s32_ClusterIdx].f32_MinY;
// //         f32_Height = pst_Cluster[s32_ClusterIdx].f32_Height = pst_Cluster[s32_ClusterIdx].f32_MaxZ - pst_Cluster[s32_ClusterIdx].f32_MinZ;
// //         f32_Area = f32_Width * f32_Length;
// //         f32_Volume = pst_Cluster[s32_ClusterIdx].f32_Volume = f32_Area * f32_Height;
// //         f32_Density = pst_Cluster[s32_ClusterIdx].f32_Density = (pst_Cluster[s32_ClusterIdx].s32_PointNum / pst_Cluster[s32_ClusterIdx].f32_Volume);
// //         s32_PointNum = pst_Cluster[s32_ClusterIdx].s32_PointNum;
        
// //         f32_MinDistXYTop = FLT_MAX;
// //         f32_MinDistXYBottom = FLT_MAX;

// //         printf("Index : %d, Distance : %f, Density : %f, PointNum : %d, Width : %f, Length : %f, Height : %f, Width/Length : %f, Length/Width : %f, Area/Height : %f\n", s32_ClusterIdx, f32_DistXY, f32_Density, s32_PointNum, f32_Width, f32_Length, f32_Height, f32_Width/f32_Length, f32_Length/f32_Width, f32_Area/f32_Height);

// //         // Sparse한 클러스터 제거 
// //         if(
// //             (f32_Volume == 0) ||
// //             ((f32_Width < 1.f || f32_Length < 1.f) &&
// //             ((f32_DistXY <= 10.f && (f32_Density < 20.f && s32_PointNum < 35)) ||
// //             (f32_DistXY > 10.f && f32_DistXY <= 20.f && ((f32_Density < 15.f || f32_Volume < 3.f) && s32_PointNum < 30)) ||
// //             (f32_DistXY > 20.f && f32_DistXY <= 30.f && ((f32_Density < 10.f || f32_Volume < 2.f) && s32_PointNum < 25)) ||
// //             (f32_DistXY > 30.f && f32_DistXY <= 40.f && ((f32_Density < 5.f || f32_Volume < 2.f) && s32_PointNum < 20)) ||
// //             (f32_DistXY > 40.f && f32_DistXY <= 50.f && ((f32_Density < 3.f || f32_Volume < 2.f) && s32_PointNum < 15)) ||
// //             (f32_DistXY > 50.f && f32_DistXY <= 75.f && ((f32_Density < 2.f || f32_Volume < 2.f) && s32_PointNum < 10)) ||
// //             (f32_DistXY > 75.f && f32_DistXY <= 100.f && ((f32_Density < 1.f || f32_Volume < 1.f) && s32_PointNum < 7)) ||
// //             (f32_DistXY > 100.f && f32_DistXY <= 125.f && ((f32_Density < 0.5f || f32_Volume < 0.5f) && s32_PointNum < 5)) ||
// //             (f32_DistXY > 125.f && s32_PointNum < 3)))
// //         )
// //         {
// //             printf("Index '%d' classified as Sparse Cluster\n", s32_ClusterIdx);
// //             pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_INVALID;
// //         }
// //         // Big Terrain/Object or Banker
// //         else if(f32_Width > 5.f && f32_Length > 5.f && s32_PointNum > 50)
// //         {
// //             if(
// //                     (f32_Area / f32_Height > 400.f && (s32_PointNum < 5000 && f32_Density < 0.1f)) ||  // 전후방에 잡히는 뱅커
// //                     (pst_Cluster[s32_ClusterIdx].f32_MinX < 0) && (pst_Cluster[s32_ClusterIdx].f32_MaxX > 0) && // 뱅커 내부에 들어왔을때
// //                     (pst_Cluster[s32_ClusterIdx].f32_MinY < 0) && (pst_Cluster[s32_ClusterIdx].f32_MaxY > 0)
// //             )
// //             {
// //                 printf("Index '%d' classified as Banker\n", s32_ClusterIdx);
// //                 pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_INVALID;
// //             }
// //             else
// //             {
// //                 printf("Index '%d' classified as Big Terrain/Object\n", s32_ClusterIdx);
// //                 pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_OBJECT;
// //             }
// //         }
// //         // Size가 작은 클러스터 제거
// //         else if(
// //                 (f32_Width < 0.1f || f32_Length < 0.1f) ||
// //                 (f32_DistXY < 100.f && f32_Height < 0.2f) ||
// //                 (f32_DistXY >= 100.f && f32_Height < 0.1f)
// //             )
// //         {
// //             printf("Index '%d' didn't satisfy Size Rule\n", s32_ClusterIdx);
// //             pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_INVALID;
// //         }
// //         // 지면 제거
// //         else if(
// //                 // 길쭉하게 나타나는 지면
// //                 (((f32_Width / f32_Length > 7.f) && (f32_Length < 0.5f)) ||
// //                 ((f32_Length / f32_Width > 7.f) && f32_Width < 0.5f) ||
// //                 // 넓적하게 나타나는 지면
// //                 ((f32_Area / f32_Height > 10.f)))                           
// //             )
// //         {
            
// //             if(f32_Area / f32_Height > 20.f && f32_Density < 50.f && f32_Height < 0.5f)
// //             {
// //                 printf("Index '%d' classified as Ground Rule1\n", s32_ClusterIdx);
// //                 pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_INVALID;
// //             }
// //             // else if(f32_Area / f32_Height > 20.f && f32_Density < 50.f && f32_Height < 0.3f)
// //             // {
// //             //     printf("Index '%d' classified as Ground Rule2\n", s32_ClusterIdx);
// //             //     pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_INVALID;
// //             // }
// //             else if(
// //                 (f32_DistXY <= 10.f && f32_Height < 0.1f && s32_PointNum < 100) ||
// //                 (f32_DistXY > 10.f && f32_DistXY <= 20.f && f32_Height < 0.2f && s32_PointNum < 50) ||
// //                 (f32_DistXY > 20.f && f32_DistXY <= 40.f && f32_Height < 0.2f && s32_PointNum < 20) ||
// //                 (f32_DistXY > 40.f && f32_Height < 0.2f && s32_PointNum < 10)
// //             )
// //             {
// //                 printf("Index '%d' classified as Ground Rule3\n", s32_ClusterIdx);
// //                 pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_INVALID;
// //             }
// //             else
// //             {
// //                 printf("Index '%d' classified as Object in Ground Rule\n", s32_ClusterIdx);
// //                 pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_OBJECT;
// //             }
// //         }
// //         // Classify Whether Non-Car or Car
// //         else
// //         {
// //             // Initialization
// //             arf32_ZCriteria[0] = f32_Height / float32_t(u32_SubClusterNum);
// //             arst_SubCluster[0].f32_MaxX = -FLT_MAX;
// //             arst_SubCluster[0].f32_MaxY = -FLT_MAX;
// //             arst_SubCluster[0].f32_MinX = FLT_MAX;
// //             arst_SubCluster[0].f32_MinY = FLT_MAX;
// //             arst_SubCluster[0].s32_PointNum = 0;
        
// //             for(s32_CriteriaIdx = 1; s32_CriteriaIdx < u32_SubClusterNum; s32_CriteriaIdx++)
// //             {
// //                 arf32_ZCriteria[s32_CriteriaIdx] = arf32_ZCriteria[0] * (s32_CriteriaIdx + 1);
// //                 arst_SubCluster[s32_CriteriaIdx].f32_MaxX = -FLT_MAX;
// //                 arst_SubCluster[s32_CriteriaIdx].f32_MaxY = -FLT_MAX;
// //                 arst_SubCluster[s32_CriteriaIdx].f32_MinX = FLT_MAX;
// //                 arst_SubCluster[s32_CriteriaIdx].f32_MinY = FLT_MAX;
// //                 arst_SubCluster[s32_CriteriaIdx].s32_PointNum = 0;
// //             }

// //             // Iteration through Points in Cluster
// //             for(s32_PointIdx = 0; s32_PointIdx < s32_PointNum; s32_PointIdx++)
// //             {
// //                 s32_MappedPointIdx = pst_Cluster[s32_ClusterIdx].ars32_PointIdx[s32_PointIdx];
// //                 f32_ZWithOffset = pst_Point[s32_MappedPointIdx].f32_Z - pst_Cluster[s32_ClusterIdx].f32_MinZ;   // Get Z Value with Offset of Specific Point

// //                 for(s32_CriteriaIdx = 0; s32_CriteriaIdx < u32_SubClusterNum; s32_CriteriaIdx++)
// //                 {
// //                     if(s32_CriteriaIdx == 0)
// //                     {
// //                         if(f32_ZWithOffset >= 0 && f32_ZWithOffset < arf32_ZCriteria[0]) break;
// //                     }
// //                     else if(s32_CriteriaIdx != 0)
// //                     {
// //                         if(f32_ZWithOffset >= arf32_ZCriteria[s32_CriteriaIdx - 1] && f32_ZWithOffset <= arf32_ZCriteria[s32_CriteriaIdx])
// //                         {
// //                             break;
// //                         }
// //                     }
// //                 }

// //                 // Get Min Distance in XY Plane in Top and Bottom of Cluster
// //                 if(s32_CriteriaIdx == 0)
// //                 {
// //                     f32_DistXY = sqrt(pst_Point[s32_MappedPointIdx].f32_X * pst_Point[s32_MappedPointIdx].f32_X + pst_Point[s32_MappedPointIdx].f32_Y * pst_Point[s32_MappedPointIdx].f32_Y);
// //                     f32_MinDistXYBottom = min(f32_MinDistXYBottom, f32_DistXY);
// //                 }
// //                 else if(s32_CriteriaIdx == (u32_SubClusterNum - 1))
// //                 {
// //                     f32_DistXY = sqrt(pst_Point[s32_MappedPointIdx].f32_X * pst_Point[s32_MappedPointIdx].f32_X + pst_Point[s32_MappedPointIdx].f32_Y * pst_Point[s32_MappedPointIdx].f32_Y);
// //                     f32_MinDistXYTop = min(f32_MinDistXYTop, f32_DistXY);   
// //                 }

// //                 // Assign Point in the SubCluster and Update Min/Max XY Value of SubCluster
// //                 arst_SubCluster[s32_CriteriaIdx].ars32_PointIdx[arst_SubCluster[s32_CriteriaIdx].s32_PointNum++] = s32_MappedPointIdx;
// //                 arst_SubCluster[s32_CriteriaIdx].f32_MaxX = max(arst_SubCluster[s32_CriteriaIdx].f32_MaxX, pst_Point[s32_MappedPointIdx].f32_X);
// //                 arst_SubCluster[s32_CriteriaIdx].f32_MaxY = max(arst_SubCluster[s32_CriteriaIdx].f32_MaxY, pst_Point[s32_MappedPointIdx].f32_Y);
// //                 arst_SubCluster[s32_CriteriaIdx].f32_MinX = min(arst_SubCluster[s32_CriteriaIdx].f32_MinX, pst_Point[s32_MappedPointIdx].f32_X);
// //                 arst_SubCluster[s32_CriteriaIdx].f32_MinY = min(arst_SubCluster[s32_CriteriaIdx].f32_MinY, pst_Point[s32_MappedPointIdx].f32_Y);
// //             }
            
// //             for(s32_CriteriaIdx = 0; s32_CriteriaIdx < u32_SubClusterNum; s32_CriteriaIdx++)
// //             {
// //                 arf32_SubWidth[s32_CriteriaIdx] = arst_SubCluster[s32_CriteriaIdx].f32_MaxX - arst_SubCluster[s32_CriteriaIdx].f32_MinX;
// //                 arf32_SubLength[s32_CriteriaIdx] = arst_SubCluster[s32_CriteriaIdx].f32_MaxY - arst_SubCluster[s32_CriteriaIdx].f32_MinY;
// //                 arf32_SubArea[s32_CriteriaIdx] = arf32_SubWidth[s32_CriteriaIdx] * arf32_SubLength[s32_CriteriaIdx];
// //                 if(s32_CriteriaIdx != 0 && arf32_SubArea[s32_CriteriaIdx] != 0)
// //                 {
// //                     if(arf32_SubArea[s32_CriteriaIdx] < arf32_SubArea[s32_CriteriaIdx - 1]) ++s32_Count;
// //                 }
// //             }
            
// //             if(arf32_SubArea[u32_SubClusterNum - 1] > arf32_SubArea[0])
// //             {
// //                 printf("SubArea Rule Not Satisfied\n");
// //                 pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_OBJECT;
// //             }
// //             else if(
// //                 (f32_DistXY <= 80 && f32_MinDistXYBottom < f32_MinDistXYTop) &&
// //                 (
// //                     (f32_DistXY < 10.f && s32_PointNum > 1500) ||
// //                     (f32_DistXY >= 10.f && f32_DistXY < 20.f && s32_PointNum > 1000) ||
// //                     (f32_DistXY >= 20.f && f32_DistXY < 30.f && s32_PointNum > 700) ||
// //                     (f32_DistXY >= 30.f && f32_DistXY < 40.f && s32_PointNum > 500) ||
// //                     (f32_DistXY >= 40.f && f32_DistXY < 50.f && s32_PointNum > 300) ||
// //                     (f32_DistXY >= 50.f && f32_DistXY < 60.f && s32_PointNum > 200) ||
// //                     (f32_DistXY >= 60.f && f32_DistXY < 70.f && s32_PointNum > 100) ||
// //                     (f32_DistXY >= 70.f && f32_DistXY <= 80.f && s32_PointNum > 50)
// //                 )
// //             )
// //             {
// //                 pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_CAR;
// //             }
// //             else
// //             {
// //                 printf("MinDistXYTop : %f, MinDistXYBottom : %f\n", f32_MinDistXYTop, f32_MinDistXYBottom);
// //                 printf("Any Rule that detects Car Not Satisfied\n");
// //                 pst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_OBJECT;
// //             }
// //         }
// //     }
// // }

// void ClassifyPointCloud(LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
// {
//     float32_t f32_EOffset, f32_NOffset, f32_UOffset;
//     float32_t f32_E, f32_N, f32_EWithOffset, f32_NWithOffset, f32_U, f32_Yaw_rad, f32_EWithYaw, f32_NWithYaw;
//     float32_t f32_ResultX, f32_ResultY;
//     float32_t f32_EgoX, f32_EgoY;
//     float64_t f64_LatitudeOrigin = 35.646802598871915f;
//     float64_t f64_LongitudeOrigin = 128.402556331900200f;
//     float64_t f64_AltitudeOrigin = 0.000000000000000f;
//     int32_t s32_PointNum = 0;

//     lla2enu(f64_LatitudeOrigin, f64_LongitudeOrigin, f64_AltitudeOrigin, f32_EOffset, f32_NOffset, f32_UOffset); 

//     VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
//     char *files[] = {
//         "src/Integration/map/KiapiRoadBoundary1.txt",
//         "src/Integration/map/KiapiRoadBoundary2.txt",
//         "src/Integration/map/KiapiRoadBoundary3.txt",
//         "src/Integration/map/KiapiRoadBoundary4.txt",
//         "src/Integration/map/KiapiRoadBoundaryJoker1.txt",
//         "src/Integration/map/KiapiRoadBoundaryJoker2.txt"
//     };
    

//     f32_Yaw_rad = pst_EgoVehicleData->f32_Yaw_rad_ENU;
//     f32_EgoX = pst_EgoVehicleData->f32_X_global;
//     f32_EgoY = pst_EgoVehicleData->f32_Y_global;

//     for (int i = 0; i < 6; i++) {
//         FILE *file = fopen(files[i], "r");

//         while (!feof(file)) 
//         {
//             fscanf(file, "%f %f %f", &f32_E, &f32_N, &f32_U);

//             f32_EWithOffset = f32_E + f32_EOffset;
//             f32_NWithOffset = f32_N + f32_NOffset;
//             getLocalCoord(f32_EgoX, f32_EgoY, f32_Yaw_rad, f32_EWithOffset, f32_NWithOffset, f32_ResultX, f32_ResultY);

//             if (fabsf(f32_ResultX) < 150.f && fabsf(f32_ResultY) < 150.f)
//             {
//                 pst_LidarData->arst_RoadPoint[s32_PointNum].f32_X = -f32_ResultY;
//                 pst_LidarData->arst_RoadPoint[s32_PointNum].f32_Y = f32_ResultX;
//                 s32_PointNum++;
//             }
//         }
//         pst_LidarData->s32_RoadPointNum = s32_PointNum;

//         fclose(file);
//     }
// }


// // void ClassifyPointCloud(LIDAR_DATA_t *pst_LidarData)
// // {
// //     int32_t s32_ClusterIdx;
// //     int32_t s32_PointIdx;
// //     int32_t s32_MappedPointIdx;

// //     arma::mat st_ArmaCluster, st_ArmaClusterNormalized;
    
// //     float32_t f32_Alpha, f32_Omega;

// //     arma::mat stf32_Coeff;
// //     arma::mat stf32_Score;
// //     arma::vec stf32_Latent;
// //     arma::vec stf32_EigenValue;
    
// //     arma::vec st_Features(11);

// //     mlpack::RandomForest<> st_RandomForest;
// //     mlpack::data::Load("src/Integration/src/LiDAR/model/rf-model-contour-1-1-1e-3-8.bin", "model", st_RandomForest, true);

// //     arma::Row<size_t> st_Predictions;
// //     arma::mat st_Probs;
    
// //     for(s32_ClusterIdx = 0; s32_ClusterIdx < pst_LidarData->s32_ClusterNum; s32_ClusterIdx++)
// //     {
// //         pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Volume = (
// //                                     (pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MaxX - pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MinX) *
// //                                     (pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MaxY - pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MinY) *
// //                                     (pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MaxZ - pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MinZ)
// //                                 );

// //         if((pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Volume == 0) ||
// //            (pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Volume > 30.f)
// //         )
// //         {
// //             pst_LidarData->arst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_NONCAR;
// //         }
// //         else
// //         {
// //             // X, Y, Z to Azimuth, Elevation
// //             f32_Omega = asinf(pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Z / pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Distance);
// //             pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Elevation = rad2deg(f32_Omega);
            
// //             f32_Alpha = asinf(pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_X / (pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Distance * cosf(f32_Omega)));
// //             pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Azimuth = rad2deg(f32_Alpha);

// //             st_ArmaCluster.reshape(pst_LidarData->arst_Cluster[s32_ClusterIdx].s32_PointNum, 3);
            
// //             for(s32_PointIdx = 0; s32_PointIdx < st_ArmaCluster.n_rows; s32_PointIdx++)
// //             {
// //                 s32_MappedPointIdx = pst_LidarData->arst_Cluster[s32_ClusterIdx].aru16_PointIdx[s32_PointIdx];
// //                 st_ArmaCluster(s32_PointIdx, 0) = pst_LidarData->arst_Point[s32_MappedPointIdx].f32_X;
// //                 st_ArmaCluster(s32_PointIdx, 1) = pst_LidarData->arst_Point[s32_MappedPointIdx].f32_Y;
// //                 st_ArmaCluster(s32_PointIdx, 2) = pst_LidarData->arst_Point[s32_MappedPointIdx].f32_Z;
// //             }

// //             arma::princomp(stf32_Coeff, stf32_Score, stf32_Latent, arma::normalise(st_ArmaCluster));

// //             if(stf32_Latent.n_elem == 3)
// //             {
// //                 st_Features(0) = pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Distance; 
// //                 st_Features(1) = pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Azimuth; 
// //                 st_Features(2) = pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Elevation;

// //                 st_Features(3) = pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MaxX - pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MinX; // Width
// //                 st_Features(4) = pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MaxY - pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MinY; // Length
// //                 st_Features(5) = pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MaxZ - pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_MinZ; // Height
// //                 st_Features(6) = pst_LidarData->arst_Cluster[s32_ClusterIdx].f32_Volume;
// //                 st_Features(7) = float32_t(pst_LidarData->arst_Cluster[s32_ClusterIdx].s32_PointNum / st_Features(6)); // Density
                
// //                 stf32_EigenValue = arma::sort(stf32_Latent, "d");

// //                 st_Features(8) = std::pow(stf32_EigenValue(0) * stf32_EigenValue(1) * stf32_EigenValue(2), 1.f/3.f);
// //                 st_Features(9) = stf32_EigenValue(2) / stf32_EigenValue(0);
// //                 st_Features(10) = 1 - std::abs(stf32_Coeff(2, 2));

// //                 st_RandomForest.Classify(st_Features, st_Predictions, st_Probs);
                
// //                 if(st_Predictions[0] == 1)
// //                 {
// //                     pst_LidarData->arst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_CAR;
// //                 }
// //                 else
// //                 {
// //                     pst_LidarData->arst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_NONCAR;
// //                 }
// //             }
// //             else
// //             {
// //                 pst_LidarData->arst_Cluster[s32_ClusterIdx].u8_Class = c_CLUSTER_NONCAR;
// //             }
// //         }
// //     }
// // }


// void SaveTrackingObject(LIDAR_DATA_t *pst_LidarData, int32_t s32_NewObjIdx, int32_t s32_PrevNewObjIdx)
// {
//     TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;
//     CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
//     CLUSTER_t *pst_PrevCluster = pst_LidarData->arst_PrevCluster;

//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_X = pst_Cluster[s32_NewObjIdx].f32_X;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_Y = pst_Cluster[s32_NewObjIdx].f32_Y;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_VelocityX_m_s = (pst_Cluster[s32_NewObjIdx].f32_X - pst_PrevCluster[s32_PrevNewObjIdx].f32_X) / f32_DeltaTime_s;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_VelocityY_m_s = (pst_Cluster[s32_NewObjIdx].f32_Y - pst_PrevCluster[s32_PrevNewObjIdx].f32_Y) / f32_DeltaTime_s;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_ClusterX = pst_Cluster[s32_NewObjIdx].f32_X;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_ClusterY = pst_Cluster[s32_NewObjIdx].f32_Y;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_MaxX = pst_Cluster[s32_NewObjIdx].f32_MaxX;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_MaxY = pst_Cluster[s32_NewObjIdx].f32_MaxY;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_MinX = pst_Cluster[s32_NewObjIdx].f32_MinX;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_MinY = pst_Cluster[s32_NewObjIdx].f32_MinY;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_PrevClusterX = pst_PrevCluster[s32_PrevNewObjIdx].f32_X;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_PrevClusterY = pst_PrevCluster[s32_PrevNewObjIdx].f32_Y;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_PrevMaxX = pst_PrevCluster[s32_NewObjIdx].f32_MaxX;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_PrevMaxY = pst_PrevCluster[s32_NewObjIdx].f32_MaxY;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_PrevMinX = pst_PrevCluster[s32_NewObjIdx].f32_MinX;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].f32_PrevMinY = pst_PrevCluster[s32_NewObjIdx].f32_MinY;
//     pst_Tracking[pst_LidarData->s32_TrackingNum].b_InitlzFlag = true;
//     pst_LidarData->s32_TrackingNum++;
// }


// void CopyClusterObject(LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I;

//     CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
//     CLUSTER_t *pst_PrevCluster = pst_LidarData->arst_PrevCluster;

//     for(s32_I = 0; s32_I < pst_LidarData->s32_ClusterNum; s32_I++)
//     {
//         pst_PrevCluster[s32_I] = pst_Cluster[s32_I];
//     }

//     pst_LidarData->s32_PrevClusterNum = pst_LidarData->s32_ClusterNum;
// }


// void DeleteTrackingObject(LIDAR_DATA_t *pst_LidarData, int32_t s32_Idx)
// {
//     int32_t s32_I;

//     TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

//     for(s32_I = s32_Idx; s32_I < pst_LidarData->s32_TrackingNum - 1; s32_I++)
//     {
//         pst_Tracking[s32_I] = pst_Tracking[s32_I + 1];
//     }

//     pst_LidarData->s32_TrackingNum--;
// }


// void DetermineDynamicOrStatic(LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I;

//     float32_t f32_Distance;
//     float32_t f32_GlobalSpeed_m_s;

//     TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

//     for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
//     {
//         f32_Distance = getDistance2d(pst_Tracking[s32_I].f32_GlobalX, pst_Tracking[s32_I].f32_GlobalY, pst_Tracking[s32_I].f32_PrevGlobalX, pst_Tracking[s32_I].f32_PrevGlobalY);

//         f32_GlobalSpeed_m_s = f32_Distance / f32_DeltaTime_s;

//         if(f32_GlobalSpeed_m_s * 3.6 > 10) // kph
//         {
//             pst_Tracking[s32_I].b_StateFlag = c_OBJECT_DYNAMIC;
//         }
//         else
//         {
//             pst_Tracking[s32_I].b_StateFlag = c_OBJECT_STATIC;
//         }
//     }
// }


// void SetMeasurementVelocity(TRACKING_t *pst_Tracking, int32_t s32_Idx)
// {
//     POINT_t arst_CurrentObject[4] = {};
//     POINT_t arst_PrevObject[4] = {};

//     arst_CurrentObject[0].f32_X = pst_Tracking[s32_Idx].f32_MaxX;
//     arst_CurrentObject[0].f32_Y = pst_Tracking[s32_Idx].f32_MaxY;
//     arst_CurrentObject[1].f32_X = pst_Tracking[s32_Idx].f32_MaxX;
//     arst_CurrentObject[1].f32_Y = pst_Tracking[s32_Idx].f32_MinY;
//     arst_CurrentObject[2].f32_X = pst_Tracking[s32_Idx].f32_MinX;
//     arst_CurrentObject[2].f32_Y = pst_Tracking[s32_Idx].f32_MaxY;
//     arst_CurrentObject[3].f32_X = pst_Tracking[s32_Idx].f32_MinX;
//     arst_CurrentObject[3].f32_Y = pst_Tracking[s32_Idx].f32_MinY;

//     arst_PrevObject[0].f32_X = pst_Tracking[s32_Idx].f32_PrevMaxX;
//     arst_PrevObject[0].f32_Y = pst_Tracking[s32_Idx].f32_PrevMaxY;
//     arst_PrevObject[1].f32_X = pst_Tracking[s32_Idx].f32_PrevMaxX;
//     arst_PrevObject[1].f32_Y = pst_Tracking[s32_Idx].f32_PrevMinY;
//     arst_PrevObject[2].f32_X = pst_Tracking[s32_Idx].f32_PrevMinX;
//     arst_PrevObject[2].f32_Y = pst_Tracking[s32_Idx].f32_PrevMaxY;
//     arst_PrevObject[3].f32_X = pst_Tracking[s32_Idx].f32_PrevMinX;
//     arst_PrevObject[3].f32_Y = pst_Tracking[s32_Idx].f32_PrevMinY;

//     int32_t s32_J;
//     int32_t s32_PointIdx;

//     float32_t f32_Distance;
//     float32_t f32_MinDistance = FLT_MAX;

//     for(s32_J = 0; s32_J < 4; s32_J++)
//     {
//         f32_Distance = getDistance2d(0, 0, arst_CurrentObject[s32_J].f32_X, arst_CurrentObject[s32_J].f32_Y);

//         if(f32_Distance < f32_MinDistance)
//         {
//             f32_MinDistance = f32_Distance;
//             s32_PointIdx = s32_J;
//         }
//     }

//     float32_t f32_OffsetX = arst_CurrentObject[s32_PointIdx].f32_X - arst_PrevObject[s32_PointIdx].f32_X;
//     float32_t f32_OffsetY = arst_CurrentObject[s32_PointIdx].f32_Y - arst_PrevObject[s32_PointIdx].f32_Y;

//     float32_t f32_DeltaL = pst_Tracking[s32_Idx].f32_ClusterX - (pst_Tracking[s32_Idx].f32_PrevClusterX + f32_OffsetX);
//     float32_t f32_DeltaW = pst_Tracking[s32_Idx].f32_ClusterY - (pst_Tracking[s32_Idx].f32_PrevClusterY + f32_OffsetY);

//     pst_Tracking[s32_Idx].st_Z(2) = ((pst_Tracking[s32_Idx].f32_ClusterX - f32_DeltaL) - pst_Tracking[s32_Idx].f32_PrevClusterX) / pst_Tracking[s32_Idx].f32_DeltaTime_s;
//     pst_Tracking[s32_Idx].st_Z(3) = ((pst_Tracking[s32_Idx].f32_ClusterY - f32_DeltaW) - pst_Tracking[s32_Idx].f32_PrevClusterY) / pst_Tracking[s32_Idx].f32_DeltaTime_s;
// }


// void SetGlobalPosition(TRACKING_t *pst_Tracking, PLANNING_DATA_t *pst_PlanningData, int32_t s32_Idx)
// {
//     VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

//     pst_Tracking[s32_Idx].f32_GlobalX = cos(pst_EgoVehicleData->f32_Yaw_rad_ENU) * pst_Tracking[s32_Idx].st_X(0) 
//                                         - sin(pst_EgoVehicleData->f32_Yaw_rad_ENU) * pst_Tracking[s32_Idx].st_X(1);
//     pst_Tracking[s32_Idx].f32_GlobalY = sin(pst_EgoVehicleData->f32_Yaw_rad_ENU) * pst_Tracking[s32_Idx].st_X(0)
//                                         + cos(pst_EgoVehicleData->f32_Yaw_rad_ENU) * pst_Tracking[s32_Idx].st_X(1);

//     pst_Tracking[s32_Idx].f32_GlobalX += pst_EgoVehicleData->f32_X_global;
//     pst_Tracking[s32_Idx].f32_GlobalY += pst_EgoVehicleData->f32_Y_global;
// }


// void UpdateMeasurement(LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
// {
//     int32_t s32_I;

//     TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

//     for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
//     {
//         if(pst_Tracking[s32_I].b_UpdateFlag == true && pst_Tracking[s32_I].b_InitlzFlag == false)
//         {
//             pst_Tracking[s32_I].f32_EndTime_s = getMillisecond() / 1000.f;
//             pst_Tracking[s32_I].f32_DeltaTime_s = pst_Tracking[s32_I].f32_EndTime_s - pst_Tracking[s32_I].f32_StartTime_s;
//             pst_Tracking[s32_I].f32_StartTime_s = getMillisecond() / 1000.f;

//             pst_Tracking[s32_I].st_Z(0) = pst_Tracking[s32_I].f32_ClusterX;
//             pst_Tracking[s32_I].st_Z(1) = pst_Tracking[s32_I].f32_ClusterY;
//             pst_Tracking[s32_I].st_Z(2) = (pst_Tracking[s32_I].f32_ClusterX - pst_Tracking[s32_I].f32_PrevClusterX) / pst_Tracking[s32_I].f32_DeltaTime_s;
//             pst_Tracking[s32_I].st_Z(3) = (pst_Tracking[s32_I].f32_ClusterY - pst_Tracking[s32_I].f32_PrevClusterY) / pst_Tracking[s32_I].f32_DeltaTime_s;

//             // SetMeasurementVelocity(pst_Tracking, s32_I);

//             pst_Tracking[s32_I].st_K = pst_Tracking[s32_I].st_P * pst_Tracking[s32_I].st_H.transpose() * (pst_Tracking[s32_I].st_H * pst_Tracking[s32_I].st_P * pst_Tracking[s32_I].st_H.transpose() + pst_Tracking[s32_I].st_R).inverse();

//             pst_Tracking[s32_I].st_P = pst_Tracking[s32_I].st_P - pst_Tracking[s32_I].st_K * pst_Tracking[s32_I].st_H * pst_Tracking[s32_I].st_P;

//             pst_Tracking[s32_I].st_X = pst_Tracking[s32_I].st_X + pst_Tracking[s32_I].st_K * (pst_Tracking[s32_I].st_Z - pst_Tracking[s32_I].st_H * pst_Tracking[s32_I].st_X);

//             pst_Tracking[s32_I].f32_PrevClusterX = pst_Tracking[s32_I].f32_ClusterX;
//             pst_Tracking[s32_I].f32_PrevClusterY = pst_Tracking[s32_I].f32_ClusterY;

//             pst_Tracking[s32_I].f32_X = pst_Tracking[s32_I].st_X(0);
//             pst_Tracking[s32_I].f32_Y = pst_Tracking[s32_I].st_X(1);
//             pst_Tracking[s32_I].f32_VelocityX_m_s = pst_Tracking[s32_I].st_X(2);
//             pst_Tracking[s32_I].f32_VelocityY_m_s = pst_Tracking[s32_I].st_X(3);

//             SetGlobalPosition(pst_Tracking, pst_PlanningData, s32_I);

//             pst_Tracking[s32_I].u32_EraseCnt = 0;
//         }
//         else
//         {
//             pst_Tracking[s32_I].b_InitlzFlag = false; // for new object
//             pst_Tracking[s32_I].f32_StartTime_s = getMillisecond() / 1000.f;
//         }

//         pst_Tracking[s32_I].b_UpdateFlag = false;
//     }
// }


// void PredictState(LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
// {
//     int32_t s32_I;

//     float32_t f32_Distance;
//     float32_t f32_Velocity_m_s;
//     float32_t f32_DeltaYaw_rad;

//     TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;
//     VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

//     MatrixXf st_Rotation = MatrixXf(4, 4);

//     for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
//     {
//         if(pst_Tracking[s32_I].b_InitlzFlag == false)
//         {
//             pst_Tracking[s32_I].f32_CurrYaw_rad = pst_EgoVehicleData->f32_Yaw_rad_ENU;
//             f32_DeltaYaw_rad = pst_Tracking[s32_I].f32_PrevYaw_rad - pst_Tracking[s32_I].f32_CurrYaw_rad;
//             pst_Tracking[s32_I].f32_PrevYaw_rad = pst_Tracking[s32_I].f32_CurrYaw_rad;

//             st_Rotation << cos(f32_DeltaYaw_rad), -sin(f32_DeltaYaw_rad), 0, 0,
//                            sin(f32_DeltaYaw_rad),  cos(f32_DeltaYaw_rad), 0, 0,
//                            0, 0, cos(f32_DeltaYaw_rad), -sin(f32_DeltaYaw_rad),
//                            0, 0, sin(f32_DeltaYaw_rad),  cos(f32_DeltaYaw_rad);

//             pst_Tracking[s32_I].st_PrevX = pst_Tracking[s32_I].st_X;
//             pst_Tracking[s32_I].f32_PrevGlobalX = pst_Tracking[s32_I].f32_GlobalX;
//             pst_Tracking[s32_I].f32_PrevGlobalY = pst_Tracking[s32_I].f32_GlobalY;

//             pst_Tracking[s32_I].st_X = st_Rotation * pst_Tracking[s32_I].st_X;

//             pst_Tracking[s32_I].st_A << 1, 0, f32_DeltaTime_s, 0,
//                                         0, 1, 0, f32_DeltaTime_s,
//                                         0, 0, 1, 0,
//                                         0, 0, 0, 1;        

//             pst_Tracking[s32_I].st_X = pst_Tracking[s32_I].st_A * pst_Tracking[s32_I].st_X;
  
//             pst_Tracking[s32_I].st_P = pst_Tracking[s32_I].st_A * pst_Tracking[s32_I].st_P * pst_Tracking[s32_I].st_A.transpose() + pst_Tracking[s32_I].st_Q;

//             pst_Tracking[s32_I].f32_X = pst_Tracking[s32_I].st_X(0);
//             pst_Tracking[s32_I].f32_Y = pst_Tracking[s32_I].st_X(1);
//             pst_Tracking[s32_I].f32_VelocityX_m_s = pst_Tracking[s32_I].st_X(2);
//             pst_Tracking[s32_I].f32_VelocityY_m_s = pst_Tracking[s32_I].st_X(3);

//             SetGlobalPosition(pst_Tracking, pst_PlanningData, s32_I);

//             f32_Distance = getDistance2d(pst_Tracking[s32_I].st_X(0), pst_Tracking[s32_I].st_X(1), pst_Tracking[s32_I].st_PrevX(0), pst_Tracking[s32_I].st_PrevX(1));
//             f32_Velocity_m_s = getDistance2d(pst_Tracking[s32_I].st_X(2), pst_Tracking[s32_I].st_X(3), 0, 0);

//             if(f32_Distance > 6)
//             {
//                 DeleteTrackingObject(pst_LidarData, s32_I);
//                 s32_I--;
//             }
//             else if(f32_Velocity_m_s * 3.6 > 200) //kph
//             {
//                 DeleteTrackingObject(pst_LidarData, s32_I);
//                 s32_I--;
//             }
//         }
//     }
// }


// void InitializeTrackingObject(LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
// {
//     int32_t s32_I;

//     TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;
//     VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

//     for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
//     {
//         if(pst_Tracking[s32_I].b_InitlzFlag == true)
//         {
//             pst_Tracking[s32_I].st_A.setZero();
//             pst_Tracking[s32_I].st_X.setZero();
//             pst_Tracking[s32_I].st_PrevX.setZero();
//             pst_Tracking[s32_I].st_Z.setZero();
//             pst_Tracking[s32_I].st_K.setZero();
//             pst_Tracking[s32_I].st_P.setZero();
//             pst_Tracking[s32_I].st_Q.setZero();
//             pst_Tracking[s32_I].st_R.setZero();

//             pst_Tracking[s32_I].st_X(0) = pst_Tracking[s32_I].f32_X;
//             pst_Tracking[s32_I].st_X(1) = pst_Tracking[s32_I].f32_Y;
//             pst_Tracking[s32_I].st_X(2) = pst_Tracking[s32_I].f32_VelocityX_m_s;
//             pst_Tracking[s32_I].st_X(3) = pst_Tracking[s32_I].f32_VelocityY_m_s;

//             pst_Tracking[s32_I].st_P(0, 0) = 10; // 10
//             pst_Tracking[s32_I].st_P(1, 1) = 10; // 10 
//             pst_Tracking[s32_I].st_P(2, 2) = 10; // 10
//             pst_Tracking[s32_I].st_P(3, 3) = 10; // 10

//             pst_Tracking[s32_I].st_Q(0, 0) = 1; // 0.1
//             pst_Tracking[s32_I].st_Q(1, 1) = 1; // 0.1
//             pst_Tracking[s32_I].st_Q(2, 2) = 1; // 0.1
//             pst_Tracking[s32_I].st_Q(3, 3) = 1; // 0.1

//             pst_Tracking[s32_I].st_R(0, 0) = 0.01; // 100
//             pst_Tracking[s32_I].st_R(1, 1) = 0.01; // 100
//             pst_Tracking[s32_I].st_R(2, 2) = 100; // 10
//             pst_Tracking[s32_I].st_R(3, 3) = 100; // 10

//             SetGlobalPosition(pst_Tracking, pst_PlanningData, s32_I);

//             pst_Tracking[s32_I].f32_PrevGlobalX = pst_Tracking[s32_I].f32_GlobalX;
//             pst_Tracking[s32_I].f32_PrevGlobalY = pst_Tracking[s32_I].f32_GlobalY;

//             pst_Tracking[s32_I].f32_CurrYaw_rad = pst_EgoVehicleData->f32_Yaw_rad_ENU;
//             pst_Tracking[s32_I].f32_PrevYaw_rad = pst_EgoVehicleData->f32_Yaw_rad_ENU;
//         }
//     }
// }


// void SearchNewObject(LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I, s32_J;
//     int32_t s32_NewObjIdx, s32_PrevNewObjIdx;
    
//     float32_t f32_Distance;
//     float32_t f32_MinDistance;

//     bool b_SaveCluObj;

//     TRACKING_t st_NewObject;

//     CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
//     CLUSTER_t *pst_PrevCluster = pst_LidarData->arst_PrevCluster;
//     TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

//     for(s32_I = 0; s32_I < pst_LidarData->s32_ClusterNum; s32_I++)
//     {
//         if(pst_Cluster[s32_I].b_TrackingFlag == true)
//         {
//             f32_MinDistance = FLT_MAX;
//             b_SaveCluObj = false;

//             for(s32_J = 0; s32_J < pst_LidarData->s32_PrevClusterNum; s32_J++)
//             {
//                 if(pst_PrevCluster[s32_J].b_TrackingFlag == true)
//                 {
//                     f32_Distance = getDistance2d(pst_Cluster[s32_I].f32_X, pst_Cluster[s32_I].f32_Y, pst_PrevCluster[s32_J].f32_X, pst_PrevCluster[s32_J].f32_Y);

//                     if(f32_Distance < f32_MinDistance && f32_Distance < 3)
//                     {
//                         f32_MinDistance = f32_Distance;
//                         s32_NewObjIdx = s32_I;
//                         s32_PrevNewObjIdx = s32_J;
//                         b_SaveCluObj = true;
//                         pst_Cluster[s32_I].b_TrackingFlag = false;
//                     }
//                 }
//             }

//             if(b_SaveCluObj == true)
//             {
//                 SaveTrackingObject(pst_LidarData, s32_NewObjIdx, s32_PrevNewObjIdx);
//             }
//         }
//     }

//     CopyClusterObject(pst_LidarData);
// }


// void SearchNearObject(LIDAR_DATA_t *pst_LidarData)
// {
//     int32_t s32_I, s32_J;
//     int32_t s32_BestObjIdx;

//     float32_t f32_Distance;
//     float32_t f32_MinDistance;

//     CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;
//     TRACKING_t *pst_Tracking = pst_LidarData->arst_Tracking;

//     for(s32_I = 0; s32_I < pst_LidarData->s32_TrackingNum; s32_I++)
//     {
//         f32_MinDistance = FLT_MAX;

//         for(s32_J = 0; s32_J < pst_LidarData->s32_ClusterNum; s32_J++)
//         {
//             if(pst_Cluster[s32_J].b_TrackingFlag == true)
//             {
//                 f32_Distance = getDistance2d(pst_Tracking[s32_I].f32_X, pst_Tracking[s32_I].f32_Y, pst_Cluster[s32_J].f32_X, pst_Cluster[s32_J].f32_Y);

//                 if(f32_Distance < f32_MinDistance && f32_Distance < 3)
//                 {
//                     f32_MinDistance = f32_Distance;
//                     s32_BestObjIdx = s32_J;
//                     pst_Tracking[s32_I].b_UpdateFlag = true;
//                 }
//             }
//         }

//         if(pst_Tracking[s32_I].b_UpdateFlag == true)
//         {
//             pst_Tracking[s32_I].f32_ClusterX = pst_Cluster[s32_BestObjIdx].f32_X;
//             pst_Tracking[s32_I].f32_ClusterY = pst_Cluster[s32_BestObjIdx].f32_Y;
//             pst_Tracking[s32_I].f32_MaxX = pst_Cluster[s32_BestObjIdx].f32_MaxX;
//             pst_Tracking[s32_I].f32_MaxY = pst_Cluster[s32_BestObjIdx].f32_MaxY;
//             pst_Tracking[s32_I].f32_MinX = pst_Cluster[s32_BestObjIdx].f32_MinX;
//             pst_Tracking[s32_I].f32_MinY = pst_Cluster[s32_BestObjIdx].f32_MinY;
//             pst_Cluster[s32_BestObjIdx].b_TrackingFlag = false;

//             for(s32_J = 0; s32_J < pst_LidarData->s32_ClusterNum; s32_J++)
//             {
//                 if(pst_Cluster[s32_J].b_TrackingFlag == true)
//                 {
//                     f32_Distance = getDistance2d(pst_Cluster[s32_BestObjIdx].f32_X, pst_Cluster[s32_BestObjIdx].f32_Y, pst_Cluster[s32_J].f32_X, pst_Cluster[s32_J].f32_Y);

//                     if(f32_Distance < 1)
//                     {
//                         pst_Cluster[s32_J].b_TrackingFlag = false;
//                     }
//                 }
//             }
//         }
//         else
//         {
//             pst_Tracking[s32_I].u32_EraseCnt++;

//             if(pst_Tracking[s32_I].u32_EraseCnt > 10)
//             {
//                 DeleteTrackingObject(pst_LidarData, s32_I);
//                 s32_I--;
//             }
//         }
//     }
// }


// void ObjectTracking(LIDAR_DATA_t *pst_LidarData, PLANNING_DATA_t *pst_PlanningData)
// {
//     f32_EndTime_s = getMillisecond() / 1000.f;
//     f32_DeltaTime_s =  f32_EndTime_s - f32_StartTime_s;
//     f32_StartTime_s = getMillisecond() / 1000.f;

//     SearchNearObject(pst_LidarData);

//     SearchNewObject(pst_LidarData);

//     InitializeTrackingObject(pst_LidarData, pst_PlanningData);

//     PredictState(pst_LidarData, pst_PlanningData);

//     UpdateMeasurement(pst_LidarData, pst_PlanningData);

//     DetermineDynamicOrStatic(pst_LidarData);

//     for(int i = 0; i < pst_LidarData->s32_TrackingNum; i++)
//     {
//         // if(pst_LidarData->arst_Tracking[i].b_StateFlag == c_OBJECT_DYNAMIC)
//         // {
//             // cout << "-----------------" << endl;
//             // cout << "idx : " << i << endl;
//             // cout << "X : " << pst_LidarData->arst_Tracking[i].f32_X << endl;
//             // cout << "Y : " << pst_LidarData->arst_Tracking[i].f32_Y << endl;
//             // cout << "VX : " << pst_LidarData->arst_Tracking[i].f32_VelocityX_m_s * 3.6f << endl;
//             // cout << "VY : " << pst_LidarData->arst_Tracking[i].f32_VelocityY_m_s * 3.6f << endl;
//             // cout << "Flag : " << pst_LidarData->arst_Tracking[i].b_StateFlag << endl;
//         // }
//     }
// }


// void RemoveGroundVelodyne128(LIDAR_DATA_t *pst_LidarData)
// {
//     // Threshold
//     float32_t f32_ThresholdAngleVer = 0.2f;
//     float32_t f32_ThresholdAngleDiffHor = 0.1f;
    
//     float32_t f32_ThresholdZDiffVer = 0.1f;
//     float32_t f32_ThresholdZDiffHor = 0.1f;

//     // Variables
//     int32_t s32_PointOffset, s32_PointIdx;
//     int32_t s32_PrevAzimuthPointIdx, s32_NextAzimuthPointIdx;
//     float32_t f32_X1, f32_X2, f32_X3, f32_X4, f32_X5, f32_Y1, f32_Y2, f32_Y3, f32_Y4, f32_Y5, f32_Z1, f32_Z2, f32_Z3, f32_Z4, f32_Z5;
//     float32_t f32_DistXY1, f32_DistXY2, f32_DistZ1, f32_DistZ2, f32_SlopeAngleVer, f32_SlopeAngleHor1, f32_SlopeAngleHor2;
//     float32_t f32_DistX1, f32_DistX2, f32_DistY1, f32_DistY2;
//     int32_t s32_LastGNDPointIdx;
    
//     for(s32_PointIdx = 0; s32_PointIdx < pst_LidarData->s32_PointNum; s32_PointIdx++)
//     {
//         if((s32_PointIdx & 127) == 0) s32_LastGNDPointIdx = -1;

//         if(pst_LidarData->arst_Point[s32_PointIdx].u8_Flag == c_POINT_INVALID) continue;
        
//         // Slope 기반 2차 제거
//         if(pst_LidarData->arst_Point[s32_PointIdx].f32_Distance > 0.5f)
//         {
//             f32_X1 = pst_LidarData->arst_Point[s32_PointIdx].f32_X;
//             f32_Y1 = pst_LidarData->arst_Point[s32_PointIdx].f32_Y;
//             f32_Z1 = pst_LidarData->arst_Point[s32_PointIdx].f32_Z;

//             f32_X2 = pst_LidarData->arst_Point[s32_PointIdx + 1].f32_X;
//             f32_Y2 = pst_LidarData->arst_Point[s32_PointIdx + 1].f32_Y;
//             f32_Z2 = pst_LidarData->arst_Point[s32_PointIdx + 1].f32_Z;

//             f32_DistX1 = f32_X2 - f32_X1;
//             f32_DistY1 = f32_Y2 - f32_Y1;
            
//             f32_DistXY1 = sqrtf((f32_DistX1 * f32_DistX1) + (f32_DistY1 * f32_DistY1));

//             // Channel 간 비교
//             if(f32_DistXY1 < 2.f)
//             {
//                 f32_DistZ1 = fabsf(f32_Z2 - f32_Z1);
//                 f32_SlopeAngleVer = atan2f(f32_DistZ1, f32_DistXY1);

//                 if
//                 (
//                     (f32_SlopeAngleVer < f32_ThresholdAngleVer) &&
//                     (pst_LidarData->arst_Point[s32_PointIdx].f32_Distance < pst_LidarData->arst_Point[s32_PointIdx + 1].f32_Distance) &&
//                     (f32_DistZ1 < f32_ThresholdZDiffVer)
//                 )
//                 {   
//                     pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_GROUND;
//                     s32_LastGNDPointIdx = s32_PointIdx;
//                 }
//                 else
//                 {
//                     pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_VALID;
//                 }
//             }
//             // Channel 내 비교
//             else
//             {
//                 s32_PrevAzimuthPointIdx = (s32_PointIdx + 230272) % 230400;
//                 s32_NextAzimuthPointIdx = (s32_PointIdx + 128) % 230400;

//                 f32_X3 = pst_LidarData->arst_Point[s32_PrevAzimuthPointIdx].f32_X;
//                 f32_Y3 = pst_LidarData->arst_Point[s32_PrevAzimuthPointIdx].f32_Y;
//                 f32_Z3 = pst_LidarData->arst_Point[s32_PrevAzimuthPointIdx].f32_Z;

//                 f32_X4 = pst_LidarData->arst_Point[s32_NextAzimuthPointIdx].f32_X;
//                 f32_Y4 = pst_LidarData->arst_Point[s32_NextAzimuthPointIdx].f32_Y;
//                 f32_Z4 = pst_LidarData->arst_Point[s32_NextAzimuthPointIdx].f32_Z;

//                 f32_DistX1 = f32_X3 - f32_X1;
//                 f32_DistY1 = f32_Y3 - f32_Y1;

//                 f32_DistX2 = f32_X4 - f32_X1;
//                 f32_DistY2 = f32_Y4 - f32_Y1;

//                 f32_DistXY1 = sqrtf((f32_DistX1 * f32_DistX1) + (f32_DistY1 * f32_DistY1));
//                 f32_DistZ1 = fabsf(f32_Z3 - f32_Z1);
//                 f32_SlopeAngleHor1 = atan2f(f32_DistZ1, f32_DistXY1);

//                 f32_DistXY2 = sqrtf((f32_DistX2 * f32_DistX2) + (f32_DistY2 * f32_DistY2));
//                 f32_DistZ2 = fabsf(f32_Z4 - f32_Z1);
//                 f32_SlopeAngleHor2 = atan2f(f32_DistZ2, f32_DistXY2);

//                 if(
//                     (fabsf(f32_SlopeAngleHor1 - f32_SlopeAngleHor2) < f32_ThresholdAngleDiffHor) &&
//                     (fabsf(f32_DistZ1 - f32_DistZ2) < f32_ThresholdZDiffHor)
//                 )
//                 {
//                     pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_GROUND;
//                     s32_LastGNDPointIdx = s32_PointIdx;
//                 }
//                 else
//                 {
//                     pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_VALID;
//                 }

//                 // 끊어지는 부분의 말단 점 제거
//                 if(pst_LidarData->arst_Point[s32_PointIdx].u8_Flag == c_POINT_VALID)
//                 {
//                     f32_X5 = pst_LidarData->arst_Point[s32_LastGNDPointIdx].f32_X;
//                     f32_Y5 = pst_LidarData->arst_Point[s32_LastGNDPointIdx].f32_Y;
//                     f32_Z5 = pst_LidarData->arst_Point[s32_LastGNDPointIdx].f32_Z;

//                     f32_DistX1 = f32_X5 - f32_X1;
//                     f32_DistY1 = f32_Y5 - f32_Y1;
//                     f32_DistXY1 = sqrtf(f32_DistX1 * f32_DistX1 + f32_DistY1 * f32_DistY1);
//                     f32_DistZ1 = fabsf(f32_Z5 - f32_Z1);

//                     f32_SlopeAngleVer = atan2f(f32_DistZ1, f32_DistXY1);
                    
//                     if(
//                         f32_SlopeAngleVer < f32_ThresholdAngleVer
//                     )
//                     {
//                         pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_GROUND;
//                     }
//                 }
                
//             }
//         }
//     }
// }

// // L-shape Fitting
// // // // float32_t calcCriterion_area(vector<float32_t> C1, vector<float32_t> C2)
// // // // {
// // // //     float32_t c1_max, c1_min, c2_max, c2_min;

// // // //     c1_max = *max_element(C1.begin(), C1.end());
// // // //     c1_min = *min_element(C1.begin(), C1.end());
// // // //     c2_max = *max_element(C2.begin(), C2.end());
// // // //     c2_min = *min_element(C2.begin(), C2.end());

// // // //     return -(c1_max - c1_min) * (c2_max - c2_min);
// // // // }

// // // // float32_t calcCriterion_closeness(vector<float32_t> C1, vector<float32_t> C2)
// // // // {
// // // //     float32_t c1_max, c1_min, c2_max, c2_min;
// // // //     float32_t d0 = 0.01;
// // // //     float32_t d;

// // // //     c1_max = *max_element(C1.begin(), C1.end());
// // // //     c1_min = *min_element(C1.begin(), C1.end());
// // // //     c2_max = *max_element(C2.begin(), C2.end());
// // // //     c2_min = *min_element(C2.begin(), C2.end());

// // // //     float32_t* D1 = new float32_t[C1.size()];
// // // //     float32_t* D2 = new float32_t[C2.size()];

// // // //     float32_t c1_norm, c2_norm;
// // // //     float32_t c1_min_norm = numeric_limits<float32_t>::max();
// // // //     float32_t c2_min_norm = numeric_limits<float32_t>::max();

// // // //     for(int i = 0; i < C1.size(); i++)
// // // //     {
// // // //         D1[i] = min(c1_max - C1[i], C1[i] - c1_min);
// // // //         D2[i] = min(c2_max - C2[i], C2[i] - c2_min);

// // // //     }

// // // //     float32_t beta = 0;

// // // //     for(int i = 1; i < C1.size(); i++)
// // // //     {
// // // //         d = max(min(D1[i], D2[i]), d0);
// // // //         beta += (1 / d);
// // // //     }

// // // //     return beta;

// // // // }

// // // // float32_t calcCriterion_var(vector<float32_t> C1, vector<float32_t> C2)
// // // // {
// // // //     float32_t c1_max, c1_min, c2_max, c2_min;

// // // //     float32_t* D1 = new float32_t[C1.size()];
// // // //     float32_t* D2 = new float32_t[C2.size()];

// // // //     vector<float32_t> E1, E2;
// // // //     float32_t E1_sum = 0;
// // // //     float32_t E2_sum = 0;
// // // //     float32_t E1_mean, E2_mean, E1_var, E2_var;

// // // //     float32_t gamma;

// // // //     c1_max = *max_element(C1.begin(), C1.end());
// // // //     c1_min = *min_element(C1.begin(), C1.end());
// // // //     c2_max = *max_element(C2.begin(), C2.end());
// // // //     c2_min = *min_element(C2.begin(), C2.end());

// // // //     for(int i = 0; i < C1.size(); i++)
// // // //     {
// // // //         D1[i] = min(c1_max - C1[i], C1[i] - c1_min);
// // // //         D2[i] = min(c2_max - C2[i], C2[i] - c2_min);

// // // //         if(D1[i] < D2[i]) E1.push_back(D1[i]);
// // // //         else if(D2[i] < D1[i]) E2.push_back(D2[i]);
// // // //     }

// // // //     for(int i = 0; i < E1.size(); i++) E1_sum += E1[i];
// // // //     for(int i = 0; i < E2.size(); i++) E2_sum += E2[i];

// // // //     E1_mean = E1_sum / E1.size();
// // // //     E2_mean = E2_sum / E2.size();

// // // //     for(int i = 0; i < E1.size(); i++)
// // // //     {
// // // //         E1_var += (pow(E1[i] - E1_mean , 2) / E1.size());
// // // //     }

// // // //     for(int i = 0; i < E2.size(); i++)
// // // //     {
// // // //         E2_var += (pow(E2[i] - E2_mean, 2) / E2.size());
// // // //     }

// // // //     gamma = - E1_var - E2_var;

// // // //     return gamma;
// // // // }

// // // // void getIntersectionPoint(float32_t a1, float32_t b1, float32_t c1, float32_t a2, float32_t b2, float32_t c2, float32_t a3, float32_t b3, float32_t c3, float32_t a4, float32_t b4, float32_t c4, vector<vector<float32_t>> *rect_coor)
// // // // {
// // // //     // 아래 2개 직교
// // // //     // a1 * x + b1 * y = c1
// // // //     // a2 * x + b2 * y = c2

// // // //     // 아래 2개 직교
// // // //     // a3 * x + b3 * y = c3
// // // //     // a4 * x + b4 * y = c4

// // // //     float32_t line12_x = (c1 - b1 * c2 / b2) / (a1 - b1 * a2 / b2);
// // // //     float32_t line12_y = (c1 - a1 * c2 / a2) / (b1 - a1 * b2 / a2);

// // // //     float32_t line23_x = (c2 - b2 * c3 / b3) / (a2 - b2 * a3 / b3);
// // // //     float32_t line23_y = (c2 - a2 * c3 / a3) / (b2 - a2 * b3 / a3);

// // // //     float32_t line14_x = (c1 - b1 * c4 / b4) / (a1 - b1 * a4 / b4);
// // // //     float32_t line14_y = (c1 - a1 * c4 / a4) / (b1 - a1 * b4 / a4);

// // // //     float32_t line34_x = (c3 - b3 * c4 / b4) / (a3 - b3 * a4 / b4);
// // // //     float32_t line34_y = (c3 - a3 * c4 / a4) / (b3 - a3 * b4 / a4);

// // // //     vector<float32_t> interPoint1 = {line12_x, line12_y};
// // // //     vector<float32_t> interPoint2 = {line23_x, line23_y};
// // // //     vector<float32_t> interPoint3 = {line14_x, line14_y};
// // // //     vector<float32_t> interPoint4 = {line34_x, line34_y};

// // // //     rect_coor->push_back(interPoint1);
// // // //     rect_coor->push_back(interPoint2);
// // // //     rect_coor->push_back(interPoint3);
// // // //     rect_coor->push_back(interPoint4);
// // // // }

// // // // void fitRectangle(Cluster *cluster, vector<vector<float32_t>> *rect_coor)
// // // // {
// // // //     map<float32_t, float32_t> Q;
// // // //     float32_t delta = 0.05;                    // theta 값을 얼마나 세밀하게 볼지 결정하는 값
// // // //     float32_t e1_hat[2], e2_hat[2];
// // // //     vector<float32_t> C1, C2;
// // // //     float32_t q;
// // // //     vector<float32_t> C1_star, C2_star;
// // // //     float32_t a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4;

// // // //     PointCloud* pcl = cluster->getCluster();

// // // //     rect_coor->clear();

// // // //     for(int clusterIdx = 0; clusterIdx < cluster->getSize(); clusterIdx++)
// // // //     {
// // // //         Point* points = pcl[clusterIdx].getPoints();

// // // //         for(float32_t theta = 0; theta < (M_PI / 2 - delta); theta += delta)
// // // //         {
// // // //             e1_hat[0] = cos(theta);
// // // //             e1_hat[1] = sin(theta);
// // // //             e2_hat[0] = -sin(theta);
// // // //             e2_hat[1] = cos(theta);

// // // //             for(int idx = 0; idx < pcl[clusterIdx].getValidSize(); idx++)
// // // //             {
// // // //                 C1.push_back(points[idx].f64_x * e1_hat[0] + points[idx].f64_y * e1_hat[1]);
// // // //                 C2.push_back(points[idx].f64_x * e2_hat[0] + points[idx].f64_y * e2_hat[1]);
// // // //             }

// // // //             q = calcCriterion_area(C1, C2);
// // // //             // q = calcCriterion_closeness(C1, C2);
// // // //             // q = calcCriterion_var(C1, C2);

// // // //             Q.insert(make_pair(theta, q));

// // // //             C1.clear();
// // // //             C2.clear();
// // // //         }

// // // //         auto max_q = max_element(Q.begin(), Q.end(),
// // // //                                 [](const auto &a, const auto &b) {
// // // //                                 return a.second < b.second;
// // // //                                 });

// // // //         float32_t theta_star = max_q->first; // q값이 최대인 theta

// // // //         for(int idx = 0; idx < pcl[clusterIdx].getValidSize(); idx++)
// // // //         {
// // // //             C1_star.push_back(points[idx].f64_x * cos(theta_star) + points[idx].f64_y * sin(theta_star));
// // // //             C2_star.push_back(points[idx].f64_x * -sin(theta_star) + points[idx].f64_y * cos(theta_star));
// // // //         }

// // // //         a1 = cos(theta_star);
// // // //         a2 = -sin(theta_star);
// // // //         a3 = cos(theta_star);
// // // //         a4 = -sin(theta_star);
// // // //         b1 = sin(theta_star);
// // // //         b2 = cos(theta_star);
// // // //         b3 = sin(theta_star);
// // // //         b4 = cos(theta_star);

// // // //         c1 = *min_element(C1_star.begin(), C1_star.end());
// // // //         c2 = *min_element(C2_star.begin(), C2_star.end());
// // // //         c3 = *max_element(C1_star.begin(), C1_star.end());
// // // //         c4 = *max_element(C2_star.begin(), C2_star.end());

// // // //         getIntersectionPoint(a1, b1, c1, a2, b2, c2, a3, b3, c3, a4, b4, c4, rect_coor);

// // // //         C1_star.clear();
// // // //         C2_star.clear();
// // // //     }
// // // // }
