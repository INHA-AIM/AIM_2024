const int32_t c_TOTAL_SECTOR_POINT_NUM = 80;
const int32_t c_SECTOR_NONGROUND = 0;
const int32_t c_SECTOR_UNKNOWN = 128;
const int32_t c_SECTOR_GROUND = 255;

typedef struct _SECTOR
{
  int32_t s32_PointNum = 0;
  int32_t ars32_PointIdx[c_TOTAL_SECTOR_POINT_NUM];
  int32_t s32_ClusterIdx;

  float32_t f32_DistanceSum = 0;
  float32_t f32_AzimuthSum = 0;
  float32_t f32_ElevationSum = 0;
  float32_t f32_XSum = 0; 
  float32_t f32_YSum = 0;
  float32_t f32_ZSum = 0;
  
  float32_t f32_DistanceMean;
  float32_t f32_AzimuthMean;
  float32_t f32_ElevationMean;
  float32_t f32_XMean = 0;
  float32_t f32_YMean = 0;
  float32_t f32_ZMean = 0;
  
  float32_t f32_CovEX; // Covariance Between Elevation and X
  float32_t f32_CovEY; // Covariance Between Elevation and Y
  float32_t f32_CovAX; // Covariance Between Azimuth and X
  float32_t f32_CovAY;  // Covariance Between Azimuth and Y
  float32_t f32_CovDZ; // Covariance Between Distance and Z
  float32_t f32_CovAZ; // Covariance Between Azimuth and Z
  float32_t f32_CovEZ; // Covariance Between Elevation and Z

  uint8_t u8_Flag;
} SECTOR_t;

typedef struct _VOXEL
{
  SECTOR_t arst_Sector[c_GRID_DISTANCE_SIZE * c_GRID_AZIMUTH_SIZE * c_GRID_ELEVATION_SIZE];
  int32_t ars32_FilledSectorIdx[c_GRID_DISTANCE_SIZE * c_GRID_AZIMUTH_SIZE * c_GRID_ELEVATION_SIZE];
  int32_t s32_SectorNum = 0;

} VOXEL_t;

void RemoveGroundVelodyne128(LIDAR_DATA_t *pst_LidarData)
{
    int32_t s32_IdxDist, s32_IdxAzim, s32_IdxElev;
    int32_t s32_PointIdx, s32_SectorPointIdx;
    uint16_t u16_VoxelIdx;
    int32_t s32_SectorIdx;
    int32_t s32_FilledSectorIdx;
    float32_t f32_DistCentralized, f32_AzimCentralized, f32_ElevCentralized, f32_XCentralized, f32_YCentralized, f32_ZCentralized;
    POINT_t* pst_Point;
    SECTOR_t* pst_Sector;
    int32_t s32_N;
    float32_t f32_N;

    // Initialization
    for (s32_SectorIdx = 0; s32_SectorIdx < pst_LidarData->st_Voxel.s32_SectorNum; s32_SectorIdx++)
    {
        s32_FilledSectorIdx = pst_LidarData->st_Voxel.ars32_FilledSectorIdx[s32_SectorIdx];
        pst_Sector = &pst_LidarData->st_Voxel.arst_Sector[s32_FilledSectorIdx];
        pst_Sector->s32_PointNum = 0;
        pst_Sector->s32_ClusterIdx = -1;
        
        pst_Sector->f32_DistanceSum = 0;
        pst_Sector->f32_AzimuthSum = 0;
        pst_Sector->f32_ElevationSum = 0;
        pst_Sector->f32_XSum = 0;
        pst_Sector->f32_YSum = 0;
        pst_Sector->f32_ZSum = 0;

        pst_Sector->f32_DistanceMean = 0;
        pst_Sector->f32_AzimuthMean = 0;
        pst_Sector->f32_ElevationMean = 0;
        pst_Sector->f32_XMean = 0;
        pst_Sector->f32_YMean = 0;
        pst_Sector->f32_ZMean = 0;

        pst_Sector->f32_CovEX = 0;
        pst_Sector->f32_CovEY = 0;
        pst_Sector->f32_CovAX = 0;
        pst_Sector->f32_CovAY = 0;
        pst_Sector->f32_CovDZ = 0;
        pst_Sector->f32_CovAZ = 0;
        pst_Sector->f32_CovEZ = 0;
        pst_Sector->u8_Flag = c_SECTOR_UNKNOWN;
    }

    pst_LidarData->st_Voxel.s32_SectorNum = 0;

    // Voxel 상의 index 계산
    for (s32_PointIdx = 0; s32_PointIdx < pst_LidarData->s32_PointNum; s32_PointIdx++)
    {
        if (pst_LidarData->arst_Point[s32_PointIdx].u8_Flag == c_POINT_VALID)
        {
            s32_IdxDist = (int32_t)((pst_LidarData->arst_Point[s32_PointIdx].f32_Distance - c_VOXEL_DISTANCE_MIN) / c_DISTANCE_LEAF_SIZE);
            s32_IdxAzim = (int32_t)((pst_LidarData->arst_Point[s32_PointIdx].f32_Azimuth_deg - c_VOXEL_AZIMUTH_MIN) / c_AZIMUTH_LEAF_SIZE);
            s32_IdxElev = (int32_t)((pst_LidarData->arst_Point[s32_PointIdx].f32_Elevation_deg - c_VOXEL_ELEVATION_MIN) / c_ELEVATION_LEAF_SIZE);

            if ((s32_IdxDist >= 0) && (s32_IdxDist < c_GRID_DISTANCE_SIZE) &&
                (s32_IdxAzim >= 0) && (s32_IdxAzim < c_GRID_AZIMUTH_SIZE) &&
                (s32_IdxElev >= 0) && (s32_IdxElev < c_GRID_ELEVATION_SIZE))
            {
                pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[0] = s32_IdxDist;
                pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[1] = s32_IdxAzim;
                pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[2] = s32_IdxElev;

                s32_SectorIdx = s32_IdxElev * (c_GRID_AZIMUTH_SIZE * c_GRID_DISTANCE_SIZE) + s32_IdxAzim * c_GRID_DISTANCE_SIZE + s32_IdxDist;
                pst_LidarData->st_Voxel.ars32_FilledSectorIdx[pst_LidarData->st_Voxel.s32_SectorNum++] = s32_SectorIdx;
                if (pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].s32_PointNum < c_TOTAL_SECTOR_POINT_NUM)
                {
                    pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].ars32_PointIdx[pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].s32_PointNum++] = s32_PointIdx;
                    pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].f32_DistanceSum += pst_LidarData->arst_Point[s32_PointIdx].f32_Distance;
                    pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].f32_AzimuthSum += pst_LidarData->arst_Point[s32_PointIdx].f32_Azimuth_deg;
                    pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].f32_ElevationSum += pst_LidarData->arst_Point[s32_PointIdx].f32_Elevation_deg;
                    pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].f32_XSum += pst_LidarData->arst_Point[s32_PointIdx].f32_X;
                    pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].f32_YSum += pst_LidarData->arst_Point[s32_PointIdx].f32_Y;
                    pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx].f32_ZSum += pst_LidarData->arst_Point[s32_PointIdx].f32_Z;
                }
                else continue;
            }
        }
    }

    for (s32_PointIdx = 0; s32_PointIdx < pst_LidarData->s32_PointNum; s32_PointIdx++)
    {
        s32_IdxDist = pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[0];
        s32_IdxAzim = pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[1];
        s32_IdxElev = pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[2];
        s32_SectorIdx = s32_IdxElev * (c_GRID_AZIMUTH_SIZE * c_GRID_DISTANCE_SIZE) + s32_IdxAzim * c_GRID_DISTANCE_SIZE + s32_IdxDist;
        pst_Sector = &pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx];
        s32_N = pst_Sector->s32_PointNum;
        f32_N = (float32_t)(s32_N);

        if (s32_N != 1)
        {
            pst_Sector->f32_DistanceMean = pst_Sector->f32_DistanceSum / f32_N;
            pst_Sector->f32_AzimuthMean = pst_Sector->f32_AzimuthSum / f32_N;
            pst_Sector->f32_ElevationMean = pst_Sector->f32_ElevationSum / f32_N;
            pst_Sector->f32_XMean = pst_Sector->f32_XSum / f32_N;
            pst_Sector->f32_YMean = pst_Sector->f32_YSum / f32_N;
            pst_Sector->f32_ZMean = pst_Sector->f32_ZSum / f32_N;

            f32_DistCentralized = pst_LidarData->arst_Point[s32_PointIdx].f32_Distance - pst_Sector->f32_DistanceMean;
            f32_AzimCentralized = pst_LidarData->arst_Point[s32_PointIdx].f32_Azimuth_deg - pst_Sector->f32_AzimuthMean;
            f32_ElevCentralized = pst_LidarData->arst_Point[s32_PointIdx].f32_Elevation_deg - pst_Sector->f32_ElevationMean;
            f32_XCentralized = pst_LidarData->arst_Point[s32_PointIdx].f32_X - pst_Sector->f32_XMean;
            f32_YCentralized = pst_LidarData->arst_Point[s32_PointIdx].f32_Y - pst_Sector->f32_YMean;
            f32_ZCentralized = pst_LidarData->arst_Point[s32_PointIdx].f32_Z - pst_Sector->f32_ZMean;

            pst_Sector->f32_CovEX += (f32_ElevCentralized * f32_XCentralized); 
            pst_Sector->f32_CovEY += (f32_ElevCentralized * f32_YCentralized); 
            pst_Sector->f32_CovAX += (f32_AzimCentralized * f32_XCentralized); 
            pst_Sector->f32_CovAY += (f32_AzimCentralized * f32_YCentralized); 

            pst_Sector->f32_CovDZ += (f32_DistCentralized * f32_ZCentralized); 
            pst_Sector->f32_CovAZ += (f32_AzimCentralized * f32_ZCentralized);
            pst_Sector->f32_CovEZ += (f32_ElevCentralized * f32_ZCentralized);
        }
        else
        {
            pst_Sector->f32_CovEX = pst_Sector->f32_CovEY = pst_Sector->f32_CovAX = pst_Sector->f32_CovAY = pst_Sector->f32_CovDZ = pst_Sector->f32_CovAZ = pst_Sector->f32_CovEZ = 0.f;
            pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_GROUND;
        }
    }

    for (s32_PointIdx = 0; s32_PointIdx < pst_LidarData->s32_PointNum; s32_PointIdx++)
    {
        s32_IdxDist = pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[0];
        s32_IdxAzim = pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[1];
        s32_IdxElev = pst_LidarData->arst_Point[s32_PointIdx].aru16_VoxelIdx[2];
        s32_SectorIdx = s32_IdxElev * (c_GRID_AZIMUTH_SIZE * c_GRID_DISTANCE_SIZE) + s32_IdxAzim * c_GRID_DISTANCE_SIZE + s32_IdxDist;
        pst_Sector = &pst_LidarData->st_Voxel.arst_Sector[s32_SectorIdx];

        if (pst_Sector->u8_Flag == c_SECTOR_UNKNOWN)
        {
            f32_N = (float32_t)(pst_Sector->s32_PointNum);

            pst_Sector->f32_CovEX /= f32_N;
            pst_Sector->f32_CovEY /= f32_N;
            pst_Sector->f32_CovAX /= f32_N;
            pst_Sector->f32_CovAY /= f32_N;

            pst_Sector->f32_CovDZ /= f32_N;
            pst_Sector->f32_CovAZ /= f32_N;
            pst_Sector->f32_CovEZ /= f32_N;
            
            if(pst_Sector->s32_PointNum >= 2)
            {
                if(
                fabsf(pst_Sector->f32_CovDZ) > 0.5f ||
                fabsf(pst_Sector->f32_CovAZ) > 0.05f ||
                fabsf(pst_Sector->f32_CovEZ) > 0.1f
                )
                {
                    pst_Sector->u8_Flag = c_SECTOR_NONGROUND;
                }
                else if(
                        (fabsf(pst_Sector->f32_CovEX) > 0.3f &&
                        fabsf(pst_Sector->f32_CovEY) < 0.1f)
                        ||
                        (fabsf(pst_Sector->f32_CovEX) < 0.1f &&
                        fabsf(pst_Sector->f32_CovEY) > 0.3f)
                        ||
                        (fabsf(pst_Sector->f32_CovAX) > 0.3f &&
                        fabsf(pst_Sector->f32_CovAY) < 0.1f)
                        ||
                        (fabsf(pst_Sector->f32_CovAX) < 0.1f &&
                        fabsf(pst_Sector->f32_CovAY) > 0.3f)
                        )
                {
                    pst_Sector->u8_Flag = c_SECTOR_GROUND;
                    pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_GROUND;
                }
                else
                {
                    pst_Sector->u8_Flag = c_SECTOR_GROUND;
                    pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_GROUND;
                }
            }
        }
        else if (pst_Sector->u8_Flag == c_SECTOR_GROUND)
        {
            pst_LidarData->arst_Point[s32_PointIdx].u8_Flag = c_POINT_GROUND;
        }
    }
}