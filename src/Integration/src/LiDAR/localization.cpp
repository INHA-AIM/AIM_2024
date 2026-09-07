#include "localization.h"

using idx_t = faiss::idx_t;

extern std::string s_OmniPointFileName;
extern VIEWER_PARAMETER_t st_ViewerParam;
float32_t f32_OmniPrevX;
float32_t f32_OmniPrevY;

faiss::IndexFlatL2 faiss_Index(c_OMNI_HISTOGRAM_SIZE);
extern int32_t s32_ICPIteration;
extern bool b_UpdateLocalMap;


void LidarLocalization(HMC_LIDAR_DATA_t *pst_LidarData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, OMNI_POINT_t *pst_OmniPoint, ICP_MAP_t *pst_ICPMap)
{
	int32_t s32_I, s32_J;
	float32_t f32_X, f32_Y, f32_Z, f32_Yaw_deg, f32_Yaw_rad;
	float32_t f32_Dist;
	float32_t f32_MinDist = 9999999.f;
	int32_t s32_NearIdx = -1;
	float32_t *pf32_Transform = pst_LidarData->arf32_ICPTransform;
	float32_t f32_DRFitness = 0;

    // lla2enu(pst_DeadReckoningData->st_GPS.f64_Latitude,
	//         pst_DeadReckoningData->st_GPS.f64_Longitude,
	//         0,
	//         pst_LidarData->f32_PredictX, pst_LidarData->f32_PredictY, pst_LidarData->f32_PredictZ);

	pst_LidarData->f32_PredictX = pst_DeadReckoningData->st_Vehicle_Odometry.f32_PredictX;
	pst_LidarData->f32_PredictY = pst_DeadReckoningData->st_Vehicle_Odometry.f32_PredictY;
	pst_LidarData->f32_PredictYaw_deg = -pst_DeadReckoningData->st_Vehicle_Odometry.f32_PredictYaw_deg + 90.f;

	GetICPPoint(pst_LidarData, pst_DeadReckoningData);

	if (pst_LidarData->st_ICPPoint.s32_PointNum > 0 && pst_ICPMap->s32_MapNum > 0)
    {
        // uint64_t u64_A = getMillisecond();
        for (s32_I = 0;s32_I < pst_ICPMap->s32_MapNum;s32_I++)
        {
            f32_X = pst_ICPMap->arst_PointCloud[s32_I].f32_X;
            f32_Y = pst_ICPMap->arst_PointCloud[s32_I].f32_Y;

            f32_Dist = getDistance2d(pst_LidarData->f32_PredictX, pst_LidarData->f32_PredictY, f32_X, f32_Y);

            if (f32_MinDist > f32_Dist)
            {
                f32_MinDist = f32_Dist;
                s32_NearIdx = s32_I;
            }

        }

        pst_LidarData->s32_ICPNearestMapIdx = s32_NearIdx;

        if (b_UpdateLocalMap == false)
        {
        	// printf("A %d %d\n", pst_LidarData->st_ICPPoint.s32_PointNum, pst_ICPMap->arst_PointCloud[s32_NearIdx].s32_PointNum);

	        f32_DRFitness = CUDA_ICP_KDTREE(&pst_LidarData->st_ICPPoint, 
				        	&pst_ICPMap->arst_PointCloud[s32_NearIdx], 
	        				s32_ICPIteration, 
	        				pst_LidarData->ars32_ICPNearestIdx, 
	        				pf32_Transform);
	        f32_X = pst_ICPMap->arst_PointCloud[s32_NearIdx].f32_X;
	        f32_Y = pst_ICPMap->arst_PointCloud[s32_NearIdx].f32_Y;
	        f32_Z = pst_ICPMap->arst_PointCloud[s32_NearIdx].f32_Z;
	        f32_Yaw_deg = pst_ICPMap->arst_PointCloud[s32_NearIdx].f32_Yaw_deg;
	        f32_Yaw_rad = deg2rad(-f32_Yaw_deg + 90);

			Eigen::Matrix4f st_Transform;
			st_Transform << pf32_Transform[0], pf32_Transform[1], pf32_Transform[2], pf32_Transform[3],
							pf32_Transform[4], pf32_Transform[5], pf32_Transform[6], pf32_Transform[7],
							pf32_Transform[8], pf32_Transform[9], pf32_Transform[10], pf32_Transform[11],
							pf32_Transform[12], pf32_Transform[13], pf32_Transform[14], pf32_Transform[15];

			Eigen::Matrix4f st_InverseTransform = st_Transform.inverse();

			for (s32_I = 0;s32_I < 4;s32_I++)
			{
				for (s32_J = 0;s32_J < 4;s32_J++)
				{
					pst_LidarData->arf32_ICPInverseTransform[s32_I * 4 + s32_J] = st_InverseTransform(s32_I, s32_J);
				}
			}
			if(f32_DRFitness < 0.24f)
			{
				pst_LidarData->f32_PredictYaw_deg = f32_Yaw_deg - rad2deg(atan2f(st_Transform(1, 0), st_Transform(0, 0)));
				getGlobalCoord(f32_X, f32_Y, deg2rad(-pst_LidarData->f32_PredictYaw_deg + 90.f),
							pst_LidarData->arf32_ICPTransform[3], pst_LidarData->arf32_ICPTransform[7], 
							pst_LidarData->f32_PredictX, pst_LidarData->f32_PredictY);
			}
			else
			{
				pst_LidarData->f32_PredictX = pst_DeadReckoningData->st_Vehicle_Odometry.f32_PredictX;
				pst_LidarData->f32_PredictY = pst_DeadReckoningData->st_Vehicle_Odometry.f32_PredictY;
				pst_LidarData->f32_PredictYaw_deg = -pst_DeadReckoningData->st_Vehicle_Odometry.f32_PredictYaw_deg + 90.f;
			}



	        // CUDA_ICP_KDTREE(&pst_ICPMap->arst_PointCloud[s32_NearIdx], 
	        // 				&pst_LidarData->st_ICPPoint, 
	        // 				s32_ICPIteration, 
	        // 				pst_LidarData->ars32_ICPNearestIdx, 
	        // 				pf32_Transform);

	        // f32_X = pst_ICPMap->arst_PointCloud[s32_NearIdx].f32_X;
	        // f32_Y = pst_ICPMap->arst_PointCloud[s32_NearIdx].f32_Y;
	        // f32_Z = pst_ICPMap->arst_PointCloud[s32_NearIdx].f32_Z;
	        // f32_Yaw_deg = pst_ICPMap->arst_PointCloud[s32_NearIdx].f32_Yaw_deg;
	        // f32_Yaw_rad = deg2rad(-f32_Yaw_deg + 90);

			// Eigen::Matrix4f st_Transform;
			// st_Transform << pf32_Transform[0], pf32_Transform[1], pf32_Transform[2], pf32_Transform[3],
			// 				pf32_Transform[4], pf32_Transform[5], pf32_Transform[6], pf32_Transform[7],
			// 				pf32_Transform[8], pf32_Transform[9], pf32_Transform[10], pf32_Transform[11],
			// 				pf32_Transform[12], pf32_Transform[13], pf32_Transform[14], pf32_Transform[15];

			// Eigen::Matrix4f st_InverseTransform = st_Transform.inverse();

			// for (s32_I = 0;s32_I < 4;s32_I++)
			// {
			// 	for (s32_J = 0;s32_J < 4;s32_J++)
			// 	{
			// 		pst_LidarData->arf32_ICPInverseTransform[s32_I * 4 + s32_J] = st_InverseTransform(s32_I, s32_J);
			// 	}
			// }

			// pst_LidarData->f32_PredictYaw_deg = f32_Yaw_deg - rad2deg(atan2f(st_InverseTransform(1, 0), st_InverseTransform(0, 0)));

	        // getGlobalCoord(f32_X, f32_Y, deg2rad(-pst_LidarData->f32_PredictYaw_deg + 90.f),
	        //                pst_LidarData->arf32_ICPInverseTransform[3], pst_LidarData->arf32_ICPInverseTransform[7], 
	        //                pst_LidarData->f32_PredictX, pst_LidarData->f32_PredictY);
	    }
    }
    else
    {
        pst_LidarData->s32_ICPNearestMapIdx = -1;
    }


    if (st_ViewerParam.s32_State & c_STATE_MODE_RECORD_MAP)
    {
    	ICPRecordMap(pst_LidarData, pst_ICPMap);
	}

    if (st_ViewerParam.s32_State & c_STATE_MODE_SAVE_MAP)
    {
    	ICPSaveMap(pst_LidarData, pst_ICPMap);
		st_ViewerParam.s32_State ^= c_STATE_MODE_SAVE_MAP;
	}

	if (st_ViewerParam.s32_State & c_STATE_MODE_LOAD_MAP)
	{
		ICPLoadMap(pst_ICPMap);
		st_ViewerParam.s32_State ^= c_STATE_MODE_LOAD_MAP;
	}

	// OmniFeatureExtraction(pst_LidarData, pst_OmniPoint);
	// OmniCoarseLocalization(pst_LidarData, pst_OmniPoint);
	// if (st_ViewerParam.s32_State & c_STATE_MODE_LOAD_MAP)
	// {
	// 	OmniLoadMap(pst_OmniPoint);
	// 	st_ViewerParam.s32_State ^= c_STATE_MODE_LOAD_MAP;
	// }
    // if (st_ViewerParam.s32_State & c_STATE_MODE_RECORD_MAP)
    // {
    // 	OmniSaveMap(&pst_LidarData->st_OmniFeature, pst_DeadReckoningData, pst_OmniPoint);
	// }	
}


void OmniFeatureExtraction(HMC_LIDAR_DATA_t *pst_LidarData, OMNI_POINT_t *pst_OmniPoint)
{
    memset(&pst_LidarData->st_OmniFeature, 0, sizeof(OMNI_FEATURE_t));

	int32_t s32_I, s32_J;
	OMNI_FEATURE_t *pst_Feature = &pst_LidarData->st_OmniFeature;
    float32_t arf32_Farthest[c_OMNI_FEATURE_MAX_NUM] = {0.f};
    int32_t ars32_FarthestIndex[c_OMNI_FEATURE_MAX_NUM];
    int32_t s32_Index;
    float32_t f32_R, f32_X, f32_Y, f32_Z, f32_X2, f32_Y2;


    for (s32_I = 0;s32_I < c_OMNI_FEATURE_MAX_NUM;s32_I++)
    {
        ars32_FarthestIndex[s32_I] = -1;
    }

    for (s32_I = 0;s32_I < pst_LidarData->s32_PointNum; s32_I += 1)
    {
        s32_Index = (int32_t)(pst_LidarData->arst_Point[s32_I].f32_Azimuth_deg + 0.5f) % 360;
        // s32_Index = max(min(359, s32_Index), 0);

        f32_X = pst_LidarData->arst_Point[s32_I].f32_X;
        f32_Y = pst_LidarData->arst_Point[s32_I].f32_Y;
        f32_Z = pst_LidarData->arst_Point[s32_I].f32_Z;
        f32_R = getDistance2d(0.f, 0.f, f32_X, f32_Y);

        if (f32_Z > 10.f && pst_LidarData->arst_Point[s32_I].u8_Flag != c_POINT_INVALID)
        {
            if ((ars32_FarthestIndex[s32_Index] == -1) || (f32_R > arf32_Farthest[s32_Index]))
            {
                ars32_FarthestIndex[s32_Index] = s32_I;
                arf32_Farthest[s32_Index] = f32_R;
            }
        }
    }

    for (s32_I = 0;s32_I < c_OMNI_FEATURE_MAX_NUM;s32_I++)
    {
        if (ars32_FarthestIndex[s32_I] == -1)
        {
        	pst_Feature->arb_Valid[s32_I] = false;
            continue;
        }
		
		s32_J = ars32_FarthestIndex[s32_I];
		pst_Feature->arb_Valid[s32_I] = true;       
		pst_Feature->arf32_X[s32_I] = pst_LidarData->arst_Point[s32_J].f32_X;
		pst_Feature->arf32_Y[s32_I] = pst_LidarData->arst_Point[s32_J].f32_Y;
    }


    for (s32_I = 0;s32_I < c_OMNI_FEATURE_MAX_NUM - 1;s32_I++)
    {
    	if (pst_Feature->arb_Valid[s32_I] == false)
    	{
    		continue;
    	}

    	f32_X = pst_Feature->arf32_X[s32_I];
    	f32_Y = pst_Feature->arf32_Y[s32_I];

    	for (s32_J = s32_I + 1; s32_J < c_OMNI_FEATURE_MAX_NUM;s32_J++)
    	{
	    	if (pst_Feature->arb_Valid[s32_J] == false)
	    	{
	    		continue;
	    	}

	    	f32_X2 = pst_Feature->arf32_X[s32_J];
	    	f32_Y2 = pst_Feature->arf32_Y[s32_J];
	    	f32_R = getDistance2d(f32_X, f32_Y, f32_X2, f32_Y2);

	    	pst_Feature->arf32_Histogram[min(c_OMNI_HISTOGRAM_SIZE - 1, (int32_t)(f32_R / c_OMNI_BINNING_SIZE))] += 1.f;
    	}
    }
}


void OmniCoarseLocalization(HMC_LIDAR_DATA_t *pst_LidarData, OMNI_POINT_t *pst_OmniPoint)
{	
	int32_t s32_I, s32_J;
	int32_t s32_Index;
    idx_t ars32_I[5] = {0};
    float32_t arf32_D[5] = {0};

    int32_t ars32_Result[360 * 360] = {0};
    float32_t arf32_Result[360 * 360] = {0};
    int32_t s32_MaxCnt = 0;
    int32_t s32_BestIndex = -1;
    int32_t s32_BestMatching = -1;

	if (faiss_Index.ntotal == 0)
	{
		;
	}
	else
	{
        faiss_Index.search(1, pst_LidarData->st_OmniFeature.arf32_Histogram, 5, arf32_D, ars32_I);

        for (s32_I = 0;s32_I < 5;s32_I++)
        {
        	s32_Index = ars32_I[s32_I];
        	pst_LidarData->ars32_CandidateIndex[s32_I] = s32_Index;

			// CUDA_BestMatching(&pst_OmniPoint->arst_Feature[s32_Index], 
			// 				  &pst_LidarData->st_OmniFeature, 
			// 				  ars32_Result, arf32_Result);

			// for (s32_J = 0;s32_J < 360 * 360;s32_J++)
			// {
			// 	if (ars32_Result[s32_J] > s32_MaxCnt)
			// 	{
			// 		s32_MaxCnt = ars32_Result[s32_J];
			// 		s32_BestIndex = s32_Index;
			// 		s32_BestMatching = s32_J;
			// 	}
			// }
        }

        // printf("%d %d\n", s32_MaxCnt, s32_BestIndex);
        pst_LidarData->s32_BestCandidateIndex = s32_BestIndex;
		pst_LidarData->s32_OmniCorrespondence = s32_BestMatching;
		pst_LidarData->s32_OmniMatchingScore = s32_MaxCnt; 

		// OmniPositionUpdate(pst_LidarData, pst_OmniPoint);

	}
}


void OmniLoadMap(OMNI_POINT_t *pst_OmniPoint)
{
	int32_t s32_Num = 0;
	FILE *f_MapFile = fopen(s_OmniPointFileName.c_str(), "rb");

	if (f_MapFile == NULL)
	{
		;
	}
	else
	{
		while (f_MapFile != NULL && !feof(f_MapFile))
		{
			fread(&pst_OmniPoint->arf32_PositionX[s32_Num], sizeof(float32_t), 1, f_MapFile);
			fread(&pst_OmniPoint->arf32_PositionY[s32_Num], sizeof(float32_t), 1, f_MapFile);
			fread(&pst_OmniPoint->arf32_PositionYaw_deg[s32_Num], sizeof(float32_t), 1, f_MapFile);
			fread(&pst_OmniPoint->arst_Feature[s32_Num], sizeof(OMNI_FEATURE_t), 1, f_MapFile);

			if (fabsf(pst_OmniPoint->arf32_PositionYaw_deg[s32_Num]) < 0.01f)
			{
				continue;
			}

			faiss_Index.add(1, pst_OmniPoint->arst_Feature[s32_Num].arf32_Histogram);
			s32_Num += 1;
		}

		pst_OmniPoint->s32_Num = s32_Num;
		fclose(f_MapFile);
	}
	
}

void OmniSaveMap(OMNI_FEATURE_t *pst_OmniFeature, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, OMNI_POINT_t *pst_OmniPoint)
{
	int32_t s32_I;
    float32_t f32_X, f32_Y, f32_Z;
    float32_t f32_D;
    float32_t f32_Yaw_deg = pst_DeadReckoningData->st_IMU.f32_Yaw_deg;

    lla2enu(pst_DeadReckoningData->st_GPS.f64_Latitude, 
            pst_DeadReckoningData->st_GPS.f64_Longitude,
            pst_DeadReckoningData->st_GPS.f64_Altitude, 
            f32_X, f32_Y, f32_Z);

    f32_D = getDistance2d(f32_X, f32_Y, f32_OmniPrevX, f32_OmniPrevY);

    if (f32_D > 5.f)
    {
    	s32_I = pst_OmniPoint->s32_Num;

    	pst_OmniPoint->arf32_PositionX[s32_I] = f32_X;
    	pst_OmniPoint->arf32_PositionY[s32_I] = f32_Y;
    	pst_OmniPoint->arf32_PositionYaw_deg[s32_I] = f32_Yaw_deg;

    	memcpy(&pst_OmniPoint->arst_Feature[s32_I], pst_OmniFeature, sizeof(OMNI_FEATURE_t));

		faiss_Index.add(1, pst_OmniFeature->arf32_Histogram);

		FILE *f_MapFile = fopen(s_OmniPointFileName.c_str(), "ab");

		if (f_MapFile != NULL)
		{
			fwrite(&f32_X, sizeof(float32_t), 1, f_MapFile);
			fwrite(&f32_Y, sizeof(float32_t), 1, f_MapFile);
			fwrite(&f32_Yaw_deg, sizeof(float32_t), 1, f_MapFile);
			fwrite(pst_OmniFeature, sizeof(OMNI_FEATURE_t), 1, f_MapFile);
			fclose(f_MapFile);
		}
		pst_OmniPoint->s32_Num += 1;

		f32_OmniPrevX = f32_X;
		f32_OmniPrevY = f32_Y;
    }
}

int32_t ars32_HashTable[c_DOWNSAMPLING_VOXEL_CAPACITY];
ICP_DOWN_POINT_t arst_HashTable[c_DOWNSAMPLING_VOXEL_CAPACITY];

void GetICPPoint(HMC_LIDAR_DATA_t *pst_LidarData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
{
	ICP_POINT_CLOUD_t *pst_ICP = &pst_LidarData->st_ICPPoint;
	int32_t s32_I;
	int32_t s32_J;
	int32_t s32_X, s32_Y, s32_Z;
	int32_t s32_PointNum = 0;
    float32_t f32_X, f32_Y, f32_Z;
    float32_t f32_E, f32_N, f32_U;
    float32_t f32_D;
    float32_t f32_Yaw_deg = pst_DeadReckoningData->st_IMU.f32_Yaw_deg;
    uint8_t u8_FlagA = 1;
    uint8_t u8_FlagB = 1;

	uint64_t u64_H1;
	uint64_t u64_H2;
	uint64_t u64_H3;
	uint64_t u64_Hash;

    lla2enu(pst_DeadReckoningData->st_GPS.f64_Latitude, 
            pst_DeadReckoningData->st_GPS.f64_Longitude,
            pst_DeadReckoningData->st_GPS.f64_Altitude, 
            f32_E, f32_N, f32_U);

	pst_ICP->f32_X = f32_E;
	pst_ICP->f32_Y = f32_N;
	pst_ICP->f32_Z = f32_U;
	pst_ICP->f32_Yaw_deg = f32_Yaw_deg;

	memset(ars32_HashTable, 0, sizeof(ars32_HashTable));

	for (s32_I = 0;s32_I < pst_LidarData->s32_PointNum && s32_PointNum < c_ICP_MAX_POINT_NUM;s32_I++)
	{
		f32_X = pst_LidarData->arst_Point[s32_I].f32_X;
		f32_Y = pst_LidarData->arst_Point[s32_I].f32_Y;
		f32_Z = pst_LidarData->arst_Point[s32_I].f32_Z;
		u64_Hash = pst_LidarData->arst_Point[s32_I].u64_Hash;
		ars32_HashTable[u64_Hash] = s32_I;
	}

    s32_PointNum = 0;
    for (s32_I = 0;s32_I < c_DOWNSAMPLING_VOXEL_CAPACITY && s32_PointNum < c_ICP_MAX_POINT_NUM;s32_I++)
    {
    	s32_J = ars32_HashTable[s32_I];
    	if (s32_J)
    	{
			if ((st_ViewerParam.s32_State & c_STATE_MODE_RECORD_MAP) == 0)
			{
				u8_FlagA += 1;
				u8_FlagA = u8_FlagA % 2;
				if (u8_FlagA)
				{
					continue;
				}
			}

   			f32_X = pst_LidarData->arst_Point[s32_J].f32_X;
			f32_Y = pst_LidarData->arst_Point[s32_J].f32_Y;
			f32_Z = pst_LidarData->arst_Point[s32_J].f32_Z;

	    	pst_ICP->arf32_X[s32_PointNum] = f32_X;
	    	pst_ICP->arf32_Y[s32_PointNum] = f32_Y;
	    	pst_ICP->arf32_Z[s32_PointNum++] = f32_Z;
    	}
    }

	pst_ICP->s32_PointNum = s32_PointNum;
	// printf("%d %d\n", s32_PointNum, st_ViewerParam.s32_State & c_STATE_MODE_RECORD_MAP);
}


void ICPLoadMap(ICP_MAP_t *pst_ICPMap)
{
	int32_t s32_Num = 0;
	FILE *f_MapFile = fopen(s_OmniPointFileName.c_str(), "rb");

	if (f_MapFile == NULL)
	{
		;
	}
	else
	{
		while (f_MapFile != NULL && !feof(f_MapFile))
		{
			fread(&pst_ICPMap->arst_PointCloud[s32_Num], sizeof(ICP_POINT_CLOUD_t), 1, f_MapFile);

			if (pst_ICPMap->arst_PointCloud[s32_Num].s32_PointNum == 0)
			{
				continue;
			}
			s32_Num += 1;

		}

		pst_ICPMap->s32_MapNum = s32_Num;
		fclose(f_MapFile);
	}

}

void ICPRecordMap(HMC_LIDAR_DATA_t *pst_LidarData, ICP_MAP_t *pst_ICPMap)
{
	int32_t s32_I;
	int32_t s32_J;
	int32_t s32_NearIdx = -1;
	int32_t s32_PointNum = 0;
    float32_t f32_X, f32_Y, f32_Z;
    float32_t f32_E, f32_N, f32_U, f32_Yaw_deg;
    float32_t f32_Dist, f32_MinDist = 99999.f;
	ICP_POINT_CLOUD_t *pst_ICP = &pst_LidarData->st_ICPPoint;

	f32_E = pst_ICP->f32_X;
	f32_N = pst_ICP->f32_Y;
	f32_U = pst_ICP->f32_Z;
	f32_Yaw_deg = pst_ICP->f32_Yaw_deg;

	if (fabsf(f32_Yaw_deg) < 0.1f)
	{
        pst_LidarData->s32_ICPNearestMapIdx = -1;
		return;
	}

    for (s32_I = 0;s32_I < pst_ICPMap->s32_MapNum;s32_I++)
    {
        f32_X = pst_ICPMap->arst_PointCloud[s32_I].f32_X;
        f32_Y = pst_ICPMap->arst_PointCloud[s32_I].f32_Y;

        f32_Dist = getDistance2d(f32_E, f32_N, f32_X, f32_Y);

        if (f32_MinDist > f32_Dist)
        {
            f32_MinDist = f32_Dist;
            s32_NearIdx = s32_I;
        }
    }

    if (b_UpdateLocalMap)
    {
   		ICPUpdateRecordMap(&pst_ICPMap->arst_PointCloud[s32_NearIdx], pst_ICP);
    }
    else if (s32_NearIdx == -1)
    {
    	s32_I = pst_ICPMap->s32_MapNum;
		CUDA_BUILD_KDTREE(pst_ICP);

    	memcpy(&pst_ICPMap->arst_PointCloud[s32_I], pst_ICP, sizeof(ICP_POINT_CLOUD_t));

		pst_ICPMap->s32_MapNum += 1;
    }
    else 
    {
    	if (f32_MinDist > 5.f)
	    {
	    	s32_I = pst_ICPMap->s32_MapNum;
			CUDA_BUILD_KDTREE(pst_ICP);

	    	memcpy(&pst_ICPMap->arst_PointCloud[s32_I], pst_ICP, sizeof(ICP_POINT_CLOUD_t));

			pst_ICPMap->s32_MapNum += 1;
	    }
	    else
	    {
   			ICPUpdateRecordMap(&pst_ICPMap->arst_PointCloud[s32_NearIdx], pst_ICP);
	    }
	}

    // else if (f32_MinDist > 5.f)
    // {
    // 	ICPAddMap(&pst_ICPMap->arst_PointCloud[s32_NearIdx], pst_ICP);

    // 	s32_I = pst_ICPMap->s32_MapNum;
	// 	CUDA_BUILD_KDTREE(pst_ICP);

    // 	memcpy(&pst_ICPMap->arst_PointCloud[s32_I], pst_ICP, sizeof(ICP_POINT_CLOUD_t));

	// 	pst_ICPMap->s32_MapNum += 1;
    // }
}


void ICPSaveMap(HMC_LIDAR_DATA_t *pst_LidarData, ICP_MAP_t *pst_ICPMap)
{
	int32_t s32_I;

	FILE *f_MapFile = fopen(s_OmniPointFileName.c_str(), "wb");

	for (s32_I = 0;s32_I < pst_ICPMap->s32_MapNum;s32_I++)
	{
		if (f_MapFile != NULL)
		{
			fwrite(&pst_ICPMap->arst_PointCloud[s32_I], sizeof(ICP_POINT_CLOUD_t), 1, f_MapFile);
		}
		else
		{
			break;
		}
	}

	fclose(f_MapFile);
}

void ICPPointPreprocessing(ICP_POINT_CLOUD_t *pst_Map, ICP_POINT_CLOUD_t *pst_New)
{
	int32_t s32_I = 0;
	float32_t f32_X, f32_Y, f32_Z;
	float32_t f32_RX, f32_RY, f32_RZ;
	float32_t f32_AX, f32_AY, f32_AZ, f32_AYaw_rad, f32_AYaw_deg;
	float32_t f32_BX, f32_BY, f32_BZ, f32_BYaw_rad;
	float32_t f32_CYaw_rad;

	Eigen::Matrix3f st_R;

	f32_AX = pst_Map->f32_X; 
	f32_AY = pst_Map->f32_Y; 
	f32_AZ = pst_Map->f32_Z; 
	f32_AYaw_deg = pst_Map->f32_Yaw_deg; 
	f32_AYaw_rad = deg2rad(f32_AYaw_deg); 

	f32_BX = pst_New->f32_X; 
	f32_BY = pst_New->f32_Y; 
	f32_BZ = pst_New->f32_Z; 
	f32_BYaw_rad = deg2rad(pst_New->f32_Yaw_deg); 

	f32_CYaw_rad = f32_AYaw_rad - f32_BYaw_rad;

	st_R << cosf(f32_CYaw_rad), -sinf(f32_CYaw_rad), 0,
			sinf(f32_CYaw_rad), cosf(f32_CYaw_rad), 0, 
			0, 0, 1;

	f32_RX = f32_BX - f32_AX;
	f32_RY = f32_BY - f32_AY;
	f32_RZ = f32_BZ - f32_AZ;

	getLocalCoord(0, 0, -deg2rad(f32_AYaw_deg - 90), f32_RX, f32_RY, f32_RX, f32_RY);

	for (s32_I = 0; s32_I < pst_New->s32_PointNum; s32_I++)
	{
		f32_X = pst_New->arf32_X[s32_I];
		f32_Y = pst_New->arf32_Y[s32_I];
		f32_Z = pst_New->arf32_Z[s32_I];

		pst_New->arf32_X[s32_I] = f32_X * st_R(0, 0) + f32_Y * st_R(0, 1) + f32_Z * st_R(0, 2) + f32_RX;
		pst_New->arf32_Y[s32_I] = f32_X * st_R(1, 0) + f32_Y * st_R(1, 1) + f32_Z * st_R(1, 2) + f32_RY;
		pst_New->arf32_Z[s32_I] = f32_X * st_R(2, 0) + f32_Y * st_R(2, 1) + f32_Z * st_R(2, 2) + f32_RZ;
	}


}


void ICPAddMap(ICP_POINT_CLOUD_t *pst_Map, ICP_POINT_CLOUD_t *pst_New)
{
	float32_t f32_X, f32_Y, f32_Z, f32_Yaw_deg, f32_Yaw_rad;
	float32_t arf32_Transform[16];
	int32_t ars32_NearestIndex[c_ICP_MAX_POINT_NUM] = {0};

	ICPPointPreprocessing(pst_Map, pst_New);

	CUDA_ICP_KDTREE(pst_New, 
				    pst_Map,
    				s32_ICPIteration, 
    				ars32_NearestIndex, 
    				arf32_Transform);

    f32_X = pst_Map->f32_X;
    f32_Y = pst_Map->f32_Y;
    f32_Z = pst_Map->f32_Z;
    f32_Yaw_deg = pst_Map->f32_Yaw_deg;
    f32_Yaw_rad = deg2rad(-f32_Yaw_deg + 90);

	pst_New->f32_Yaw_deg = f32_Yaw_deg - rad2deg(atan2f(arf32_Transform[4], arf32_Transform[0]));

    getGlobalCoord(f32_X, f32_Y, deg2rad(-f32_Yaw_deg + 90.f),
                   arf32_Transform[3], arf32_Transform[7], 
                   pst_New->f32_X, pst_New->f32_Y);

    pst_New->f32_Z = f32_Z + arf32_Transform[11];
}


void ICPUpdateRecordMap(ICP_POINT_CLOUD_t *pst_Map, ICP_POINT_CLOUD_t *pst_New)
{
	ICPPointPreprocessing(pst_Map, pst_New);
	// std::map<std::tuple<int32_t, int32_t, int32_t>, std::tuple<float32_t, float32_t, float32_t> > st_DownsampleMap;
	// std::map<uint64_t, std::tuple<float32_t, float32_t, float32_t> > st_DownsampleMap;

	int32_t s32_I;
	int32_t s32_PointNum = pst_Map->s32_PointNum;
	uint64_t u64_H1;
	uint64_t u64_H2;
	uint64_t u64_H3;
	uint64_t u64_Hash;
	int32_t s32_X, s32_Y, s32_Z;
	float32_t f32_X, f32_Y, f32_Z;
	float32_t f32_RX, f32_RY, f32_RZ;
	std::tuple<int32_t, int32_t, int32_t> st_Key;
	uint8_t u8_FlagA = 1;
	
	int32_t ars32_NearestIndex[c_ICP_MAX_POINT_NUM] = {0};
	float32_t arf32_Transform[16];

    float32_t f32_Fitness = CUDA_ICP_KDTREE(pst_New, 
								        	pst_Map, 
						    				s32_ICPIteration, 
						    				ars32_NearestIndex, 
						    				arf32_Transform);

	printf("Update Local Map : %.4f\n", f32_Fitness);

	memset(ars32_HashTable, 0, sizeof(ars32_HashTable));
	memset(arst_HashTable, 0, sizeof(arst_HashTable));

    for (s32_I = 0;s32_I < pst_Map->s32_PointNum;s32_I++)
    {
    	f32_X = pst_Map->arf32_X[s32_I];
    	f32_Y = pst_Map->arf32_Y[s32_I];
    	f32_Z = pst_Map->arf32_Z[s32_I];   	

    	s32_X = (int32_t)(f32_X / c_ICP_DOWNSAMPLING_SIZE_XY);
    	s32_Y = (int32_t)(f32_Y / c_ICP_DOWNSAMPLING_SIZE_XY);
    	s32_Z = (int32_t)(f32_Z / c_ICP_DOWNSAMPLING_SIZE_Z);

    	u64_Hash = 0;
	    // u64_Hash ^= std::hash<int>{}(s32_X) + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2);
	    // u64_Hash ^= std::hash<int>{}(s32_Y) + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2);
	    // u64_Hash ^= std::hash<int>{}(s32_Z) + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2);

	    u64_Hash ^= ((uint64_t)s32_X + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
	    u64_Hash ^= ((uint64_t)s32_Y + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
	    u64_Hash ^= ((uint64_t)s32_Z + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
	    u64_Hash %= c_DOWNSAMPLING_VOXEL_CAPACITY;

	    arst_HashTable[u64_Hash].f32_X = f32_X;
	    arst_HashTable[u64_Hash].f32_Y = f32_Y;
	    arst_HashTable[u64_Hash].f32_Z = f32_Z;
	    ars32_HashTable[u64_Hash] = 1;
    }


    for (s32_I = 0;s32_I < pst_New->s32_PointNum;s32_I++)
    {
    	f32_X = pst_New->arf32_X[s32_I];
    	f32_Y = pst_New->arf32_Y[s32_I];
    	f32_Z = pst_New->arf32_Z[s32_I];

		f32_RX = f32_X * arf32_Transform[0] + f32_Y * arf32_Transform[1] + f32_Z * arf32_Transform[2] + arf32_Transform[3];
		f32_RY = f32_X * arf32_Transform[4] + f32_Y * arf32_Transform[5] + f32_Z * arf32_Transform[6] + arf32_Transform[7];
		f32_RZ = f32_X * arf32_Transform[8] + f32_Y * arf32_Transform[9] + f32_Z * arf32_Transform[10] + arf32_Transform[11];

		pst_New->arf32_X[s32_I] = f32_RX;
		pst_New->arf32_Y[s32_I] = f32_RY;
		pst_New->arf32_Z[s32_I] = f32_RZ;

    	s32_X = (int32_t)(f32_RX / c_ICP_DOWNSAMPLING_SIZE_XY);
    	s32_Y = (int32_t)(f32_RY / c_ICP_DOWNSAMPLING_SIZE_XY);
    	s32_Z = (int32_t)(f32_RZ / c_ICP_DOWNSAMPLING_SIZE_Z);

    	u64_Hash = 0;
	    u64_Hash ^= ((uint64_t)s32_X + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
	    u64_Hash ^= ((uint64_t)s32_Y + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
	    u64_Hash ^= ((uint64_t)s32_Z + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
	    u64_Hash %= c_DOWNSAMPLING_VOXEL_CAPACITY;

	    arst_HashTable[u64_Hash].f32_X = f32_RX;
	    arst_HashTable[u64_Hash].f32_Y = f32_RY;
	    arst_HashTable[u64_Hash].f32_Z = f32_RZ;
	    ars32_HashTable[u64_Hash] = 1;
    }

    s32_PointNum = 0;
    for (s32_I = 0;s32_I < c_DOWNSAMPLING_VOXEL_CAPACITY && s32_PointNum < c_ICP_MAX_POINT_NUM;s32_I++)
    {
    	if (ars32_HashTable[s32_I] == 0)
    	{
    		continue;
    	}
		f32_X = arst_HashTable[s32_I].f32_X;
		f32_Y = arst_HashTable[s32_I].f32_Y;
		f32_Z = arst_HashTable[s32_I].f32_Z;

    	pst_Map->arf32_X[s32_PointNum] = f32_X;
    	pst_Map->arf32_Y[s32_PointNum] = f32_Y;
    	pst_Map->arf32_Z[s32_PointNum++] = f32_Z;
    }

    printf("Updated Point Num %d -> %d\n", pst_Map->s32_PointNum, s32_PointNum);
    pst_Map->s32_PointNum = s32_PointNum;
	CUDA_BUILD_KDTREE(pst_Map);
}





void OmniPositionUpdate(HMC_LIDAR_DATA_t *pst_LidarData, OMNI_POINT_t* pst_OmniPoint)
{
	int32_t s32_BestIndex = pst_LidarData->s32_BestCandidateIndex;
	int32_t s32_BestMatching = pst_LidarData->s32_OmniCorrespondence;

	int32_t s32_I = s32_BestMatching / 360;
	int32_t s32_J = s32_BestMatching % 360;

	float32_t f32_X = pst_OmniPoint->arf32_PositionX[s32_BestIndex];
	float32_t f32_Y = pst_OmniPoint->arf32_PositionY[s32_BestIndex];
	float32_t f32_Yaw_rad = deg2rad(pst_OmniPoint->arf32_PositionYaw_deg[s32_BestIndex]);

	float32_t f32_X1 = 0.f, f32_Y1 = 0.f;
	float32_t f32_X2 = 0.f, f32_Y2 = 0.f;
	float32_t f32_X3 = 0.f, f32_Y3 = 0.f;
	float32_t f32_X4 = 0.f, f32_Y4 = 0.f;
	float32_t f32_Azimuth12_rad = 0.f, f32_Azimuth34_rad = 0.f;

	float32_t f32_Rotation_rad = 0.f;
	float32_t f32_TX = 0.f, f32_TY = 0.f;

	float32_t f32_OriginX = 0.f, f32_OriginY = 0.f;
	float32_t f32_RotateX = 0.f, f32_RotateY = 0.f;
	float32_t f32_ResultX = 0.f, f32_ResultY = 0.f;
	float32_t f32_ForwardX = 0.f, f32_ForwardY = 0.f;
	float32_t f32_ResultRotation_rad = 0.f;

	f32_X1 = pst_OmniPoint->arst_Feature[s32_BestIndex].arf32_X[s32_I];
	f32_Y1 = pst_OmniPoint->arst_Feature[s32_BestIndex].arf32_Y[s32_I];
	f32_X2 = pst_OmniPoint->arst_Feature[s32_BestIndex].arf32_X[(s32_I + 2) % 360];
	f32_Y2 = pst_OmniPoint->arst_Feature[s32_BestIndex].arf32_Y[(s32_I + 2) % 360];

	f32_Azimuth12_rad = atan2f(f32_Y2 - f32_Y1, f32_X2 - f32_X1);

	f32_X3 = pst_LidarData->st_OmniFeature.arf32_X[s32_J];
	f32_Y3 = pst_LidarData->st_OmniFeature.arf32_Y[s32_J];
	f32_X4 = pst_LidarData->st_OmniFeature.arf32_X[(s32_J + 2) % 360];
	f32_Y4 = pst_LidarData->st_OmniFeature.arf32_Y[(s32_J + 2) % 360];

	f32_Azimuth34_rad = atan2f(f32_Y4 - f32_Y3, f32_X4 - f32_X3);

	f32_Rotation_rad = f32_Azimuth12_rad - f32_Azimuth34_rad;

	//printf("%f\n", rad2deg(f32_Rotation));

	f32_OriginX = -f32_X3;
	f32_OriginY = -f32_Y3;

	f32_RotateX = (f32_OriginX * cosf(f32_Rotation_rad) - f32_OriginY * sinf(f32_Rotation_rad)) + f32_X1;
	f32_RotateY = (f32_OriginX * sinf(f32_Rotation_rad) + f32_OriginY * cosf(f32_Rotation_rad)) + f32_Y1;

	f32_ResultX = (f32_RotateX * cosf(f32_Yaw_rad) - f32_RotateY * sinf(f32_Yaw_rad));
	f32_ResultY = (f32_RotateX * sinf(f32_Yaw_rad) + f32_RotateY * cosf(f32_Yaw_rad));

	//f32_ForwardX = 10 - f32_X3;
	//f32_ForwardY = 0 - f32_Y3;

	//f32_RotateX = (f32_ForwardX * cosf(f32_Rotation_rad) - f32_ForwardY * sinf(f32_Rotation_rad)) + f32_X1;
	//f32_RotateY = (f32_ForwardX * sinf(f32_Rotation_rad) + f32_ForwardY * cosf(f32_Rotation_rad)) + f32_Y1;

	//f32_ForwardX = (f32_RotateX * cosf(f32_Yaw_rad) - f32_RotateY * sinf(f32_Yaw_rad)) + f32_X;
	//f32_ForwardY = (f32_RotateX * sinf(f32_Yaw_rad) + f32_RotateY * cosf(f32_Yaw_rad)) + f32_Y;

	//printf("Yaw? %f %f\n", f32_RetYaw, rad2deg(atan2f(f32_ForwardY - f32_RetY, f32_ForwardX - f32_RetX)));

	pst_LidarData->f32_PredictX = f32_X + f32_ResultX;
	pst_LidarData->f32_PredictY = f32_Y + f32_ResultY;
	pst_LidarData->f32_PredictYaw_deg = rad2deg(f32_Rotation_rad + f32_Yaw_rad);
}


void GetVehicleHeaidngUsingLane(HMC_LIDAR_DATA_t *pst_LidarData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, PLANNING_DATA_t *pst_PlanningData)
{
	int32_t s32_I, s32_J, s32_K, s32_L, s32_Lane;
	float32_t f32_E, f32_N, f32_U;
	float32_t f32_X, f32_Y, f32_Z, f32_Dist, f32_Yaw_deg;
	float32_t f32_X2, f32_Y2;
	float32_t f32_LocalX, f32_LocalY;
	float32_t f32_LocalX2, f32_LocalY2;
	float32_t f32_DX, f32_DY;
	uint8_t u8_Flag, u8_Intensity;
	int32_t s32_NearIdx;

    lla2enu(pst_DeadReckoningData->st_GPS.f64_Latitude, 
            pst_DeadReckoningData->st_GPS.f64_Longitude,
            0, 
            f32_E, f32_N, f32_U);

    PATH_t *pst_Boundary[] = {&pst_PlanningData->st_Boundary1_global,
                              &pst_PlanningData->st_Boundary2_global,
                              &pst_PlanningData->st_Boundary3_global,
                              &pst_PlanningData->st_Boundary4_global,
                              &pst_PlanningData->st_Boundary5_global,
                              &pst_PlanningData->st_Boundary6_global};

    float32_t arf32_LaneCandPointX[1000] = {0.f};
    float32_t arf32_LaneCandPointY[1000] = {0.f};
    float32_t arf32_LanePointX[2000] = {0.f};
    float32_t arf32_LanePointY[2000] = {0.f};
    int32_t s32_LaneCandPointNum = 0;
    int32_t s32_LanePointNum = 0;

    std::map<pair<int32_t, int32_t>,  int32_t > st_LaneCandidate;

    for (s32_I = 0;s32_I < pst_LidarData->s32_PointNum;s32_I++)
    {
        u8_Intensity = pst_LidarData->arst_Point[s32_I].u8_Intensity;
        u8_Flag = pst_LidarData->arst_Point[s32_I].u8_Flag;
        f32_X = pst_LidarData->arst_Point[s32_I].f32_X;
        f32_Y = pst_LidarData->arst_Point[s32_I].f32_Y;
        f32_Z = pst_LidarData->arst_Point[s32_I].f32_Z;
        f32_Dist = pst_LidarData->arst_Point[s32_I].f32_Distance;

        if (u8_Flag != c_POINT_GROUND || f32_Dist > 20.f)
        {
            continue;
        }

        if (u8_Intensity > 100.f)
        {
            st_LaneCandidate.insert(make_pair(make_pair((int32_t)(f32_X * 10.f + 200.f), (int32_t)(f32_Y * 10.f + 200.f)), s32_I));
        }
    }


    for (auto st_Point : st_LaneCandidate)
    {
        s32_I = st_Point.second;
        f32_X = pst_LidarData->arst_Point[s32_I].f32_X;
        f32_Y = pst_LidarData->arst_Point[s32_I].f32_Y;
        f32_Z = pst_LidarData->arst_Point[s32_I].f32_Z;

        arf32_LaneCandPointX[s32_LaneCandPointNum] = f32_X;
        arf32_LaneCandPointY[s32_LaneCandPointNum++] = f32_Y;

        if (s32_LaneCandPointNum >= 1000)
        {
        	break;
        }
    }

    for (int s32_Lane : {0, 1, 2, 3, 4, 5})
    {
        if (pst_Boundary[s32_Lane]->s32_Num == 0)
        {
            continue;
        }

        CalcNearestDistIdx(f32_E, f32_N, 
                        pst_Boundary[s32_Lane]->arf32_X,
                        pst_Boundary[s32_Lane]->arf32_Y,
                        pst_Boundary[s32_Lane]->s32_Num,
                        f32_Dist, s32_NearIdx);

        for (s32_I = -10; s32_I < 10; s32_I += 1)
        {
            s32_J = (s32_NearIdx + s32_I + pst_Boundary[s32_Lane]->s32_Num) % pst_Boundary[s32_Lane]->s32_Num;
            s32_K = (s32_NearIdx + s32_I + pst_Boundary[s32_Lane]->s32_Num + 1) % pst_Boundary[s32_Lane]->s32_Num;

            f32_X = pst_Boundary[s32_Lane]->arf32_X[s32_J]; 
            f32_Y = pst_Boundary[s32_Lane]->arf32_Y[s32_J]; 
            f32_X2 = pst_Boundary[s32_Lane]->arf32_X[s32_K]; 
            f32_Y2 = pst_Boundary[s32_Lane]->arf32_Y[s32_K]; 

            f32_DX = (f32_X2 - f32_X) / 10.f;
            f32_DY = (f32_Y2 - f32_Y) / 10.f;

            for (s32_L = 0;s32_L < 10;s32_L++)
            {
            	arf32_LanePointX[s32_LanePointNum] = f32_X + (f32_DX * s32_L);
            	arf32_LanePointY[s32_LanePointNum++] = f32_Y + (f32_DY * s32_L);
            }

            // getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg), f32_X, f32_Y, f32_LocalX, f32_LocalY);
            // getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg), f32_X2, f32_Y2, f32_LocalX2, f32_LocalY2);

            // if (s32_Lane == 3)
            // {
            //     st_BoundLeft.push_back(make_pair(f32_LocalX, f32_LocalY));
            // }
            // else
            // {
            //     st_BoundRight.push_back(make_pair(f32_LocalX, f32_LocalY));
            // }
        }
    }

    // printf("LaneNum : %d %d\n", s32_LaneCandPointNum, s32_LanePointNum);

    if (s32_LaneCandPointNum > 100)
    {
	    CUDA_LaneICP(f32_E, f32_N, arf32_LaneCandPointX, arf32_LaneCandPointY, s32_LaneCandPointNum,
	    			arf32_LanePointX, arf32_LanePointY, s32_LanePointNum, pst_LidarData->f32_PredictYaw_deg);
	}
	else
	{
		pst_LidarData->f32_PredictYaw_deg = pst_DeadReckoningData->st_IMU.f32_Yaw_deg;
	}
}