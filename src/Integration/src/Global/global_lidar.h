#pragma once

typedef unsigned char uint8_t;
typedef unsigned short uint16_t; 
typedef unsigned int uint32_t;
typedef short int16_t;
typedef float float32_t;
typedef double float64_t;


const int32_t c_TOTAL_POINT_NUM = 240000;

const uint8_t c_POINT_INVALID = 0;
const uint8_t c_POINT_VALID = 255;
const uint8_t c_POINT_GROUND = 128;
const uint8_t c_POINT_FEATURE = 64;
const uint8_t c_CLUSTER_CAR = 255;
const uint8_t c_CLUSTER_NONCAR = 0;

const int32_t c_UNCLUSTERED = -1;
const int32_t c_TOTAL_CLUSTER_NUM = 500;
const int32_t c_CLUSTER_POINT_NUM = 100000;

const uint8_t c_OBJECT_STATIC = 0;
const uint8_t c_OBJECT_DYNAMIC = 1;
const uint16_t c_TOTAL_TRACKING_NUM = 500;

const int32_t c_HMC_MODE_OS2 = 0;
const int32_t c_HMC_MODE_VLS = 1;
const int32_t c_HMC_CUDA_BLOCK_OS2 = 1024;
const int32_t c_HMC_CUDA_THREAD_OS2 = 128;
const int32_t c_HMC_CUDA_PARSE_BLOCK_VLS = 600;
const int32_t c_HMC_CUDA_PARSE_THREAD_VLS = 384;
const int32_t c_HMC_CUDA_BLOCK_VLS = 1800;
const int32_t c_HMC_CUDA_THREAD_VLS = 128;

const int32_t c_HMC_VOXEL_SIZE_R = 100;
const float32_t c_HMC_VOXEL_STEP_R = 1.f;
const int32_t c_HMC_VOXEL_SIZE_AZIMUTH = 150;
const int32_t c_HMC_VOXEL_SIZE_Z = 8;
const float32_t c_HMC_VOXEL_STEP_AZIMUTH = 2.4f;
const float32_t c_HMC_VOXEL_STEP_Z = 0.5f;
const int32_t c_HMC_VOXEL_SIZE = c_HMC_VOXEL_SIZE_R * c_HMC_VOXEL_SIZE_AZIMUTH * c_HMC_VOXEL_SIZE_Z;

const uint8_t c_HMC_CLUSTER_NONE = 0;
const uint8_t c_HMC_CLUSTER_TRAFFIC_SIGN = 1;
const uint8_t c_HMC_CLUSTER_VEHICLE = 2;
const uint8_t c_HMC_CLUSTER_OBSTACLE = 3;
const int32_t c_HMC_CLUSTER_MAX_POINT_NUM = 3000;

const int32_t c_HMC_MAX_POINT_NUM = 250000;
const int32_t c_HMC_MAX_VOXEL_NUM = 5000;
const int32_t c_HMC_MAX_CLUSTER_NUM = 300;

const uint8_t c_HMC_OBJECT_STATIC = 0;
const uint8_t c_HMC_OBJECT_DYNAMIC = 1;
const uint8_t c_HMC_OBJECT_TEMP = 2;
const uint16_t c_HMC_TOTAL_TRACKING_NUM = 500;

const int32_t c_OMNI_FEATURE_MAX_NUM = 360;
const int32_t c_OMNI_POINT_MAX_NUM = 10000;
const int32_t c_OMNI_HISTOGRAM_SIZE = 200;
const float32_t c_OMNI_BINNING_SIZE = 4.f;

const int32_t c_ICP_MAX_POINT_NUM = 50000;
const int32_t c_ICP_MAP_SIZE = 1000;
const float32_t c_ICP_DOWNSAMPLING_SIZE_XY = 1.f; // 0.7f
const float32_t c_ICP_DOWNSAMPLING_SIZE_Z = 0.3f;  // 0.3f
const int32_t c_DOWNSAMPLING_VOXEL_CAPACITY = 250000;

//////////////////////////////////////////////////////

const int32_t c_PLANNING_MAX_PATH_NUM = 5000;

typedef struct _ICP_DOWN_POINT
{
  float32_t f32_X;
  float32_t f32_Y;
  float32_t f32_Z;
} ICP_DOWN_POINT_t;

typedef struct _DOWNSAMPLE_VOXEL
{
    int32_t s32_Count;
    int32_t s32_Index;
} DOWNSAMPLE_VOXEL_t;

typedef struct _ICP_POINT_CLOUD
{
  float32_t f32_X;
  float32_t f32_Y;
  float32_t f32_Z;
  float32_t f32_Yaw_deg;

  float32_t arf32_X[c_ICP_MAX_POINT_NUM];
  float32_t arf32_Y[c_ICP_MAX_POINT_NUM];
  float32_t arf32_Z[c_ICP_MAX_POINT_NUM];
  int32_t s32_PointNum;
} ICP_POINT_CLOUD_t;



typedef struct _ICP_MAP
{
  ICP_POINT_CLOUD_t arst_PointCloud[c_ICP_MAP_SIZE];
  int32_t s32_MapNum;
} ICP_MAP_t;




typedef struct _OMNI_FEATURE
{
  bool arb_Valid[c_OMNI_FEATURE_MAX_NUM];
  float32_t arf32_X[c_OMNI_FEATURE_MAX_NUM];
  float32_t arf32_Y[c_OMNI_FEATURE_MAX_NUM];
  // float32_t arf32_Z[c_OMNI_FEATURE_MAX_NUM];

  float32_t arf32_Histogram[c_OMNI_HISTOGRAM_SIZE];
} OMNI_FEATURE_t;



typedef struct _OMNI_POINT
{
  int32_t s32_Num;
  float32_t arf32_PositionX[c_OMNI_POINT_MAX_NUM];
  float32_t arf32_PositionY[c_OMNI_POINT_MAX_NUM];
  float32_t arf32_PositionYaw_deg[c_OMNI_POINT_MAX_NUM];
  
  // float32_t f32_PositionZ[c_OMNI_POINT_MAX_NUM];

  OMNI_FEATURE_t arst_Feature[c_OMNI_POINT_MAX_NUM];
} OMNI_POINT_t;





typedef struct _HMC_TRACKING
{
    float32_t f32_X;
    float32_t f32_Y;
    float32_t f32_Z;
    float32_t f32_VelocityX_m_s;
    float32_t f32_VelocityY_m_s;
    float32_t f32_AbsoluteVelocityX_m_s;
    float32_t f32_AbsoluteVelocityY_m_s;
    float32_t f32_Yaw_rad;

    float32_t f32_GlobalX;
    float32_t f32_GlobalY;
    float32_t f32_PrevGlobalX;
    float32_t f32_PrevGlobalY;
    float32_t f32_Yaw_rad_ENU;

    float32_t f32_ClusterX;
    float32_t f32_ClusterY;
    float32_t f32_Length;
    float32_t f32_Width;
    float32_t f32_Height;

    float32_t f32_PrevClusterX;
    float32_t f32_PrevClusterY;
    float32_t f32_PrevLength;
    float32_t f32_PrevWidth;
    float32_t f32_PrevHeight;

    float32_t f32_MaxLength;
    float32_t f32_MaxWidth;
    float32_t f32_MaxHeight;

    float32_t f32_StartTime_s;
    float32_t f32_EndTime_s;
    float32_t f32_DeltaTime_s;

    float32_t arf32_X[4];
    float32_t arf32_PrevX[4];
    float32_t arf32_K[16];
    float32_t arf32_P[16];
    float32_t arf32_Q[16];
    float32_t arf32_R[16];

    int32_t ars32_PointIndex[c_HMC_CLUSTER_MAX_POINT_NUM];
    int32_t s32_PointNum;

    uint32_t u32_EraseCnt = 0;
    float32_t f32_Score;
    bool b_InitlzFlag;
    bool b_UpdateFlag = false;
    uint8_t u8_StateFlag;

} HMC_TRACKING_t;


typedef struct _HMC_POINT
{
  float32_t f32_X;
  float32_t f32_Y;
  float32_t f32_Z;
  float32_t f32_R;
  float32_t f32_Distance;
  float32_t f32_Azimuth_deg;
  float32_t f32_Azimuth_rad;
  float32_t f32_Theta_deg;
  float32_t f32_Theta_rad;
  uint8_t u8_Intensity;
  uint8_t u8_Layer;
  uint8_t u8_GroundFilteringFlag;
  uint8_t u8_Flag;
  int32_t s32_VoxelIndex;
  uint64_t u64_Hash;
} HMC_POINT_t;


typedef struct _HMC_VOXEL
{
  float32_t   arf32_VoxelBaseX[8];
  float32_t   arf32_VoxelBaseY[8];
  float32_t   arf32_VoxelBaseZ[8];
  float32_t   arf32_VoxelPointX[8];
  float32_t   arf32_VoxelPointY[8];
  float32_t   arf32_VoxelPointZ[8];
  bool      arb_PointUpdate[8];
  bool    b_HasPoint;
  uint8_t u8_VoxelColor = -1;
  int32_t   s32_ClusterID;
  int32_t   s32_PointNum;
  int32_t   s32_Parent;
} HMC_VOXEL_t;


typedef struct _HMC_CLUSTER
{
  float32_t f32_X1;
  float32_t f32_Y1;
  float32_t f32_Z1;
  float32_t f32_X2;
  float32_t f32_Y2;
  float32_t f32_Z2;

  float32_t f32_CenterX;
  float32_t f32_CenterY;
  float32_t f32_CenterZ;
  float32_t f32_Length;
  float32_t f32_Size;

  int32_t ars32_PointIndex[c_HMC_CLUSTER_MAX_POINT_NUM];
  int32_t s32_PointNum;
  int32_t s32_VoxelNum;
  uint8_t u8_Flag;
  bool b_TrackingFlag;

} HMC_CLUSTER_t;


typedef struct _LIDAR_PARAM
{
  float32_t f32_MaxNearObjectDistance;
  float32_t f32_MaxNearNeighborDistance;
  float32_t f32_MaxNewObjectDistance;

  uint32_t u32_MaxEraseCnt;

  float32_t f32_InitialP0;
  float32_t f32_InitialP5;
  float32_t f32_InitialP10;
  float32_t f32_InitialP15;

  float32_t f32_InitialQ0;
  float32_t f32_InitialQ5;
  float32_t f32_InitialQ10;
  float32_t f32_InitialQ15;

  float32_t f32_InitialR0;
  float32_t f32_InitialR5;
  float32_t f32_InitialR10;
  float32_t f32_InitialR15;

  float32_t f32_MaxObjectDistance;
  float32_t f32_MaxObjectVelocity_kph;

  float32_t f32_MinIoU;

  float32_t f32_MaxStaticVelocity_kph;

  /////////////////////////////////////////////////////////

  ICP_POINT_CLOUD_t st_VehiclePointCloud;

  /////////////////////////////////////////////////////////

  int32_t s32_MinClusterPointNum;
  float32_t f32_MaxClusterLength;
  float32_t f32_MinClusterSize;

  float32_t f32_MapDistanceOffset;

} LIDAR_PARAM_t;


typedef struct _HMC_LIDAR_DATA
{
  uint64_t u64_Time_ms;

  int32_t s32_PointNum;
  int32_t s32_ClusterNum;
  int32_t s32_LayerNum;
  int32_t s32_TrackingNum;

  HMC_POINT_t arst_Point[c_HMC_MAX_POINT_NUM];
  HMC_VOXEL_t arst_Voxel[c_HMC_VOXEL_SIZE];
  HMC_CLUSTER_t arst_Cluster[c_HMC_MAX_CLUSTER_NUM];
  HMC_TRACKING_t arst_Tracking[c_TOTAL_TRACKING_NUM];

  OMNI_FEATURE_t st_OmniFeature;
  ICP_POINT_CLOUD_t st_ICPPoint;
  int32_t ars32_CandidateIndex[5];
  int32_t s32_BestCandidateIndex;
  int32_t s32_OmniCorrespondence;
  int32_t s32_OmniMatchingScore;
  float32_t f32_PredictX;
  float32_t f32_PredictY;
  float32_t f32_PredictZ;
  float32_t f32_PredictYaw_deg;

  int32_t s32_ICPNearestMapIdx;
  int32_t ars32_ICPNearestIdx[c_ICP_MAX_POINT_NUM];
  float32_t arf32_ICPTransform[16];
  float32_t arf32_ICPInverseTransform[16];
} HMC_LIDAR_DATA_t;
 




typedef struct _PATH
{
  float32_t arf32_X[c_PLANNING_MAX_PATH_NUM];
  float32_t arf32_Y[c_PLANNING_MAX_PATH_NUM];
  float32_t arf32_D[c_PLANNING_MAX_PATH_NUM];
  float32_t arf32_Yaw_rad_ENU[c_PLANNING_MAX_PATH_NUM];
  float32_t arf32_Yaw_rad_NED[c_PLANNING_MAX_PATH_NUM];
  float32_t arf32_LaneCenterX[1000];
  float32_t arf32_LaneCenterY[1000];

  int32_t s32_Num;
  int32_t s32_NearIdx = 0;
  float32_t f32_NearDist = 0.0f;

} PATH_t;


typedef struct _VEHICLE_DATA
  {

    int32_t s32_VehicleIdx;

    float32_t f32_X_local;
    float32_t f32_Y_local;
    float32_t f32_Z_local;

    float32_t f32_X_global;
    float32_t f32_Y_global;
    float32_t f32_Z_global;

    float32_t f32_VelocityX_m_s_global;
    float32_t f32_VelocityY_m_s_global;
    float32_t f32_Velocity_m_s_global;

    float32_t f32_VelocityX_kph_global;
    float32_t f32_VelocityY_kph_global;
    float32_t f32_Velocity_kph_global;

    float32_t f32_VelocityX_m_s_local;
    float32_t f32_VelocityY_m_s_local;
    float32_t f32_Velocity_m_s_local;

    float32_t f32_VelocityX_kph_local;
    float32_t f32_VelocityY_kph_local;
    float32_t f32_Velocity_kph_local;

    float32_t f32_Yaw_rad_ENU;
    float32_t f32_Yaw_rad_NED;
    float32_t f32_roll_rad;
    float32_t f32_roll_deg;

    float32_t f32_PrevX_global;
    float32_t f32_PrevY_global;
    float32_t f32_PrevZ_global;
    float32_t f32_DT;

    float32_t f32_AccelX;
    float32_t f32_AccelY;

    float32_t f32_MaxX;
    float32_t f32_MinX;
    float32_t f32_MaxY;
    float32_t f32_MinY;

    float32_t f32_Vertex[4][2];
    float32_t f32_MarginVertex[4][2];
    bool b_IsCollision = false;
    float32_t f32_FrontHangOver;
    float32_t f32_RearHangOver;
    float32_t f32_GpsToFront;
    float32_t f32_GpsToRear;
    float32_t f32_WholeLength;
    float32_t f32_Width;

    float32_t f32_Score;

    float32_t f32_S;
    float32_t f32_D;
    float32_t f32_DS;
    float32_t f32_DD;

    float32_t f32_RelativeDistance;
    float32_t f32_RelativeVelocity;

    float32_t f32_TargetVelocity_m_s;
    float32_t f32_TargetVelocity_kph;

    int32_t s32_CurrentLane;
    float32_t f32_ThresHold;

    float32_t f32_TimeToCollision;
    float32_t f32_TTC_Min;
    float32_t f32_SafetyDistance;
    float32_t f32_AbsDistance;

    bool b_ACC_Mode;
    bool b_AEB_Mode;
    bool b_ABS_Mode;

  } VEHICLE_DATA_t;
