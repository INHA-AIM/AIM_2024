#ifndef GLOBAL_H
#define GLOBAL_H

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <vector>
#include <cstdlib>
#include <tuple>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <stdint.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <sys/stat.h>
#include <sys/types.h>
#include <filesystem>
#include <array>
#include <chrono>
#include <random>

#include "global_lidar.h"

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"

#include "casadi/casadi.hpp"
#include "casadi/core/sparsity_interface.hpp"

#include "morai_msgs/CtrlCmd.h"
#include "morai_msgs/EgoVehicleStatus.h"
#include "morai_msgs/GPSMessage.h"

#include "Integration/GNSSInfo.h"
#include "Integration/object_msg_arr.h"

#include <yaml-cpp/yaml.h>
#include <canlib.h>
#include "PCANBasic.h"

#define NONE -1e9

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef short int16_t;
typedef float float32_t;
typedef double float64_t;

using namespace std;
using namespace Eigen;
using namespace chrono;
using namespace cv;

using PointArray2D = array<float32_t, 2>;

const float64_t PI = 3.14159265358979323846;

const int32_t c_PARSING_UDP = 0;
const int32_t c_PARSING_ROS = 1;
const int32_t c_PARSING_SERIAL = 2;
const int32_t c_PARSING_CAN = 3;

const int32_t c_PARSING_MORAI = 0;
const int32_t c_PARSING_AVANTE = 1;

const int32_t c_LIDAR_BUFFER_SIZE = 2000000;
const int32_t c_CAMERA_BUFFER_SIZE = 2000000;
const int32_t c_GPSINS_BUFFER_SIZE = 100;
const int32_t c_IMU_BUFFER_SIZE = 100000;
const int32_t c_CAN_BUFFER_SIZE = 30000;
const int32_t c_MAX_FRAME_SIZE = 5000;

const int32_t c_ROUND_TEST = 0;
const int32_t c_ROUND_TESTDRIVE1 = 1;
const int32_t c_ROUND_TESTDRIVE2 = 2;
const int32_t c_ROUND_PRELIMINARY1 = 3;
const int32_t c_ROUND_PRELIMINARY2 = 4;
const int32_t c_ROUND_FINAL = 5;

const int32_t c_VIEWER_NULL = -1;
const int32_t c_VIEWER_MENU = 0;
const int32_t c_VIEWER_LOCAL = 1;
const int32_t c_VIEWER_GLOBAL = 2;
const int32_t c_VIEWER_FRAME = 3;
const int32_t c_VIEWER_CAMERA = 4;

const int32_t c_VIEWER_SIMUL_MODE = 0;
const int32_t c_VIEWER_REAL_MODE = 1;

const int32_t c_VIEWER_BUTTON_NUM_SIMUL = 6;
const int32_t c_VIEWER_BUTTON_NUM_REAL = 3;

const int32_t c_VIEWER_BUTTON_NULL = -1;

const int32_t c_VIEWER_BUTTON_SIMUL_MODE = 0;
const int32_t c_VIEWER_BUTTON_SIMUL_LOAD_MAP = 1;
const int32_t c_VIEWER_BUTTON_SIMUL_RECORD_MAP = 2;
const int32_t c_VIEWER_BUTTON_SIMUL_SAVE_MAP = 3;
const int32_t c_VIEWER_BUTTON_SIMUL_LOGGING = 4;
const int32_t c_VIEWER_BUTTON_SIMUL_PLAY = 5;

const int32_t c_VIEWER_BUTTON_REAL_MODE = 0;
const int32_t c_VIEWER_BUTTON_REAL_LOGGING = 1;
const int32_t c_VIEWER_BUTTON_REAL_AUTOMODE_ON = 2;

const int32_t c_VIEWER_BUTTON_EXIT = 6;

const int32_t c_STATE_MODE_LOGGING = 1;
const int32_t c_STATE_MODE_PLAY = 2;
const int32_t c_STATE_MODE_STOP = 4;
const int32_t c_STATE_MODE_RECORD_MAP = 8;
const int32_t c_STATE_MODE_LOAD_MAP = 16;
const int32_t c_STATE_MODE_SAVE_MAP = 32;
const int32_t c_STATE_MODE_NULL = 0;

const int32_t c_STATE_1XZOOMFACTOR = 5;
const int32_t c_STATE_2XZOOMFACTOR = 6;
const int32_t c_STATE_3XZOOMFACTOR = 7;

const int32_t c_STATE_GLOBALFIX = 0;
const int32_t c_STATE_GLOBALNONFIX = 1;

const int32_t c_STATE_PATHMAKEROFF = 0;
const int32_t c_STATE_PATHMAKERON = 1;

const float32_t c_LEAF_SIZE = 1.f;

const float32_t c_DISTANCE_LEAF_SIZE = 1.f;
const float32_t c_AZIMUTH_LEAF_SIZE = 1.f;
const float32_t c_ELEVATION_LEAF_SIZE = 2.5f;

const int32_t c_VOXEL_DISTANCE_MIN = 0;
const int32_t c_VOXEL_DISTANCE_MAX = 150;
const int32_t c_VOXEL_AZIMUTH_MIN = 0;
const int32_t c_VOXEL_AZIMUTH_MAX = 360;
const int32_t c_VOXEL_ELEVATION_MIN = -15;
const int32_t c_VOXEL_ELEVATION_MAX = 5;

const uint32_t c_GRID_DISTANCE_SIZE = (c_VOXEL_DISTANCE_MAX - c_VOXEL_DISTANCE_MIN) / c_DISTANCE_LEAF_SIZE;
const uint32_t c_GRID_AZIMUTH_SIZE = (c_VOXEL_AZIMUTH_MAX - c_VOXEL_AZIMUTH_MIN) / c_AZIMUTH_LEAF_SIZE;
const uint32_t c_GRID_ELEVATION_SIZE = (c_VOXEL_ELEVATION_MAX - c_VOXEL_ELEVATION_MIN) / c_ELEVATION_LEAF_SIZE;

extern float64_t c_ORIGIN_LATITUDE_DEG;
extern float64_t c_ORIGIN_LONGITUDE_DEG;
extern float64_t c_ORIGIN_LATITUDE_RAD;
extern float64_t c_ORIGIN_LONGITUDE_RAD;
extern float64_t c_ORIGIN_ALTITUDE;
extern float64_t c_ORIGIN_REFERENCE_X;
extern float64_t c_ORIGIN_REFERENCE_Y;
extern float64_t c_ORIGIN_REFERENCE_Z;

const float64_t c_LLA2ENU_A = 6378137.0;
const float64_t c_LLA2ENU_FLAT_RATIO = 1.0 / 298.257223563;
const float64_t c_LLA2ENU_N_2 = (2 * c_LLA2ENU_FLAT_RATIO) - pow(c_LLA2ENU_FLAT_RATIO, 2);

const int32_t c_PLANNING_MAX_SPLINE_NUM = 20000;
const int32_t c_PLANNING_MAX_FRENET_NUM = 200;
const int32_t c_PLANNING_MAX_FRENET_PATH_NUM = 250;

// SELine에 대한 Near 점에 대해 유효함
const int32_t c_REFERENCE_IDX = 261;
const int32_t c_ARRIVE_IDX = 282;
const int32_t c_START_IDX = 448;

// RefLine2_global에 대한 Near 점에 대해 유효함
const int32_t c_FINAL_IDX = 1;
const int32_t c_FIRSTBANKSTART_IDX = 354;
const int32_t c_FIRSTBANKEND_IDX = 765;
const int32_t c_SECONDBANKSTART_IDX = 1582;
const int32_t c_SECONDBANKEND_IDX = 2016;

const uint8_t c_CONTROL_FLAG_NONE = 0;
const uint8_t c_CONTROL_FLAG_ACC = 1;
const uint8_t c_CONTROL_FLAG_AEB = 2;
const uint8_t c_CONTROL_FLAG_OVERTAKING = 3;

const int32_t c_CONTROL_MAX_HORIZON = 30;

typedef struct _RAW_LIDAR_DATA
{
  uint64_t u64_Timestamp;
  int32_t s32_Num;
  char arc_Buffer[c_LIDAR_BUFFER_SIZE];
  string s_LIDAR_Status;
  string s_LIDAR_Mode;
  int64_t s64_LIDAR_Temp;
  int32_t s32_LiDARHeader;
} RAW_LIDAR_DATA_t;

typedef struct _RAW_CAMERA_DATA
{
  uint64_t u64_Timestamp;
  int32_t s32_Num;
  char arc_Buffer[c_CAMERA_BUFFER_SIZE];
  int32_t s32_CameraHeader;
} RAW_CAMERA_DATA_t;

typedef struct _RAW_GPSINS_DATA
{
  uint64_t u64_Timestamp;
  int32_t s32_Num;
  char arc_Buffer[c_GPSINS_BUFFER_SIZE];
  int32_t s32_GPSINSHeader;
} RAW_GPSINS_DATA_t;

typedef struct _RAW_IMU_DATA
{
  uint64_t u64_Timestamp;
  int32_t s32_Num;
  char arc_Buffer[c_IMU_BUFFER_SIZE];
  int32_t s32_IMUHeader;
} RAW_IMU_DATA_t;

typedef struct _RAW_CAN_DATA
{
  uint64_t u64_Timestamp;
  int32_t s32_Num;
  char arc_Buffer[c_CAN_BUFFER_SIZE];
  int32_t s32_CANHeader;
} RAW_CAN_DATA_t;

typedef struct _SENSOR_DATA
{
  uint64_t u64_Timestamp;

  RAW_LIDAR_DATA_t st_RawLIDAR;
  RAW_CAMERA_DATA_t st_RawCamera;
  RAW_GPSINS_DATA_t st_RawGPSINS;
  RAW_IMU_DATA_t st_RawIMU;
  RAW_CAN_DATA_t st_RawCAN;

  pthread_mutex_t st_MutexLIDAR;
  pthread_mutex_t st_MutexCamera;
  pthread_mutex_t st_MutexGPSINS;
  pthread_mutex_t st_MutexIMU;
  pthread_mutex_t st_MutexCAN;

} SENSOR_DATA_t;

typedef struct _VIEWER_PARAMETER
{
  int32_t s32_ScreenX = 0;
  int32_t s32_ScreenY = 0;
  int32_t s32_ScreenWidth = 0;
  int32_t s32_ScreenHeight = 0;

  int32_t s32_LocalX = 0;
  int32_t s32_LocalY = 0;
  int32_t s32_LocalWidth = 0;
  int32_t s32_LocalHeight = 0;

  int32_t s32_GlobalX = 0;
  int32_t s32_GlobalY = 0;
  int32_t s32_GlobalWidth = 0;
  int32_t s32_GlobalHeight = 0;

  int32_t s32_MenuX = 0;
  int32_t s32_MenuY = 0;
  int32_t s32_MenuWidth = 0;
  int32_t s32_MenuHeight = 0;

  int32_t s32_LoggingX = 0;
  int32_t s32_LoggingY = 0; // constant
  int32_t s32_LoggingWidth = 0;
  int32_t s32_LoggingHeight = 0;

  int32_t s32_CameraX = 0;
  int32_t s32_CameraY = 0;
  int32_t s32_CameraWidth = 0;
  int32_t s32_CameraHeight = 0;

  int32_t s32_FrameX = 0;
  int32_t s32_FrameY = 0;
  int32_t s32_FrameWidth = 0;
  int32_t s32_FrameHeight = 0;
  int32_t s32_FrameBarX = 0;

  float32_t f32_GlobalZoomFactor = 3.4f;
  float32_t f32_LocalZoomFactor = 3.4f;

  bool b_MoveFrame = false;
  bool b_TransLocalMap = false;
  bool b_TransGlobalMap = false;
  bool b_RotateLocalMap = false;
  bool b_RotateGlobalMap = false;

  float32_t f32_PrevX;
  float32_t f32_PrevY;

  int32_t s32_CurrentMode;
  int32_t s32_CurrentWindow = c_VIEWER_NULL;
  int32_t s32_CurrentButton = c_VIEWER_BUTTON_NULL;

  int32_t s32_State = c_STATE_MODE_NULL;
  int32_t s32_ZoomFactorState = c_STATE_1XZOOMFACTOR;
  int32_t s32_GlobalWindowState = 0;

  bool b_SatToggle = false;
  bool b_FrenetToggle = false;
  bool b_AccToggle = false;
  bool b_VertexToggle = false;

} VIEWER_PARAMETER_t;

typedef struct _POINT
{
  float32_t f32_X;
  float32_t f32_Y;
  float32_t f32_Z;

  float32_t f32_Azimuth_deg;
  float32_t f32_Azimuth_rad;
  float32_t f32_Elevation_deg;
  float32_t f32_Elevation_rad;
  float32_t f32_Distance;
  float32_t f32_R;

  uint8_t u8_Intensity;
  uint8_t u8_Channel;
  uint8_t u8_Flag;
  uint8_t u8_GroundFilteringFlag;
  int32_t ars32_VoxelIdx[3];
  int32_t s32_VoxelIdx;

} POINT_t;

typedef struct _VOXEL
{
  int32_t ars32_Grid[c_GRID_DISTANCE_SIZE][c_GRID_AZIMUTH_SIZE][c_GRID_ELEVATION_SIZE];
  int32_t ars32_ClusterIdx[c_GRID_DISTANCE_SIZE][c_GRID_AZIMUTH_SIZE][c_GRID_ELEVATION_SIZE];

} VOXEL_t;

typedef struct _CLUSTER
{
  int32_t ars32_PointIdx[c_CLUSTER_POINT_NUM];
  int32_t s32_PointNum;

  float32_t f32_X;
  float32_t f32_Y;
  float32_t f32_Z;

  float32_t f32_MaxX;
  float32_t f32_MaxY;
  float32_t f32_MaxZ;
  float32_t f32_MinX;
  float32_t f32_MinY;
  float32_t f32_MinZ;

  float32_t f32_Volume;
  float32_t f32_Distance;
  float32_t f32_Azimuth;
  float32_t f32_Elevation;
  uint8_t u8_Class;

  bool b_State = false;
  bool b_TrackingFlag = false;

} CLUSTER_t;

typedef struct _TRACKING
{
  float32_t f32_X;
  float32_t f32_Y;
  float32_t f32_VelocityX_m_s;
  float32_t f32_VelocityY_m_s;

  float32_t f32_GlobalX;
  float32_t f32_GlobalY;
  float32_t f32_PrevGlobalX;
  float32_t f32_PrevGlobalY;

  float32_t f32_ClusterX;
  float32_t f32_ClusterY;
  float32_t f32_MaxX;
  float32_t f32_MaxY;
  float32_t f32_MinX;
  float32_t f32_MinY;

  float32_t f32_PrevClusterX;
  float32_t f32_PrevClusterY;
  float32_t f32_PrevMaxX;
  float32_t f32_PrevMaxY;
  float32_t f32_PrevMinX;
  float32_t f32_PrevMinY;

  float32_t f32_StartTime_s;
  float32_t f32_EndTime_s;
  float32_t f32_DeltaTime_s;

  float32_t f32_CurrYaw_rad;
  float32_t f32_PrevYaw_rad;

  MatrixXf st_A = MatrixXf(4, 4);
  VectorXf st_X = VectorXf(4);
  VectorXf st_PrevX = VectorXf(4);
  VectorXf st_Z = VectorXf(4);
  MatrixXf st_H = Matrix4f::Identity();
  MatrixXf st_K = MatrixXf(4, 4);
  MatrixXf st_P = MatrixXf(4, 4);
  MatrixXf st_Q = MatrixXf(4, 4);
  MatrixXf st_R = MatrixXf(4, 4);

  uint32_t u32_EraseCnt = 0;

  bool b_InitlzFlag;
  bool b_UpdateFlag = false;
  bool b_StateFlag;

} TRACKING_t;

typedef struct _LIDAR_DATA
{
  uint64_t u64_Timestamp;

  POINT_t arst_Point[c_TOTAL_POINT_NUM];
  POINT_t arst_RoadPoint[22230];
  VOXEL_t st_Voxel;
  CLUSTER_t arst_Cluster[c_TOTAL_CLUSTER_NUM];
  CLUSTER_t arst_PrevCluster[c_TOTAL_CLUSTER_NUM];
  TRACKING_t arst_Tracking[c_TOTAL_TRACKING_NUM];

  int32_t s32_PointNum;
  int32_t s32_RoadPointNum;
  int32_t s32_ClusterNum = 0;
  int32_t s32_PrevClusterNum = 0;
  int32_t s32_TrackingNum = 0;

  float32_t f32_LiDARRoll_deg;
  float32_t f32_LiDARPitch_deg;
  float32_t f32_LiDARYaw_deg;

  float32_t f32_LiDARRoll_rad;
  float32_t f32_LiDARPitch_rad;
  float32_t f32_LiDARYaw_rad;

  OMNI_FEATURE_t st_OmniFeature;
  int32_t ars32_CandidateIndex[5];
  int32_t s32_BestCandidateIndex;
  int32_t s32_OmniCorrespondence;
  int32_t s32_OmniMatchingScore;
  float32_t f32_PredictX;
  float32_t f32_PredictY;
  float32_t f32_PredictYaw_deg;

} LIDAR_DATA_t;

typedef struct _LANE_COEFFICIENT
{
  float32_t f32_Slope;
  float32_t f32_Intercept;

} LANE_COEFFICIENT_t;

typedef struct _LANE_STATUS
{
  bool b_NoLaneLeft = true;
  bool b_NoLaneRight = true;
  bool b_IsNotCornerLeft = true;
  bool b_IsNotCornerRight = true;

  int32_t s32_CntValidLaneLeft = 0;
  int32_t s32_CntValidLaneRight = 0;

} LANE_STATUS_t;

typedef struct _CMAMERA_LANEINFO
{
  cv::Point arst_LaneSample[40];
  int32_t s32_SampleCount;
  LANE_COEFFICIENT_t st_LaneCoefficient;
  bool b_IsValid = false;

} CAMERA_LANEINFO_t;

typedef struct _CAMERA_DATA
{
  uint64_t u64_Timestamp;
  // PATH_t st_CameraLine_local;

  // Perspective Image
  int32_t s32_WayPntX;
  int32_t s32_WayPntY;

  float32_t f32_DistLeft;
  float32_t f32_DistRight;

  LANE_COEFFICIENT_t st_LastLaneCoefLeft;
  LANE_COEFFICIENT_t st_LastLaneCoefRight;

  int32_t s32_Lane2Near;
  int32_t s32_PitStopNear;
  bool b_PitStop = true;
  bool b_NearPitStopL = false;
  bool b_NearPitStopR = false;

} CAMERA_DATA_t;

typedef struct _VISUAL_ODOMETRY
{
  vector<KeyPoint> st_KeyPointLast;
  Mat st_DescriptorLast;
  Mat st_BGRImageLast;
  vector<KeyPoint> st_KeyPointCurrent;
  Mat st_DescriptorCurrent;
  Mat st_BGRImageCurrent;

  Mat st_RotationT;
  Mat st_TranslationT;

  vector<Point2f> st_Trajectory;

} VISUAL_ODOMETRY_t;

typedef struct _GPS_DATA
{
  float64_t f64_Latitude;
  float64_t f64_Longitude;
  float64_t f64_Altitude;
  int32_t s32_mode;
  float32_t f32_E;
  float32_t f32_N;

  int32_t s32_LabCount;
  int32_t ars32_TotalTime[3];
  int32_t ars32_CurrentLapTime[3];
  int32_t ars32_BestLapTime[3];

} GPS_DATA_t;

typedef struct _IMU_DATA
{
  float32_t f32_AccelX;
  float32_t f32_AccelY;
  float32_t f32_AccelZ;

  float32_t f32_GyroX;
  float32_t f32_GyroY;
  float32_t f32_GyroZ;

  float32_t f32_QuaternionX;
  float32_t f32_QuaternionY;
  float32_t f32_QuaternionZ;
  float32_t f32_QuaternionW;

  float32_t arf32_VelocityNED_m_s[3] = {0};
  float32_t arf32_VelocityXYZ_m_s[3] = {0};

  float32_t f32_Roll_rad;
  float32_t f32_Pitch_rad;
  float32_t f32_Yaw_rad;

  float32_t f32_Roll_deg;
  float32_t f32_Pitch_deg;
  float32_t f32_Yaw_deg;

} IMU_DATA_t;

typedef struct _CAN_DATA
{
  canHandle st_Handle;
  canStatus st_Stat;

  TPCANStatus st_PStat;

  unsigned int u32_DLC;
  uint32_t u32_Flag;
  long unsigned int u32_Time;

  uint32_t u32_TimeOut;

  // ID
  long int ID;
  long int ID_0x156;
  long int ID_0x157;
  long int ID_0x710;
  long int ID_0x711;
  long int ID_0x712;
  long int ID_0x713;

  // 0x710
  uint8_t u8_EPS_En_Status;
  uint8_t u8_EPS_Control_Board_Status;
  uint8_t u8_EPS_Control_Status;
  uint8_t u8_EPS_USER_CAN_ERR;
  uint8_t u8_EPS_ERR;
  uint8_t u8_EPS_Veh_CAN_ERR;
  uint8_t u8_EPS_SAS_ERR;
  uint8_t u8_Override_Ignore_Status;
  uint8_t u8_Override_Status;
  float32_t f32_StrAng;
  float32_t f32_Str_Drv_Tq;
  float32_t f32_Str_Out_Tq;
  uint8_t u8_EPS_Alive_Cnt;

  // 0x711
  uint8_t u8_ACC_En_Status;
  uint8_t u8_ACC_Control_Board_Status;
  uint8_t u8_ACC_Control_Status;
  uint8_t u8_ACC_USER_CAN_ERR;
  uint8_t u8_ACC_Veh_ERR;
  uint8_t u8_ACC_ERR;
  uint8_t u8_VS;
  uint8_t u8_Turn_Left_En;
  uint8_t u8_Hazard_En;
  uint8_t u8_Turn_Right_En;
  uint8_t u8_G_SEL_DISP;
  float32_t f32_LONG_ACCEL;
  uint8_t u8_ACC_Alive_Cnt;

  // 0x712
  float32_t f32_WHEEL_SPD_FL;
  float32_t f32_WHEEL_SPD_FR;
  float32_t f32_WHEEL_SPD_RL;
  float32_t f32_WHEEL_SPD_RR;

  // 0x713
  float32_t f32_LAT_ACCEL;
  float32_t f32_Long_ACCEL;
  float32_t f32_YAW_RATE;
  float32_t f32_BRK_CYLINDER;

  // 0x156
  uint8_t u8_EPS_En;
  uint8_t u8_EPS_Override_ignore;
  uint8_t u8_EPS_Speed;
  uint8_t u8_ACC_En;
  uint8_t u8_AEB_En;
  uint8_t u8_Hazard;
  uint8_t u8_Turn_sig_right;
  uint8_t u8_Turn_sig_left;
  uint8_t u8_AEB_decel_value; // 0 ~ 100
  uint8_t u8_Alive_Cnt;
  uint32_t u64_CanSync;

  // 0x157, can_control steering, velocity
  int16_t s16_EPS_Cmd;
  int16_t s16_ACC_Cmd;

  // 0x124 Receive signal
  uint8_t u8_SIG_GO;
  uint8_t u8_SIG_STOP;
  uint8_t u8_SIG_PIT_STOP;
  uint8_t u8_SIG_SLOW_ON;
  uint8_t u8_SIG_SLOW_OFF;

  // 0x125 Receive PIT STOP ZONE
  float32_t f32_PITZONE_LAT;
  float32_t f32_PITZONE_LONG;

  // 0x126 Send REPAIR
  uint8_t u8_SIG_REPAIR;

  // msg
  TPCANTimestamp timestamp;
  TPCANMsg message;
  TPCANMsg PRequest_Msg;
  TPCANMsg PControl_Msg;
  TPCANMsg PREPAIR_Msg;
  char strMsg[256];

  uint8_t msg[8];
  uint8_t Request_Msg[8];
  uint8_t Control_Msg[8];
  uint8_t REPAIR_Msg[8];
  uint8_t Read_EPS_Msg[8];
  uint8_t Read_ACC_Msg[8];
  uint8_t Read_WS_Msg[8];
  uint8_t Read_STAT_Msg[8];

  // control
  float32_t f32_TargetSpeed;    // -3 ~ 1
  float32_t f32_TargetSteering; // -500 ~ 500

  float32_t f32_Speed_kph;
  float32_t f32_Speed_m_s;
  float32_t f32_AccelX;
  float32_t f32_AccelY;
  float32_t f32_SteerAngle_deg;
  float32_t f32_SteerAngle_rad;
  float32_t f32_Heading_deg;
  float32_t f32_Heading_rad;

} CAN_DATA_t;

typedef struct _GPS_IMU_EKF_MATRIX
{
  MatrixXf st_Q = MatrixXf(15, 15), st_H = MatrixXf(15, 15), st_K = MatrixXf(15, 15), st_P = MatrixXf(15, 15),
           st_F = MatrixXf(15, 15), st_F2 = MatrixXf(15, 15), st_A = MatrixXf(15, 15), st_R = MatrixXf(15, 15);

  MatrixXf st_X = MatrixXf(1, 9);

  MatrixXf st_dX = MatrixXf(15, 1), st_dX0 = MatrixXf(15, 1), st_m_7 = MatrixXf(15, 1);
  MatrixXf st_Cbn = MatrixXf(3, 3), st_f_ned = MatrixXf(3, 1), st_V = MatrixXf(3, 1),
           st_V_ned = MatrixXf(3, 1), st_c3 = MatrixXf(3, 1), st_Vt = MatrixXf(3, 1), st_Vt2 = MatrixXf(3, 1),
           st_Pt = MatrixXf(3, 1), st_Pt2 = MatrixXf(3, 1), st_av = MatrixXf(3, 3), st_g = MatrixXf(3, 1),
           st_accel_b = MatrixXf(3, 1), st_omega_b = MatrixXf(3, 1);

  MatrixXf st_w_ibb = MatrixXf(1, 3), st_w_enn = MatrixXf(1, 3), st_w_ien = MatrixXf(1, 3), st_w_inn = MatrixXf(1, 3),
           st_c1 = MatrixXf(1, 3), st_c2 = MatrixXf(1, 3), st_c31 = MatrixXf(1, 3), st_c32 = MatrixXf(1, 3),
           st_c33 = MatrixXf(1, 3), st_c34 = MatrixXf(1, 3), st_c35 = MatrixXf(1, 3);

  MatrixXf st_acc = MatrixXf(3, 1), st_av_exp = MatrixXf(3, 3), st_av_2 = MatrixXf(3, 3), st_Rotation = MatrixXf(3, 3), st_F_pp = MatrixXf(3, 3),
           st_F_pv = MatrixXf(3, 3), st_F_vp = MatrixXf(3, 3), st_F_vv = MatrixXf(3, 3), st_F_vphi = MatrixXf(3, 3), st_F_phip = MatrixXf(3, 3),
           st_F_phiv = MatrixXf(3, 3), st_F_phiphi = MatrixXf(3, 3), st_Cbn_minus = MatrixXf(3, 3), st_C_B2N = MatrixXf(3, 3), minCbn = MatrixXf(3, 3);

  VectorXf euler_angles = VectorXf(3);

} GPS_IMU_EKF_MATRIX_t;

typedef struct _EKF_DATA
{
  float64_t lat, lon, alt, h;
  float64_t gps_lat, gps_lon, gps_alt;
  float64_t lat_origin, lon_origin, alt_origin; // origin LLA(rad)
  float64_t wgs84_f, wgs84_e2;                  // wgs84 model
  float64_t ecc, R0, Ome, gravity, ecc_2;       // earth value
  float64_t INSTime, lastINST, GPSTime, lastGPST;
  float64_t cos_lat, sin_lat, tan_lat;

  float32_t cur_E_prev, cur_N_prev, cur_E_aft, cur_N_aft;
  float32_t av_x, av_y, av_z;
  float32_t la_x, la_y, la_z;
  float32_t qu_x, qu_y, qu_z, qu_w;
  float32_t t0, t1, t2, t3, t4, t5;
  float32_t setting_yaw;
  float32_t roll, pitch, yaw, delta_yaw;
  float32_t q_roll, q_pitch, q_yaw;
  float32_t rho_n, rho_e, rho_d;
  float32_t f_n, f_e, f_d;
  float32_t x, y, z;
  float32_t Rm, Rt, Rmm, Rtt;
  float32_t v_e, v_n, v_u;
  float32_t gps_v_e, gps_v_n, gps_v_u;
  float32_t cur_E, cur_N, cur_U, prev_E, prev_N, prev_U;
  float32_t gps_E, gps_N, gps_U, d_E, d_N, d_U;
  float32_t dt, dt_gps;
  float32_t prev_roll, prev_pitch, prev_yaw, prev_lat, prev_lon, prev_alt, prev_v_e, prev_v_n, prev_v_u;

  int32_t TAU_A;
  int32_t SIG_G_D;
  int32_t TAU_G;

  bool flag0, flag1, flag2, IMU_flag, GPS_flag, Ref_flag;

} EKF_DATA_t;

typedef struct _VEHICLE_ODOMETRY
{
  float32_t f32_Speed_ms;
  float32_t f32_Range;
  float32_t f32_EgoPrevHeading_rad;
  float32_t f32_EgoPrevHeading_deg;
  float32_t f32_EgoHeading_rad;
  float32_t f32_EgoHeading_deg;
  float32_t f32_AnglerVelcity_ms;
  float32_t f32_TrajectoryX[10000];
  float32_t f32_TrajectoryY[10000];
  float32_t f32_PredictX;
  float32_t f32_PredictY;
  float32_t f32_PredictYaw_rad;
  float32_t f32_PredictYaw_deg;

  int32_t s32_TrajectoryCount;
  uint8_t u8_StateFlag[10000];
  uint64_t u64_TimeStamp;

} VEHICLE_ODOMETRY_t;

typedef struct _TRAJECTORY
{

  float32_t f32_OdomTrajectoryX[10000];
  float32_t f32_OdomTrajectoryY[10000];
  float32_t f32_OdomTrajectoryHeadingX[10000];
  float32_t f32_OdomTrajectoryHeadingY[10000];
  int32_t s32_OdomTrajectoryCount;

  float32_t f32_ICPTrajectoryX[10000];
  float32_t f32_ICPTrajectoryY[10000];
  float32_t f32_ICPTrajectoryHeadingX[10000];
  float32_t f32_ICPTrajectoryHeadingY[10000];
  int32_t s32_ICPTrajectoryCount;

  float32_t f32_GPSTrajectoryX[10000];
  float32_t f32_GPSTrajectoryY[10000];
  float32_t f32_GPSTrajectoryHeadingX[10000];
  float32_t f32_GPSTrajectoryHeadingY[10000];
  int32_t s32_GPSTrajectoryCount;

} TRAJECTORY_t;

typedef struct _DEAD_RECKONING_DATA
{
  uint64_t u64_Timestamp;

  GPS_DATA_t st_GPS;
  IMU_DATA_t st_IMU;

  float64_t f64_Latitude;
  float64_t f64_Longitude;
  float64_t f64_Altitude;

  float32_t f32_Roll_rad;
  float32_t f32_Pitch_rad;
  float32_t f32_Yaw_rad;

  float32_t f32_AccelX;
  float32_t f32_AccelY;
  float32_t f32_AccelZ;

  float32_t f32_GyroX;
  float32_t f32_GyroY;
  float32_t f32_GyroZ;

  float32_t f32_VehicleSpeed_m_s;
  float32_t f32_VehicleSpeed_kph;

  EKF_DATA_t st_EKF_data;
  GPS_IMU_EKF_MATRIX_t st_EKF_Matrix;
  VEHICLE_ODOMETRY_t st_Vehicle_Odometry;
} DEAD_RECKONING_DATA_t;

typedef struct FILE_INFO
{
  FILE *pFile;
  string st_FileName;
  int32_t s32_CntFrame;
  int32_t s32_AllSensorBufferSize;
  uint64_t ars64_Frame[c_MAX_FRAME_SIZE];
} FILE_INFO_t;

typedef struct _FRENET_PARAM
{

  float32_t f32_WheelBase;

  float32_t f32_MaxSpeed_kph;
  float32_t f32_MaxAccel;
  float32_t f32_MaxCurvature;
  float32_t f32_DT;
  float32_t f32_MaxT;
  float32_t f32_MinT;
  float32_t f32_TargetSpeed_m_s;
  float32_t f32_TargetSpeed_kph;
  float32_t f32_SpeedSamplingTime;

  int32_t s32_SampleNum;
  int32_t s32_PathNum;
  float32_t f32_KJ;
  float32_t f32_KT;
  float32_t f32_KD;
  float32_t f32_KLat;
  float32_t f32_KLon;
  float32_t f32_Ratio;

  //////////////////////////////////////////////////////////////////////////
  float32_t f32_EgoParam;
  float32_t f32_RelativeParam;
  float32_t f32_MinMarginParam;
  float32_t f32_MaxBrakeThreshold;

  //////////////////////////////////////////////////////////////////////////
  float32_t f32_CurV;
  float32_t f32_CurA;
  float32_t f32_CurD0;
  float32_t f32_CurD1;
  float32_t f32_CurD2;
  float32_t f32_CurS0;
  float32_t f32_CurS1;
  float32_t f32_CurS2;
  /////////////////////////////////////////////////////////////////////////

} FRENET_PARAM_t;

typedef struct _QUINTIC
{

  float32_t f32_XS;
  float32_t f32_VXS;
  float32_t f32_AXS;
  float32_t f32_XE;
  float32_t f32_VXE;
  float32_t f32_AXE;
  float32_t f32_A0;
  float32_t f32_A1;
  float32_t f32_A2;
  float32_t f32_A3;
  float32_t f32_A4;
  float32_t f32_A5;

  void Init(float32_t f32_tXS, float32_t f32_tVXS, float32_t f32_tAXS, float32_t f32_tXE, float32_t f32_tVXE, float32_t f32_tAXE, float32_t f32_T);
  float32_t CalcPoint(float32_t f32_T);
  float32_t CalcFirstDerivative(float32_t f32_T);
  float32_t CalcSecondDerivative(float32_t f32_T);
  float32_t CalcThirdDerivative(float32_t f32_T);

} QUINTIC_t;

typedef struct _QUARTIC
{

  float32_t f32_XS;
  float32_t f32_VXS;
  float32_t f32_AXS;
  float32_t f32_VXE;
  float32_t f32_AXE;
  float32_t f32_A0;
  float32_t f32_A1;
  float32_t f32_A2;
  float32_t f32_A3;
  float32_t f32_A4;

  void Init(float32_t f32_tXS, float32_t f32_tVXS, float32_t f32_tAXS, float32_t f32_tVXE, float32_t f32_tAXE, float32_t f32_T);
  float32_t CalcPoint(float32_t f32_T);
  float32_t CalcFirstDerivative(float32_t f32_T);
  float32_t CalcSecondDerivative(float32_t f32_T);
  float32_t CalcThirdDerivative(float32_t f32_T);

} QUARTIC_t;

typedef struct _SPLINE_PARAM
{

  float32_t arf32_X[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_Y[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_H[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_A[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_B[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_C[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_D[c_PLANNING_MAX_SPLINE_NUM];
  int32_t s32_Num;

  _SPLINE_PARAM();
  _SPLINE_PARAM(float32_t x_[], float32_t y_[], int32_t s32_Num);

  float32_t calc(float32_t f32_T);
  float32_t calc_d(float32_t f32_T);
  float32_t calc_dd(float32_t f32_T);

  Eigen::MatrixXf calc_A();
  Eigen::VectorXf calc_B();
  int32_t CalcNearestIdx(float32_t f32_T, int32_t s32_Start, int32_t s32_End);

} SPLINE_PARAM_t;

typedef struct _SPLINE2D_PARAM
{
  SPLINE_PARAM_t st_SplineX;
  SPLINE_PARAM_t st_SplineY;
  float32_t arf32_S[c_PLANNING_MAX_SPLINE_NUM];
  int32_t s32_Size;

  _SPLINE2D_PARAM();
  void initialize(float32_t arf32_X[], float32_t arf32_Y[], int32_t s32_Num);
  PointArray2D calc_position(float32_t f32_T);
  float32_t calc_curvature(float32_t f32_T);
  float32_t calc_yaw(float32_t f32_T);
  int32_t calc_s(float32_t arf32_X[], float32_t arf32_Y[], int32_t s32_Num);
} SPLINE2D_PARAM_t;

typedef struct _SPLINE_PATH
{
  float32_t arf32_X[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_Y[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_Yaw_rad_ENU[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_Yaw_rad_NED[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_Curvature[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_Speed_kph[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_S[c_PLANNING_MAX_SPLINE_NUM];
  float32_t arf32_D[c_PLANNING_MAX_SPLINE_NUM];
  int32_t s32_Num;
  int32_t s32_NearestIndex = 0;
  float32_t f32_LastS;
  float32_t f32_LastBeforeS;

} SPLINE_PATH_t;

typedef struct _FRENET_PATH
{

  float32_t arf32_T[c_PLANNING_MAX_FRENET_NUM];

  float32_t arf32_D0[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_D1[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_D2[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_D3[c_PLANNING_MAX_FRENET_NUM];

  float32_t arf32_S0[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_S1[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_S2[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_S3[c_PLANNING_MAX_FRENET_NUM];

  float32_t f32_CostD;
  float32_t f32_CostV;
  float32_t f32_CostF;

  float32_t arf32_X[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_Y[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_Yaw_rad_ENU[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_Yaw_rad_NED[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_DS[c_PLANNING_MAX_FRENET_NUM];
  float32_t arf32_C[c_PLANNING_MAX_FRENET_NUM];
  int32_t s32_CurrentLane;
  int32_t s32_Num;
  bool b_IsCollision = false;
  int32_t ars32_Collision[100];
  int32_t s32_CollisionNum = 0;
} FRENET_PATH_t;

typedef struct _ACCCHECK
{

  int32_t ars32_ProbAccObjIdx[100];
  int32_t s32_Num = 0;

} ACCCHECK_t;

typedef struct _PLANNER
{

  int32_t s32_TargetLane;

  int32_t s32_LabCount;

  bool b_Bank;
  bool b_PlanningBank;

  bool b_RestartStop = false;
  uint64_t u64_RestartStopEndTime = 0;

  uint8_t u8_GoSignal;
  uint8_t u8_StopSignal;
  uint8_t u8_SlowOnSignal;
  uint8_t u8_SlowOffSignal;
  uint8_t u8_PitStopSignal;

  uint8_t u8_PitStopMode = false;
  uint8_t u8_PitStopReady = false;

  bool b_AccMode = false;
  bool b_AebMode = false;
  bool b_AbsMode = false;
  bool b_OnceAbs = false;

  uint64_t u64_OverTakingCnt = 0;
  uint64_t u64_OverTakingSuccessCnt = 0;
  uint64_t u64_UnStableCnt = 0;
  bool b_OverTakingMode = false;

  int32_t ars32_AvailFrenetPath[17];
  int32_t ars32_AvailGridPath[17];
  bool b_NonAvailLane = false;

  bool b_PitStop = false;
  uint64_t u64_EndTime_s;
  float32_t f32_DeltaTime_s;
  uint64_t u64_StartTime_s;

  uint64_t u64_OvertakingTime = 0;
  uint64_t u64_OvertakingSuccessTime = 0;
  uint64_t u64_UnStableTime = 0;

} PLANNER_t;

typedef enum _VEHICLE_STATE
{
  STATE_IDLE,
  STATE_IDLE_ACC_MODE,
  STATE_OVERTAKING,
  STATE_NON_STABLE_MODE,
  STATE_NON_STABLE_ACC_MODE,
  STATE_STATIC,
  STATE_PITSTOP,
  STATE_EMERGENCY
} VEHICLE_STATE_t;

typedef struct _PLANNING_DATA
{
  uint64_t u64_Timestamp;

  int32_t s32_Round;
  int32_t s32_StartLane;

  PATH_t st_ReferenceLine1_global;
  PATH_t st_ReferenceLine2_global;
  PATH_t st_ReferenceLine3_global;

  PATH_t st_ReferenceSELine1_global;
  PATH_t st_ReferenceSELine2_global;
  PATH_t st_ReferenceSELine3_global;

  PATH_t st_IDX_global;

  // check
  PATH_t st_Boundary5_global;
  PATH_t st_Boundary6_global;
  //

  PATH_t st_LineGap1;
  PATH_t st_LineGap3;

  PATH_t st_SELineGap1;
  PATH_t st_SELineGap3;

  PATH_t st_Boundary1_global;
  PATH_t st_Boundary2_global;
  PATH_t st_Boundary3_global;
  PATH_t st_Boundary4_global;

  PATH_t st_SEBoundary1_global;
  PATH_t st_SEBoundary2_global;
  PATH_t st_SEBoundary3_global;
  PATH_t st_SEBoundary4_global;

  PATH_t st_BoundaryInOut;
  PATH_t st_BoundaryIn2;

  PATH_t st_BoundaryGap1;
  PATH_t st_BoundaryGap2;
  PATH_t st_BoundaryGap3;

  PATH_t st_SEBoundaryGap1;
  PATH_t st_SEBoundaryGap2;
  PATH_t st_SEBoundaryGap3;

  // Final Path
  PATH_t st_FinalPath_global;

  // Spline Line2
  SPLINE2D_PARAM_t st_Spline2DParam;
  SPLINE_PATH_t st_SplinePath_global;

  // Spline PitStop
  SPLINE2D_PARAM_t st_PisStopSpline2DParam;
  SPLINE_PATH_t st_PitStopSplinePath_global;

  // Frenet
  FRENET_PATH_t arst_FrenetPath_global[c_PLANNING_MAX_FRENET_PATH_NUM];
  FRENET_PATH_t arst_CollisonCheckPath_global[c_PLANNING_MAX_FRENET_PATH_NUM];
  FRENET_PATH_t arst_ControlPath_global[c_PLANNING_MAX_FRENET_PATH_NUM];
  FRENET_PARAM_t st_FrenetParam;
  int32_t s32_FrenetPathNum;
  int32_t s32_BestPath;
  int32_t s32_FrenetBestIdx[3];

  float32_t f32_PathGapList[3];

  // Object Data
  VEHICLE_DATA_t arst_ObjectData[c_TOTAL_CLUSTER_NUM];
  int32_t s32_ObjectNum;
  int32_t s32_AccObjIdx = -1;

  // Car Data
  VEHICLE_DATA_t arst_CarData[c_TOTAL_CLUSTER_NUM];
  int32_t s32_CarNum;

  // Ego Vehicle Data
  VEHICLE_DATA_t st_EgoVehicleData;

  // Planner
  PLANNER_t st_Planner;

  // AccCheck
  ACCCHECK_t st_AccCheck;

  // Speed to Control
  int32_t s32_SpeedSignal = 0;
  float32_t f32_TargetSpeed_kph;

} PLANNING_DATA_t;

typedef struct _State
{
  float32_t f32_CX;
  float32_t f32_CY;
  float32_t f32_VelocityX_m_s;
  float32_t f32_VelocityY_m_s;
  float32_t f32_Yaw_rad_ENU;
  float32_t f32_Yaw_rad_NED;
  float32_t f32_Yawrate_rad_s;

} State_t;

typedef struct _Lat_MPC
{
  // Current State
  State_t st_CurrentState;

  // MPC Tmp State
  State_t st_PredictState;

  // reference State
  float32_t arf32_Ref_Y[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Ref_VY_m_s[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Ref_Yaw_rad[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Ref_Yawrate_rad_s[c_CONTROL_MAX_HORIZON + 1];
  casadi::MX MX_refState;

  // predict State
  float32_t arf32_Predict_Y[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Predict_VY_m_s[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Predict_Yaw_rad[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Predict_Yawrate_rad_s[c_CONTROL_MAX_HORIZON + 1];

  // Solve State
  float32_t arf32_Sol_Y[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Sol_VY_m_s[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Sol_Yaw_rad[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Sol_Yawrate_rad_s[c_CONTROL_MAX_HORIZON + 1];
  float64_t arf32_SolSteer[c_CONTROL_MAX_HORIZON] = {0};

  // MPC Parameters
  float32_t arf32_Q[4];
  float32_t arf32_Qf[4];
  float32_t f32_R;
  float32_t f32_Rd;

} LAT_MPC_t;

typedef struct _Long_MPC
{
  // Current State
  float32_t f32_X;
  float32_t f32_Velocity_m_s;

  // Tmp State
  float32_t f32_Predict_X;
  float32_t f32_Predict_Velocity_m_s;

  // Reference State
  casadi::MX MX_refState;

  // predict State
  float32_t arf32_Predict_X[c_CONTROL_MAX_HORIZON + 1];
  float32_t arf32_Predict_Velocity[c_CONTROL_MAX_HORIZON + 1];

  // Solve State
  float64_t arf32_Sol_X[c_CONTROL_MAX_HORIZON + 1];
  float64_t arf32_Sol_Velocity[c_CONTROL_MAX_HORIZON + 1];
  float64_t arf32_Sol_Throttle[c_CONTROL_MAX_HORIZON] = {0};

  // Pre Velocity
  float32_t f32_PreVelocity_m_s = 0;

  // MPC Parameters
  float32_t arf32_Q[2];
  float32_t arf32_Qf[2];
  float32_t f32_R;
  float32_t f32_Rd;

} LONG_MPC_t;

typedef struct _LKAS
{
  const float32_t f32_k = 1.0;
  const float32_t f32_ke = 0.5;

  float32_t f32_MAX_DSTEER_rad_s;
  float32_t f32_PreSteerAngle_rad = 0;
  float32_t f32_HeadingError = 0;
  float32_t f32_PathError = 0;

} LKAS_t;

typedef struct _ACC
{
  float32_t f32_ErrorVelocity = 0;
  float32_t f32_ErrorDistance = 0;
  float32_t f32_GoalDistance = 0;
  float32_t f32_GoalVelocity = 0;
  float32_t f32_MaxVelocity = 120 / 3.6;
  float32_t f32_MaxDistance = 100;

  float32_t f32_SafetyDist = 0;
  float32_t f32_RelDist = 0;
  float32_t f32_RelVel = 0;

} ACC_t;

typedef struct _AdvancedStanley
{
  float32_t f32_lowk;
  float32_t f32_highk;
  float32_t f32_ke;

  float32_t f32_FX;
  float32_t f32_FY;

  float32_t f32_Error;
  float32_t f32_FrontDist = 0;
  float32_t f32_speed = 0;
  float32_t f32_ReferenceYaw = 0;

  float32_t f32_LD = 0;
  int32_t s32_LDlen = 0;
  int32_t s32_FrontNear = 0;
  float32_t f32_Factor = 2.5;
  float32_t f32_PreSteer_rad = 0;

} Stanley_t;

typedef struct _PID
{
  float32_t f32_KP;
  float32_t f32_KI;
  float32_t f32_KD;

  float32_t f32_Error = 0;
  float32_t f32_PreError = 0;
  float32_t f32_I_Error = 0;

  float32_t f32_InputP = 0;
  float32_t f32_InputI = 0;
  float32_t f32_InputD = 0;

} PID_t;

typedef struct _Control_Param
{
  // vehicle parameters
  const float32_t f32_WB = 2.720;
  const float32_t f32_LF = 1.139;
  const float32_t f32_LR = 1.581;
  const float32_t f32_IZ = 1392.97; // [kg*m^2] Temporary
  const float32_t f32_Mass = 1380.0;
  const float32_t f32_GPS_to_Front = 1.07;

  // Tire Model
  const float32_t f32_CF = 163264.75; // [N/rad]
  const float32_t f32_CR = 163264.75; // [N/rad] Temporay

  // MPC Parameters
  uint32_t u32_NX;
  uint32_t u32_NU;
  uint32_t u32_T;
  float32_t f32_DT;
  float32_t f32_Sample_DT;

  // Longitudinal Dynamics
  float32_t f32_MAX_SPEED_m_s;       // [m/s]
  float32_t f32_MIN_SPEED_m_s = 0.0; // [m/s]
  float32_t f32_MAX_SPEED_kph;       // [kph]
  float32_t f32_MIN_SPEED_kph;       // [kph]

  // Lateral Dynamics
  float32_t f32_MAX_STEER_rad = 0.6088; // [rad]
  float32_t f32_MAX_DSTEER_rad_s;       // [rad/s]
  float32_t f32_MAX_ACCEL_m_ss = 1.66;  // [m/ss]

  float32_t arf32_bankSpeed[3] = {110.0, 90.0, 80.0};
  float32_t arf32_bankOutSpeed[3] = {80.0, 90.0, 100.0};
  float32_t arf32_SlipRatio[3] = {0.6f, 0.5f, 0.48f};

} CONTROL_PARAM_t;

typedef struct _GG_Diagram
{
  float32_t arf32_AccelX[c_CONTROL_MAX_HORIZON] = {0};
  float32_t arf32_AccelY[c_CONTROL_MAX_HORIZON] = {0};

  float32_t f32_MaxAccelUpperX = 0.3f;
  float32_t f32_MaxAccelLowerX = -1.0f;
  float32_t f32_MaxAccelY = 0.6f;

} GG_Diagram_t;

typedef struct _CONTROL_DATA
{
  // Current State
  State_t st_OriginState;

  // SetUp
  int32_t s32_ControlMethod;
  int32_t s32_SpeedProfileMode;
  int32_t s32_V2XMode;
  int32_t s32_Judgement;
  bool b_PreBank = false;
  bool b_BankInControl = false;
  bool b_BankOutControl = false;

  // PitStop
  bool b_PitStopFlag = false;
  float32_t f32_FinalDistance;
  float32_t f32_FinalVelocity_kph;

  // Target Control Input
  float32_t f32_TargetSteer_deg;
  float32_t f32_TargetSteer_rad;
  float32_t f32_TargetSpeed_kph;
  float32_t f32_TargetSpeed_m_s;
  float32_t f32_TargetThrottle;

  // dl
  float64_t arf64_dl[c_PLANNING_MAX_PATH_NUM];
  float64_t f64_Max_Distance;

  // GG Diagram
  GG_Diagram_t st_GG_Diagram;

  // Control
  CONTROL_PARAM_t st_Param;
  LKAS_t st_LKAS;
  Stanley_t st_Stanley;
  ACC_t st_ACC;

  // MPC
  LONG_MPC_t st_LongMPC;
  LAT_MPC_t st_LatMPC;
  float32_t f32_PreYaw = 0;

  // PID
  PID_t st_PID_LKAS;
  PID_t st_PID_Throttle;
  PID_t st_PID_ACC;
  PID_t st_PID_MPC;

} CONTROL_DATA_t;

typedef struct _LOGIC_HZ
{
  uint64_t u64_LiDARHz;
  uint64_t u64_CameraHz;
  uint64_t u64_DeadReckoningHz;
  uint64_t u64_PlanningHz;
  uint64_t u64_ControlHz;
  uint64_t u64_CANHz;
} LOGIC_HZ_t;

void ms2hms(uint64_t &u64_Time, int32_t *ars_Time);

uint64_t getMillisecond();
uint64_t getMicrosecond();

float32_t deg2rad(float32_t f32_Degree);
float32_t rad2deg(float32_t f32_Radian);

float64_t deg2rad(float64_t f64_Degree);
float64_t rad2deg(float64_t f64_Radian);

float32_t ms2kph(float32_t f32_Speed);
float32_t kph2ms(float32_t f32_Speed);

float64_t ms2kph(float64_t f64_Speed);
float64_t kph2ms(float64_t f64_Speed);

float32_t getDistance3d(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2);
float64_t getDistance3d(float64_t f64_X1, float64_t f64_Y1, float64_t f64_Z1, float64_t f64_X2, float64_t f64_Y2, float64_t f64_Z2);

float32_t getDistance2d(float32_t f32_X1, float32_t f32_Y1, float32_t f32_X2, float32_t f32_Y2);
float64_t getDistance2d(float64_t f64_X1, float64_t f64_Y1, float64_t f64_X2, float64_t f64_Y2);

float32_t getNorm2d(float32_t f32_X, float32_t f32_Y);
float64_t getNorm2d(float64_t f32_X, float64_t f32_Y);

float32_t pi2pi(float32_t f32_Angle);
float64_t pi2pi(float64_t f64_Angle);

float32_t axisRotate(float32_t f32_Heading_rad);
float64_t axisRotate(float64_t f64_Heading_rad);

void CalcVertex(float32_t f32_X_global, float32_t f32_Y_global, float32_t f32_MaxX, float32_t f32_MaxY, float32_t f32_MinX, float32_t f32_MinY, float32_t f32_heading, float32_t (&f32_Vertex)[4][2], int32_t s32_Mode = -1);

void PolygonProjection(POINT_t axis, POINT_t vertices[], int numVertices, float32_t *min, float32_t *max);
bool OverlappingTest(float32_t minA, float32_t maxA, float32_t minB, float32_t maxB);
bool CalcSAT(float32_t vertices1[4][2], float32_t vertices2[4][2]);

POINT_t CalcTangentVector(POINT_t p1, POINT_t p2);
POINT_t CalcNormalVector(POINT_t edge);
float32_t CalcDotProduct(POINT_t v1, POINT_t v2);

void GetModData(float32_t min, float32_t max, float32_t &data);
void GetModData(float64_t min, float64_t max, float64_t &data);
void GetModData(int32_t min, int32_t max, int32_t &data);

void GetRadius(float32_t f32_X1, float32_t f32_Y1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_X3, float32_t f32_Y3, float32_t &Radius);
void GetRadius(float64_t f32_X1, float64_t f32_Y1, float64_t f32_X2, float64_t f32_Y2, float64_t f32_X3, float64_t f32_Y3, float64_t &Radius);

bool InRange(float32_t min, float32_t max, float32_t data);
bool InRange(float64_t min, float64_t max, float64_t data);
bool InRange(int32_t min, int32_t max, int32_t data);

void OverShootFilter(float32_t f32_Maxtick, float32_t f32_Pre_data, float32_t &f32_Target_data);
void OverShootFilter(float64_t f64_Maxtick, float64_t f64_Pre_data, float64_t &f64_Target_data);

void CalcNearestDistIdx(float64_t f64_X, float64_t f64_Y, float64_t *f64_MapX, float64_t *f64_MapY, int32_t s32_MapLength, float64_t &f64_NearDist, int32_t &s32_NearIdx);
void CalcNearestDistIdx(float32_t f32_X, float32_t f32_Y, float32_t *f32_MapX, float32_t *f32_MapY, int32_t s32_MapLength, float32_t &f32_NearDist, int32_t &s32_NearIdx);

void lla2enu(float64_t f64_Lat_deg, float64_t f64_Lon_deg, float64_t f64_Alt, float32_t &f32_E, float32_t &f32_N, float32_t &f32_U);

void CreateBagFileName(char *datetime);
void CreateFile();
void LoggingFirstRead();
bool CompareFilenames(const std::string &a, const std::string &b);
int32_t extractNumberFromFilename(const std::string &filename);
void rotationMatrix(float32_t roll, float32_t pitch, float32_t yaw, float32_t matrix[3][3]);
void rotatePoint(float32_t f32_X, float32_t f32_Y, float32_t f32_Z, float32_t roll, float32_t pitch, float32_t yaw, float32_t &f32_RX, float32_t &f32_RY, float32_t &f32_RZ);
void getLocalCoord(float32_t f32_X, float32_t f32_Y, float32_t f32_Yaw_rad, float32_t f32_GlobalX, float32_t f32_GlobalY, float32_t &f32_ResultX, float32_t &f32_ResultY);
void getGlobalCoord(float32_t f32_X, float32_t f32_Y, float32_t f32_Yaw_rad, float32_t f32_LocalX, float32_t f32_LocalY, float32_t &f32_ResultX, float32_t &f32_ResultY);

void DeadReckoningProcessing(RAW_GPSINS_DATA_t *pst_GPSINSData,
                             HMC_LIDAR_DATA_t *pst_LidarData,
                             RAW_IMU_DATA_t *pst_IMUData,
                             RAW_CAN_DATA_t *pst_CANData,
                             DEAD_RECKONING_DATA_t *pst_DeadReckoningData);

void Gray2RGB(float32_t f32_Gray, float32_t &f32_R, float32_t &f32_G, float32_t &f32_B);

#endif
