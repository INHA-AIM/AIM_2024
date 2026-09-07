#include "Global/global.h"
#include "LiDAR/lidar128.h"
#include "LiDAR/HMC_LiDAR.h"
#include "Viewer/viewer.h"
#include "Parser/parser.h"
#include "Planning/Test/testplanning.h"
#include "Planning/TestDrive1/testdrive1.h"
#include "Planning/TestDrive2/testdrive2.h"
#include "Planning/Preliminary2/preliminary2.h"
#include "Planning/Final/final.h"
#include "Planning/Offline/offline.h"
#include "Control/control.h"
#include "DeadReckoning/deadReckoning.h"
#include "CAN/Can.h"
#include <yaml-cpp/yaml.h>

float64_t c_ORIGIN_LATITUDE_DEG;
float64_t c_ORIGIN_LONGITUDE_DEG;
float64_t c_ORIGIN_LATITUDE_RAD;
float64_t c_ORIGIN_LONGITUDE_RAD;
float64_t c_ORIGIN_ALTITUDE;
float64_t c_ORIGIN_REFERENCE_X;
float64_t c_ORIGIN_REFERENCE_Y;
float64_t c_ORIGIN_REFERENCE_Z;

int s32_Va = 0;

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Database 구축 ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

string s_GlobalPath = ros::package::getPath("Integration");
SENSOR_DATA_t st_SensorData;
SENSOR_DATA_t st_CurrentData;
FILE_INFO_t st_FileInfo;

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Bag File 구축 ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

string s_LoggingPath;              // = s_GlobalPath + "/rosbag";
vector<string> st_LoggingFileList; // vector 자료형에 대한 변수명 추가 설정
int32_t s32_FileCnt = 0;
int32_t s32_FileNum = 0;
int32_t s32_FileFrameCnt = 0;
int32_t s32_SelectFileNum = -1;
int32_t s32_CurrentFileNum = -1;
uint64_t u64_AllFileSize = 0;
uint64_t u64_AllFilePointer = 0;
bool b_LoggingFileOpen = false;

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Viewer 관련 ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
bool b_Running = true;

VIEWER_PARAMETER_t st_ViewerParam = {0};
POS_t st_Pos;
EGO_POS_t st_Ego_Pos;
F32_INFO_t st_Prev = {0.0f};  // 이전 state value를 저장시킴으로써 유지
F32_INFO_t st_Trans = {0.0f}; // 변화량
GLuint WebcamTexture;
GLFWwindow *pst_MainWindow;
LOGIC_HZ_t st_LogicHz;
TRAJECTORY_t st_Trajectory;

float64_t f64_PrevXPos_G = 0.0;
float64_t f64_PrevYPos_G = 0.0;
float64_t f64_PrevXPos_L = 0.0;
float64_t f64_PrevYPos_L = 0.0;

bool b_switchToMain = true; // no need to use
bool b_Flag = false;        // 마우스가 클릭 된 상황인지 확인

int32_t b_Start = false;
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// MAP & Path ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

bool b_RefLLA = true;
int32_t s32_MapMake = 0;
float32_t f32_PathGap = 0;
std::string s_OriginAdress = "./src/Integration/map";
std::string s_DirectoryName = "";
std::string s_PathName = "";
std::string s_MapName = "";
float32_t f32_LabCheckX1 = 0, f32_LabCheckX2 = 0, f32_LabCheckY1 = 0, f32_LabCheckY2 = 0;
int32_t s32_LabCount = 0;

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Camera ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
VISUAL_ODOMETRY_t st_VisualOdometry; // <- Temp
Mat st_IntrinsicParam = (Mat_<double>(3, 3) << 1716.1244, 0, 1440,
                         0, 1716.1244, 930,
                         0, 0, 1);
cv::BFMatcher st_BFMatcher(cv::NORM_HAMMING);

CAMERA_DATA_t st_CameraData = {0};
Point arst_LaneWayPnt[7];
int32_t s32_CntWayPnt = 0;
cv::Mat st_BGRImage;
CAMERA_LANEINFO_t st_MainLaneInfoLeft, st_MainLaneInfoRight;
LANE_COEFFICIENT_t st_LastLaneCoefLeft, st_LastLaneCoefRight;
int32_t s32_ImageCount = 0;
bool b_RunOnGPU = true;
bool b_FirstFrame = true;
int32_t s32_ImShowKey = 0;
int32_t s32_CntInvalidVanishingPnt = 0;
bool b_ResetLaneWayPnt = false;
bool b_LastValidVanishingPnt = false;
LANE_STATUS_t st_LaneStatus;

// Inference st_Inference("./src/Integration/Param/yolov8s.onnx", cv::Size(640, 480), "./src/Integration/Param/classes.txt", b_RunOnGPU);
// Inference st_Inference("./src/Integration/Param/best2.onnx", cv::Size(416, 416), "./src/Integration/Param/classes.txt", b_RunOnGPU);

////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// LIDAR ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
std::string s_LIDARName = "";
LIDAR_DATA_t st_LidarData = {0};
HMC_LIDAR_DATA_t st_HMCLidarData = {0};
LIDAR_PARAM_t st_LidarParam = {0};

string s_LiDARHzMode;
////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// DeadReckoning ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
DEAD_RECKONING_DATA_t st_DeadReckoningData = {0};
serial::Serial GPSSerial;

////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// Planning ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
PLANNING_DATA_t st_PlanningData = {0};
VEHICLE_STATE_t st_VehicleState = STATE_IDLE;

int32_t s32_RoundName;
////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// Control ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
CONTROL_DATA_t st_ControlData = {0};

////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// CAN ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
CAN_DATA_t st_CANData = {0};
int32_t s32_CanMode;
bool b_CANMODE = false;
int32_t s32_HZContolNum = 0;

bool goActive = false, stopActive = false, pitStopActive = false, slowOnActive = false, slowOffActive = false;
bool Go = false, Stop = false, PitStop = false, SlowOn = false, SlowOff = false;

pthread_mutex_t st_ControlToCAN;
pthread_mutex_t st_CopyMutex;
pthread_mutex_t Logging_Mutex;
pthread_mutex_t st_DataTemporal;
pthread_mutex_t pth_GpsImuThread;
pthread_cond_t st_DataTemporalcond;
pthread_cond_t pth_GpsImuCond;

CAMERA_DATA_t st_CameraDataTemporal = {0};
CAMERA_DATA_t st_CameraDataViewer = {0};

LIDAR_DATA_t st_LidarDataTemporal = {0};
LIDAR_DATA_t st_LidarDataViewer = {0};

HMC_LIDAR_DATA_t st_HMCLidarDataTemporal = {0};
HMC_LIDAR_DATA_t st_HMCLidarDataViewer = {0};

OMNI_POINT_t st_OmniPoint = {0};
std::string s_OmniPointFileName;
ICP_MAP_t st_ICPMap = {0};

DEAD_RECKONING_DATA_t st_DeadReckoningDataTemporal = {0};
DEAD_RECKONING_DATA_t st_DeadReckoningDataViewer = {0};

PLANNING_DATA_t st_PlanningDataTemporal = {0};
PLANNING_DATA_t st_PlanningDataViewer = {0};

CONTROL_DATA_t st_ControlDataTemporal = {0};
CONTROL_DATA_t st_ControlDataViewer = {0};

CONTROL_DATA_t st_CANDataTemporal = {0};
CONTROL_DATA_t st_CANDataViewer = {0};

void InitMutex()
{
    pthread_mutex_init(&st_CopyMutex, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexLIDAR, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexCamera, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexGPSINS, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexIMU, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexCAN, NULL);
    pthread_mutex_init(&st_ControlToCAN, NULL);
    pthread_mutex_init(&st_DataTemporal, NULL);
}

void InitPlanning()
{

    YAML::Node st_Config2 = YAML::LoadFile("configuration.yaml");

    st_PlanningData.s32_Round = st_Config2["Round"].as<int32_t>();
    st_PlanningData.s32_StartLane = st_Config2["StartLane"].as<int32_t>();
    st_PlanningData.st_Planner.s32_TargetLane = st_Config2["TargetLane"].as<int32_t>();

    // Frenet Parameters
    YAML::Node st_Config = YAML::LoadFile("PlanningParam.yaml");
    st_PlanningData.st_FrenetParam.f32_EgoParam = st_Config["EgoParam"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_RelativeParam = st_Config["RelativeParam"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_MinMarginParam = st_Config["MinMarginParam"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_MaxBrakeThreshold = st_Config["MaxBrakeThreshold"].as<float32_t>();

    st_PlanningData.st_FrenetParam.s32_PathNum = st_Config["PathNum"].as<int32_t>();

    st_PlanningData.st_FrenetParam.f32_TargetSpeed_kph = st_Config["TargetSpeedkph"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_TargetSpeed_m_s = kph2ms(st_PlanningData.st_FrenetParam.f32_TargetSpeed_kph);

    st_PlanningData.st_FrenetParam.f32_MaxAccel = st_Config["MaxAccel"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_MaxCurvature = st_Config["MaxCurvature"].as<float32_t>();

    st_PlanningData.st_FrenetParam.f32_DT = st_Config["SamplingTime"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_MaxT = st_Config["MaxPredictionTime"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_MinT = st_Config["MinPredictionTime"].as<float32_t>();

    st_PlanningData.st_FrenetParam.f32_SpeedSamplingTime = st_Config["SpeedSamplingTime"].as<float32_t>();
    st_PlanningData.st_FrenetParam.s32_SampleNum = st_Config["SpeedSamplingNum"].as<int32_t>();

    st_PlanningData.st_FrenetParam.f32_KJ = st_Config["CostJ"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_KT = st_Config["CostT"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_KD = st_Config["CostD"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_KLat = st_Config["CostLat"].as<float32_t>();
    st_PlanningData.st_FrenetParam.f32_KLon = st_Config["costLon"].as<float32_t>();

    st_PlanningData.st_FrenetParam.f32_Ratio = st_Config["Ratio"].as<float32_t>();

    st_PlanningData.st_EgoVehicleData.f32_FrontHangOver = st_Config["FrontHangOver"].as<float32_t>();
    st_PlanningData.st_EgoVehicleData.f32_RearHangOver = st_Config["RearHangOver"].as<float32_t>();
    st_PlanningData.st_EgoVehicleData.f32_GpsToFront = st_Config["GpsToFront"].as<float32_t>();
    st_PlanningData.st_EgoVehicleData.f32_GpsToRear = st_Config["GpsToRear"].as<float32_t>();
    st_PlanningData.st_EgoVehicleData.f32_WholeLength = st_Config["WholeLength"].as<float32_t>();
    st_PlanningData.st_EgoVehicleData.f32_Width = st_Config["Width"].as<float32_t>();

    st_PlanningData.st_EgoVehicleData.f32_MaxX = st_PlanningData.st_EgoVehicleData.f32_FrontHangOver + st_PlanningData.st_EgoVehicleData.f32_GpsToFront;
    st_PlanningData.st_EgoVehicleData.f32_MinX = -st_PlanningData.st_EgoVehicleData.f32_RearHangOver - st_PlanningData.st_EgoVehicleData.f32_GpsToRear;
    st_PlanningData.st_EgoVehicleData.f32_MaxY = st_PlanningData.st_EgoVehicleData.f32_Width / 2;
    st_PlanningData.st_EgoVehicleData.f32_MinY = -st_PlanningData.st_EgoVehicleData.f32_Width / 2;

    // Frenet Parameters

    st_PlanningData.s32_BestPath = 0;
}

void InitSerial()
{
    std::string s_GPSPort = "";
    int32_t s32_Boudrate = 0;
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");
    s_GPSPort = st_Config["GPS_Port"].as<std::string>();
    s32_Boudrate = st_Config["GPS_Boudrate"].as<int32_t>();
    try
    {
        GPSSerial.setPort(s_GPSPort);
        GPSSerial.setBaudrate(s32_Boudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(50); // check
        GPSSerial.setTimeout(to);
        GPSSerial.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return;
    }
}

void InitGlobal()
{
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");
    s_MapName = st_Config["MapName"].as<std::string>();
    std::string s_MapAdress = "./src/Integration/map/" + s_MapName + "/ref.txt";
    s_OmniPointFileName = st_Config["OmniPoint"].as<std::string>();

    s32_MapMake = st_Config["MapMake"].as<int32_t>();
    f32_PathGap = st_Config["PathGap"].as<float32_t>();
    s_DirectoryName = st_Config["DirectoryName"].as<std::string>();
    s_PathName = st_Config["PathName"].as<std::string>();

    std::ifstream st_ReferenceLLA(s_MapAdress);
    st_ReferenceLLA >> c_ORIGIN_LATITUDE_DEG >> c_ORIGIN_LONGITUDE_DEG >> c_ORIGIN_ALTITUDE;

    st_ReferenceLLA.close();
    c_ORIGIN_LATITUDE_RAD = deg2rad(c_ORIGIN_LATITUDE_DEG);
    c_ORIGIN_LONGITUDE_RAD = deg2rad(c_ORIGIN_LONGITUDE_DEG);

    float64_t f64_Lat_rad = c_ORIGIN_LATITUDE_RAD;
    float64_t f64_Lon_rad = c_ORIGIN_LONGITUDE_RAD;
    float64_t f64_Alt = c_ORIGIN_ALTITUDE;

    float64_t f64_Chi = sqrt(1 - c_LLA2ENU_N_2 * pow(sin(f64_Lat_rad), 2));
    float64_t f64_Q = (c_LLA2ENU_A / f64_Chi + f64_Alt) * cos(f64_Lat_rad);

    c_ORIGIN_REFERENCE_X = f64_Q * cos(f64_Lon_rad);
    c_ORIGIN_REFERENCE_Y = f64_Q * sin(f64_Lon_rad);
    c_ORIGIN_REFERENCE_Z = ((c_LLA2ENU_A * (1 - c_LLA2ENU_N_2) / f64_Chi) + f64_Alt) * sin(f64_Lat_rad);
}

void InitViewer()
{
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");

    // st_ViewerParam.s32_CurrentMode = c_VIEWER_REAL_MODE;
    st_ViewerParam.s32_ScreenWidth = st_Config["width"].as<int32_t>();
    st_ViewerParam.s32_ScreenHeight = st_Config["height"].as<int32_t>();
    s_LoggingPath = s_GlobalPath + "/" + st_Config["logging"].as<std::string>();
}

void InitControl()
{
    YAML::Node st_Config = YAML::LoadFile("ControlParam.yaml");

    st_ControlData.s32_ControlMethod = st_Config["ControlMethod"].as<int32_t>();
    st_ControlData.s32_SpeedProfileMode = st_Config["SpeedProfile"].as<int32_t>();
    st_ControlData.s32_V2XMode = st_Config["V2X"].as<int32_t>();

    st_ControlData.f32_TargetSpeed_kph = st_Config["TargetSpeed"].as<float32_t>();
    st_ControlData.f32_TargetSpeed_m_s = st_ControlData.f32_TargetSpeed_kph / 3.6;
    st_ControlData.st_Stanley.f32_lowk = st_Config["lowk"].as<float32_t>();
    st_ControlData.st_Stanley.f32_highk = st_Config["highk"].as<float32_t>();
    st_ControlData.st_Stanley.f32_ke = st_Config["ke"].as<float32_t>();

    st_ControlData.st_PID_Throttle.f32_KP = st_Config["P"].as<float32_t>();
    st_ControlData.st_PID_Throttle.f32_KI = st_Config["I"].as<float32_t>();
    st_ControlData.st_PID_Throttle.f32_KD = st_Config["D"].as<float32_t>();

    st_ControlData.st_PID_ACC.f32_KP = st_Config["ACC_P"].as<float32_t>();
    st_ControlData.st_PID_ACC.f32_KI = st_Config["ACC_I"].as<float32_t>();
    st_ControlData.st_PID_ACC.f32_KD = st_Config["ACC_D"].as<float32_t>();

    st_ControlData.st_Param.f32_MAX_SPEED_kph = st_Config["MaxSpeed"].as<float32_t>();
    st_ControlData.st_Param.f32_MIN_SPEED_kph = st_Config["MinSpeed"].as<float32_t>();
    st_ControlData.st_Param.f32_MAX_DSTEER_rad_s = st_Config["MaxDSteer"].as<float32_t>();

    st_ControlData.st_Param.f32_MAX_SPEED_m_s = st_ControlData.st_Param.f32_MAX_SPEED_kph / 3.6;
    st_ControlData.st_Param.f32_MIN_SPEED_m_s = st_ControlData.st_Param.f32_MIN_SPEED_kph / 3.6;

    st_ControlData.st_Param.u32_NX = st_Config["State"].as<uint32_t>();
    st_ControlData.st_Param.u32_NU = st_Config["Input"].as<uint32_t>();
    st_ControlData.st_Param.u32_T = st_Config["Horizon"].as<uint32_t>();
    st_ControlData.st_Param.f32_DT = st_Config["RealTime"].as<float32_t>();
    st_ControlData.st_Param.f32_Sample_DT = st_Config["SamplingTime"].as<float32_t>();

    st_ControlData.st_PID_LKAS.f32_KP = st_Config["LKAS_P"].as<float32_t>();
    st_ControlData.st_LKAS.f32_MAX_DSTEER_rad_s = st_Config["LKAS_MAXDsteer"].as<float32_t>();

    YAML::Node st_Configs = st_Config["LonQ"];
    st_ControlData.st_LongMPC.arf32_Q[0] = st_Configs[0].as<float32_t>();
    st_ControlData.st_LongMPC.arf32_Q[1] = st_Configs[1].as<float32_t>();
    st_ControlData.st_LongMPC.f32_R = st_Config["LonR"].as<float32_t>();
    st_ControlData.st_LongMPC.f32_Rd = st_Config["LonRd"].as<float32_t>();

    YAML::Node st_Configs2 = st_Config["LatQ"];
    st_ControlData.st_LatMPC.arf32_Q[0] = st_Configs2[0].as<float32_t>();
    st_ControlData.st_LatMPC.arf32_Q[1] = st_Configs2[1].as<float32_t>();
    st_ControlData.st_LatMPC.arf32_Q[2] = st_Configs2[2].as<float32_t>();
    st_ControlData.st_LatMPC.arf32_Q[3] = st_Configs2[3].as<float32_t>();

    st_ControlData.st_LatMPC.f32_R = st_Config["LatR"].as<float32_t>();
    st_ControlData.st_LatMPC.f32_Rd = st_Config["LatRd"].as<float32_t>();
}

void InitCAN()
{
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");

    st_CANData.u32_TimeOut = 1;

    // ID
    st_CANData.ID_0x156 = 0x156;
    st_CANData.ID_0x157 = 0x157;
    st_CANData.ID_0x710 = 0x710;
    st_CANData.ID_0x711 = 0x711;
    st_CANData.ID_0x712 = 0x712;
    st_CANData.ID_0x713 = 0x713;

    // 0x710
    st_CANData.u8_EPS_En_Status = 0;
    st_CANData.u8_EPS_Control_Board_Status = 0;
    st_CANData.u8_EPS_Control_Status = 0;
    st_CANData.u8_EPS_USER_CAN_ERR = 0;
    st_CANData.u8_EPS_ERR = 0;
    st_CANData.u8_EPS_Veh_CAN_ERR = 0;
    st_CANData.u8_EPS_SAS_ERR = 0;
    st_CANData.u8_Override_Ignore_Status = 0;
    st_CANData.u8_Override_Status = 0;
    st_CANData.f32_StrAng = 0;
    st_CANData.f32_Str_Drv_Tq = 0;
    st_CANData.f32_Str_Out_Tq = 0;
    st_CANData.u8_EPS_Alive_Cnt = 0;

    // 0x711
    st_CANData.u8_ACC_En_Status = 0;
    st_CANData.u8_ACC_Control_Board_Status = 0;
    st_CANData.u8_ACC_Control_Status = 0;
    st_CANData.u8_ACC_USER_CAN_ERR = 0;
    st_CANData.u8_ACC_Veh_ERR = 0;
    st_CANData.u8_ACC_ERR = 0;
    st_CANData.u8_VS = 0;
    st_CANData.u8_Turn_Left_En = 0;
    st_CANData.u8_Hazard_En = 0;
    st_CANData.u8_Turn_Right_En = 1;
    st_CANData.u8_G_SEL_DISP = 0;
    st_CANData.f32_LONG_ACCEL = 0;
    st_CANData.u8_ACC_Alive_Cnt = 0;

    // 0x712
    st_CANData.f32_WHEEL_SPD_FL = 0;
    st_CANData.f32_WHEEL_SPD_FR = 0;
    st_CANData.f32_WHEEL_SPD_RL = 0;
    st_CANData.f32_WHEEL_SPD_RR = 0;

    // 0x713
    st_CANData.f32_LAT_ACCEL = 0;
    st_CANData.f32_Long_ACCEL = 0;
    st_CANData.f32_YAW_RATE = 0;
    st_CANData.f32_BRK_CYLINDER = 0;

    // 0x156
    st_CANData.u8_EPS_En = st_Config["EPS"].as<uint8_t>();
    st_CANData.u8_ACC_En = st_Config["EPS"].as<uint8_t>();
    st_CANData.u8_EPS_Override_ignore = 1;
    st_CANData.u8_EPS_Speed = 250;
    st_CANData.u8_AEB_En = 0;
    st_CANData.u8_Hazard = 0;
    st_CANData.u8_Turn_sig_right = 0;
    st_CANData.u8_Turn_sig_left = 0;
    st_CANData.u8_AEB_decel_value = 0; // 0 ~ 100
    st_CANData.u8_Alive_Cnt = 0;

    // 0x157, can_control steering, velocity
    st_CANData.s16_EPS_Cmd = 0;
    st_CANData.s16_ACC_Cmd = 0;

    // 0x124 Receive signal
    st_CANData.u8_SIG_GO = 0;
    st_CANData.u8_SIG_STOP = 0;
    st_CANData.u8_SIG_PIT_STOP = 0;
    st_CANData.u8_SIG_SLOW_ON = 0;
    st_CANData.u8_SIG_SLOW_OFF = 0;

    // 0x125 Receive PIT STOP ZONE
    st_CANData.f32_PITZONE_LAT = 0;
    st_CANData.f32_PITZONE_LONG = 0;

    // 0x126 Send REPAIR
    st_CANData.u8_SIG_REPAIR = 0;

    // control
    st_CANData.f32_TargetSpeed = -3;   // -3 ~ 1
    st_CANData.f32_TargetSteering = 0; // -500 ~ 500
}

void InitLidar()
{
    int32_t s32_Num;

    float32_t f32_X, f32_Y, f32_Z;

    YAML::Node st_Config = YAML::LoadFile("LidarParam.yaml");
    std::string s_Line;
    std::string s_VehiclePointCloudAdress = "./src/Integration/VehiclePointCloud/";
    std::ifstream st_Ioniq5File(s_VehiclePointCloudAdress + "ioniq5.txt");

    st_LidarParam.f32_MaxNearObjectDistance = st_Config["MaxNearObjectDistance"].as<float32_t>();
    st_LidarParam.f32_MaxNearNeighborDistance = st_Config["MaxNearNeighborDistance"].as<float32_t>();
    st_LidarParam.f32_MaxNewObjectDistance = st_Config["MaxNewObjectDistance"].as<float32_t>();

    st_LidarParam.u32_MaxEraseCnt = st_Config["MaxEraseCnt"].as<uint32_t>();

    st_LidarParam.f32_InitialP0 = st_Config["InitialP0"].as<float32_t>();
    st_LidarParam.f32_InitialP5 = st_Config["InitialP5"].as<float32_t>();
    st_LidarParam.f32_InitialP10 = st_Config["InitialP10"].as<float32_t>();
    st_LidarParam.f32_InitialP15 = st_Config["InitialP15"].as<float32_t>();

    st_LidarParam.f32_InitialQ0 = st_Config["InitialQ0"].as<float32_t>();
    st_LidarParam.f32_InitialQ5 = st_Config["InitialQ5"].as<float32_t>();
    st_LidarParam.f32_InitialQ10 = st_Config["InitialQ10"].as<float32_t>();
    st_LidarParam.f32_InitialQ15 = st_Config["InitialQ15"].as<float32_t>();

    st_LidarParam.f32_InitialR0 = st_Config["InitialR0"].as<float32_t>();
    st_LidarParam.f32_InitialR5 = st_Config["InitialR5"].as<float32_t>();
    st_LidarParam.f32_InitialR10 = st_Config["InitialR10"].as<float32_t>();
    st_LidarParam.f32_InitialR15 = st_Config["InitialR15"].as<float32_t>();

    st_LidarParam.f32_MaxObjectDistance = st_Config["MaxObjectDistance"].as<float32_t>();
    st_LidarParam.f32_MaxObjectVelocity_kph = st_Config["MaxObjectVelocity_kph"].as<float32_t>();

    st_LidarParam.f32_MinIoU = st_Config["MinIoU"].as<float32_t>();

    st_LidarParam.f32_MaxStaticVelocity_kph = st_Config["MaxStaticVelocity_kph"].as<float32_t>();

    s32_Num = 0;
    while (std::getline(st_Ioniq5File, s_Line))
    {
        std::istringstream st_Stream(s_Line);
        st_Stream >> f32_X >> f32_Y >> f32_Z;
        st_LidarParam.st_VehiclePointCloud.arf32_X[s32_Num] = f32_X;
        st_LidarParam.st_VehiclePointCloud.arf32_Y[s32_Num] = f32_Y;
        st_LidarParam.st_VehiclePointCloud.arf32_Z[s32_Num] = f32_Z;
        s32_Num += 1;
    }
    st_LidarParam.st_VehiclePointCloud.s32_PointNum = s32_Num;
    st_Ioniq5File.close();
    st_LidarParam.st_VehiclePointCloud.f32_X = 0;
    st_LidarParam.st_VehiclePointCloud.f32_Y = 0;
    st_LidarParam.st_VehiclePointCloud.f32_Z = 0;
    CUDA_BUILD_KDTREE(&st_LidarParam.st_VehiclePointCloud);

    st_LidarParam.s32_MinClusterPointNum = st_Config["MinClusterPointNum"].as<int32_t>();
    st_LidarParam.f32_MaxClusterLength = st_Config["MaxClusterLength"].as<float32_t>();
    st_LidarParam.f32_MinClusterSize = st_Config["MinClusterSize"].as<float32_t>();

    st_LidarParam.f32_MapDistanceOffset = st_Config["MapDistanceOffset"].as<float32_t>();
}

void CreateFile()
{
    char filename[50] = {0};
    string combined_string;
    const char *combined_char_ptr;
    CreateBagFileName(filename);
    combined_string = s_LoggingPath + "/" + filename;
    combined_char_ptr = combined_string.c_str();

    st_FileInfo.st_FileName = combined_string;
    st_FileInfo.pFile = fopen64(combined_char_ptr, "ab");

    if (st_FileInfo.pFile == NULL)
    {
        exit(1); // 혹은 적절한 오류 처리를 수행하고 프로그램을 종료
    }

    fclose(st_FileInfo.pFile);
}

void CopyData(SENSOR_DATA_t *pst_Data)
{
    pthread_mutex_lock(&st_SensorData.st_MutexLIDAR);
    memcpy(&(pst_Data->st_RawLIDAR), &st_SensorData.st_RawLIDAR, sizeof(RAW_LIDAR_DATA_t));
    pthread_mutex_unlock(&st_SensorData.st_MutexLIDAR);

    pthread_mutex_lock(&st_SensorData.st_MutexCamera);
    memcpy(&(pst_Data->st_RawCamera), &st_SensorData.st_RawCamera, sizeof(RAW_CAMERA_DATA_t));
    pthread_mutex_unlock(&st_SensorData.st_MutexCamera);

    pthread_mutex_lock(&st_SensorData.st_MutexGPSINS);
    memcpy(&(pst_Data->st_RawGPSINS), &st_SensorData.st_RawGPSINS, sizeof(RAW_GPSINS_DATA_t));
    pthread_mutex_unlock(&st_SensorData.st_MutexGPSINS);

    pthread_mutex_lock(&st_SensorData.st_MutexIMU);
    memcpy(&(pst_Data->st_RawIMU), &st_SensorData.st_RawIMU, sizeof(RAW_IMU_DATA_t));
    pthread_mutex_unlock(&st_SensorData.st_MutexIMU);

    pthread_mutex_lock(&st_SensorData.st_MutexCAN);
    memcpy(&(pst_Data->st_RawCAN), &st_SensorData.st_RawCAN, sizeof(RAW_CAN_DATA_t));
    pthread_mutex_unlock(&st_SensorData.st_MutexCAN);
}

void LoggingFileWrite()
{

    char filename[50] = {0};
    uint64_t u64_AllFrameSize = 0;
    string combined_string;
    const char *combined_char_ptr;
    // printf("logging start\n");

    if (s32_FileFrameCnt >= 1000)
    {
        s32_FileNum += 1;
        CreateFile();
        s32_FileFrameCnt = 0;
    }

    u64_AllFrameSize = sizeof(st_CurrentData.st_RawLIDAR) +
                       sizeof(st_CurrentData.st_RawCamera) +
                       sizeof(st_CurrentData.st_RawGPSINS) +
                       sizeof(st_CurrentData.st_RawIMU) +
                       sizeof(st_CurrentData.st_RawCAN);

    st_FileInfo.pFile = fopen64(st_FileInfo.st_FileName.c_str(), "ab");
    fwrite(&u64_AllFrameSize, sizeof(uint64_t), 1, st_FileInfo.pFile);

    fwrite(&st_CurrentData.st_RawLIDAR, sizeof(char), sizeof(st_CurrentData.st_RawLIDAR), st_FileInfo.pFile);

    fwrite(&st_CurrentData.st_RawCamera, sizeof(char), sizeof(st_CurrentData.st_RawCamera), st_FileInfo.pFile);

    fwrite(&st_CurrentData.st_RawGPSINS, sizeof(char), sizeof(st_CurrentData.st_RawGPSINS), st_FileInfo.pFile);

    fwrite(&st_CurrentData.st_RawIMU, sizeof(char), sizeof(st_CurrentData.st_RawIMU), st_FileInfo.pFile);

    fwrite(&st_CurrentData.st_RawCAN, sizeof(char), sizeof(st_CurrentData.st_RawCAN), st_FileInfo.pFile);

    // fwrite(&st_CurrentData.st_RawCamera.s32_CameraHeader, sizeof(int), 1, st_FileInfo.pFile);
    // fwrite(&st_CurrentData.st_RawCamera.s32_Num, sizeof(int), 1, st_FileInfo.pFile);
    // fwrite(&st_CurrentData.st_RawCamera, sizeof(char), sizeof(st_CurrentData.st_RawCamera), st_FileInfo.pFile);

    s32_FileFrameCnt += 1;
    fclose(st_FileInfo.pFile);
}

void LoggingFirstRead()
{
    if (s32_SelectFileNum >= 0)
    {
        s32_CurrentFileNum = s32_SelectFileNum;
        string combined_string = s_LoggingPath + "/" + st_LoggingFileList[s32_CurrentFileNum];
        const char *combined_char_ptr = combined_string.c_str();
        memset(&st_FileInfo, 0, sizeof(FILE_INFO_t));

        st_FileInfo.pFile = fopen64(combined_char_ptr, "rb");
        if (st_FileInfo.pFile == NULL)
        {
            printf("Logging First Read File Open Failed\n");
        }
        uint64_t u64_CurrentFrameSize = 0;
        u64_AllFilePointer = 0;

        fseeko64(st_FileInfo.pFile, 0, SEEK_END);
        u64_AllFileSize = ftello64(st_FileInfo.pFile);
        fseeko64(st_FileInfo.pFile, 0, SEEK_SET);

        while (u64_AllFilePointer < u64_AllFileSize)
        {
            fread(&u64_CurrentFrameSize, sizeof(uint64_t), 1, st_FileInfo.pFile);
            u64_AllFilePointer += 8;
            st_FileInfo.ars64_Frame[st_FileInfo.s32_CntFrame++] = u64_AllFilePointer;
            u64_AllFilePointer += u64_CurrentFrameSize;
            fseeko64(st_FileInfo.pFile, u64_CurrentFrameSize, SEEK_CUR);
        }

        if (u64_AllFilePointer > u64_AllFileSize)
        {
            st_FileInfo.s32_CntFrame = max(st_FileInfo.s32_CntFrame - 10, 0);
        }

        u64_AllFilePointer = 0;
        s32_FileFrameCnt = 0;
    }
}

void LoggingFileRead(SENSOR_DATA_t *pst_Data)
{
    int32_t s32_TempFrameCnt = 0;
    float32_t f32_speed = 0;
    if (s32_CurrentFileNum >= 0)
    {

        string combined_string = s_LoggingPath + "/" + st_LoggingFileList[s32_CurrentFileNum];
        const char *combined_char_ptr = combined_string.c_str();
        st_FileInfo.pFile = fopen64(combined_char_ptr, "rb");
        uint64_t u64_FrameSize = 0;

        if (st_FileInfo.pFile == NULL)
        {
            printf("FIle Open Failed %p\n", (void *)st_FileInfo.pFile);
        }
        // printf("file path : %s\n", combined_char_ptr);

        s32_TempFrameCnt = s32_FileFrameCnt;
        if (s32_TempFrameCnt < st_FileInfo.s32_CntFrame - 1)
        {
            fseeko64(st_FileInfo.pFile, st_FileInfo.ars64_Frame[s32_TempFrameCnt], SEEK_SET);

            fread(&st_CurrentData.st_RawLIDAR, sizeof(char), sizeof(st_CurrentData.st_RawLIDAR), st_FileInfo.pFile);

            fread(&st_CurrentData.st_RawCamera, sizeof(char), sizeof(st_CurrentData.st_RawCamera), st_FileInfo.pFile);

            fread(&st_CurrentData.st_RawGPSINS, sizeof(char), sizeof(st_CurrentData.st_RawGPSINS), st_FileInfo.pFile);

            fread(&st_CurrentData.st_RawIMU, sizeof(char), sizeof(st_CurrentData.st_RawIMU), st_FileInfo.pFile);

            fread(&st_CurrentData.st_RawCAN, sizeof(char), sizeof(st_CurrentData.st_RawCAN), st_FileInfo.pFile);

            u64_AllFilePointer += u64_FrameSize + 8;

            if (st_ViewerParam.s32_State & c_STATE_MODE_STOP)
            {
                ;
            }
            else if (st_ViewerParam.s32_State & c_STATE_MODE_PLAY)
            {

                s32_TempFrameCnt += 1;
                st_ViewerParam.s32_FrameBarX = s32_TempFrameCnt;

                if (s32_TempFrameCnt == st_FileInfo.s32_CntFrame)
                {
                    st_ViewerParam.s32_State |= c_STATE_MODE_STOP;
                    s32_TempFrameCnt = 0;
                    st_FileInfo.s32_CntFrame = 0;
                }
            }
        }
        else
        {
            s32_TempFrameCnt = st_FileInfo.s32_CntFrame - 1;
        }

        s32_FileFrameCnt = s32_TempFrameCnt;
        fclose(st_FileInfo.pFile);
    }
}

void *TemporalThread(void *p_Arg)
{

    uint64_t u64_StartTime_ms = 0;

    while (b_Running)
    {
        pthread_mutex_lock(&st_DataTemporal);
        pthread_cond_wait(&st_DataTemporalcond, &st_DataTemporal);
        CopyData(&st_CurrentData);

        if (st_ViewerParam.s32_CurrentMode == c_VIEWER_SIMUL_MODE)
        {
            if ((st_ViewerParam.s32_State & c_STATE_MODE_PLAY) || (st_ViewerParam.s32_State & c_STATE_MODE_STOP))
            {
                LoggingFileRead(&st_CurrentData);
            }
        }

        if (st_ViewerParam.s32_State & c_STATE_MODE_LOGGING)
        {
            LoggingFileWrite();
        }

        //----------------CAN PARSING--------------
        memcpy(&st_CANData, st_CurrentData.st_RawCAN.arc_Buffer, sizeof(CAN_DATA_t));
        //----------------CAN PARSING--------------

        // u64_StartTime_ms = getMillisecond();
        // CameraProcessing(&st_CurrentData.st_RawCamera, &st_CameraData, &st_PlanningData, &st_DeadReckoningData);
        // st_LogicHz.u64_CameraHz = getMillisecond() - u64_StartTime_ms;

        // // 추후 별도 Thread로 동작 예정
        u64_StartTime_ms = getMillisecond();
        DeadReckoningProcessing(&st_CurrentData.st_RawGPSINS, &st_HMCLidarData, &st_CurrentData.st_RawIMU, &st_CurrentData.st_RawCAN, &st_DeadReckoningData);

        st_LogicHz.u64_DeadReckoningHz = getMillisecond() - u64_StartTime_ms;

        u64_StartTime_ms = getMillisecond();
        HMC_LIDARProcessing(&st_CurrentData.st_RawLIDAR, &st_HMCLidarData, &st_PlanningData, &st_DeadReckoningData, &st_OmniPoint, &st_ICPMap);
        st_LogicHz.u64_LiDARHz = getMillisecond() - u64_StartTime_ms;

        // Plannig
        u64_StartTime_ms = getMicrosecond();
        if (s32_RoundName == c_ROUND_TEST)
        {
            TEST_PlanningProcessing(&st_HMCLidarData,
                                    &st_DeadReckoningData,
                                    &st_CameraData,
                                    &st_PlanningData,
                                    &st_CANData);
        }
        else if (s32_RoundName == c_ROUND_TESTDRIVE1)
        {
            TESTDRIVE1_PlanningProcessing(&st_HMCLidarData,
                                          &st_DeadReckoningData,
                                          &st_CameraData,
                                          &st_PlanningData,
                                          &st_CANData);
        }
        else if (s32_RoundName == c_ROUND_TESTDRIVE2)
        {
            TESTDRIVE2_PlanningProcessing(&st_HMCLidarData,
                                          &st_DeadReckoningData,
                                          &st_CameraData,
                                          &st_PlanningData,
                                          &st_CANData);
        }
        else if (s32_RoundName == c_ROUND_PRELIMINARY1)
        {
        }
        else if (s32_RoundName == c_ROUND_PRELIMINARY2)
        {
            PRE2_PlanningProcessing(&st_HMCLidarData,
                                    &st_DeadReckoningData,
                                    &st_CameraData,
                                    &st_PlanningData,
                                    &st_CANData);
        }
        else if (s32_RoundName == c_ROUND_FINAL)
        {
            FINAL_PlanningProcessing(&st_HMCLidarData,
                                     &st_DeadReckoningData,
                                     &st_CameraData,
                                     &st_PlanningData,
                                     &st_CANData);
        }
        st_LogicHz.u64_PlanningHz = getMicrosecond() - u64_StartTime_ms;

        // Control
        u64_StartTime_ms = getMillisecond();

        ControlProcessing(&st_PlanningData,
                          &st_DeadReckoningData,
                          &st_ControlData,
                          &st_CANData,
                          &st_CameraData);
        st_LogicHz.u64_ControlHz = getMillisecond() - u64_StartTime_ms;

        pthread_mutex_unlock(&st_DataTemporal);

        pthread_mutex_lock(&st_CopyMutex);
        memcpy(&st_CameraDataTemporal, &st_CameraData, sizeof(CAMERA_DATA_t));
        // memcpy(&st_LidarDataTemporal, &st_LidarData, sizeof(LIDAR_DATA_t));
        memcpy(&st_HMCLidarDataTemporal, &st_HMCLidarData, sizeof(HMC_LIDAR_DATA_t));
        memcpy(&st_DeadReckoningDataTemporal, &st_DeadReckoningData, sizeof(DEAD_RECKONING_DATA_t));
        memcpy(&st_PlanningDataTemporal, &st_PlanningData, sizeof(PLANNING_DATA_t));
        memcpy(&st_ControlDataTemporal, &st_ControlData, sizeof(CONTROL_DATA_t));
        memcpy(&st_CANDataTemporal, &st_CANData, sizeof(CAN_DATA_t));
        pthread_mutex_unlock(&st_CopyMutex);
    }
}

int main(int argc, char **argv)
{
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");
    s_LIDARName = st_Config["LIDAR_Name"].as<std::string>();
    s32_RoundName = st_Config["Round"].as<int32_t>();
    s32_CanMode = st_Config["CAN_Mode"].as<int32_t>();

    pthread_t pth_TemporalThread;
    pthread_t pth_OpenGLThread;
    pthread_t pth_LiDARThread;
    pthread_t pth_CameraThread;
    pthread_t pth_GpsThread;
    pthread_t pth_ImuTHread;
    pthread_t pth_CanThread;
    pthread_t pth_LiDARImuThread;
    pthread_t pth_LiDARStatusThread;

    ros::init(argc, argv, "Integration");
    ros::NodeHandle st_NodeHandle;

    // Load Global Path & Make Spline Path
    InitGlobal();
    InitSerial();

    LoadPath(&st_PlanningData);
    CalcPathYaw(&st_PlanningData);
    // LoadParam(&st_CameraData);
    // LoadMappingParam(&st_CameraData);

    InitMutex();
    InitPlanning();
    InitViewer();
    InitControl();
    InitCAN();
    InitLidar();
    glutInit(&argc, argv);

    if (pthread_create(&pth_GpsThread, NULL, GPSParserWrapper, (void *)&st_NodeHandle) != 0)
    { // 2번 thread 생성
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&pth_OpenGLThread, NULL, Run_OpenGL, NULL) != 0)
    { // 2번 thread 생성
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&pth_CanThread, NULL, CANParserWrapper, (void *)&st_NodeHandle) != 0)
    { // 2번 thread 생성
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&pth_LiDARThread, NULL, LIDARParserWrapper, (void *)&st_NodeHandle) != 0)
    { // 2번 thread 생성
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&pth_ImuTHread, NULL, IMUParserWrapper, (void *)&st_NodeHandle) != 0)
    { // 2번 thread 생성
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    // if (pthread_create(&pth_CameraThread, NULL, CameraParserWrapper, (void *)&st_NodeHandle) != 0)
    // { // 2번 thread 생성
    //     fprintf(stderr, "thread create error\n");
    //     exit(1);
    // }

    if (pthread_create(&pth_TemporalThread, NULL, TemporalThread, NULL) != 0)
    { // 1번 thread 생성
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&pth_LiDARStatusThread, NULL, LiDARStatusWrapper, NULL) != 0)
    { // 1번 thread 생성
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    // if(s_LIDARName == "OS2")
    // {
    //     if(pthread_create(&pth_LiDARImuThread, NULL, OusterIMUParserWrapper, (void*)&st_NodeHandle) != 0) { //2번 thread 생성
    //         fprintf(stderr, "thread create error\n");
    //         exit(1);
    //     }
    // }

    while (b_Running)
    {
        usleep(100);
    }

    return 0;
}