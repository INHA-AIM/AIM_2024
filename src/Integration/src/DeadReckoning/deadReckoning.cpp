#include "deadReckoning.h"

extern SENSOR_DATA_t st_SensorData;
extern CAN_DATA_t st_CANData;
extern PLANNING_DATA_t st_PlanningData;
extern LOGIC_HZ_t st_LogicHz;
static bool IMU_flag = false;
static bool GPS_flag = false;
static bool initial_flag = false;
static bool flag1 = false;
static bool flag2 = false;

static int cnt = 0;
bool OdometricOneFlag = false;
bool DeadReckoningFlag = false;
float32_t f32_PredictX = 0;
float32_t f32_PredictY = 0;
float32_t f32_EgoCurHeading_rad = 0.0f;
float32_t f32_EgoCurHeading_deg = 0.0f;
float32_t f32_EgoPredictHeading_rad = 0.0f;
float32_t f32_EgoPredictHeading_deg = 0.0f;


uint64_t u64_PrevTimeStamp;

using Eigen::MatrixXd;
using Eigen::VectorXd;


void DeadReckoningProcessing(RAW_GPSINS_DATA_t *pst_GPSINSData,
                            HMC_LIDAR_DATA_t *pst_LidarData,
                            RAW_IMU_DATA_t *pst_IMUData, 
                            RAW_CAN_DATA_t *pst_CANData, 
                            DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
{

    memcpy(&pst_DeadReckoningData->st_GPS, pst_GPSINSData->arc_Buffer, sizeof(GPS_DATA_t));
    memcpy(&pst_DeadReckoningData->st_IMU, pst_IMUData->arc_Buffer, sizeof(IMU_DATA_t));
    int32_t s32_CANMode = 0;
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");
    s32_CANMode = st_Config["CAN_Mode"].as<int32_t>();

    if(s32_CANMode == 0 || s32_CANMode == 1)
    {   
        memcpy(&st_CANData, pst_CANData->arc_Buffer, sizeof(CAN_DATA_t));
    }
    
    // Odometry Blocked
    OdometricLocalization(pst_DeadReckoningData, pst_CANData, pst_LidarData);



    // printf("LLA, Yaw Difference: %.9f, %.9f, %.9f, %.9f\n",pst_DeadReckoningData->st_EKF_data.lat - pst_DeadReckoningData->st_GPS.f64_Latitude, 
                                                                // pst_DeadReckoningData->st_EKF_data.lon - pst_DeadReckoningData->st_GPS.f64_Longitude, 
                                                                // pst_DeadReckoningData->st_EKF_data.alt - pst_DeadReckoningData->st_GPS.f64_Altitude);

    // Initialization(pst_DeadReckoningData);
    
    // get_GPSData(pst_GPSINSData, pst_DeadReckoningData);

    // get_IMUData(pst_IMUData, pst_DeadReckoningData);

    // EKF(pst_GPSINSData, pst_DeadReckoningData);

        // printf("LLA , yaw: %f, %f, %f, %f",pst_DeadReckoningData->st_EKF_data.lat, pst_DeadReckoningData->st_EKF_data.lon, pst_DeadReckoningData->st_EKF_data.alt, pst_DeadReckoningData->st_EKF_data.yaw);


    // printf("Call DR %lf %lf %f %f %f\n", pst_DeadReckoningData->st_GPS.f64_Latitude, 
    //                                 pst_DeadReckoningData->st_GPS.f64_Longitude, 
    //                                 pst_DeadReckoningData->st_IMU.f32_AccelX,
    //                                 pst_DeadReckoningData->st_IMU.f32_AccelY,
    //                                 pst_DeadReckoningData->st_IMU.f32_GyroZ);

    // lla2enu(pst_DeadReckoningData->st_GPS.f64_Latitude, 
    //         pst_DeadReckoningData->st_GPS.f64_Longitude,
    //         pst_DeadReckoningData->st_GPS.f64_Altitude, 
    //         f32_E, f32_N, f32_U);


    // printf("%f %f %f\n", f32_E, f32_N, f32_U);

}

float64_t updateYawAngle(float64_t f64_currentYaw, float64_t f64_yawRate, float64_t f64_deltaTime) {
    return f64_currentYaw + (f64_yawRate * f64_deltaTime);
}
void GetEgoVehicleHeading(float64_t &f64_Yaw, float64_t f64_deltaTime)
{
    float64_t f64_initialYaw = f64_Yaw;
    float64_t f64_speed = st_CANData.f32_Speed_m_s;                // 속도 in m/s
    float64_t f64_wheelbase = 2.720;             // 휠베이스 in meters    
             // 시간 간격 in seconds
    float64_t f64_yawRate = (f64_speed / f64_wheelbase) * tan(st_CANData.f32_SteerAngle_rad);
    
    float64_t f64_newYaw = updateYawAngle(f64_initialYaw, f64_yawRate, f64_deltaTime);
    
    float64_t f64_newYawDegrees = f64_newYaw * (180.0 / PI);
    f64_Yaw = f64_newYawDegrees;
    // std::cout <<  f64_newYawDegrees << " degrees)" << std::endl;
}



                                                                      
bool isKalmanFilterInitialized = false;

const int state_dim = 3; // [x, y, yaw]
VectorXd StateVector_x(state_dim);  // 상태 벡터
VectorXd MeasurementVector_z(state_dim);  // 관측 벡터
MatrixXd PredictedCovariance_P(state_dim, state_dim); // 예측한 공분산 행렬
MatrixXd StateTransitionMatrix_A(state_dim, state_dim); // 상태 전이 행렬
MatrixXd PrNoiseCovMatrix_Q(state_dim, state_dim);  // 프로세스 노이즈 공분산 행렬    
MatrixXd ObservationMatrix_H(3, state_dim); // 관측 행렬
MatrixXd ObsNoiseCovMatrix_R(3, 3);  // 관측 노이즈 공분산 행렬


// 예측 단계
void Predict(float64_t f64_DeltaTime, float64_t f64_Range, float32_t &f32_PredictYaw_deg, float32_t &f32_PredictX, float32_t &f32_PredictY)
{
    float32_t f32_NewHeading = f32_PredictYaw_deg + st_CANData.f32_YAW_RATE * f64_DeltaTime;
    float32_t f32_EquationYaw = 1 / f32_PredictYaw_deg;

    StateTransitionMatrix_A <<  1, 0,  f32_EquationYaw * f64_Range  * (sin(deg2rad(f32_NewHeading)) - sin(deg2rad(f32_PredictYaw_deg))),
                                0, 1,  -f32_EquationYaw * f64_Range * (cos(deg2rad(f32_NewHeading)) - cos(deg2rad(f32_PredictYaw_deg))),
                                0, 0,  f32_EquationYaw * (f32_PredictYaw_deg + st_CANData.f32_YAW_RATE * f64_DeltaTime);

    StateVector_x = StateTransitionMatrix_A * StateVector_x;
    PredictedCovariance_P = StateTransitionMatrix_A * PredictedCovariance_P * StateTransitionMatrix_A.transpose() + PrNoiseCovMatrix_Q;
}

// 갱신 단계
void Update(VectorXd MeasurementVector)
{
    MatrixXd ErrorCovMatrix = ObservationMatrix_H * PredictedCovariance_P * ObservationMatrix_H.transpose() + ObsNoiseCovMatrix_R;
    MatrixXd KalmanGainMatrix = PredictedCovariance_P * ObservationMatrix_H.transpose() * ErrorCovMatrix.inverse();
    StateVector_x = StateVector_x + KalmanGainMatrix * (MeasurementVector - ObservationMatrix_H * StateVector_x);

    MatrixXd I = MatrixXd::Identity(state_dim, state_dim);
    PredictedCovariance_P = (I - KalmanGainMatrix * ObservationMatrix_H) * PredictedCovariance_P;
}

void OdometricLocalization(DEAD_RECKONING_DATA_t *pst_DeadReckoningData, RAW_CAN_DATA_t *pst_CANData, HMC_LIDAR_DATA_t *pst_LidarData)
{
    uint64_t u64_CurTimeStamp = pst_CANData->u64_Timestamp;

    float64_t f64_Speed_ms = st_CANData.f32_Speed_m_s;
    float64_t f64_Range = 0.0;
    float64_t f64_InitCurHeading_rad = 0.0f;
    float64_t f64_InitCurHeading_deg = 0.0f;
    float64_t f64_GTHeading_deg = 0.0f;
    float64_t f64_CurX = 0.0f;
    float64_t f64_CurY = 0.0f;

    float64_t f64_TimeDelay = fabsf(u64_CurTimeStamp - u64_PrevTimeStamp) * 0.001;

    if(f64_TimeDelay > 0.1 || f64_TimeDelay < 0.01)
    {
        f64_TimeDelay = 0.05f;
    }


    float32_t f32_ErrorDistance = 0;
    float32_t f32_AnglerVelcity_rad = deg2rad(st_CANData.f32_YAW_RATE);
    float32_t f32_AnglerVelcity_deg = st_CANData.f32_YAW_RATE;
    float32_t f32_InitialX = -99999.0f;
    float32_t f32_InitialY = -99999.0f;
    float32_t f32_InitialZ = -99999.0f;

    float32_t f32_Coefficient_Q = 0.8f;
    float32_t f32_Coefficient_R = 0.1f;


    PredictedCovariance_P = MatrixXd::Identity(state_dim, state_dim);
    PrNoiseCovMatrix_Q = MatrixXd::Identity(state_dim, state_dim); // 프로세스 잡음 공분산 행렬 설정
    PrNoiseCovMatrix_Q << f32_Coefficient_Q, 0, 0,
                          0, f32_Coefficient_Q, 0,
                          0, 0, f32_Coefficient_Q;  
    ObservationMatrix_H << 1, 0, 0,
                           0, 1, 0,
                           0, 0, 1;
    ObsNoiseCovMatrix_R = MatrixXd::Identity(state_dim, state_dim); // 관측 잡음 공분산 행렬 설정

    ObsNoiseCovMatrix_R << f32_Coefficient_R, 0, 0,
                           0, f32_Coefficient_R, 0,
                           0, 0, f32_Coefficient_R;

    int32_t s32_GPSMode = 0;
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");


    VEHICLE_ODOMETRY_t *pst_Vehicle_Odometry = &pst_DeadReckoningData->st_Vehicle_Odometry;

    s32_GPSMode = st_Config["GPS_Mode"].as<int32_t>();
    lla2enu(pst_DeadReckoningData->st_GPS.f64_Latitude,
            pst_DeadReckoningData->st_GPS.f64_Longitude,
            0,
            f32_InitialX, f32_InitialY, f32_InitialZ);
    // printf("(after)px : %f py : %f gtx : %f gty : %f\n",pst_LidarData->f32_PredictX, pst_LidarData->f32_PredictY, f32_InitialX, f32_InitialY);

    if(s32_GPSMode == c_PARSING_MORAI)
    {
        f64_InitCurHeading_rad = deg2rad(pst_DeadReckoningData->st_IMU.f32_Yaw_deg);
        f64_InitCurHeading_deg = rad2deg(f64_InitCurHeading_rad);
        f64_GTHeading_deg = pst_DeadReckoningData->st_IMU.f32_Yaw_deg;

    }
    if(s32_GPSMode == c_PARSING_AVANTE)
    {
        f64_InitCurHeading_rad = deg2rad(-pst_DeadReckoningData->st_IMU.f32_Yaw_deg + 90.f);
        f64_InitCurHeading_deg = rad2deg(f64_InitCurHeading_rad);
        f64_GTHeading_deg = -pst_DeadReckoningData->st_IMU.f32_Yaw_deg + 90.f;
    }
    if (fabs(f32_AnglerVelcity_rad) > 1e-9) 
    {
        f64_Range = f64_Speed_ms / f32_AnglerVelcity_rad; 
        pst_Vehicle_Odometry->u8_StateFlag[pst_Vehicle_Odometry->s32_TrajectoryCount] = 0;

    }
    else
    {
        f64_Range =  f64_Speed_ms * f64_TimeDelay; 
        pst_Vehicle_Odometry->u8_StateFlag[pst_Vehicle_Odometry->s32_TrajectoryCount] = 0;

    }
    if(f32_InitialX > -2000.0f)
    {
        if(!OdometricOneFlag)
        {

            f32_PredictX = f32_InitialX;
            f32_PredictY = f32_InitialY;
            f32_EgoPredictHeading_deg = f64_InitCurHeading_deg;
            pst_Vehicle_Odometry->f32_PredictX = f32_PredictX;
            pst_Vehicle_Odometry->f32_PredictY = f32_PredictY;
            pst_Vehicle_Odometry->f32_PredictYaw_deg = f32_EgoPredictHeading_deg;
            pst_Vehicle_Odometry->f32_PredictYaw_rad = deg2rad(f32_EgoPredictHeading_deg);
            
            
            OdometricOneFlag = true;
        }
        else
        {
            StateVector_x << f32_PredictX, f32_PredictY, f32_EgoPredictHeading_deg; 
            Predict(f64_TimeDelay, f64_Range, f32_EgoPredictHeading_deg, f32_PredictX, f32_PredictY);
            if(!DeadReckoningFlag)
            {
                MeasurementVector_z << f32_InitialX, f32_InitialY, f64_GTHeading_deg; 
                Update(MeasurementVector_z);
            }
            // printf("predict yaw : %f, gt Yaw : %f\n", StateVector_x(2), f64_GTHeading_deg);
            pst_Vehicle_Odometry->f32_PredictX = StateVector_x(0);
            pst_Vehicle_Odometry->f32_PredictY = StateVector_x(1);
            pst_Vehicle_Odometry->f32_PredictYaw_deg = StateVector_x(2);
            pst_Vehicle_Odometry->f32_PredictYaw_rad = deg2rad(StateVector_x(2));
            f32_PredictX = StateVector_x(0);
            f32_PredictY = StateVector_x(1);
            f32_EgoPredictHeading_deg = StateVector_x(2);
            f32_EgoPredictHeading_rad = deg2rad(StateVector_x(2));

        }
    }
    // else
    // {
    //     OdometricOneFlag = false;
    // }
    u64_PrevTimeStamp = u64_CurTimeStamp;
}
void Initialization(DEAD_RECKONING_DATA_t *pst_DeadReckoningData){


    if(initial_flag ==  false){
        InitialsettingEKFData(pst_DeadReckoningData);
        InitialsettingEKFObject(pst_DeadReckoningData);
        initial_flag = true;
    }
}

void EKF(RAW_GPSINS_DATA_t *pst_GPSINSData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData){

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;


    // printf("EKF is running\n");



    if(GPS_flag == true && IMU_flag == true){
        UpdataState(pst_GPSINSData, pst_DeadReckoningData);
        // printf("Update LLA , yaw: %.9f, %.9f, %.9f, %.9f\n",pst_DeadReckoningData->st_EKF_data.lat, pst_DeadReckoningData->st_EKF_data.lon, pst_DeadReckoningData->st_EKF_data.alt, pst_DeadReckoningData->st_EKF_data.yaw);

        INS_ERROR_MODEL(pst_DeadReckoningData);
        
        INS_MECHANIZATION(pst_DeadReckoningData);

        PredictState(pst_DeadReckoningData);
        // printf("Predict LLA , yaw: %.9f, %.9f, %.9f, %.9f\n",pst_DeadReckoningData->st_EKF_data.lat, pst_DeadReckoningData->st_EKF_data.lon, pst_DeadReckoningData->st_EKF_data.alt, pst_DeadReckoningData->st_EKF_data.yaw);

        pst_EKF_data->prev_lat = pst_EKF_data->lat;
        pst_EKF_data->prev_lon = pst_EKF_data->lon;
        pst_EKF_data->prev_alt = pst_EKF_data->alt;

    }

}

void get_GPSData(RAW_GPSINS_DATA_t *pst_GPSINSData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData){
    
    memcpy(&pst_DeadReckoningData->st_GPS, pst_GPSINSData->arc_Buffer, sizeof(GPS_DATA_t));

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;


    
        if (GPS_flag == false) 
        {

            if(pst_DeadReckoningData->st_GPS.f64_Latitude > 1 && pst_DeadReckoningData->st_GPS.f64_Latitude > 1 && pst_DeadReckoningData->st_GPS.f64_Latitude && 1){
                
                pst_EKFObject->st_X(0, 0) = pst_DeadReckoningData->st_GPS.f64_Latitude;
                pst_EKFObject->st_X(0, 1) = pst_DeadReckoningData->st_GPS.f64_Longitude;
                pst_EKFObject->st_X(0, 2) = pst_DeadReckoningData->st_GPS.f64_Altitude; 
                pst_EKFObject->st_X(0, 3) = 0;
                pst_EKFObject->st_X(0, 4) = 0;
                pst_EKFObject->st_X(0, 5) = 0;
    
                pst_EKF_data->gps_lat = pst_DeadReckoningData->st_GPS.f64_Latitude;
                pst_EKF_data->gps_lon = pst_DeadReckoningData->st_GPS.f64_Longitude;
                pst_EKF_data->gps_alt = pst_DeadReckoningData->st_GPS.f64_Altitude;
    
                pst_EKF_data->lat = pst_EKF_data->gps_lat;
                pst_EKF_data->lon = pst_EKF_data->gps_lon;
                pst_EKF_data->alt = pst_EKF_data->gps_alt;
                pst_EKF_data->h = pst_EKF_data->gps_alt;
                pst_EKF_data->dt_gps = 0.05;

                GPS_flag = true;
            }

            
        }

        else{
            pst_EKF_data->GPSTime = pst_GPSINSData->u64_Timestamp * 0.001;

            if (flag2 == false)
            {
                pst_EKF_data->lastGPST = pst_EKF_data->GPSTime;
                flag2 = true;
            }
            else
            {
                pst_EKF_data->dt_gps = (pst_EKF_data->GPSTime - pst_EKF_data->lastGPST);
                pst_EKF_data->lastGPST = (pst_EKF_data->GPSTime);
            }
        }

    pst_EKF_data->lat = pst_DeadReckoningData->st_GPS.f64_Latitude; 
    pst_EKF_data->lon = pst_DeadReckoningData->st_GPS.f64_Longitude; 
    pst_EKF_data->alt = pst_DeadReckoningData->st_GPS.f64_Altitude;

}

void get_IMUData(RAW_IMU_DATA_t *pst_IMUData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData){
    
    memcpy(&pst_DeadReckoningData->st_IMU, pst_IMUData->arc_Buffer, sizeof(IMU_DATA_t));

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;



    pst_EKF_data->la_x = pst_DeadReckoningData->st_IMU.f32_AccelX; 
    pst_EKF_data->la_y = pst_DeadReckoningData->st_IMU.f32_AccelY; 
    pst_EKF_data->la_z = pst_DeadReckoningData->st_IMU.f32_AccelZ;

    pst_EKF_data->q_roll = pst_DeadReckoningData->st_IMU.f32_Roll_deg;
    pst_EKF_data->q_pitch = pst_DeadReckoningData->st_IMU.f32_Pitch_deg;
    pst_EKF_data->q_yaw = pst_DeadReckoningData->st_IMU.f32_Yaw_deg;

    if (IMU_flag == false)
    {
        IMUSetting(pst_DeadReckoningData);
        
    }
    else{
        pst_EKF_data->INSTime = pst_IMUData->u64_Timestamp * 0.001;

        if (flag1 == false)
        {
            pst_EKF_data->lastINST = pst_EKF_data->INSTime;
            flag1 = true;
            pst_EKF_data->dt = 0.02;
        }
    
        else
        {
            pst_EKF_data->dt = (pst_EKF_data->INSTime - pst_EKF_data->lastINST);
            pst_EKF_data->lastINST = pst_EKF_data->INSTime;
    
        }
    }

    pst_EKF_data->av_x = pst_DeadReckoningData->st_IMU.f32_GyroX; 
    pst_EKF_data->av_y = pst_DeadReckoningData->st_IMU.f32_GyroY; 
    pst_EKF_data->av_z = pst_DeadReckoningData->st_IMU.f32_GyroZ;

    pst_EKF_data->qu_x = pst_DeadReckoningData->st_IMU.f32_QuaternionX; 
    pst_EKF_data->qu_y = pst_DeadReckoningData->st_IMU.f32_QuaternionY; 
    pst_EKF_data->qu_z = pst_DeadReckoningData->st_IMU.f32_QuaternionZ;
    pst_EKF_data->qu_w = pst_DeadReckoningData->st_IMU.f32_QuaternionW;

}

void InitialsettingEKFData(DEAD_RECKONING_DATA_t *pst_DeadReckoningData){

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;

    pst_EKF_data->lat = 0.0;
    pst_EKF_data->lon = 0.0;
    pst_EKF_data->alt = 0.0;
    pst_EKF_data->h = 0.0;

    pst_EKF_data->gps_lat = 1.0;
    pst_EKF_data->gps_lon = 1.0;
    pst_EKF_data->gps_alt = 1.0;
    pst_EKF_data->wgs84_f = 1 / 298.257223563;
    pst_EKF_data->wgs84_e2 = 2 * pst_EKF_data->wgs84_f - pow(pst_EKF_data->wgs84_f, 2);                      // wgs84 model
    pst_EKF_data->ecc = 0.0818192;
    pst_EKF_data->R0 = 6378137.0;
    pst_EKF_data->Ome = 7.2921151467e-5;
    pst_EKF_data->gravity = -9.81;
    pst_EKF_data->ecc_2 = pst_EKF_data->ecc * pst_EKF_data->ecc; // earth value

    pst_EKF_data->setting_yaw = 0.0;

    pst_EKF_data->v_e = 0.0;
    pst_EKF_data->v_n = 0.0;
    pst_EKF_data->v_u = 0.0;

    pst_EKF_data->gps_v_e = 0.0;
    pst_EKF_data->gps_v_n = 0.0;
    pst_EKF_data->gps_v_u = 0.0;

    pst_EKF_data->prev_E = 0.0;
    pst_EKF_data->prev_N = 0.0;
    pst_EKF_data->prev_U = 0.0;

    pst_EKF_data->d_E = 0.0; 
    pst_EKF_data->d_N = 0.0;
    pst_EKF_data->d_U = 0.0;

    pst_EKF_data->TAU_A = 100;
    pst_EKF_data->SIG_G_D = 0;
    pst_EKF_data->TAU_G = 50;

    pst_EKF_data->Ref_flag = 0;

}


void InitialsettingEKFObject(DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
{
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;

    pst_EKFObject->st_X.setZero();

    pst_EKFObject->st_dX(0, 0) = 0.000528385112155;
    pst_EKFObject->st_dX(1, 0) = 0.001903292298067;
    pst_EKFObject->st_dX(2, 0) = 0.119029140416519;
    pst_EKFObject->st_dX(3, 0) = 0.001136779904715;
    pst_EKFObject->st_dX(4, 0) = 0.025140765509064;
    pst_EKFObject->st_dX(5, 0) = 0.767850705055676;
    pst_EKFObject->st_dX(6, 0) = 0.000095216205449;
    pst_EKFObject->st_dX(7, 0) = 0.000007047578971;
    pst_EKFObject->st_dX(8, 0) = 0.000064377190190;
    pst_EKFObject->st_dX(9, 0) = 0.000049586430756;
    pst_EKFObject->st_dX(10, 0) = 0.000137711460060;
    pst_EKFObject->st_dX(11, 0) = 0.000341346308273;
    pst_EKFObject->st_dX(12, 0) = 0.000004785728765;
    pst_EKFObject->st_dX(13, 0) = 0.000001781730025;
    pst_EKFObject->st_dX(14, 0) = 0.000000026285999;

    pst_EKFObject->st_dX0 = pst_EKFObject->st_dX;

    pst_EKFObject->st_Q(0, 0) = 100000000;
    pst_EKFObject->st_Q(1, 1) = 100000000;
    pst_EKFObject->st_Q(2, 2) = 10;
    pst_EKFObject->st_Q(3, 3) = 100;
    pst_EKFObject->st_Q(4, 4) = 100;
    pst_EKFObject->st_Q(5, 5) = 1;
    pst_EKFObject->st_Q(6, 6) = 1000000;
    pst_EKFObject->st_Q(7, 7) = 100000000;
    pst_EKFObject->st_Q(8, 8) = 100000000;
    pst_EKFObject->st_Q(9, 9) = 100;
    pst_EKFObject->st_Q(10, 10) = 100;
    pst_EKFObject->st_Q(11, 11) = 100;
    pst_EKFObject->st_Q(12, 12) = 100;
    pst_EKFObject->st_Q(13, 13) = 100;
    pst_EKFObject->st_Q(14, 14) = 100;

    pst_EKFObject->st_Q = pst_EKFObject->st_Q * 10000;

    pst_EKFObject->st_R(0, 0) = 0.0000000000000001; // lla
    pst_EKFObject->st_R(1, 1) = 0.0000000000000001;
    pst_EKFObject->st_R(2, 2) = 0.0000000000000001;
    pst_EKFObject->st_R(3, 3) = 0.01; // v_ned
    pst_EKFObject->st_R(4, 4) = 0.0000001;
    pst_EKFObject->st_R(5, 5) = 0.0000000000001;
    pst_EKFObject->st_R(6, 6) = 0.00000000000001; // euler
    pst_EKFObject->st_R(7, 7) = 0.00000000000001;
    pst_EKFObject->st_R(8, 8) = 0.000001;
    pst_EKFObject->st_R(9, 9) = 1; // l_a
    pst_EKFObject->st_R(10, 10) = 1;
    pst_EKFObject->st_R(11, 11) = 1;
    pst_EKFObject->st_R(12, 12) = 1; // a_v
    pst_EKFObject->st_R(13, 13) = 1;
    pst_EKFObject->st_R(14, 14) = 1;

    int32_t i;
    for (i = 0; i < pst_EKFObject->st_H.cols(); i++)
    {
        pst_EKFObject->st_H(i, i) = 1;
    }

    int32_t j;
    for (j = 0; j < pst_EKFObject->st_P.cols(); j++)
    {
        pst_EKFObject->st_P(j, j) = 0.1;
    }
    pst_EKFObject->st_V_ned.setZero();
}

void PredictState(DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
{
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;
    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;

    Eigen::MatrixXf st_F2 = Eigen::MatrixXf::Zero(15,15);
    Eigen::MatrixXf st_A = Eigen::MatrixXf::Zero(15,15);

    st_F2 = pst_EKF_data->dt * pst_EKFObject->st_F;

    pst_EKFObject->st_A = pst_EKFObject->st_F2.exp();
    pst_EKFObject->st_dX = pst_EKFObject->st_A * pst_EKFObject->st_dX;
    pst_EKFObject->st_P = pst_EKFObject->st_A * pst_EKFObject->st_P * pst_EKFObject->st_A.transpose() + pst_EKFObject->st_Q;
}

Eigen::MatrixXf skew(float32_t x, float32_t y, float32_t z)
{
    Eigen::MatrixXf M(3, 3);
    M << 0, -z, y,
         z, 0, -x,
        -y, x, 0;

    return M;
}

void DCM2eul_bn(DEAD_RECKONING_DATA_t *pst_DeadReckoningData, const Eigen::MatrixXf &matrix)
{
    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;

    pst_EKFObject->euler_angles[0] = rad2deg(atan2(matrix(2, 1), matrix(2, 2)));
    pst_EKFObject->euler_angles[1] = rad2deg(asin(-matrix(2, 0)));
    pst_EKFObject->euler_angles[2] = rad2deg(atan2(matrix(1, 0), matrix(0, 0)));

    pst_EKF_data->roll  = pst_EKFObject->euler_angles[0];
    pst_EKF_data->pitch = pst_EKFObject->euler_angles[1];
    pst_EKF_data->yaw   = pst_EKFObject->euler_angles[2];
}

MatrixXf quat2DCM(DEAD_RECKONING_DATA_t *pst_DeadReckoningData){

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;

    Eigen::Matrix<float32_t,3,3> C_N2B;
    C_N2B(0,0) = 2 * pow(pst_EKF_data->qu_x, 2)- 1 + 2 * pow(pst_EKF_data->qu_y, 2);
    C_N2B(1,1) = 2 * pow(pst_EKF_data->qu_x, 2)- 1 + 2 * pow(pst_EKF_data->qu_z, 2);
    C_N2B(2,2) = 2 * pow(pst_EKF_data->qu_x, 2)- 1 + 2 * pow(pst_EKF_data->qu_w, 2);   
    C_N2B(0,1) = 2 * pst_EKF_data->qu_y * pst_EKF_data->qu_z + 2 * pst_EKF_data->qu_x * pst_EKF_data->qu_w;
    C_N2B(0,2) = 2 * pst_EKF_data->qu_y * pst_EKF_data->qu_w - 2 * pst_EKF_data->qu_x * pst_EKF_data->qu_z;
    C_N2B(1,0) = 2 * pst_EKF_data->qu_y * pst_EKF_data->qu_z - 2 * pst_EKF_data->qu_x * pst_EKF_data->qu_w;
    C_N2B(1,2) = 2 * pst_EKF_data->qu_z * pst_EKF_data->qu_w + 2 * pst_EKF_data->qu_x * pst_EKF_data->qu_y;
    C_N2B(2,0) = 2 * pst_EKF_data->qu_y * pst_EKF_data->qu_w + 2 * pst_EKF_data->qu_x * pst_EKF_data->qu_z;
    C_N2B(2,1) = 2 * pst_EKF_data->qu_z * pst_EKF_data->qu_w - 2 * pst_EKF_data->qu_x * pst_EKF_data->qu_y;

    pst_EKFObject->st_C_B2N = C_N2B.transpose();

    return pst_EKFObject->st_C_B2N;
}


void IMUSetting(DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
{

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;


    if(abs(pst_EKF_data->la_x) > 0.00001 && abs(pst_EKF_data->la_y) > 0.00001){

        pst_EKFObject->st_X(0, 6) = pst_EKF_data->q_roll;
        pst_EKFObject->st_X(0, 7) = pst_EKF_data->q_pitch;
        pst_EKFObject->st_X(0, 8) = pst_EKF_data->q_yaw;
    
        pst_EKF_data->yaw = pst_EKF_data->q_yaw;

        IMU_flag = true;

    }

}

Eigen::MatrixXf reshape(Eigen::MatrixXf m, int32_t col, int32_t row)
{
    Eigen::MatrixXf M(col, row);
    M << m(0, 0), m(0, 1), m(0, 2);
    return M;
}

void INS_ERROR_MODEL(DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
{
    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;

    pst_EKF_data->lat = pst_EKFObject->st_X(0, 0);
    pst_EKF_data->lon = pst_EKFObject->st_X(0, 1);
    pst_EKF_data->alt = pst_EKFObject->st_X(0, 2);
    pst_EKF_data->v_n = pst_EKFObject->st_X(0, 3);
    pst_EKF_data->v_e = pst_EKFObject->st_X(0, 4);
    pst_EKF_data->v_u = -pst_EKFObject->st_X(0, 5);
    pst_EKF_data->roll = pst_EKFObject->st_X(0, 6);
    pst_EKF_data->pitch = pst_EKFObject->st_X(0, 7);
    pst_EKF_data->yaw = pst_EKFObject->st_X(0, 8);

    float32_t rad_lat = deg2rad(pst_EKF_data->lat);
    float32_t rad_roll = deg2rad(pst_EKF_data->roll);
    float32_t rad_pitch = deg2rad(pst_EKF_data->pitch);
    float32_t rad_yaw = deg2rad(pst_EKF_data->yaw);

    //earth model
    pst_EKF_data->Rm = pst_EKF_data->R0 * (1 - pst_EKF_data->ecc_2) / pow((1 - pst_EKF_data->ecc_2 * pow((sin(rad_lat)), 2)), 1.5); 
    pst_EKF_data->Rt = pst_EKF_data->R0 / (1 - pst_EKF_data->ecc_2 * pow(pow((sin(rad_lat)), 2), 0.5));
    pst_EKF_data->Rt = pst_EKF_data->R0 / (1 - pst_EKF_data->ecc_2 * pow(pow((sin(rad_lat)), 2), 0.5));                                                                   // À§µµ(lat)¿¡ µû¶ó µ¿-Œ  ¹æÇâÀž·ÎÀÇ ¿ø°æ°Åž®

    pst_EKF_data->Rmm = (3 * pst_EKF_data->R0 * (1 - pst_EKF_data->ecc_2) * pst_EKF_data->ecc_2 * sin(rad_lat)) * cos(rad_lat) / pow((1 - pst_EKF_data->ecc_2 * ((sin(rad_lat)), 2)), 2.5); 
    pst_EKF_data->Rtt = pst_EKF_data->R0 * pst_EKF_data->ecc_2 * sin(rad_lat) * cos(rad_lat) / pow((1 - pst_EKF_data->ecc_2 * pow((sin(rad_lat)), 2)), 1.5);                 

    pst_EKFObject->st_av.setZero();
    pst_EKFObject->st_g.setZero();

    pst_EKFObject->st_w_ibb << pst_EKF_data->av_x, pst_EKF_data->av_y, pst_EKF_data->av_z;

    // ECEF nav frame angular rate
    pst_EKFObject->st_w_enn << pst_EKF_data->v_e / (pst_EKF_data->Rt + pst_EKF_data->h), -pst_EKF_data->v_n / (pst_EKF_data->Rm + pst_EKF_data->h), -pst_EKF_data->v_e * tan(rad_lat) / (pst_EKF_data->Rt + pst_EKF_data->h);
    
    pst_EKF_data->rho_n = pst_EKFObject->st_w_enn(0);
    pst_EKF_data->rho_e = pst_EKFObject->st_w_enn(1);
    pst_EKF_data->rho_d = pst_EKFObject->st_w_enn(2);

    pst_EKFObject->st_w_ien << pst_EKF_data->Ome * cos(rad_lat), 0, -pst_EKF_data->Ome * sin(rad_lat);

    pst_EKFObject->st_w_inn << pst_EKFObject->st_w_ien(0, 0) + pst_EKFObject->st_w_enn(0, 0), pst_EKFObject->st_w_ien(0, 1) + pst_EKFObject->st_w_enn(0, 1), pst_EKFObject->st_w_ien(0, 2) + pst_EKFObject->st_w_enn(0, 2);

    //Rotation Matrix
    pst_EKFObject->st_Cbn << cos(rad_pitch) * cos(rad_yaw),
        cos(rad_yaw) * sin(rad_pitch) * sin(rad_roll) -
            cos(rad_roll) * sin(rad_yaw),
        cos(rad_roll) * cos(rad_yaw) * sin(rad_pitch) +
            sin(rad_roll) * sin(rad_yaw),
        cos(rad_pitch) * sin(rad_yaw),
        cos(rad_roll) * cos(rad_yaw) +
            sin(rad_pitch) * sin(rad_roll) * sin(rad_yaw),
        cos(rad_roll) * sin(rad_pitch) * sin(rad_yaw) -
            cos(rad_yaw) * sin(rad_roll),
        -sin(rad_pitch),
        cos(rad_pitch) * sin(rad_roll),
        cos(rad_pitch) * cos(rad_roll); // body to ned DCM  eul2DCM_bn

    pst_EKFObject->st_c1 = skew(pst_EKFObject->st_w_ibb(0, 0) - pst_EKFObject->st_dX(12, 0), pst_EKFObject->st_w_ibb(0, 1) - pst_EKFObject->st_dX(13, 0), pst_EKFObject->st_w_ibb(0, 2) - pst_EKFObject->st_dX(14, 0)); // w_ibb(0,3) -> w_ibb(0,2)
    pst_EKFObject->st_c2 = skew(pst_EKFObject->st_w_inn(0, 0), pst_EKFObject->st_w_inn(0, 1), pst_EKFObject->st_w_inn(0, 2));                        

    pst_EKFObject->st_acc << pst_EKF_data->la_x, pst_EKF_data->la_y, pst_EKF_data->la_z;

    pst_EKFObject->st_f_ned = pst_EKFObject->st_Cbn * pst_EKFObject->st_acc; // Cosine direction matrix @=matrix x matrix body to ned accel
    pst_EKF_data->f_n = pst_EKFObject->st_f_ned(0, 0); // n accel
    pst_EKF_data->f_e = pst_EKFObject->st_f_ned(1, 0); // e accel
    pst_EKF_data->f_d = pst_EKFObject->st_f_ned(2, 0); // d accel

    pst_EKFObject->st_Cbn = pst_EKFObject->st_Cbn + (pst_EKFObject->st_Cbn * pst_EKFObject->st_c1 - pst_EKFObject->st_c2 * pst_EKFObject->st_Cbn) * pst_EKF_data->dt; // mechanization DCM differential equation
    DCM2eul_bn(pst_DeadReckoningData, pst_EKFObject->st_Cbn);    

    pst_EKFObject->st_c31 << pst_EKFObject->st_w_ien(0, 0) * 2, pst_EKFObject->st_w_ien(0, 1) * 2, pst_EKFObject->st_w_ien(0, 2) * 2;
    pst_EKFObject->st_c32 << pst_EKFObject->st_w_enn(0, 0), pst_EKFObject->st_w_enn(0, 1), pst_EKFObject->st_w_enn(0, 2);
    pst_EKFObject->st_c33 = pst_EKFObject->st_c31 + pst_EKFObject->st_c32;

    pst_EKFObject->st_c34 << pst_EKFObject->st_c33(0, 1) * pst_EKFObject->st_V_ned(2, 0) - pst_EKFObject->st_c33(0, 2) * pst_EKFObject->st_V_ned(1, 0),
        pst_EKFObject->st_c33(0, 2) * pst_EKFObject->st_V_ned(0, 0) - pst_EKFObject->st_c33(0, 0) * pst_EKFObject->st_V_ned(2, 0),
        pst_EKFObject->st_c33(0, 1) * pst_EKFObject->st_V_ned(1, 0) - pst_EKFObject->st_c33(0, 1) * pst_EKFObject->st_V_ned(0, 0);

    pst_EKFObject->st_c35 = reshape(pst_EKFObject->st_c34, 3, 1);
    pst_EKFObject->st_c3 << pst_EKF_data->la_x - pst_EKFObject->st_dX(9, 0) - pst_EKFObject->st_c35(0, 0), pst_EKF_data->la_y - pst_EKFObject->st_dX(10, 0) - pst_EKFObject->st_c35(1, 0), pst_EKF_data->la_z - pst_EKFObject->st_dX(11, 0) - pst_EKFObject->st_c35(2, 0);

    pst_EKFObject->st_V_ned << pst_EKF_data->v_n, pst_EKF_data->v_e, -pst_EKF_data->v_u;
    pst_EKFObject->st_g << 0, 0, pst_EKF_data->gravity;
    pst_EKFObject->st_m_7 = pst_EKFObject->st_Cbn * pst_EKFObject->st_c3 - pst_EKFObject->st_c35;
    pst_EKFObject->st_V = pst_EKFObject->st_V_ned + (pst_EKFObject->st_m_7 + pst_EKFObject->st_g) * pst_EKF_data->dt; // <- V = reshape(V_ned, 3, 1) + (m_7 + g) * dt;

    pst_EKF_data->v_n = pst_EKFObject->st_V(0, 0);
    pst_EKF_data->v_e = pst_EKFObject->st_V(1, 0);
    pst_EKF_data->v_u = -pst_EKFObject->st_V(2, 0);

    // Position Update
    pst_EKF_data->lat = pst_EKF_data->lat + rad2deg((pst_EKF_data->v_n / (pst_EKF_data->Rm + pst_EKF_data->h)) * pst_EKF_data->dt);
    pst_EKF_data->lon = pst_EKF_data->lon + rad2deg((pst_EKF_data->v_e / ((pst_EKF_data->Rt + pst_EKF_data->h) * cos(rad_lat))) * pst_EKF_data->dt);
    pst_EKF_data->alt = pst_EKF_data->alt + (pst_EKF_data->v_u) * pst_EKF_data->dt;
    pst_EKF_data->h = pst_EKF_data->h + (pst_EKF_data->v_u)*pst_EKF_data->dt;



    pst_EKFObject->st_X << pst_EKF_data->lat, pst_EKF_data->lon, pst_EKF_data->alt, 
                            pst_EKF_data->v_n, pst_EKF_data->v_e, -pst_EKF_data->v_u, 
                            pst_EKF_data->roll, pst_EKF_data->pitch, pst_EKF_data->yaw;


}

void INS_MECHANIZATION(DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
    {
        EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
        GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;
        
        float32_t rad_lat = deg2rad(pst_EKF_data->lat);
        float32_t cos_lat = cos(rad_lat); 
        float32_t sin_lat = sin(rad_lat); 
        float32_t tan_lat = tan(rad_lat); 


        pst_EKFObject->st_F_pp << pst_EKF_data->Rmm * pst_EKF_data->rho_e / (pst_EKF_data->Rm + pst_EKF_data->h), 0, pst_EKF_data->rho_e / (pst_EKF_data->Rm + pst_EKF_data->h),
            pst_EKF_data->rho_n * (tan_lat - pst_EKF_data->Rtt / (pst_EKF_data->Rt + pst_EKF_data->h)) / cos_lat, 0, -pst_EKF_data->rho_n / (cos_lat * (pst_EKF_data->Rt + pst_EKF_data->h)),
            0, 0, 0;

        pst_EKFObject->st_F_pv << 1 / (pst_EKF_data->Rm + pst_EKF_data->h), 0, 0,
            0, 1 / (cos_lat * (pst_EKF_data->Rt + pst_EKF_data->h)), 0,
            0, 0, -1;

        pst_EKFObject->st_F_vp << pst_EKF_data->Rmm * pst_EKF_data->rho_e * (-pst_EKF_data->v_u) / (pst_EKF_data->Rm + pst_EKF_data->h) - 
            (pst_EKF_data->rho_n / (pow(cos_lat, 2)) + 2 * pst_EKFObject->st_w_ien(0)) * pst_EKF_data->v_e - 
            pst_EKF_data->rho_n * pst_EKF_data->rho_d * pst_EKF_data->Rtt, 0,
            pst_EKF_data->rho_e * (-pst_EKF_data->v_u) / (pst_EKF_data->Rm + pst_EKF_data->h) - pst_EKF_data->rho_n * pst_EKF_data->rho_d,
            (2 * pst_EKFObject->st_w_ien(0) + pst_EKF_data->rho_n / (pow(cos_lat, 2)) + pst_EKF_data->rho_d * 
            pst_EKF_data->Rtt / (pst_EKF_data->Rt + pst_EKF_data->h)) * pst_EKF_data->v_n - (pst_EKF_data->rho_n * 
            pst_EKF_data->Rtt / (pst_EKF_data->Rt + pst_EKF_data->h) - 2 * pst_EKFObject->st_w_ien(2)) * (-pst_EKF_data->v_u), 
            0, pst_EKF_data->rho_d * pst_EKF_data->v_n / (pst_EKF_data->Rt + pst_EKF_data->h) - pst_EKF_data->rho_n *
             (-pst_EKF_data->v_u) / (pst_EKF_data->Rt + pst_EKF_data->h),
            (pow(pst_EKF_data->rho_n, 2)) * pst_EKF_data->Rtt + (pow(pst_EKF_data->rho_e, 2)) * pst_EKF_data->Rmm - 2 *
             pst_EKFObject->st_w_ien(2) * pst_EKF_data->v_e, 0, pow(pst_EKF_data->rho_n, 2) + pow(pst_EKF_data->rho_e, 2);

        pst_EKFObject->st_F_vv << (-pst_EKF_data->v_u) / (pst_EKF_data->Rm + pst_EKF_data->h), 2 * pst_EKF_data->rho_d + 2 * pst_EKFObject->st_w_ien(2), -pst_EKF_data->rho_e,
            -2 * pst_EKFObject->st_w_ien(2) - pst_EKF_data->rho_d, (pst_EKF_data->v_n * tan_lat - pst_EKF_data->v_u) / (pst_EKF_data->Rt + pst_EKF_data->h), 2 * pst_EKFObject->st_w_ien(0) + pst_EKF_data->rho_n,
            2 * pst_EKF_data->rho_e, -2 * pst_EKFObject->st_w_ien(0) - 2 * pst_EKF_data->rho_n, 0;

        pst_EKFObject->st_F_vphi << 0, -pst_EKF_data->f_d, pst_EKF_data->f_e,
            pst_EKF_data->f_d, 0, -pst_EKF_data->f_n,
            -pst_EKF_data->f_e, pst_EKF_data->f_n, 0;

        pst_EKFObject->st_F_phip << pst_EKFObject->st_w_ien(2) - pst_EKF_data->rho_n * pst_EKF_data->Rtt / (pst_EKF_data->Rt + pst_EKF_data->h), 0, -pst_EKF_data->rho_n / (pst_EKF_data->Rt + pst_EKF_data->h),
            -pst_EKF_data->rho_e * pst_EKF_data->Rmm / (pst_EKF_data->Rm + pst_EKF_data->h), 0, -pst_EKF_data->rho_e / (pst_EKF_data->Rm + pst_EKF_data->h),
            -pst_EKFObject->st_w_ien(0) - pst_EKF_data->rho_n / (pow(pst_EKF_data->cos_lat, 2)) - pst_EKF_data->rho_d * pst_EKF_data->Rtt / (pst_EKF_data->Rt + pst_EKF_data->h), 0,
             -pst_EKF_data->rho_d / (pst_EKF_data->Rt + pst_EKF_data->h);

        pst_EKFObject->st_F_phiv << 0, 1 / (pst_EKF_data->Rt + pst_EKF_data->h), 0,
            -1 / (pst_EKF_data->Rm + pst_EKF_data->h), 0, 0,
            0, -tan_lat / (pst_EKF_data->Rt + pst_EKF_data->h), 0;

        pst_EKFObject->st_F_phiphi << 0, pst_EKFObject->st_w_ien(2) + pst_EKF_data->rho_d, -pst_EKF_data->rho_e,
            -pst_EKFObject->st_w_ien(2) - pst_EKF_data->rho_d, 0, pst_EKFObject->st_w_ien(0) + pst_EKF_data->rho_n,
            pst_EKF_data->rho_e, -pst_EKFObject->st_w_ien(0) - pst_EKF_data->rho_n, 0;

        pst_EKFObject->st_Cbn_minus = (-1) * pst_EKFObject->st_Cbn;

        pst_EKFObject->st_F.setZero();

        int32_t i, k;

        for (i = 0; i < 3; i++)
        {
            for (k = 0; k < 3; k++)
            {
                pst_EKFObject->st_F(i, k) = pst_EKFObject->st_F_pp(i, k);
                pst_EKFObject->st_F(i, k + 3) = pst_EKFObject->st_F_pv(i, k);
                pst_EKFObject->st_F(i + 3, k) = pst_EKFObject->st_F_vp(i, k);
                pst_EKFObject->st_F(i + 3, k + 3) = pst_EKFObject->st_F_vv(i, k);
                pst_EKFObject->st_F(i + 3, k + 6) = pst_EKFObject->st_F_vphi(i, k);
                pst_EKFObject->st_F(i + 3, k + 9) = pst_EKFObject->st_Cbn(i, k);
                pst_EKFObject->st_F(i + 6, k) = pst_EKFObject->st_F_phip(i, k);
                pst_EKFObject->st_F(i + 6, k + 3) = pst_EKFObject->st_F_phiv(i, k);
                pst_EKFObject->st_F(i + 6, k + 6) = pst_EKFObject->st_F_phiphi(i, k);
                pst_EKFObject->st_F(i + 6, k + 12) = pst_EKFObject->st_Cbn_minus(i , k);

            }
        }

    }

void updateBias(DEAD_RECKONING_DATA_t *pst_DeadReckoningData) {

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;

    pst_EKFObject->st_accel_b(0,0) = pst_EKF_data->la_x - pst_EKFObject->st_dX(9,0);
    pst_EKFObject->st_accel_b(1,0) = pst_EKF_data->la_y - pst_EKFObject->st_dX(10,0);
    pst_EKFObject->st_accel_b(2,0) = pst_EKF_data->la_z - pst_EKFObject->st_dX(11,0);
    pst_EKFObject->st_omega_b(0,0) = pst_EKF_data->av_x - pst_EKFObject->st_dX(12,0);
    pst_EKFObject->st_omega_b(1,0) = pst_EKF_data->av_y - pst_EKFObject->st_dX(13,0);
    pst_EKFObject->st_omega_b(2,0) = pst_EKF_data->av_z - pst_EKFObject->st_dX(14,0);
}

void updateJacobianMatrix(DEAD_RECKONING_DATA_t *pst_DeadReckoningData) {

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;

    // Jacobian
    // printf("update Jacobian");
    // F.setZero();
    // ... pos2gs
    pst_EKFObject->st_F.block(0,3,3,3) = Eigen::Matrix<float32_t,3,3>::Identity();
    // ... gs2pos
    pst_EKFObject->st_F(5,2) = -2 * pst_EKF_data->gravity / pst_EKF_data->R0;
    // ... gs2att
    updateBias(pst_DeadReckoningData);

    pst_EKFObject->st_C_B2N = quat2DCM(pst_DeadReckoningData);
    pst_EKFObject->st_F.block(3,6,3,3) = -2 * pst_EKFObject->st_C_B2N * skew(pst_EKFObject->st_accel_b(0), pst_EKFObject->st_accel_b(1), pst_EKFObject->st_accel_b(2));
    // ... gs2acc
    pst_EKFObject->st_F.block(3,9,3,3) = -pst_EKFObject->st_C_B2N;
    // cout << "C_B2N: " << pst_EKFObject->C_B2N << endl;
    // ... att2att
    pst_EKFObject->st_F.block(6,6,3,3) = -skew(pst_EKFObject->st_omega_b(0,0), pst_EKFObject->st_omega_b(1,0), pst_EKFObject->st_omega_b(2,0));
    // ... att2gyr
    pst_EKFObject->st_F.block(6,12,3,3) = -0.5 * Matrix<float32_t,3,3>::Identity();
    // ... Accel Markov Bias
    pst_EKFObject->st_F.block(9,9,3,3) = -1/ pst_EKF_data->TAU_A * Matrix<float32_t,3,3>::Identity();
    pst_EKFObject->st_F.block(12,12,3,3) = -1 / pst_EKF_data->TAU_G * Matrix<float32_t,3,3>::Identity();
}

void UpdataState(RAW_GPSINS_DATA_t *pst_GPSINSData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData){

    memcpy(&pst_DeadReckoningData->st_GPS, pst_GPSINSData->arc_Buffer, sizeof(GPS_DATA_t));

    EKF_DATA_t *pst_EKF_data = &pst_DeadReckoningData->st_EKF_data;
    GPS_IMU_EKF_MATRIX_t *pst_EKFObject = &pst_DeadReckoningData->st_EKF_Matrix;

    Eigen::MatrixXf z(15, 1);

    pst_EKF_data->gps_lat = pst_DeadReckoningData->st_GPS.f64_Latitude;
    pst_EKF_data->gps_lon = pst_DeadReckoningData->st_GPS.f64_Longitude;
    pst_EKF_data->gps_alt = pst_DeadReckoningData->st_GPS.f64_Altitude;

    pst_EKF_data->lat = pst_EKF_data->gps_lat;
    pst_EKF_data->lon = pst_EKF_data->gps_lon;
    pst_EKF_data->alt = pst_EKF_data->gps_alt;
    pst_EKF_data->h = pst_EKF_data->gps_alt;

    lla2enu(pst_EKF_data->lat, pst_EKF_data->lon, pst_EKF_data->alt, pst_EKF_data->cur_E, pst_EKF_data->cur_N, pst_EKF_data->cur_U); 

    pst_EKF_data->d_E = pst_EKF_data->cur_E - pst_EKF_data->prev_E;
    pst_EKF_data->d_N = pst_EKF_data->cur_N - pst_EKF_data->prev_N;
    pst_EKF_data->d_U = pst_EKF_data->cur_U - pst_EKF_data->prev_U;

    if(pst_EKF_data->dt_gps < 0.0001){ // 이거 안하면 dt_gps 너무 작을 때 nan떠요
        pst_EKF_data->dt_gps = 0.05;
    }

    pst_EKF_data->gps_v_e = pst_EKF_data->d_E / pst_EKF_data->dt_gps;
    pst_EKF_data->gps_v_n = pst_EKF_data->d_N / pst_EKF_data->dt_gps;
    pst_EKF_data->gps_v_u = pst_EKF_data->d_U / pst_EKF_data->dt_gps;

    pst_EKF_data->v_e = pst_EKF_data->gps_v_e;
    pst_EKF_data->v_n = pst_EKF_data->gps_v_n;
    pst_EKF_data->v_u = pst_EKF_data->gps_v_u;

    pst_EKF_data->prev_N = pst_EKF_data->cur_N;
    pst_EKF_data->prev_E = pst_EKF_data->cur_E;
    pst_EKF_data->prev_U = pst_EKF_data->cur_U;

            
    pst_EKFObject->st_K = pst_EKFObject->st_P * (pst_EKFObject->st_H.transpose()) * ((pst_EKFObject->st_H * pst_EKFObject->st_P * pst_EKFObject->st_H.transpose() + pst_EKFObject->st_R).inverse());

    z.setZero();

    z(0, 0) = pst_EKFObject->st_X(0, 0) - pst_EKF_data->gps_lat;
    z(1, 0) = pst_EKFObject->st_X(0, 1) - pst_EKF_data->gps_lon;
    z(2, 0) = pst_EKFObject->st_X(0, 2) - pst_EKF_data->gps_alt;
    z(3, 0) = pst_EKFObject->st_X(0, 3) - pst_EKF_data->gps_v_n;
    z(4, 0) = pst_EKFObject->st_X(0, 4) - pst_EKF_data->gps_v_e;
    z(5, 0) = pst_EKFObject->st_X(0, 5) - pst_EKF_data->gps_v_u;

    pst_EKFObject->st_dX = pst_EKFObject->st_dX + (pst_EKFObject->st_K * (z - pst_EKFObject->st_H * pst_EKFObject->st_dX));
    pst_EKFObject->st_P = pst_EKFObject->st_P - pst_EKFObject->st_K * pst_EKFObject->st_H * pst_EKFObject->st_P;

    pst_EKFObject->st_X(0, 0) = pst_EKFObject->st_X(0, 0) - pst_EKFObject->st_dX(0, 0); // lla
    pst_EKFObject->st_X(0, 1) = pst_EKFObject->st_X(0, 1) - pst_EKFObject->st_dX(1, 0);
    pst_EKFObject->st_X(0, 2) = pst_EKFObject->st_X(0, 2) - pst_EKFObject->st_dX(2, 0);
    pst_EKFObject->st_X(0, 3) = pst_EKFObject->st_X(0, 3) - pst_EKFObject->st_dX(3, 0); // V_ned
    pst_EKFObject->st_X(0, 4) = pst_EKFObject->st_X(0, 4) - pst_EKFObject->st_dX(4, 0);
    pst_EKFObject->st_X(0, 5) = pst_EKFObject->st_X(0, 5) - pst_EKFObject->st_dX(5, 0);
    pst_EKFObject->st_X(0, 6) = pst_EKFObject->st_X(0, 6) - pst_EKFObject->st_dX(6, 0); // Roll Pitch Yaw
    pst_EKFObject->st_X(0, 7) = pst_EKFObject->st_X(0, 7) - pst_EKFObject->st_dX(7, 0);
    pst_EKFObject->st_X(0, 8) = pst_EKFObject->st_X(0, 8) - pst_EKFObject->st_dX(8, 0);


    pst_EKFObject->st_dX = pst_EKFObject->st_dX0;
}
