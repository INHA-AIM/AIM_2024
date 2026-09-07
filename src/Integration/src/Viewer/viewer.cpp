#include "viewer.h"
#include <list>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h" // 이미지 로딩 라이브러리

std::list<std::pair<float32_t, float32_t> > st_GPSTrajectory;
std::list<std::pair<float32_t, float32_t> > st_EKFTrajectory;

int32_t s32_TotalFileNum;
int32_t s32_ActionButtonCount = 0;
uint64_t u64_PrevButtonActionTime = 0;
uint64_t u64_NextButtonActionTime = 0;
float32_t SqaurePoint[8][3];
extern int32_t s32_FileNum;
extern int32_t s32_FileFrameCnt;   

extern bool b_Running;
extern SENSOR_DATA_t st_SensorData;
extern SENSOR_DATA_t st_CurrentData;

extern pthread_mutex_t st_CopyMutex;

extern CAN_DATA_t st_CANDataTemporal;
extern CAN_DATA_t st_CANDataViewer;

extern CAMERA_DATA_t st_CameraDataTemporal;
extern CAMERA_DATA_t st_CameraDataViewer;

extern LIDAR_DATA_t st_LidarDataTemporal;
extern LIDAR_DATA_t st_LidarDataViewer;

extern HMC_LIDAR_DATA_t st_HMCLidarDataTemporal;
extern HMC_LIDAR_DATA_t st_HMCLidarDataViewer;

extern PLANNING_DATA_t st_PlanningDataTemporal;
extern PLANNING_DATA_t st_PlanningDataViewer;

extern DEAD_RECKONING_DATA_t st_DeadReckoningDataTemporal;
extern DEAD_RECKONING_DATA_t st_DeadReckoningDataViewer;

extern CONTROL_DATA_t st_ControlDataTemporal;
extern CONTROL_DATA_t st_ControlDataViewer;


extern FILE_INFO_t st_FileInfo;
extern LOGIC_HZ_t st_LogicHz;

extern OMNI_POINT_t st_OmniPoint;
extern ICP_MAP_t st_ICPMap;

extern TRAJECTORY_t st_Trajectory;

extern int32_t s32_Va;
extern POS_t st_Pos;
extern EGO_POS_t st_Ego_Pos;
extern F32_INFO_t st_Prev;         // 이전 state value를 저장시킴으로써 유지
extern F32_INFO_t st_Trans;        // 변화량  

extern float64_t f64_PrevXPos_G; 
extern float64_t f64_PrevYPos_G; 
extern float64_t f64_PrevXPos_L; 
extern float64_t f64_PrevYPos_L; 

extern vector<string> st_LoggingFileList;      // vector 자료형에 대한 변수명 추가 설정
extern int32_t s32_FileCnt;          // constant 조정 필요
extern int32_t s32_SelectFileNum;

extern GLuint WebcamTexture;
extern GLFWwindow* pst_MainWindow;

extern bool b_Flag;            // 마우스가 클릭 된 상황인지 확인
extern bool b_LoggingFileOpen;

extern int32_t b_Start;
extern string s_GlobalPath;
extern string s_LoggingPath;
extern string s_Username;

// Map & Path
extern vector<vector<float64_t>> st_Load_Map_data;
extern vector<vector<float64_t>> st_Left;
extern vector<vector<float64_t>> st_Right;

// Camera
extern int32_t s32_ImShowKey;

// LiDAR

// Planning
extern vector<float64_t> st_Local_Waypoint;
extern PLANNING_DATA_t st_PlanningData;  //테스트용
extern CONTROL_DATA_t st_ControlData;
extern VEHICLE_STATE_t st_VehicleState;

extern float32_t arf32_VelocityNED_m_s[];
extern float32_t arf32_VelocityXYZ_m_s[];
extern float32_t f32_GroundVelocity_m_s;


// CAN
extern VIEWER_PARAMETER_t st_ViewerParam;
extern bool goActive , stopActive , pitStopActive , slowOnActive , slowOffActive ;
extern bool Go , Stop , PitStop , SlowOn , SlowOff ;

extern bool b_SwitchToMain;    // no need to use
bool b_SwitchPlayNStop = false;
bool b_VisibleLidarGround = false;
bool b_VisibleLidarVoxel = false;
bool b_VisibleLidarVoxelCoor = false;
bool b_VisibleLidarCluster = false;
bool b_VisibleLidarLshape = false;
bool b_VisibleLidarObject = false;
bool b_VisibleLidarTracking = true;
bool b_VisibleLidarIntensity = false;
bool b_VisibleSingleAzimuth = false;
bool b_UpdateLocalMap = false;
int32_t s32_TargetAzimuthIndex = 0;
int32_t s32_ICPIteration = 40;

int32_t ars32_ICPNearestIdx[20000] = {0};

bool b_DrawICPInit = false;
list<tuple<float32_t, float32_t, float32_t> > st_ICPPoint;

void DrawHMCLocalMap(HMC_LIDAR_DATA_t* pst_LidarPointData)
{
    // DrawModeStatus();
    // DrawControlBox();
    InitLocalMap();
    int32_t s32_I = 0;
    int32_t s32_J = 0;
    int32_t s32_K = 0;

    uint8_t u8_Flag = 0;
    uint8_t u8_Intensity = 0;
    int32_t s32_Min = 0;
    int32_t s32_Max = 0;

    int32_t s32_ValidNum = 0;

    float32_t f32_LocalX, f32_LocalY;
    float32_t f32_E, f32_N, f32_U, f32_Yaw_deg;
    int32_t s32_NearIdx = 0;
    float32_t f32_Dist = 9999.f;

    vector<pair<float32_t, float32_t> > st_BoundLeft;
    vector<pair<float32_t, float32_t> > st_BoundRight;

    float32_t f32_R, f32_G, f32_B;
    float32_t f32_ObjectHeading_rad;
    float32_t f32_X1, f32_X2, f32_Y1, f32_Y2, f32_Z1, f32_Z2;
    float32_t f32_X, f32_Y, f32_Z;
    float32_t f32_PrevX, f32_PrevY, f32_PrevZ;
    float32_t f32_Slope_rad = 0.f;
    float32_t f32_RX, f32_RY, f32_RZ;
    float32_t f32_Azimuth;
    uint8_t u8_Layer;
    HMC_VOXEL_t* pst_Voxel;
    HMC_POINT_t* pst_Point;
    HMC_CLUSTER_t* pst_Cluster;
    HMC_TRACKING_t* pst_Tracking;
    float32_t f32_MinDist = 999999.f;

    char arc_BufferText[256] = {0};

    int32_t s32_GPSMode = 0;
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");
    s32_GPSMode = st_Config["GPS_Mode"].as<int32_t>();

    DrawCar(1.07+0.8940, 1.826/2, 2.0, -1.81-1.059, -1.826/2, 0);

    if (b_VisibleSingleAzimuth)
    {
        glLineWidth(3.f);
        glColor3f(0.5f, 0.5f, 0.5f);
        glBegin(GL_LINES);
        glVertex3f(-100.f, 0.f, 0.f);
        glVertex3f(-100.f, -100.f, 0.f);
        glEnd();
        glPointSize(3.0f);
    }
    else
    {
        glPointSize(1.0f);
    }

    glBegin(GL_POINTS);
    for (s32_I = 0; s32_I < pst_LidarPointData->s32_PointNum; s32_I++)
    {
        pst_Point = &pst_LidarPointData->arst_Point[s32_I];
        f32_X = pst_Point->f32_X;
        f32_Y = pst_Point->f32_Y;
        f32_Z = pst_Point->f32_Z;
        f32_Azimuth = pst_Point->f32_Azimuth_deg;
        u8_Layer = pst_Point->u8_Layer;

        if ((pst_Point->u8_Flag != c_POINT_INVALID) && b_VisibleSingleAzimuth)
        {
            if ((s32_I / 128) == s32_TargetAzimuthIndex)
            {

                if (s32_I % 128 == 0)
                {
                    sprintf(arc_BufferText, "#%d(%d), (%.2f, %.2f, %.2f)", 0, pst_Point->u8_Flag, f32_X, f32_Y, f32_Z);
                }
                else
                {
                    f32_Dist = getDistance2d(f32_X, f32_Y, f32_PrevX, f32_PrevY);
                    f32_Slope_rad = atan2f(f32_Z - f32_PrevZ, f32_Dist);
                    sprintf(arc_BufferText, "#%d(%d), (%.2f, %.2f, %.2f) - (%.2f, %.3f)", s32_I % 128, pst_Point->u8_Flag, f32_X, f32_Y, f32_Z, f32_Dist, f32_Slope_rad);
                }


                DrawText(arc_BufferText, 190 - (s32_I % 128) * 3, -70, 12);
                // glRasterPos3f(f32_X, f32_Y, f32_Z);
                // for (int8_t s8_Char : "TEST")
                // {
                //     glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, s8_Char); // 폰트와 문자 크기 선택하여 출력
                // }

                glBegin(GL_POINTS);
                if (pst_Point->u8_Flag == c_POINT_GROUND)
                {
                    Gray2RGB(s32_I % 128 * 2, f32_R, f32_G, f32_B);
                    glColor3f(f32_R, f32_G, f32_B);
                }
                else
                {
                    glPointSize(1.0f);
                    glColor3f(1.f, 1.f, 1.f);
                }
                glVertex3f(f32_X, f32_Y, f32_Z);
                glVertex3f(f32_Z - 100.f, -getDistance2d(0.f, 0.f, f32_X, f32_Y), 0.f);
                glEnd();
                                sprintf(arc_BufferText, "%d", s32_I % 128);
                DrawText(arc_BufferText, f32_Z - 100.5f, -getDistance2d(0.f, 0.f, f32_X, f32_Y), 10);

                f32_PrevX = f32_X;
                f32_PrevY = f32_Y;
                f32_PrevZ = f32_Z;
            }


            continue;
        }

        if (b_VisibleLidarIntensity)
        {
            if (pst_Point->u8_Flag == c_POINT_INVALID)
            {
                continue;
            }
            else if (!b_VisibleLidarGround && pst_Point->u8_Flag == c_POINT_GROUND)
            {
                continue;
            }

            Gray2RGB(pst_Point->u8_Intensity, f32_R, f32_G, f32_B);
            glColor3f(f32_R, f32_G, f32_B);
        }
        else
        {

            // if (pst_Point->u8_Flag != c_POINT_INVALID)
            // {
            //     glColor3f(1.f, 1.f, 1.f);
            // }
            // else
            // {
            //     continue;
            // }
            if (pst_Point->u8_Flag == c_POINT_VALID)
            {
                glColor3f(1.0f, 1.0f, 1.0f);

                // if (!(fabsf(f32_X) < 50.f) && (fabsf(f32_Y) < 50.f))
                // {
                //     s32_ValidNum += 1;
                // }
                if(b_VisibleLidarGround && pst_Point->u8_GroundFilteringFlag == 1)
                {
                    glColor3f(1.0f, 1.0f, 0.0f);
                }  
                if(b_VisibleLidarGround && pst_Point->u8_GroundFilteringFlag == 2)
                {
                    glColor3f(1.0f, 0.f, 1.0f);
                }   

            }
            else if (pst_Point->u8_Flag == c_POINT_GROUND && b_VisibleLidarGround)
            {
                if(pst_Point->u8_GroundFilteringFlag == 0)
                {
                    glColor3f(0.0f, 1.0f, 0.0f);
                }
                else if(pst_Point->u8_GroundFilteringFlag == 1)
                {
                    glColor3f(0.0f, 1.0f, 1.0f);
                }
                else if(pst_Point->u8_GroundFilteringFlag == 2)
                {
                    glColor3f(0.0f, 0.0f, 1.0f);
                }
                else if(pst_Point->u8_GroundFilteringFlag == 3)
                {
                    glColor3f(1.0f, 0.f, 0.0f);
                }                                       
            }   
            else
            {
                continue;
            }
        }

        glVertex3f(pst_Point->f32_X, pst_Point->f32_Y, pst_Point->f32_Z);

    }
    glEnd();

    // printf("ValidNum : %d\n", s32_ValidNum);


    if (b_VisibleLidarVoxel)
    {
        glLineWidth(1.0f);

        for (s32_I = 0; s32_I < c_HMC_VOXEL_SIZE_R * c_HMC_VOXEL_SIZE_AZIMUTH * c_HMC_VOXEL_SIZE_Z; s32_I++)
        {
            pst_Voxel = &(pst_LidarPointData->arst_Voxel[s32_I]);


            if (pst_Voxel->b_HasPoint == false)
            {
                continue;
            }
            if(pst_Voxel->s32_PointNum == 0)
            {
                continue;
            }
            if(pst_Voxel->u8_VoxelColor == 0)
            {
                glColor3f(1.f, 0.0f, 0.0f);
                glBegin(GL_LINE_STRIP);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[1], pst_Voxel->arf32_VoxelBaseY[1], pst_Voxel->arf32_VoxelBaseZ[1]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[3], pst_Voxel->arf32_VoxelBaseY[3], pst_Voxel->arf32_VoxelBaseZ[3]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[2], pst_Voxel->arf32_VoxelBaseY[2], pst_Voxel->arf32_VoxelBaseZ[2]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glEnd();
            }
            else if(pst_Voxel->u8_VoxelColor == 1)
            {
                glColor3f(0.f, 1.f, 0.f);
                glBegin(GL_LINE_STRIP);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[1], pst_Voxel->arf32_VoxelBaseY[1], pst_Voxel->arf32_VoxelBaseZ[1]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[3], pst_Voxel->arf32_VoxelBaseY[3], pst_Voxel->arf32_VoxelBaseZ[3]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[2], pst_Voxel->arf32_VoxelBaseY[2], pst_Voxel->arf32_VoxelBaseZ[2]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glEnd();
                // printf("green point : %d\n", pst_Voxel->s32_PointNum);
            }
            else if(pst_Voxel->u8_VoxelColor == 2)
            {
                glColor3f(0.f, 0.f, 1.f);
                glBegin(GL_LINE_STRIP);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[1], pst_Voxel->arf32_VoxelBaseY[1], pst_Voxel->arf32_VoxelBaseZ[1]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[3], pst_Voxel->arf32_VoxelBaseY[3], pst_Voxel->arf32_VoxelBaseZ[3]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[2], pst_Voxel->arf32_VoxelBaseY[2], pst_Voxel->arf32_VoxelBaseZ[2]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glEnd();
            }
            else if(pst_Voxel->u8_VoxelColor == 3)
            {
                glColor3f(1.f, 0.f, 1.f);
                glBegin(GL_LINE_STRIP);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[1], pst_Voxel->arf32_VoxelBaseY[1], pst_Voxel->arf32_VoxelBaseZ[1]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[3], pst_Voxel->arf32_VoxelBaseY[3], pst_Voxel->arf32_VoxelBaseZ[3]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[2], pst_Voxel->arf32_VoxelBaseY[2], pst_Voxel->arf32_VoxelBaseZ[2]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glEnd();
            }
            else if(pst_Voxel->u8_VoxelColor == 4)
            {
                glColor3f(1.f, 1.f, 1.f);
                glBegin(GL_LINE_STRIP);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[1], pst_Voxel->arf32_VoxelBaseY[1], pst_Voxel->arf32_VoxelBaseZ[1]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[3], pst_Voxel->arf32_VoxelBaseY[3], pst_Voxel->arf32_VoxelBaseZ[3]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[2], pst_Voxel->arf32_VoxelBaseY[2], pst_Voxel->arf32_VoxelBaseZ[2]);
                glVertex3f(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseZ[0]);
                glEnd();
            }   
        }
    }


    if (b_VisibleLidarCluster)
    {
        for (s32_I = 0; s32_I < pst_LidarPointData->s32_ClusterNum; s32_I++)
        {
            pst_Cluster = &pst_LidarPointData->arst_Cluster[s32_I];

            f32_X1 = pst_Cluster->f32_X1;
            f32_X2 = pst_Cluster->f32_X2;
            f32_Y1 = pst_Cluster->f32_Y1;
            f32_Y2 = pst_Cluster->f32_Y2;
            f32_Z1 = pst_Cluster->f32_Z1;
            f32_Z2 = pst_Cluster->f32_Z2;

            //glLineWidth(1.f);
            //DrawBox(f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2, f32_Z2);
            if (pst_Cluster->u8_Flag == c_HMC_CLUSTER_NONE)
            {
                glColor3f(0.5f, 0.5f, 0.5f);
                glLineWidth(0.1f);
                DrawBox(f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2, f32_Z2);
            }
            //     else if (pst_Cluster->u8_Flag == c_HMC_CLUSTER_OBSTACLE)
        //     {
        //         glColor3f(1.0f, 0.5f, 0.3f);
        //         glLineWidth(1.f);
        //         DrawBox(f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2, f32_Z2);
        //     }
        //     else if (pst_Cluster->u8_Flag == c_HMC_CLUSTER_VEHICLE)
        //     {
        //         glColor3f(1.f, 0.0f, 0.f);
        //         glLineWidth(1.f);
        //         DrawBox(f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2, f32_Z2);

        //         glLineWidth(4.f);
        //         glBegin(GL_LINE_STRIP);
        //         glColor3f(0.f, 1.0f, 0.5f);

        //         s32_Min = 999;
        //         s32_Max = -1;
        //         for (s32_J = 0; s32_J < 360; s32_J++)
        //         {
        //             if (pst_Cluster->arb_Flag[s32_J])
        //             {
        //                 glVertex3f(pst_Cluster->arf32_X[s32_J], pst_Cluster->arf32_Y[s32_J], 5.f);

        //                 if (pst_Cluster->f32_X2 > 0 && pst_Cluster->f32_Y1 >= 0 && pst_Cluster->f32_Y2 < 0)
        //                 {
        //                     s32_K = s32_J;

        //                     if (s32_J < 180)
        //                     {
        //                         s32_K += 360;
        //                     }

        //                     s32_Min = min(s32_Min, s32_K);
        //                     s32_Max = max(s32_Max, s32_K);
        //                 }
        //                 else
        //                 {
        //                     s32_Min = min(s32_Min, s32_J);
        //                     s32_Max = max(s32_Max, s32_J);
        //                 }
        //             }
        //         }

        //         glEnd();

        //         // DrawVehicleBox(pst_Cluster->arf32_X, pst_Cluster->arf32_Y, pst_Cluster->arb_Flag, s32_Min, s32_Max);
        //     }
        }

    }

    if (b_VisibleLidarTracking)
    {

        glLineWidth(1.f);

        pst_Tracking = pst_LidarPointData->arst_Tracking;
        // printf("%d\n", pst_LidarPointData->s32_TrackingNum);
        for(s32_I = 0; s32_I < pst_LidarPointData->s32_TrackingNum; s32_I++)
        {
            if(pst_Tracking[s32_I].u8_StateFlag != c_HMC_OBJECT_TEMP)
            {
                f32_X1 = pst_Tracking[s32_I].f32_X + (pst_Tracking[s32_I].f32_Length / 2.f);
                f32_Y1 = pst_Tracking[s32_I].f32_Y + (pst_Tracking[s32_I].f32_Width / 2.f);
                f32_Z1 = pst_Tracking[s32_I].f32_Z + (pst_Tracking[s32_I].f32_Height / 2.f);
                f32_X2 = pst_Tracking[s32_I].f32_X - (pst_Tracking[s32_I].f32_Length / 2.f);
                f32_Y2 = pst_Tracking[s32_I].f32_Y - (pst_Tracking[s32_I].f32_Width / 2.f);
                f32_Z2 = pst_Tracking[s32_I].f32_Z - (pst_Tracking[s32_I].f32_Height / 2.f);

                if (pst_Tracking[s32_I].u8_StateFlag == c_HMC_OBJECT_DYNAMIC)
                {
                    glColor3f(0.f, 1.f, 1.f);
                    DrawBox(pst_Tracking[s32_I].f32_X, pst_Tracking[s32_I].f32_Y, pst_Tracking[s32_I].f32_Z,
                            pst_Tracking[s32_I].f32_Length, pst_Tracking[s32_I].f32_Width, pst_Tracking[s32_I].f32_Height,
                            pst_Tracking[s32_I].f32_Yaw_rad);

                    f32_X1 = pst_Tracking[s32_I].f32_X;
                    f32_Y1 = pst_Tracking[s32_I].f32_Y;
                    f32_Z = pst_Tracking[s32_I].f32_Z;
     
                    glColor3f(1.f, 1.f, 0.f);
                    glBegin(GL_LINES);
                    
                    f32_ObjectHeading_rad = atan2f(pst_Tracking[s32_I].f32_VelocityY_m_s, pst_Tracking[s32_I].f32_VelocityX_m_s);

                    f32_Dist = getDistance2d(0.f, 0.f, pst_Tracking[s32_I].f32_VelocityX_m_s, pst_Tracking[s32_I].f32_VelocityY_m_s);
                    f32_X2 = f32_X1 + f32_Dist * cosf(f32_ObjectHeading_rad);
                    f32_Y2 = f32_Y1 + f32_Dist * sinf(f32_ObjectHeading_rad);

                    glVertex3f(f32_X1, f32_Y1, f32_Z);
                    glVertex3f(f32_X2, f32_Y2, f32_Z);
                    glEnd();

                    glColor3f(1.f, 0.f, 0.f);
                    glBegin(GL_LINES);

                    f32_ObjectHeading_rad = pst_Tracking[s32_I].f32_Yaw_rad;

                    f32_Dist = getDistance2d(0.f, 0.f, pst_Tracking[s32_I].f32_AbsoluteVelocityX_m_s, pst_Tracking[s32_I].f32_AbsoluteVelocityY_m_s);
                    f32_X2 = f32_X1 + f32_Dist * cosf(f32_ObjectHeading_rad);
                    f32_Y2 = f32_Y1 + f32_Dist * sinf(f32_ObjectHeading_rad);

                    glVertex3f(f32_X1, f32_Y1, f32_Z);
                    glVertex3f(f32_X2, f32_Y2, f32_Z);
                    glEnd();
                }
                else
                {
                    glColor3f(0.1f, 0.4f, 0.8f);
                    DrawBox(f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2, f32_Z2);
                }

                // printf("%f %f %f %f\n", pst_Tracking[s32_I].f32_X, pst_Tracking[s32_I].f32_Y, 
                //     pst_Tracking[s32_I].f32_VelocityX_m_s, pst_Tracking[s32_I].f32_VelocityY_m_s);
            }
        }
        float32_t f32_Factor = st_ViewerParam.f32_LocalZoomFactor;

        for (s32_I = 0;s32_I < st_PlanningDataViewer.s32_ObjectNum;s32_I++)
        {
            glColor3f(1.f, 1.f, 1.f);
            DrawLabelWithNumber3D("Idx:", s32_I,st_PlanningDataViewer.arst_ObjectData[s32_I].f32_X_local + 8.f * f32_Factor , st_PlanningDataViewer.arst_ObjectData[s32_I].f32_Y_local, 8.0f * f32_Factor);
            glEnd();

            glColor3f(1.f, 1.f, 1.f);
            DrawLabelWithNumber3D("Vel:",st_PlanningDataViewer.arst_ObjectData[s32_I].f32_Velocity_m_s_global * 3.6f, st_PlanningDataViewer.arst_ObjectData[s32_I].f32_X_local + 6.f * f32_Factor, st_PlanningDataViewer.arst_ObjectData[s32_I].f32_Y_local, 6.0f * f32_Factor);
            glEnd();

            glColor3f(1.f, 1.f, 1.f);
            DrawLabelWithNumber3D("Dist:",st_PlanningDataViewer.arst_ObjectData[s32_I].f32_DS, st_PlanningDataViewer.arst_ObjectData[s32_I].f32_X_local + 4.f * f32_Factor, st_PlanningDataViewer.arst_ObjectData[s32_I].f32_Y_local, 4.0f * f32_Factor);
            glEnd();

            glColor3f(1.f, 1.f, 1.f);
            DrawLabelWithNumber3D("ICP:0.",st_PlanningDataViewer.arst_ObjectData[s32_I].f32_Score * 1000.f, st_PlanningDataViewer.arst_ObjectData[s32_I].f32_X_local + 2.f * f32_Factor, st_PlanningDataViewer.arst_ObjectData[s32_I].f32_Y_local, 2.0f * f32_Factor);
            glEnd();

        }
    }

    // lla2enu(st_DeadReckoningDataViewer.st_GPS.f64_Latitude, 
    //         st_DeadReckoningDataViewer.st_GPS.f64_Longitude,
    //         0, 
    //         f32_E, f32_N, f32_U);
    // f32_Yaw_deg = -st_DeadReckoningDataViewer.st_IMU.f32_Yaw_deg + 90.f;

    f32_E = st_HMCLidarDataViewer.f32_PredictX;
    f32_N = st_HMCLidarDataViewer.f32_PredictY;

    if(s32_GPSMode == c_PARSING_MORAI) f32_Yaw_deg = rad2deg(st_PlanningDataViewer.st_EgoVehicleData.f32_Yaw_rad_ENU);
    else if(s32_GPSMode == c_PARSING_AVANTE) f32_Yaw_deg = -st_HMCLidarDataViewer.f32_PredictYaw_deg + 90.f;

    int32_t s32_LanePointNum = st_PlanningDataViewer.st_ReferenceLine1_global.s32_Num;
    
    PATH_t *pst_SELane[] = {&st_PlanningDataViewer.st_SEBoundary1_global,
                          &st_PlanningDataViewer.st_SEBoundary2_global,
                          &st_PlanningDataViewer.st_SEBoundary3_global,
                          &st_PlanningDataViewer.st_SEBoundary4_global};

    PATH_t *pst_Boundary[] = {&st_PlanningDataViewer.st_Boundary1_global,
                              &st_PlanningDataViewer.st_Boundary2_global,
                              &st_PlanningDataViewer.st_Boundary3_global,
                              &st_PlanningDataViewer.st_Boundary4_global,
                              };

    // for (s32_K = 0;s32_K < 3;s32_K++)
    // {
    //     glLineWidth(2.f);  
    //     glBegin(GL_LINE_STRIP); 
    //     glColor3f(0.4f, .4f, .4f); 

    //     CalcNearestDistIdx(f32_E, f32_N, 
    //                     pst_Lane[s32_K]->arf32_X,
    //                     pst_Lane[s32_K]->arf32_Y,
    //                     pst_Lane[s32_K]->s32_Num,
    //                     f32_Dist, s32_NearIdx);


    //     for (s32_I = -100;s32_I < 100;s32_I++)
    //     {
    //         s32_J = (s32_NearIdx + s32_I + s32_LanePointNum) % s32_LanePointNum;

    //         f32_X = pst_Lane[s32_K]->arf32_X[s32_J]; 
    //         f32_Y = pst_Lane[s32_K]->arf32_Y[s32_J]; 

    //         getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg - 90), f32_X, f32_Y, f32_LocalX, f32_LocalY);
    //         glVertex3f(f32_LocalX, f32_LocalY, 0.f);
    //     }

    //     glEnd();
    // }

    for (int s32_K : {0, 1, 2, 3})
    {
        if (pst_Boundary[s32_K]->s32_Num == 0)
        {
            continue;
        }

        CalcNearestDistIdx(f32_E, f32_N, 
                        pst_Boundary[s32_K]->arf32_X,
                        pst_Boundary[s32_K]->arf32_Y,
                        pst_Boundary[s32_K]->s32_Num,
                        f32_Dist, s32_NearIdx);

        glLineWidth(2.f);  
        glBegin(GL_LINE_STRIP); 
        glColor3f(0.3f, 0.8f, 0.6f); 

        for (s32_I = -100; s32_I <= 100; s32_I += 1)
        {
            s32_J = (s32_NearIdx + s32_I + pst_Boundary[s32_K]->s32_Num) % pst_Boundary[s32_K]->s32_Num;

            f32_X = pst_Boundary[s32_K]->arf32_X[s32_J]; 
            f32_Y = pst_Boundary[s32_K]->arf32_Y[s32_J]; 

            getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg), f32_X, f32_Y, f32_LocalX, f32_LocalY);
            glVertex3f(f32_LocalX, f32_LocalY, 0.f);

            if (s32_K == 3)
            {
                st_BoundLeft.push_back(make_pair(f32_LocalX, f32_LocalY));
            }
            else
            {
                st_BoundRight.push_back(make_pair(f32_LocalX, f32_LocalY));
            }
        }

        glEnd();
    }

    for (int s32_K : {0, 1, 2, 3})
    {
        if (pst_SELane[s32_K]->s32_Num == 0)
        {
            continue;
        }

        CalcNearestDistIdx(f32_E, f32_N,
                        pst_SELane[s32_K]->arf32_X,
                        pst_SELane[s32_K]->arf32_Y,
                        pst_SELane[s32_K]->s32_Num,
                        f32_Dist, s32_NearIdx);

        glLineWidth(2.f);
        glBegin(GL_LINE_STRIP);
        glColor3f(0.3f, 0.8f, 0.6f);

        for (s32_I = -100; s32_I <= 100; s32_I += 1)
        {
            s32_J = (s32_NearIdx + s32_I + pst_SELane[s32_K]->s32_Num) % pst_SELane[s32_K]->s32_Num;

            f32_X = pst_SELane[s32_K]->arf32_X[s32_J];
            f32_Y = pst_SELane[s32_K]->arf32_Y[s32_J];

            getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg), f32_X, f32_Y, f32_LocalX, f32_LocalY);
            glVertex3f(f32_LocalX, f32_LocalY, 0.f);

            if (s32_K == 3)
            {
                st_BoundLeft.push_back(make_pair(f32_LocalX, f32_LocalY));
            }
            else
            {
                st_BoundRight.push_back(make_pair(f32_LocalX, f32_LocalY));
            }
        }

        glEnd();
    }


    // glPointSize(4.f);
    // glBegin(GL_POINTS);
    // glColor3f(1.f, 0.f, 0.f);

    // for (s32_I = 0;s32_I < c_OMNI_FEATURE_MAX_NUM;s32_I++)
    // {
    //     if (pst_LidarPointData->st_OmniFeature.arb_Valid[s32_I] == false)
    //     {
    //         continue;
    //     }

    //     f32_X = pst_LidarPointData->st_OmniFeature.arf32_X[s32_I];
    //     f32_Y = pst_LidarPointData->st_OmniFeature.arf32_Y[s32_I];

    //     glVertex3f(f32_X, f32_Y, 5.f);
    // }

    // glEnd();


    s32_NearIdx = pst_LidarPointData->s32_ICPNearestMapIdx;
    if (s32_NearIdx != -1)
    {
        if (b_UpdateLocalMap)
        {
            for (s32_I = 0;s32_I < st_ICPMap.arst_PointCloud[s32_NearIdx].s32_PointNum;s32_I += 5)
            {
                f32_X = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_X[s32_I];
                f32_Y = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_Y[s32_I];
                f32_Z = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_Z[s32_I];

                // glColor3f(0.5f, 1.0f, 0.2f);
                glColor3f(0.5f, 0.4f, 1.0f);
                glPointSize(1.f);
                glBegin(GL_POINTS);
                glVertex3f(f32_X, f32_Y, f32_Z);
                glEnd();
            }

            for (s32_I = 0; s32_I < pst_LidarPointData->st_ICPPoint.s32_PointNum; s32_I++)
            {
                f32_X = pst_LidarPointData->st_ICPPoint.arf32_X[s32_I];
                f32_Y = pst_LidarPointData->st_ICPPoint.arf32_Y[s32_I];
                f32_Z = pst_LidarPointData->st_ICPPoint.arf32_Z[s32_I];

                glColor3f(0.5f, 1.0f, 0.7f);
                glPointSize(1.f);
                glBegin(GL_POINTS);
                glVertex3f(f32_X, f32_Y, f32_Z);
                glEnd();
            }

            // Eigen::Matrix3f st_R;
            // float32_t f32_AX, f32_AY, f32_AZ, f32_AYaw_rad, f32_AYaw_deg;
            // float32_t f32_BX, f32_BY, f32_BZ, f32_BYaw_rad, f32_CYaw_rad;

            // f32_AX = st_ICPMap.arst_PointCloud[s32_NearIdx].f32_X;
            // f32_AY = st_ICPMap.arst_PointCloud[s32_NearIdx].f32_Y;
            // f32_AZ = st_ICPMap.arst_PointCloud[s32_NearIdx].f32_Z;
            // f32_AYaw_deg = st_ICPMap.arst_PointCloud[s32_NearIdx].f32_Yaw_deg;
            // f32_AYaw_rad = deg2rad(f32_AYaw_deg); 

            // f32_BX = pst_LidarPointData->st_ICPPoint.f32_X;
            // f32_BY = pst_LidarPointData->st_ICPPoint.f32_Y;
            // f32_BZ = pst_LidarPointData->st_ICPPoint.f32_Z;
            // f32_BYaw_rad = deg2rad(pst_LidarPointData->st_ICPPoint.f32_Yaw_deg); 

            // f32_CYaw_rad = f32_AYaw_rad - f32_BYaw_rad;

            // st_R << cosf(f32_CYaw_rad), -sinf(f32_CYaw_rad), 0,
            //         sinf(f32_CYaw_rad), cosf(f32_CYaw_rad), 0, 
            //         0, 0, 1;

            // f32_RX = f32_BX - f32_AX;
            // f32_RY = f32_BY - f32_AY;
            // f32_RZ = f32_BZ - f32_AZ;

            // getLocalCoord(0, 0, -deg2rad(f32_AYaw_deg - 90), f32_RX, f32_RY, f32_RX, f32_RY);

            // for (s32_I = 0; s32_I < pst_LidarPointData->st_ICPPoint.s32_PointNum; s32_I++)
            // {
            //     f32_X = pst_LidarPointData->st_ICPPoint.arf32_X[s32_I];
            //     f32_Y = pst_LidarPointData->st_ICPPoint.arf32_Y[s32_I];
            //     f32_Z = pst_LidarPointData->st_ICPPoint.arf32_Z[s32_I];

            //     f32_X1 = f32_X * st_R(0, 0) + f32_Y * st_R(0, 1) + f32_Z * st_R(0, 2) + f32_RX;
            //     f32_Y1 = f32_X * st_R(1, 0) + f32_Y * st_R(1, 1) + f32_Z * st_R(1, 2) + f32_RY;
            //     f32_Z1 = f32_X * st_R(2, 0) + f32_Y * st_R(2, 1) + f32_Z * st_R(2, 2) + f32_RZ;

            //     glColor3f(0.5f, 1.0f, 0.7f);
            //     glPointSize(1.f);
            //     glBegin(GL_POINTS);
            //     glVertex3f(f32_X1, f32_Y1, f32_Z1);
            //     glEnd();
            // }


        }
        else
        {

            float32_t *pf32_Transform = pst_LidarPointData->arf32_ICPInverseTransform;
            // printf("Map Point Num: %d\n", st_ICPMap.arst_PointCloud[s32_NearIdx].s32_PointNum);

            for (s32_I = 0;s32_I < st_ICPMap.arst_PointCloud[s32_NearIdx].s32_PointNum;s32_I += 5)
            {
                f32_X = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_X[s32_I];
                f32_Y = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_Y[s32_I];
                f32_Z = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_Z[s32_I];

                f32_RX = f32_X * pf32_Transform[0] + f32_Y * pf32_Transform[1] + f32_Z * pf32_Transform[2] + pf32_Transform[3];
                f32_RY = f32_X * pf32_Transform[4] + f32_Y * pf32_Transform[5] + f32_Z * pf32_Transform[6] + pf32_Transform[7];
                f32_RZ = f32_X * pf32_Transform[8] + f32_Y * pf32_Transform[9] + f32_Z * pf32_Transform[10] + pf32_Transform[11];

                // glColor3f(0.5f, 1.0f, 0.2f);
                glColor3f(0.7f, 0.6f, 1.0f);
                glPointSize(1.f);
                glBegin(GL_POINTS);
                glVertex3f(f32_RX, f32_RY, f32_RZ);
                glEnd();
            }

            // for (s32_I = 0;s32_I < pst_LidarPointData->st_ICPPoint.s32_PointNum;s32_I++)
            // {
            //     s32_J = pst_LidarPointData->ars32_ICPNearestIdx[s32_I];

            //     if (s32_J != -1)
            //     {
            //         f32_X = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_X[s32_J];
            //         f32_Y = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_Y[s32_J];
            //         f32_Z = st_ICPMap.arst_PointCloud[s32_NearIdx].arf32_Z[s32_J];

            //         f32_RX = f32_X * pf32_Transform[0] + f32_Y * pf32_Transform[1] + f32_Z * pf32_Transform[2] + pf32_Transform[3];
            //         f32_RY = f32_X * pf32_Transform[4] + f32_Y * pf32_Transform[5] + f32_Z * pf32_Transform[6] + pf32_Transform[7];
            //         f32_RZ = f32_X * pf32_Transform[8] + f32_Y * pf32_Transform[9] + f32_Z * pf32_Transform[10] + pf32_Transform[11];

            //         f32_X1 = pst_LidarPointData->st_ICPPoint.arf32_X[s32_I];
            //         f32_Y1 = pst_LidarPointData->st_ICPPoint.arf32_Y[s32_I];
            //         f32_Z1 = pst_LidarPointData->st_ICPPoint.arf32_Z[s32_I];

            //         glLineWidth(0.1f);
            //         glColor3f(0.2f, 0.2f, 0.2f);
            //         glBegin(GL_LINES);
            //         glVertex3f(f32_X1, f32_Y1, f32_Z1);
            //         glVertex3f(f32_RX, f32_RY, f32_RZ);
            //         glEnd();
            //     }

            //     // glColor3f(0.5f, 1.0f, 0.7f);
            //     // glPointSize(1.f);
            //     // glBegin(GL_POINTS);
            //     // f32_X = pst_LidarPointData->st_ICPPoint.arf32_X[s32_I];
            //     // f32_Y = pst_LidarPointData->st_ICPPoint.arf32_Y[s32_I];
            //     // f32_Z = pst_LidarPointData->st_ICPPoint.arf32_Z[s32_I];
            //     // glVertex3f(f32_X, f32_Y, f32_Z);
            //     // glEnd();
            // }

            // pf32_Transform = pst_LidarPointData->arf32_ICPInverseTransform;

            // for (s32_I = 0;s32_I < pst_LidarPointData->st_ICPPoint.s32_PointNum;s32_I++)
            // {
            //     f32_X = pst_LidarPointData->st_ICPPoint.arf32_X[s32_I];
            //     f32_Y = pst_LidarPointData->st_ICPPoint.arf32_Y[s32_I];
            //     f32_Z = pst_LidarPointData->st_ICPPoint.arf32_Z[s32_I];

            //     // f32_RX = f32_X * pf32_Transform[0] + f32_Y * pf32_Transform[1] + f32_Z * pf32_Transform[2] + pf32_Transform[3];
            //     // f32_RY = f32_X * pf32_Transform[4] + f32_Y * pf32_Transform[5] + f32_Z * pf32_Transform[6] + pf32_Transform[7];
            //     // f32_RZ = f32_X * pf32_Transform[8] + f32_Y * pf32_Transform[9] + f32_Z * pf32_Transform[10] + pf32_Transform[11];

            //     glColor3f(0.7f, 0.0f, 1.0f);
            //     glPointSize(1.f);
            //     glBegin(GL_POINTS);
            //     glVertex3f(f32_X, f32_Y, f32_Z);
            //     glEnd();
            // }
        }
    }
    DrawModeStatus();
    DrawControlBox();
}



void DrawLocalMap()
{
    InitLocalMap();

    int32_t s32_I = 0;
    int32_t s32_J = 0;
    int32_t s32_K = 0;

    float32_t f32_X, f32_Y, f32_Z;

    float32_t f32_X1, f32_X2, f32_Y1, f32_Y2, f32_Z1, f32_Z2;
    float32_t f32_Distance1, f32_Distance2, f32_Azimuth_deg1, f32_Azimuth_deg2, f32_Elevation_deg1, f32_Elevation_deg2;
    float32_t f32_Omega1, f32_Omega2, f32_Alpha1, f32_Alpha2;
    int32_t s32_Radius, s32_MaxRadius;
    float32_t f32_Angle;
    float32_t f32_LocalX, f32_LocalY;
    float32_t f32_E, f32_N, f32_U;
    float32_t f32_Yaw_deg = st_DeadReckoningDataViewer.st_IMU.f32_Yaw_deg;
    int32_t s32_NearIdx = 0;
    float32_t f32_Dist = 9999.f;

    vector<pair<float32_t, float32_t> > st_BoundLeft;
    vector<pair<float32_t, float32_t> > st_BoundRight;

    CLUSTER_t *pst_Cluster;
    TRACKING_t *pst_Tracking;

    // Road Boundary Points
    glPointSize(0.5); // 점의 크기
    glBegin(GL_POINTS);
    glColor3f(0.f, 0.5f, 1.f);

    for (s32_I = 0;s32_I < st_LidarDataViewer.s32_RoadPointNum; s32_I++)
    {
    	f32_X = st_LidarDataViewer.arst_RoadPoint[s32_I].f32_X;
    	f32_Y = st_LidarDataViewer.arst_RoadPoint[s32_I].f32_Y;
        glVertex2f(f32_X, f32_Y);
    }
    glEnd();

    glPointSize(1.f); // 점의 크기
    glBegin(GL_POINTS);

    for (s32_I = 0;s32_I < st_LidarDataViewer.s32_PointNum;s32_I++) //st_LidarDataViewer.s32_PointNum/4
    {
    	f32_X = st_LidarDataViewer.arst_Point[s32_I].f32_X;
    	f32_Y = st_LidarDataViewer.arst_Point[s32_I].f32_Y;
    	f32_Z = st_LidarDataViewer.arst_Point[s32_I].f32_Z ;

    	if(st_LidarDataViewer.arst_Point[s32_I].u8_Flag == c_POINT_VALID)
        {
            glColor3f(1.f, 1.f, 1.f);
            // LiDARIMU_Calibration(f32_X, f32_Y, f32_Z, 180.-st_DeadReckoningDataViewer.st_IMU.f32_Roll_deg, st_DeadReckoningDataViewer.st_IMU.f32_Pitch_deg, 0);
            glVertex3f(f32_X, f32_Y, f32_Z);


            // printf("%f %f\n", st_DeadReckoningDataViewer.st_IMU.f32_Roll_deg, st_DeadReckoningDataViewer.st_IMU.f32_Pitch_deg);
            // printf("x : %f y : %f\n", f32_X, f32_Y);
        }
    
        else if (b_VisibleLidarGround && st_LidarDataViewer.arst_Point[s32_I].u8_Flag == c_POINT_GROUND)
        {
		    glColor3f(0.f, 1.f, 0.f);
            // LiDARIMU_Calibration(f32_X, f32_Y, f32_Z, -st_DeadReckoningDataViewer.st_IMU.f32_Pitch_deg, st_DeadReckoningDataViewer.st_IMU.f32_Roll_deg, 0);
        	glVertex3f(f32_X, f32_Y, f32_Z);
        }
    }
    glEnd();


    glLineWidth(0.1f); // 점의 크기
    
    if (b_VisibleLidarVoxel)
    {
	    glColor3f(1.f, 0.0f, 0.0f);
	    for (s32_I = 0;s32_I < c_GRID_DISTANCE_SIZE;s32_I++)
	    {
	    	for (s32_J = 0;s32_J < c_GRID_AZIMUTH_SIZE;s32_J++)
	    	{
	    		for (s32_K = 0;s32_K < c_GRID_ELEVATION_SIZE;s32_K++)
	    		{
	    			if (st_LidarDataViewer.st_Voxel.ars32_Grid[s32_I][s32_J][s32_K])
	    			{
                        f32_Distance1 = (float32_t(s32_I) * c_DISTANCE_LEAF_SIZE) + c_VOXEL_DISTANCE_MIN;
						f32_Azimuth_deg1 = (float32_t(s32_J) * c_AZIMUTH_LEAF_SIZE) + c_VOXEL_AZIMUTH_MIN;
						f32_Elevation_deg1 = (float32_t(s32_K) * c_ELEVATION_LEAF_SIZE) + c_VOXEL_ELEVATION_MIN;
                        f32_Distance2 = f32_Distance1 + c_DISTANCE_LEAF_SIZE;
                        f32_Azimuth_deg2 = f32_Azimuth_deg1 + c_AZIMUTH_LEAF_SIZE;
                        f32_Elevation_deg2 = f32_Elevation_deg1 + c_ELEVATION_LEAF_SIZE;

                        f32_Omega1 = deg2rad(f32_Elevation_deg1);
                        f32_Omega2 = deg2rad(f32_Elevation_deg2);
                        f32_Alpha1 = deg2rad(f32_Azimuth_deg1);
                        f32_Alpha2 = deg2rad(f32_Azimuth_deg2);

                        f32_X1 = f32_Distance1 * cosf(f32_Omega1) * sinf(f32_Alpha1);
                        f32_Y1 = f32_Distance1 * cosf(f32_Omega1) * cosf(f32_Alpha1);
                        f32_Z1 = f32_Distance1 * sinf(f32_Omega1);
                        f32_X2 = f32_Distance2 * cosf(f32_Omega2) * sinf(f32_Alpha2);
                        f32_Y2 = f32_Distance2 * cosf(f32_Omega2) * cosf(f32_Alpha2);
                        f32_Z2 = f32_Distance2 * sinf(f32_Omega2);
                        
                        // glBegin(GL_LINE);

                        // 아래
                        glBegin(GL_LINE_LOOP);
                        glVertex3f(f32_X1, f32_Y1, f32_Z1);
                        glVertex3f(f32_X1, f32_Y2, f32_Z1);
                        glVertex3f(f32_X2, f32_Y2, f32_Z1);
                        glVertex3f(f32_X2, f32_Y1, f32_Z1);
                        glEnd();

                        // 위
                        glBegin(GL_LINE_LOOP);
                        glVertex3f(f32_X1, f32_Y1, f32_Z2);
                        glVertex3f(f32_X1, f32_Y2, f32_Z2);
                        glVertex3f(f32_X2, f32_Y2, f32_Z2);
                        glVertex3f(f32_X2, f32_Y1, f32_Z2);
                        glEnd();

                        // 옆면
                        glBegin(GL_LINE_LOOP);
                        glVertex3f(f32_X1, f32_Y1, f32_Z1);
                        glVertex3f(f32_X1, f32_Y2, f32_Z1);
                        glVertex3f(f32_X1, f32_Y2, f32_Z2);
                        glVertex3f(f32_X1, f32_Y1, f32_Z2);
                        glEnd();

                        glBegin(GL_LINE_LOOP);
                        glVertex3f(f32_X2, f32_Y1, f32_Z1);
                        glVertex3f(f32_X2, f32_Y2, f32_Z1);
                        glVertex3f(f32_X2, f32_Y2, f32_Z2);
                        glVertex3f(f32_X2, f32_Y1, f32_Z2);
                        glEnd();
	    			}
	    		}
	    	}
	    }

	}

    if (b_VisibleLidarVoxelCoor)
    {
        s32_MaxRadius = c_GRID_DISTANCE_SIZE; // 최대 반지름
        
        glColor3f(0.5f, 0.3f, 1.f);
        for (s32_Radius = c_DISTANCE_LEAF_SIZE; s32_Radius <= s32_MaxRadius; s32_Radius += c_DISTANCE_LEAF_SIZE)
        {
            glBegin(GL_LINE_LOOP);
            for (s32_I = 0; s32_I < c_GRID_AZIMUTH_SIZE; s32_I++) 
            {
                f32_Angle = deg2rad(s32_I * c_AZIMUTH_LEAF_SIZE);
                f32_X = cosf(f32_Angle) * float32_t(s32_Radius);
                f32_Y = sinf(f32_Angle) * float32_t(s32_Radius);
                glVertex2f(f32_X, f32_Y);
            }
            glEnd();
        }

        for (s32_I = 0; s32_I < c_GRID_AZIMUTH_SIZE; s32_I++)
        {
            f32_Angle = deg2rad(s32_I * c_AZIMUTH_LEAF_SIZE);
            glBegin(GL_LINES);
            glVertex2f(0.0f, 0.0f);
            glVertex2f(cosf(f32_Angle) * float32_t(s32_MaxRadius), sinf(f32_Angle) * float32_t(s32_MaxRadius));
            glEnd();
        }
    }
    
	if (b_VisibleLidarCluster)
	{
        glColor3f(0.f, 1.f, 1.f);
        for(s32_I = 0;s32_I < st_LidarDataViewer.s32_ClusterNum; s32_I++)
	    {
	    	pst_Cluster = &st_LidarDataViewer.arst_Cluster[s32_I];

            if(pst_Cluster->u8_Class == c_CLUSTER_CAR)
            {
                glColor3f(1.f, 1.f, 0.f);
                glRasterPos3f(pst_Cluster->f32_X, pst_Cluster->f32_Y, pst_Cluster->f32_Z);
                for (int8_t s8_Char : "car")
                {
                    glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, s8_Char); // 폰트와 문자 크기 선택하여 출력
                }
            }
            else
            {
                glColor3f(0.f, 1.f, 1.f);
            }

            // 아래
			glBegin(GL_LINE_LOOP);
			glVertex3f(pst_Cluster->f32_MaxX, pst_Cluster->f32_MaxY, pst_Cluster->f32_MinZ);
			glVertex3f(pst_Cluster->f32_MaxX, pst_Cluster->f32_MinY, pst_Cluster->f32_MinZ);
			glVertex3f(pst_Cluster->f32_MinX, pst_Cluster->f32_MinY, pst_Cluster->f32_MinZ);
			glVertex3f(pst_Cluster->f32_MinX, pst_Cluster->f32_MaxY, pst_Cluster->f32_MinZ);
			glEnd();
    
            // 위
            glBegin(GL_LINE_LOOP);
			glVertex3f(pst_Cluster->f32_MaxX, pst_Cluster->f32_MaxY, pst_Cluster->f32_MaxZ);
			glVertex3f(pst_Cluster->f32_MaxX, pst_Cluster->f32_MinY, pst_Cluster->f32_MaxZ);
			glVertex3f(pst_Cluster->f32_MinX, pst_Cluster->f32_MinY, pst_Cluster->f32_MaxZ);
			glVertex3f(pst_Cluster->f32_MinX, pst_Cluster->f32_MaxY, pst_Cluster->f32_MaxZ);
			glEnd();

            // 옆면
            glBegin(GL_LINE_LOOP);
            glVertex3f(pst_Cluster->f32_MinX, pst_Cluster->f32_MaxY, pst_Cluster->f32_MinZ);
			glVertex3f(pst_Cluster->f32_MinX, pst_Cluster->f32_MinY, pst_Cluster->f32_MinZ);
            glVertex3f(pst_Cluster->f32_MinX, pst_Cluster->f32_MinY, pst_Cluster->f32_MaxZ);
            glVertex3f(pst_Cluster->f32_MinX, pst_Cluster->f32_MaxY, pst_Cluster->f32_MaxZ);
            glEnd();

            glBegin(GL_LINE_LOOP);
            glVertex3f(pst_Cluster->f32_MaxX, pst_Cluster->f32_MaxY, pst_Cluster->f32_MinZ);
			glVertex3f(pst_Cluster->f32_MaxX, pst_Cluster->f32_MinY, pst_Cluster->f32_MinZ);
            glVertex3f(pst_Cluster->f32_MaxX, pst_Cluster->f32_MinY, pst_Cluster->f32_MaxZ);
            glVertex3f(pst_Cluster->f32_MaxX, pst_Cluster->f32_MaxY, pst_Cluster->f32_MaxZ);
            glEnd();
        }
	}

    if(b_VisibleLidarTracking)
    {
        pst_Tracking = st_LidarDataViewer.arst_Tracking;

        for(s32_I = 0; s32_I < st_LidarDataViewer.s32_TrackingNum; s32_I++)
        {
            glColor3f(1.f, 0.f, 0.f);

            glBegin(GL_LINE_LOOP);
            glVertex3f(pst_Tracking[s32_I].f32_MaxX, pst_Tracking[s32_I].f32_MaxY, 0);
            glVertex3f(pst_Tracking[s32_I].f32_MaxX, pst_Tracking[s32_I].f32_MinY, 0);
            glVertex3f(pst_Tracking[s32_I].f32_MinX, pst_Tracking[s32_I].f32_MinY, 0);
            glVertex3f(pst_Tracking[s32_I].f32_MinX, pst_Tracking[s32_I].f32_MaxY, 0);
            glEnd();
        }
    }


    lla2enu(st_DeadReckoningDataViewer.st_GPS.f64_Latitude, 
            st_DeadReckoningDataViewer.st_GPS.f64_Longitude,
            0, 
            f32_E, f32_N, f32_U);

    // printf("%f %f %f %f\n", f32_E, f32_N, f32_U, f32_Yaw_deg);

    int32_t s32_LanePointNum = st_PlanningDataViewer.st_ReferenceLine1_global.s32_Num;
    
    PATH_t *pst_Lane[] = {&st_PlanningDataViewer.st_ReferenceLine1_global, 
                          &st_PlanningDataViewer.st_ReferenceLine2_global, 
                          &st_PlanningDataViewer.st_ReferenceLine3_global};

    PATH_t *pst_Boundary[] = {&st_PlanningDataViewer.st_Boundary1_global,
                              &st_PlanningDataViewer.st_Boundary2_global,
                              &st_PlanningDataViewer.st_Boundary3_global,
                              &st_PlanningDataViewer.st_Boundary4_global,
                              &st_PlanningDataViewer.st_Boundary5_global,
                              &st_PlanningDataViewer.st_Boundary6_global};

    // for (s32_K = 0;s32_K < 3;s32_K++)
    // {
    //     glLineWidth(2.f);  
    //     glBegin(GL_LINE_STRIP); 
    //     glColor3f(0.4f, .4f, .4f); 

    //     CalcNearestDistIdx(f32_E, f32_N, 
    //                     pst_Lane[s32_K]->arf32_X,
    //                     pst_Lane[s32_K]->arf32_Y,
    //                     pst_Lane[s32_K]->s32_Num,
    //                     f32_Dist, s32_NearIdx);


    //     for (s32_I = -100;s32_I < 100;s32_I++)
    //     {
    //         s32_J = (s32_NearIdx + s32_I + s32_LanePointNum) % s32_LanePointNum;

    //         f32_X = pst_Lane[s32_K]->arf32_X[s32_J]; 
    //         f32_Y = pst_Lane[s32_K]->arf32_Y[s32_J]; 

    //         getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg - 90), f32_X, f32_Y, f32_LocalX, f32_LocalY);
    //         glVertex3f(f32_LocalX, f32_LocalY, 0.f);
    //     }

    //     glEnd();
    // }

    for (int s32_K : {0, 3})
    {
        if (pst_Boundary[s32_K]->s32_Num == 0)
        {
            continue;
        }

        CalcNearestDistIdx(f32_E, f32_N, 
                        pst_Boundary[s32_K]->arf32_X,
                        pst_Boundary[s32_K]->arf32_Y,
                        pst_Boundary[s32_K]->s32_Num,
                        f32_Dist, s32_NearIdx);

        glLineWidth(2.f);  
        glBegin(GL_LINE_STRIP); 
        glColor3f(0.3f, 0.8f, 0.6f); 

        for (s32_I = -100; s32_I <= 100; s32_I += 10)
        {
            s32_J = (s32_NearIdx + s32_I + pst_Boundary[s32_K]->s32_Num) % pst_Boundary[s32_K]->s32_Num;

            f32_X = pst_Boundary[s32_K]->arf32_X[s32_J]; 
            f32_Y = pst_Boundary[s32_K]->arf32_Y[s32_J]; 

            getLocalCoord(f32_E, f32_N, deg2rad(f32_Yaw_deg - 90), f32_X, f32_Y, f32_LocalX, f32_LocalY);
            glVertex3f(f32_LocalX, f32_LocalY, 0.f);

            if (s32_K == 3)
            {
                st_BoundLeft.push_back(make_pair(f32_LocalX, f32_LocalY));
            }
            else
            {
                st_BoundRight.push_back(make_pair(f32_LocalX, f32_LocalY));
            }
        }

        glEnd();
    }


    glPointSize(4.f);
    glBegin(GL_POINTS);
    glColor3f(1.f, 0.f, 0.f);

    for (s32_I = 0;s32_I < c_OMNI_FEATURE_MAX_NUM;s32_I++)
    {
        if (st_LidarDataViewer.st_OmniFeature.arb_Valid[s32_I] == false)
        {
            continue;
        }

        f32_X = st_LidarDataViewer.st_OmniFeature.arf32_X[s32_I];
        f32_Y = st_LidarDataViewer.st_OmniFeature.arf32_Y[s32_I];

        glVertex3f(f32_X, f32_Y, 5.f);
    }

    glEnd();
}



void DrawGlobalMap(PLANNING_DATA_t *pst_PlanningData)
{
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;
    FRENET_PARAM_t *pst_FrenetParam = &pst_PlanningData->st_FrenetParam;
    FRENET_PATH_t *pst_FrenetPath = pst_PlanningData->arst_FrenetPath_global;
    FRENET_PATH_t *pst_CollisionCheckPath = pst_PlanningData->arst_CollisonCheckPath_global;
    VEHICLE_DATA_t *pst_ObjectData = pst_PlanningData->arst_ObjectData;
    VEHICLE_DATA_t *pst_CarData = pst_PlanningData->arst_CarData;
    PATH_t *pst_FinalPath = &pst_PlanningData->st_FinalPath_global;
    PLANNER_t *pst_Planner = &pst_PlanningData->st_Planner;


    int32_t s32_I;
    int32_t s32_J;
    int32_t s32_K;

    const char* buttonLabels[] = {"Go", "Stop", "Slow On", "Slow Off", "PitStop"};
    const float32_t positions[] = {0.05f, 0.12f, 0.19f, 0.26f, 0.33f};
    const uint8_t states[] = {st_CANDataViewer.u8_SIG_GO,
                           st_CANDataViewer.u8_SIG_STOP,
                           st_CANDataViewer.u8_SIG_SLOW_ON,
                           st_CANDataViewer.u8_SIG_SLOW_OFF,
                           st_CANDataViewer.u8_SIG_PIT_STOP};

    float32_t y = st_ViewerParam.s32_ScreenHeight * 0.360;
    float32_t height = st_ViewerParam.s32_ScreenHeight * 0.02;
    float32_t width = st_ViewerParam.s32_ScreenWidth * 0.07;

    for (s32_I = 0;s32_I < 5;s32_I++)
    {
        DrawButton(y, width, height, buttonLabels[s32_I], st_ViewerParam.s32_ScreenWidth * positions[s32_I]);
        if (states[s32_I])
        {
            DrawButtonColor(y, width, height, buttonLabels[s32_I], 1.0f, 0.0f, 0.0f, st_ViewerParam.s32_ScreenWidth * positions[s32_I]);
        }
    }

    if(st_PlanningDataViewer.s32_Round == 0)
    {
            DrawButton(st_ViewerParam.s32_ScreenHeight * 0.99, st_ViewerParam.s32_ScreenWidth * 0.1, st_ViewerParam.s32_ScreenHeight * 0.02, "Round: TEST", st_ViewerParam.s32_ScreenWidth * 0.05f );
    }
    else if(st_PlanningDataViewer.s32_Round == 1)
    {
            DrawButton(st_ViewerParam.s32_ScreenHeight * 0.99, st_ViewerParam.s32_ScreenWidth * 0.1, st_ViewerParam.s32_ScreenHeight * 0.02, "Round: PRETEST1", st_ViewerParam.s32_ScreenWidth * 0.05f );
    }
    else if(st_PlanningDataViewer.s32_Round == 2)
    {
            DrawButton(st_ViewerParam.s32_ScreenHeight * 0.99, st_ViewerParam.s32_ScreenWidth * 0.1, st_ViewerParam.s32_ScreenHeight * 0.02, "Round: PRETEST2", st_ViewerParam.s32_ScreenWidth * 0.05f );
    }
    else if(st_PlanningDataViewer.s32_Round == 3)
    {
            DrawButton(st_ViewerParam.s32_ScreenHeight * 0.99, st_ViewerParam.s32_ScreenWidth * 0.1, st_ViewerParam.s32_ScreenHeight * 0.02, "Round: PRE1", st_ViewerParam.s32_ScreenWidth * 0.05f );
    }
    else if(st_PlanningDataViewer.s32_Round == 4)
    {
            DrawButton(st_ViewerParam.s32_ScreenHeight * 0.99, st_ViewerParam.s32_ScreenWidth * 0.1, st_ViewerParam.s32_ScreenHeight * 0.02, "Round: PRE2", st_ViewerParam.s32_ScreenWidth * 0.05f );
    }
    else if(st_PlanningDataViewer.s32_Round == 5)
    {
            DrawButton(st_ViewerParam.s32_ScreenHeight * 0.99, st_ViewerParam.s32_ScreenWidth * 0.1, st_ViewerParam.s32_ScreenHeight * 0.02, "Round: FINAL", st_ViewerParam.s32_ScreenWidth * 0.05f );
    }
    DrawButton(st_ViewerParam.s32_ScreenHeight * 0.99, st_ViewerParam.s32_ScreenWidth * 0.1, st_ViewerParam.s32_ScreenHeight * 0.02, GetVehicleStateString(st_VehicleState), st_ViewerParam.s32_ScreenWidth * 0.15f );

    DrawButton(st_ViewerParam.s32_ScreenHeight * 0.99, st_ViewerParam.s32_ScreenWidth * 0.1, st_ViewerParam.s32_ScreenHeight * 0.02, GetBankStateString(&st_PlanningDataViewer), st_ViewerParam.s32_ScreenWidth * 0.25f );

    DrawButton(
    st_ViewerParam.s32_ScreenHeight * 0.99,
    st_ViewerParam.s32_ScreenWidth * 0.1,
    st_ViewerParam.s32_ScreenHeight * 0.02,
    ("EgoLane: " + to_string(st_PlanningDataViewer.st_EgoVehicleData.s32_CurrentLane)).c_str(),
    st_ViewerParam.s32_ScreenWidth * 0.35f
    );


    InitGlobalMap(pst_PlanningData);

    float32_t f32_OmniX, f32_OmniY;
    float32_t f32_OmniYaw_deg;
    float32_t f32_OmniYaw_rad;
    float32_t f32_X, f32_Y;
    float32_t f32_RX, f32_RY;

    float32_t f32_E, f32_N, f32_U;
    float32_t f32_ForwardN, f32_ForwardE;
    float32_t f32_EKF_N, f32_EKF_E, f32_EKF_U;
    float32_t f32_Heading_rad;
    // f32_E = st_HMCLidarDataViewer.f32_PredictX;
    // f32_N = st_HMCLidarDataViewer.f32_PredictY;
    // f32_U = st_HMCLidarDataViewer.f32_PredictZ;
    // f32_Heading_rad = deg2rad(-st_HMCLidarDataViewer.f32_PredictYaw_deg + 90.f);

    // f32_E = pst_EgoVehicleData->f32_X_global;
    // f32_N = pst_EgoVehicleData->f32_Y_global;
    // f32_U = pst_EgoVehicleData->f32_Z_global;

    lla2enu(st_CANDataViewer.f32_PITZONE_LAT, st_CANDataViewer.f32_PITZONE_LONG, 0.f, f32_E, f32_N, f32_U);
    drawFlag(f32_E, f32_N);

    lla2enu(st_DeadReckoningDataViewer.st_GPS.f64_Latitude,
            st_DeadReckoningDataViewer.st_GPS.f64_Longitude,
            0,
            f32_E, f32_N, f32_U);

    st_Trajectory.f32_GPSTrajectoryX[st_Trajectory.s32_GPSTrajectoryCount] = f32_E;
    st_Trajectory.f32_GPSTrajectoryY[st_Trajectory.s32_GPSTrajectoryCount] = f32_N;
    st_Trajectory.f32_GPSTrajectoryHeadingX[st_Trajectory.s32_GPSTrajectoryCount] = f32_E + 0.5 * cos(axisRotate(st_DeadReckoningDataViewer.st_IMU.f32_Yaw_rad));
    st_Trajectory.f32_GPSTrajectoryHeadingY[st_Trajectory.s32_GPSTrajectoryCount] = f32_N + 0.5 * sin(axisRotate(st_DeadReckoningDataViewer.st_IMU.f32_Yaw_rad));
    st_Trajectory.s32_GPSTrajectoryCount++;

    f32_Heading_rad = axisRotate(st_DeadReckoningDataViewer.st_IMU.f32_Yaw_rad);

    // InitEgoBox(f32_E, f32_N, f32_U);

    // True GPS
    glColor3f(0.f, 0.f, 1.f);
    drawCircle(f32_E, f32_N, 2.5f, 36, 1);

    

    // // Odometry
    glPointSize(15.f);
    glBegin(GL_POINTS);
    glColor3f(1.f,0.f,0.f);
    glVertex3f(st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictX, st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictY, 5.f);
    glEnd();



    glLineWidth(4.f);
    glBegin(GL_LINES);
    glColor3f(1.f,0.f,0.f);
    float32_t f32_PX = st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictX + 0.5 *cos(st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictYaw_rad);
    float32_t f32_PY = st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictY + 0.5 *sin(st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictYaw_rad);
    glVertex3f(st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictX, st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictY, 5.f);
    glVertex3f(f32_PX, f32_PY, 5.f);
    glEnd();

    st_Trajectory.f32_OdomTrajectoryX[st_Trajectory.s32_OdomTrajectoryCount] = st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictX;
    st_Trajectory.f32_OdomTrajectoryY[st_Trajectory.s32_OdomTrajectoryCount] = st_DeadReckoningDataViewer.st_Vehicle_Odometry.f32_PredictY;
    st_Trajectory.f32_OdomTrajectoryHeadingX[st_Trajectory.s32_OdomTrajectoryCount] = f32_PX;
    st_Trajectory.f32_OdomTrajectoryHeadingY[st_Trajectory.s32_OdomTrajectoryCount] = f32_PY;
    st_Trajectory.s32_OdomTrajectoryCount++;


    // Ego Box
    // glBegin(GL_QUADS);
    // glColor3f(0.53f, 0.81f, 0.92f);
    // for (s32_I = 0;s32_I < 4;s32_I++) 
    // {
    //     glVertex3f(pst_EgoVehicleData->f32_Vertex[s32_I][0], pst_EgoVehicleData->f32_Vertex[s32_I][1], 1.0f);
    // }
    // glEnd();

    // glColor3f(0.f, 1.f, 0.f);
    // DrawPosition(f32_E, f32_N, f32_Heading_rad);

    // Draw OmniPoint
    // DrawOmni(&st_HMCLidarDataViewer);
    DrawICP(&st_HMCLidarDataViewer);



    // //Odom Trajectory
    // glColor3f(1.f,0.f,0.f);
    // for(s32_I = 0; s32_I < st_Trajectory.s32_OdomTrajectoryCount; s32_I++)
    // {
    //     glPointSize(7.f);
    //     glBegin(GL_POINTS);
    //     glVertex3f(st_Trajectory.f32_OdomTrajectoryX[s32_I], st_Trajectory.f32_OdomTrajectoryY[s32_I], 7.f);
    //     glEnd();

    //     glLineWidth(1.f);
    //     glBegin(GL_LINES);
    //     glVertex3f(st_Trajectory.f32_OdomTrajectoryX[s32_I], st_Trajectory.f32_OdomTrajectoryY[s32_I], 7.f);
    //     glVertex3f(st_Trajectory.f32_OdomTrajectoryHeadingX[s32_I], st_Trajectory.f32_OdomTrajectoryHeadingY[s32_I], 7.f);
    //     glEnd();
    // }

    // // ICP Trajectory
    // glColor3f(1.f, 1.f, 0.f);
    // for(s32_I = 0; s32_I < st_Trajectory.s32_ICPTrajectoryCount; s32_I++)
    // {
    //     glPointSize(7.f);
    //     glBegin(GL_POINTS);
    //     glVertex3f(st_Trajectory.f32_ICPTrajectoryX[s32_I], st_Trajectory.f32_ICPTrajectoryY[s32_I], 5.f);
    //     glEnd();

    //     glLineWidth(1.f);
    //     glBegin(GL_LINES);
    //     glVertex3f(st_Trajectory.f32_ICPTrajectoryX[s32_I], st_Trajectory.f32_ICPTrajectoryY[s32_I], 5.f);
    //     glVertex3f(st_Trajectory.f32_ICPTrajectoryHeadingX[s32_I], st_Trajectory.f32_ICPTrajectoryHeadingY[s32_I], 5.f);
    //     glEnd();
    // }

    // // GPS(GT) Trajectory
    // glColor3f(0.f, 1.f, 0.f);
    // for(s32_I = 0; s32_I < st_Trajectory.s32_GPSTrajectoryCount; s32_I++)
    // {
    //     glPointSize(7.f);
    //     glBegin(GL_POINTS);
    //     glVertex3f(st_Trajectory.f32_GPSTrajectoryX[s32_I], st_Trajectory.f32_GPSTrajectoryY[s32_I], 5.f);
    //     glEnd();

    //     glLineWidth(1.f);
    //     glBegin(GL_LINES);
    //     glVertex3f(st_Trajectory.f32_GPSTrajectoryX[s32_I], st_Trajectory.f32_GPSTrajectoryY[s32_I], 5.f);
    //     glVertex3f(st_Trajectory.f32_GPSTrajectoryHeadingX[s32_I], st_Trajectory.f32_GPSTrajectoryHeadingY[s32_I], 5.f);
    //     glEnd();
    // }
    if(st_Trajectory.s32_OdomTrajectoryCount == 9999)
    {
        st_Trajectory.s32_OdomTrajectoryCount = 0;
        st_Trajectory.s32_ICPTrajectoryCount = 0;
        st_Trajectory.s32_GPSTrajectoryCount = 0;
    }


    // Path
    PATH_t *pst_Lane[] = {&pst_PlanningData->st_ReferenceLine1_global, 
                          &pst_PlanningData->st_ReferenceLine2_global, 
                          &pst_PlanningData->st_ReferenceLine3_global,
                          &pst_PlanningData->st_ReferenceSELine1_global,
                          &pst_PlanningData->st_ReferenceSELine2_global,
                          &pst_PlanningData->st_ReferenceSELine3_global,
                          &pst_PlanningData->st_IDX_global
                        };

    PATH_t *pst_Boundary[] = {&pst_PlanningData->st_Boundary1_global,
                              &pst_PlanningData->st_Boundary2_global,
                              &pst_PlanningData->st_Boundary3_global,
                              &pst_PlanningData->st_Boundary4_global,
                              &pst_PlanningData->st_SEBoundary1_global,
                              &pst_PlanningData->st_SEBoundary2_global,
                              &pst_PlanningData->st_SEBoundary3_global,
                              &pst_PlanningData->st_SEBoundary4_global
                              };

    for (auto s32_Index : {0, 1, 2, 3, 4, 5, 6})
    {
        glLineWidth(1.f);
        glBegin(GL_LINE_LOOP); 
        glColor3f(0.4f, 0.4f, 0.4f);

        for (s32_I = 0; s32_I < pst_Lane[s32_Index]->s32_Num; s32_I++)
        {
            f32_X = pst_Lane[s32_Index]->arf32_X[s32_I];
            f32_Y = pst_Lane[s32_Index]->arf32_Y[s32_I];
            if(s32_Index != 6)
            {
                glVertex3f(f32_X, f32_Y, 0.f);
            }
            else if(s32_Index == 6)
            {
                drawFlag(f32_X, f32_Y,"G");
            }
        }
        glEnd();
    }


    for (auto s32_Index : {0, 1, 2, 3, 4, 5, 6, 7})
    {
        glLineWidth(3.f);
        glBegin(GL_LINE_LOOP); 
        glColor3f(0.8f, 0.8f, 0.8f);
        for (s32_I = 0;s32_I < pst_Boundary[s32_Index]->s32_Num;s32_I++)
        {
            f32_X = pst_Boundary[s32_Index]->arf32_X[s32_I]; 
            f32_Y = pst_Boundary[s32_Index]->arf32_Y[s32_I]; 
            glVertex3f(f32_X, f32_Y, 0.f);
        }
        glEnd();        
    }

    // Key "A"
    if (st_ViewerParam.b_AccToggle)
    {
        for (s32_I = 20;s32_I < pst_PlanningData->arst_FrenetPath_global[pst_PlanningData->s32_BestPath].s32_Num;s32_I++)
        {
            float32_t f32_Vertex[4][2];
            CalcVertex(pst_PlanningData->arst_FrenetPath_global[pst_PlanningData->s32_BestPath].arf32_X[s32_I],
                       pst_PlanningData->arst_FrenetPath_global[pst_PlanningData->s32_BestPath].arf32_Y[s32_I],
                       pst_PlanningData->st_EgoVehicleData.f32_MaxX * 1.f,
                       pst_PlanningData->st_EgoVehicleData.f32_MaxY * 1.f * 1.7f,
                       pst_PlanningData->st_EgoVehicleData.f32_MinX * 1.f,
                       pst_PlanningData->st_EgoVehicleData.f32_MinY * 1.f * 1.7f,
                       pst_PlanningData->arst_FrenetPath_global[pst_PlanningData->s32_BestPath].arf32_Yaw_rad_ENU[s32_I], f32_Vertex,
                       0);
            glLineWidth(0.1f);
            glColor3f(0.1f, 0.9f, 0.9f);
            glBegin(GL_LINE_LOOP);
            for (s32_K = 0;s32_K < 4;s32_K++)
            {
                glVertex3f(f32_Vertex[s32_K][0], f32_Vertex[s32_K][1], 1.5f);
            }
            glEnd();
        }
    }


    // Key "S"
    if (st_ViewerParam.b_SatToggle)
    {
        int32_t s32_PathNum = pst_PlanningData->st_FrenetParam.s32_PathNum;
        float32_t f32_Ratio = pst_FrenetParam->f32_Ratio;

        for (s32_I = 0; s32_I < s32_PathNum; s32_I++)
        {
            for (s32_J = 20; s32_J < pst_FrenetPath[s32_I].s32_Num; s32_J++)
            {
                    float32_t f32_Vertex[4][2];
                    CalcVertex(
                        pst_FrenetPath[s32_I].arf32_X[s32_J],
                        pst_FrenetPath[s32_I].arf32_Y[s32_J],
                        pst_EgoVehicleData->f32_MaxX * 1.f,
                        pst_EgoVehicleData->f32_MaxY * 1.f * f32_Ratio,
                        pst_EgoVehicleData->f32_MinX * 1.f,
                        pst_EgoVehicleData->f32_MinY * 1.f * f32_Ratio,
                        pst_FrenetPath[s32_I].arf32_Yaw_rad_ENU[s32_J],
                        f32_Vertex, 0
                    );

                    int32_t isCollision = 0;

                    for (int32_t s32_M = 0; s32_M < pst_FrenetPath[s32_I].s32_CollisionNum; s32_M++) {
                        if (s32_J == pst_FrenetPath[s32_I].ars32_Collision[s32_M]) {
                            isCollision = 1;
                            break;
                        }
                    }

                    if (isCollision)
                    {
                        glLineWidth(0.1f);
                        glColor3f(1.f, 0.f, 0.f);
                        glBegin(GL_QUADS);
                        for (s32_K = 0; s32_K < 4; s32_K++) {
                            glVertex3f(f32_Vertex[s32_K][0], f32_Vertex[s32_K][1], 1.0f);
                        }
                        glEnd();

                        glLineWidth(0.1f);
                        glColor3f(0.f, 0.f, 0.f);
                        glBegin(GL_LINE_LOOP);
                        for (s32_K = 0; s32_K < 4; s32_K++) {
                            glVertex3f(f32_Vertex[s32_K][0], f32_Vertex[s32_K][1], 1.5f);
                        }
                        glEnd();
                    }
                    else
                    {
                        glLineWidth(0.1f);
                        glColor3f(0.f, 1.f, 0.f);
                        glBegin(GL_LINE_LOOP);
                        for (s32_K = 0; s32_K < 4; s32_K++) {
                            glVertex3f(f32_Vertex[s32_K][0], f32_Vertex[s32_K][1], 1.0f);
                        }
                        glEnd();
                    }
            }
        }
        glEnd();

        for (s32_I = 0; s32_I < s32_PathNum; s32_I++)
        {
            for (s32_J = 0; s32_J < pst_CollisionCheckPath[s32_I].s32_Num; s32_J++)
            {
                    float32_t f32_Vertex[4][2];
                    CalcVertex(
                        pst_CollisionCheckPath[s32_I].arf32_X[s32_J],
                        pst_CollisionCheckPath[s32_I].arf32_Y[s32_J],
                        pst_EgoVehicleData->f32_MaxX * 1.f,
                        pst_EgoVehicleData->f32_MaxY * 1.f * f32_Ratio,
                        pst_EgoVehicleData->f32_MinX * 1.f,
                        pst_EgoVehicleData->f32_MinY * 1.f * f32_Ratio,
                        pst_CollisionCheckPath[s32_I].arf32_Yaw_rad_ENU[s32_J],
                        f32_Vertex, 0
                    );

                    int32_t isCollision = 0;

                    for (int32_t s32_M = 0; s32_M < pst_CollisionCheckPath[s32_I].s32_CollisionNum; s32_M++)
                    {
                        if (s32_J == pst_CollisionCheckPath[s32_I].ars32_Collision[s32_M])
                        {
                            isCollision = 1;
                            break;
                        }
                    }

                    if (isCollision)
                    {
                        glLineWidth(0.1f);
                        glColor3f(1.f, 0.f, 0.f);
                        glBegin(GL_QUADS);
                        for (s32_K = 0; s32_K < 4; s32_K++) {
                            glVertex3f(f32_Vertex[s32_K][0], f32_Vertex[s32_K][1], 1.0f);
                        }
                        glEnd();

                        glLineWidth(0.1f);
                        glColor3f(0.f, 0.f, 0.f);
                        glBegin(GL_LINE_LOOP);
                        for (s32_K = 0; s32_K < 4; s32_K++) {
                            glVertex3f(f32_Vertex[s32_K][0], f32_Vertex[s32_K][1], 1.5f);
                        }
                        glEnd();
                    }
                    else
                    {
                        glLineWidth(0.1f);
                        glColor3f(0.f, 1.f, 0.f);
                        glBegin(GL_LINE_LOOP);
                        for (s32_K = 0; s32_K < 4; s32_K++) {
                            glVertex3f(f32_Vertex[s32_K][0], f32_Vertex[s32_K][1], 1.0f);
                        }
                        glEnd();
                    }
            }
        }
        glEnd();
    }

    // Key "D"
    if (st_ViewerParam.b_VertexToggle)
    {

        int32_t s32_PathNum = pst_CollisionCheckPath->s32_Num;
        float32_t f32_Ratio = pst_FrenetParam->f32_Ratio;

        for (s32_I = 0;s32_I < s32_PathNum;s32_I++)
        {
            glLineWidth(5.0f);
            glBegin(GL_LINE_STRIP);
            if (pst_Planner->ars32_AvailGridPath[s32_I] == 1)
            {
                glColor3f(1.f, 1.f, 1.f);
            }
            else
            {
                glColor3f(1.f, 0.f, 0.1f);
            }
            for (s32_J = 0; s32_J < pst_CollisionCheckPath[s32_I].s32_Num; s32_J++)
            {
                f32_X = pst_CollisionCheckPath[s32_I].arf32_X[s32_J];
                f32_Y = pst_CollisionCheckPath[s32_I].arf32_Y[s32_J];

                glVertex3f(f32_X, f32_Y, 1.f);
            }
            glEnd();
        }
        for (s32_I = 0;s32_I < s32_PathNum;s32_I++)
        {
            glLineWidth(5.0f);
            glBegin(GL_LINE_STRIP);
            if (pst_Planner->ars32_AvailFrenetPath[s32_I] == 1)
            {
                glColor3f(1.f, 1.f, 1.f);
            }
            else
            {
                glColor3f(1.f, 0.f, 0.1f);
            }
            for (s32_J = 0; s32_J < pst_FrenetPath[s32_I].s32_Num; s32_J++)
            {
                f32_X = pst_FrenetPath[s32_I].arf32_X[s32_J];
                f32_Y = pst_FrenetPath[s32_I].arf32_Y[s32_J];

                glVertex3f(f32_X, f32_Y, 1.f);
            }
            glEnd();
        }


    }


    // Key "F"
    if (st_ViewerParam.b_FrenetToggle)
    {
        glLineWidth(1.0f);
        glColor3f(0.f, 1.f, 1.f);
        for (s32_I = 0; s32_I < pst_PlanningData->s32_FrenetPathNum; s32_I++)
        {
            glBegin(GL_LINE_STRIP);
            for (s32_J = 0; s32_J < pst_FrenetPath[s32_I].s32_Num; s32_J++)
            {
                f32_X = pst_FrenetPath[s32_I].arf32_X[s32_J];
                f32_Y = pst_FrenetPath[s32_I].arf32_Y[s32_J];
                glVertex3f(f32_X, f32_Y, 2.f);
            }
            glEnd();

        }
    }

    // Final Path
    glLineWidth(5.0f);
    glBegin(GL_LINE_STRIP); 
    glColor3f(0.2f, 0.4f, 0.1f); 
    for (s32_J = 0; s32_J < pst_FinalPath->s32_Num; s32_J++)
    {
        f32_X = pst_FinalPath->arf32_X[s32_J]; 
        f32_Y = pst_FinalPath->arf32_Y[s32_J];

        glVertex3f(f32_X, f32_Y, 1.f);
    }
    glEnd();

    // Object Data Box
    for (s32_I = 0; s32_I < pst_PlanningData->s32_ObjectNum; s32_I++)
    {
        glBegin(GL_QUADS);
        if (s32_I == pst_PlanningData->s32_AccObjIdx)
        {
            glColor3f(1.f,0.f,0.f);
        }
        else
        {
            glColor3f(0.f,1.f,0.f);
        }

        for (s32_J = 0;s32_J < 4;s32_J++)
        {
            glVertex3f(pst_ObjectData[s32_I].f32_Vertex[s32_J][0], pst_ObjectData[s32_I].f32_Vertex[s32_J][1], 1.0f);
        }
        glEnd();
    }

    // Car Data Circle
    glColor3f(0.f, 0.f, 1.f);
    for (s32_I = 0; s32_I < pst_PlanningData->s32_CarNum; ++s32_I)
    {
        float32_t x = pst_CarData[s32_I].f32_X_global;
        float32_t y = pst_CarData[s32_I].f32_Y_global;
        drawCircle(x, y, 5, 36, 1);
    }
}



void DrawPosition(float32_t f32_E, float32_t f32_N, float32_t f32_Heading_rad)
{
    float32_t f32_ForwardN;
    float32_t f32_ForwardE;

    glPointSize(15.f);
    glBegin(GL_POINTS);
    glVertex3f(f32_E, f32_N, 5.f);
    glEnd();
    f32_ForwardE = f32_E + 0.5 * cosf(f32_Heading_rad);
    f32_ForwardN = f32_N + 0.5 * sinf(f32_Heading_rad);
    st_Trajectory.f32_ICPTrajectoryX[st_Trajectory.s32_ICPTrajectoryCount] = f32_E;
    st_Trajectory.f32_ICPTrajectoryY[st_Trajectory.s32_ICPTrajectoryCount] = f32_N;
    st_Trajectory.f32_ICPTrajectoryHeadingX[st_Trajectory.s32_ICPTrajectoryCount] = f32_ForwardE;
    st_Trajectory.f32_ICPTrajectoryHeadingY[st_Trajectory.s32_ICPTrajectoryCount] = f32_ForwardN;
    st_Trajectory.s32_ICPTrajectoryCount++;

    f32_ForwardE = f32_E + 20 * cosf(f32_Heading_rad);
    f32_ForwardN = f32_N + 20 * sinf(f32_Heading_rad);
    glLineWidth(3.f);
    glBegin(GL_LINES);
    glVertex3f(f32_E, f32_N, 5.f);
    glVertex3f(f32_ForwardE, f32_ForwardN, 5.f);
    glEnd();
}

void DrawOmni(HMC_LIDAR_DATA_t* pst_LidarData)
{
    int32_t s32_I, s32_J;
    float32_t f32_OmniX, f32_OmniY, f32_OmniYaw_rad, f32_OmniYaw_deg;
    float32_t f32_X, f32_Y, f32_RX, f32_RY;

    glColor3f(1.f, 1.f, 0.f);
    DrawPosition(pst_LidarData->f32_PredictX, pst_LidarData->f32_PredictY, deg2rad(-pst_LidarData->f32_PredictYaw_deg + 90));

    for (s32_I = 0;s32_I < st_OmniPoint.s32_Num;s32_I++)
    {
        f32_OmniX = st_OmniPoint.arf32_PositionX[s32_I];
        f32_OmniY = st_OmniPoint.arf32_PositionY[s32_I];
        f32_OmniYaw_deg = st_OmniPoint.arf32_PositionYaw_deg[s32_I];
        f32_OmniYaw_rad = deg2rad(-f32_OmniYaw_deg + 90);

        glPointSize(5.f);
        glBegin(GL_POINTS);
        glColor3f(0.2f, 0.2f, 0.2f);
        glVertex3f(f32_OmniX, f32_OmniY, 0.f);
        glEnd();


        glPointSize(1.f);
        glBegin(GL_POINTS);
        glColor3f(0.5f, 0.4f, 1.0f);

        for (s32_J = 0;s32_J < c_OMNI_FEATURE_MAX_NUM;s32_J++)
        {
            if (st_OmniPoint.arst_Feature[s32_I].arb_Valid[s32_J] == false)
            {
                continue;
            }

            f32_X = st_OmniPoint.arst_Feature[s32_I].arf32_X[s32_J];
            f32_Y = st_OmniPoint.arst_Feature[s32_I].arf32_Y[s32_J];

            getGlobalCoord(f32_OmniX, f32_OmniY, f32_OmniYaw_rad, f32_X, f32_Y, f32_RX, f32_RY);
            glVertex3f(f32_RX, f32_RY, 0.f);
        }

        glEnd();
    }

    for (s32_I = 0;s32_I < 5;s32_I++)
    {
        s32_J = pst_LidarData->ars32_CandidateIndex[s32_I];
        f32_X = st_OmniPoint.arf32_PositionX[s32_J];
        f32_Y = st_OmniPoint.arf32_PositionY[s32_J];

        glPointSize(8.f);
        glBegin(GL_POINTS);
        glColor3f(0.9f, 0.5f, 0.5f);
        glVertex3f(f32_X, f32_Y, 0.f);
        glEnd();

    }

    s32_J = pst_LidarData->s32_BestCandidateIndex;
    f32_X = st_OmniPoint.arf32_PositionX[s32_J];
    f32_Y = st_OmniPoint.arf32_PositionY[s32_J];
    glPointSize(8.f);
    glBegin(GL_POINTS);
    glColor3f(0.9f, 0.2f, 0.1f);
    glVertex3f(f32_X, f32_Y, 1.f);
    glEnd();

}



void DrawICP(HMC_LIDAR_DATA_t* pst_LidarPointData)
{
    int32_t s32_I, s32_J;
    float32_t f32_ICPX, f32_ICPY, f32_ICPZ, f32_ICPYaw_deg, f32_ICPYaw_rad;
    float32_t f32_X, f32_Y, f32_Z, f32_RX, f32_RY, f32_RZ, f32_GX, f32_GY;
    float32_t f32_Yaw_rad;
    float32_t f32_EgoYaw_rad;

    int32_t s32_NearIdx = pst_LidarPointData->s32_ICPNearestMapIdx;
    float32_t *pf32_Transform = pst_LidarPointData->arf32_ICPTransform;

    glColor3f(1.f, 1.f, 0.f);
    DrawPosition(pst_LidarPointData->f32_PredictX, pst_LidarPointData->f32_PredictY, deg2rad(-pst_LidarPointData->f32_PredictYaw_deg + 90));

    if (st_ICPMap.s32_MapNum > 0)
    {
        if (st_ViewerParam.s32_State & c_STATE_MODE_RECORD_MAP)
        {
            // printf("%d\n", st_ICPMap.s32_MapNum);
            for (s32_I = 0;s32_I < st_ICPMap.s32_MapNum;s32_I++)
            {
                f32_ICPX = st_ICPMap.arst_PointCloud[s32_I].f32_X;
                f32_ICPY = st_ICPMap.arst_PointCloud[s32_I].f32_Y;
                f32_ICPZ = st_ICPMap.arst_PointCloud[s32_I].f32_Z;
                f32_ICPYaw_deg = st_ICPMap.arst_PointCloud[s32_I].f32_Yaw_deg;
                f32_ICPYaw_rad = deg2rad(-f32_ICPYaw_deg + 90);

                glPointSize(5.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.2f, 0.2f);
                glVertex3f(f32_ICPX, f32_ICPY, f32_ICPZ);
                glEnd();


                glPointSize(1.f);
                glBegin(GL_POINTS);
                glColor3f(0.5f, 0.4f, 1.0f);

                for (s32_J = 0;s32_J < st_ICPMap.arst_PointCloud[s32_I].s32_PointNum;s32_J += 10)
                {
                    f32_X = st_ICPMap.arst_PointCloud[s32_I].arf32_X[s32_J];
                    f32_Y = st_ICPMap.arst_PointCloud[s32_I].arf32_Y[s32_J];
                    f32_Z = st_ICPMap.arst_PointCloud[s32_I].arf32_Z[s32_J];

                    getGlobalCoord(f32_ICPX, f32_ICPY, f32_ICPYaw_rad, f32_X, f32_Y, f32_RX, f32_RY);
                    glVertex3f(f32_RX, f32_RY, f32_Z);
                }

                glEnd();
            }

            b_DrawICPInit = false;
        }
        else
        {
            if (b_DrawICPInit == false)
            {
                st_ICPPoint.clear();

                for (s32_I = 0;s32_I < st_ICPMap.s32_MapNum;s32_I++)
                {
                    f32_ICPX = st_ICPMap.arst_PointCloud[s32_I].f32_X;
                    f32_ICPY = st_ICPMap.arst_PointCloud[s32_I].f32_Y;
                    f32_ICPZ = st_ICPMap.arst_PointCloud[s32_I].f32_Z;
                    f32_ICPYaw_deg = st_ICPMap.arst_PointCloud[s32_I].f32_Yaw_deg;
                    f32_ICPYaw_rad = deg2rad(-f32_ICPYaw_deg + 90);

                    for (s32_J = 0;s32_J < st_ICPMap.arst_PointCloud[s32_I].s32_PointNum;s32_J += 20)
                    {
                        f32_X = st_ICPMap.arst_PointCloud[s32_I].arf32_X[s32_J];
                        f32_Y = st_ICPMap.arst_PointCloud[s32_I].arf32_Y[s32_J];
                        f32_Z = st_ICPMap.arst_PointCloud[s32_I].arf32_Z[s32_J];

                        getGlobalCoord(f32_ICPX, f32_ICPY, f32_ICPYaw_rad, f32_X, f32_Y, f32_RX, f32_RY);

                        st_ICPPoint.push_back(std::make_tuple(f32_RX, f32_RY, f32_Z));
                    }
                }

                b_DrawICPInit = true;
            }

            for (s32_I = 0;s32_I < st_ICPMap.s32_MapNum;s32_I++)
            {
                f32_ICPX = st_ICPMap.arst_PointCloud[s32_I].f32_X;
                f32_ICPY = st_ICPMap.arst_PointCloud[s32_I].f32_Y;
                f32_ICPZ = st_ICPMap.arst_PointCloud[s32_I].f32_Z;
                f32_ICPYaw_deg = st_ICPMap.arst_PointCloud[s32_I].f32_Yaw_deg;
                f32_ICPYaw_rad = deg2rad(-f32_ICPYaw_deg + 90);

                glPointSize(5.f);
                glBegin(GL_POINTS);
                glColor3f(0.5f, 0.8f, 0.2f);
                glVertex3f(f32_ICPX, f32_ICPY, f32_ICPZ);
                glEnd();
            }

            glPointSize(1.f);
            glBegin(GL_POINTS);
            glColor3f(0.5f, 0.4f, 1.0f);
            for (auto &st_Point : st_ICPPoint)
            {
                f32_X = std::get<0>(st_Point); 
                f32_Y = std::get<1>(st_Point); 
                f32_Z = std::get<2>(st_Point); 
                glVertex3f(f32_X, f32_Y, f32_Z);
            }
            glEnd();

        }
    }

    if (s32_NearIdx != -1)
    {
        f32_ICPX = st_ICPMap.arst_PointCloud[s32_NearIdx].f32_X;
        f32_ICPY = st_ICPMap.arst_PointCloud[s32_NearIdx].f32_Y;
        f32_ICPZ = st_ICPMap.arst_PointCloud[s32_NearIdx].f32_Z;
        f32_ICPYaw_deg = st_ICPMap.arst_PointCloud[s32_NearIdx].f32_Yaw_deg;
        f32_ICPYaw_rad = deg2rad(-f32_ICPYaw_deg + 90);


        if (b_UpdateLocalMap)
        {
            glColor3f(0.5f, 1.0f, 0.7f);
            glPointSize(1.f);
            glBegin(GL_POINTS);
            for (s32_I = 0; s32_I < pst_LidarPointData->st_ICPPoint.s32_PointNum; s32_I++)
            {
                f32_X = pst_LidarPointData->st_ICPPoint.arf32_X[s32_I];
                f32_Y = pst_LidarPointData->st_ICPPoint.arf32_Y[s32_I];
                f32_Z = pst_LidarPointData->st_ICPPoint.arf32_Z[s32_I];

                getGlobalCoord(f32_ICPX, f32_ICPY, f32_ICPYaw_rad,
                               f32_X, f32_Y, f32_GX, f32_GY);

                glVertex3f(f32_GX, f32_GY, f32_Z);
            }
            glEnd();
        }
        else
        {

            glColor3f(1.0f, 1.0f, 1.0f);
            glPointSize(1.f);
            glBegin(GL_POINTS);
            for (s32_I = 0;s32_I < pst_LidarPointData->st_ICPPoint.s32_PointNum;s32_I++)
            {
                f32_X = pst_LidarPointData->st_ICPPoint.arf32_X[s32_I];
                f32_Y = pst_LidarPointData->st_ICPPoint.arf32_Y[s32_I];
                f32_Z = pst_LidarPointData->st_ICPPoint.arf32_Z[s32_I];

                f32_RX = f32_X * pf32_Transform[0] + f32_Y * pf32_Transform[1] + f32_Z * pf32_Transform[2] + pf32_Transform[3];
                f32_RY = f32_X * pf32_Transform[4] + f32_Y * pf32_Transform[5] + f32_Z * pf32_Transform[6] + pf32_Transform[7];
                f32_RZ = f32_X * pf32_Transform[8] + f32_Y * pf32_Transform[9] + f32_Z * pf32_Transform[10] + pf32_Transform[11];

                getGlobalCoord(f32_ICPX, f32_ICPY, f32_ICPYaw_rad,
                               f32_RX, f32_RY, f32_GX, f32_GY);

                glVertex3f(f32_GX, f32_GY, f32_RZ);
            }
            glEnd();
        }

    }
}

void drawFlag(float32_t x, float32_t y, string color)
{
    // 깃발을 그리기 위한 좌표
    float32_t flagHeight = 1.0f; // 깃발의 높이
    float32_t flagWidth = 2.0f;  // 깃발의 너비
    float32_t poleHeight = 3.0f; // 깃발대의 높이

    // 깃발대
    glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex3f(x, y, 0.0f);
    glVertex3f(x, y + poleHeight, 0.0f);
    glEnd();

    // 깃발
    glBegin(GL_TRIANGLES);
    if(color == "R")
    {
        glColor3f(1.0f, 0.0f, 0.0f);
    }
    else if(color == "G")
    {
        glColor3f(0.0f, 1.0f, 0.0f);
    }
    else if(color =="B")
    {
        glColor3f(0.0f, 0.0f, 1.0f);
    }
    else
    {
        glColor3f(1.0f, 1.0f, 1.0f);
    }
    glVertex3f(x, y + poleHeight, 0.0f);
    glVertex3f(x + flagWidth, y + poleHeight - (flagHeight / 2.0f), 0.0f);
    glVertex3f(x, y + poleHeight - flagHeight, 0.0f);
    glEnd();
}

void drawCircle(float x, float y, float radius, int segments, int mode)
{
    if(mode == 0)
    {
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(x, y, 1.0f);
    }
    else
    {
        glBegin(GL_LINE_LOOP);
    }
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * float(i) / float(segments);
        float dx = radius * cosf(angle);
        float dy = radius * sinf(angle);
        glVertex3f(x + dx, y + dy, 1.0f);
    }
    glEnd();
}

void InitGlobalMap(PLANNING_DATA_t *pst_PlanningData)
{
    VEHICLE_DATA_t *pst_EgoVehicleData = &pst_PlanningData->st_EgoVehicleData;

    int32_t s32_I;
    int32_t s32_J;

    float32_t f32_N, f32_E, f32_U;
    float32_t f32_EKF_N, f32_EKF_E, f32_EKF_U;

    f32_E = pst_EgoVehicleData->f32_X_global;
    f32_N = pst_EgoVehicleData->f32_Y_global;
    f32_U = pst_EgoVehicleData->f32_Z_global;
    glViewport(st_ViewerParam.s32_GlobalX, st_ViewerParam.s32_GlobalY, st_ViewerParam.s32_GlobalWidth, st_ViewerParam.s32_GlobalHeight);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-500.0f * st_ViewerParam.f32_GlobalZoomFactor, 500.0f * st_ViewerParam.f32_GlobalZoomFactor, -500.0f * st_ViewerParam.f32_GlobalZoomFactor, 500.0f * st_ViewerParam.f32_GlobalZoomFactor, -1200.0f, 1200.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if(st_ViewerParam.s32_GlobalWindowState == c_STATE_GLOBALNONFIX)
    {
        glTranslatef(st_Trans.f32_CentorX_G, 0.0f, 0.0f);    // move to x
        glTranslatef(0.0f, st_Trans.f32_CentorY_G, 0.0f);    // move to y
    }
    
    else
    {
        glTranslatef(st_Trans.f32_CentorX_G - f32_E, st_Trans.f32_CentorY_G - f32_N, f32_U);
    }


    glRotatef(st_Trans.f32_AngleX_G, 0, 1, 0);    //rotate y
    glRotatef(st_Trans.f32_AngleY_G, 1, 0, 0);    //rotate x
    // printf("zoomfactor : %f\n", st_ViewerParam.f32_GlobalZoomFactor);

    if(st_ViewerParam.f32_GlobalZoomFactor >= 1.0f)
    {
        st_ViewerParam.s32_ZoomFactorState = c_STATE_1XZOOMFACTOR;
	    glLineWidth(1.0f);  //Map Grid 
	    glBegin(GL_LINES);
	    glColor3f(0.2f, 0.2f, 0.2f);

        for (int i = -2000; i <= 2000; i += 200) 
        {
            glVertex3f(i, -2000.0f, 0.0f);
            glVertex3f(i, 2000.0f, 0.0f);
            glVertex3f(-2000.0f, i, 0.0f);
            glVertex3f(2000.0f, i, 0.0f);
        }

        glEnd();
        DrawMeterUnit(-2000, 2000, 200, 1.5);
        DrawGridPoint(-2000, 2000, 200);
    }
    else if(st_ViewerParam.f32_GlobalZoomFactor >= 0.2f && st_ViewerParam.f32_GlobalZoomFactor < 1.0f)
    {
        st_ViewerParam.s32_ZoomFactorState = c_STATE_2XZOOMFACTOR;
	    glLineWidth(1.0f);  //Map Grid 
	    glBegin(GL_LINES);
	    glColor3f(0.2f, 0.2f, 0.2f);

        for (int i = -2000; i <= 2000; i += 50) 
        {
            glVertex3f(i, -2000.0f, 0.0f);
            glVertex3f(i, 2000.0f, 0.0f);
            glVertex3f(-2000.0f, i, 0.0f);
            glVertex3f(2000.0f, i, 0.0f);
        }
        glEnd();
        DrawMeterUnit(-2000, 2000, 50, 1);
        DrawGridPoint(-2000, 2000, 50);
    }
    else if(st_ViewerParam.f32_GlobalZoomFactor < 0.2f)
    {
        st_ViewerParam.s32_ZoomFactorState = c_STATE_3XZOOMFACTOR;
	    glLineWidth(1.0f);  //Map Grid 
	    glBegin(GL_LINES);
	    glColor3f(0.2f, 0.2f, 0.2f);

        for (int i = -2000; i <= 2000; i += 10) 
        {
            glVertex3f(i, -2000.0f, 0.0f);
            glVertex3f(i, 2000.0f, 0.0f);
            glVertex3f(-2000.0f, i, 0.0f);
            glVertex3f(2000.0f, i, 0.0f);
        }
        glEnd();
        DrawMeterUnit(-2000, 2000, 10, 0.5);
        DrawGridPoint(-2000, 2000, 10);

    }

    glLineWidth(1.f);
    glBegin(GL_LINES);
    glColor3f(1.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(20.f, 0.f, 0.f);
    glColor3f(0.f, 1.f, 0.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 20.f, 0.f);
    glColor3f(0.f, 0.f, 1.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, 20.f);
    glEnd();


}


void InitLocalMap()
{
    int32_t s32_I = 0;

    glViewport(st_ViewerParam.s32_LocalX, st_ViewerParam.s32_LocalY, st_ViewerParam.s32_LocalWidth, st_ViewerParam.s32_LocalHeight);  //(꼭짓점 x, 꼭짓점 y, width, height )
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-50.0f * st_ViewerParam.f32_LocalZoomFactor, 50.0f * st_ViewerParam.f32_LocalZoomFactor, -50.0f * st_ViewerParam.f32_LocalZoomFactor, 50.0f * st_ViewerParam.f32_LocalZoomFactor, -1200.0f, 1200.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef(st_Trans.f32_CentorX_L, 0.0f, 0.0f);
    glTranslatef(0.0f, st_Trans.f32_CentorY_L, 0.0f);

    glRotatef(st_Trans.f32_AngleX_L, 0, 1, 0);
    glRotatef(st_Trans.f32_AngleY_L, 1, 0, 0);
    glRotatef(90.f, 0, 0, 1);

    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glColor3f(1.f,0.f,0.f);
    glVertex2f(0, 0);
    glVertex2f(st_ViewerParam.f32_LocalZoomFactor * 4.f,0);
    glEnd();


    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glColor3f(0.f,1.f,0.f);
    glVertex2f(0, 0);
    glVertex2f(0, st_ViewerParam.f32_LocalZoomFactor * 4.f);
    glEnd();

    if(st_ViewerParam.f32_LocalZoomFactor > 1.3f)
    {
        st_ViewerParam.s32_ZoomFactorState = c_STATE_1XZOOMFACTOR;
	    glLineWidth(1.0f);
	    glBegin(GL_LINES);
	    glColor3f(0.2f,0.2f,0.2f);

        for (s32_I = -120; s32_I <= 120; s32_I += 10) 
        {
            glVertex3f(s32_I, -120.0f, 0.0f);
            glVertex3f(s32_I, 120.0f, 0.0f);
            glVertex3f(-120.0f, s32_I, 0.0f);
            glVertex3f(120.0f, s32_I, 0.0f);
        }
        glEnd();
        DrawMeterUnit(-120, 120, 10, 1.5);
        DrawGridPoint(-120, 120, 10);
    }
    else if(st_ViewerParam.f32_LocalZoomFactor >= 0.6f && st_ViewerParam.f32_LocalZoomFactor <= 1.3f)
    {
        st_ViewerParam.s32_ZoomFactorState = c_STATE_2XZOOMFACTOR;
	    glLineWidth(1.0f);
	    glBegin(GL_LINES);
	    glColor3f(0.2f,0.2f,0.2f);

        for (s32_I = -120; s32_I <= 120; s32_I += 5) 
        {
            glVertex3f(s32_I, -120.0f, 0.0f);
            glVertex3f(s32_I, 120.0f, 0.0f);
            glVertex3f(-120.0f, s32_I, 0.0f);
            glVertex3f(120.0f, s32_I, 0.0f);
        }
        glEnd();
        DrawMeterUnit(-120, 120, 5, 1);
        DrawGridPoint(-120, 120, 5);
    }
    else if(st_ViewerParam.f32_LocalZoomFactor < 0.6f)
    {
        st_ViewerParam.s32_ZoomFactorState = c_STATE_3XZOOMFACTOR;
    	glLineWidth(1.0f);
		glBegin(GL_LINES);
		glColor3f(0.2f,0.2f,0.2f);

        for (s32_I = -120; s32_I <= 120; s32_I += 2) 
        {
            glVertex3f(s32_I, -120.0f, 0.0f);
            glVertex3f(s32_I, 120.0f, 0.0f);
            glVertex3f(-120.0f, s32_I, 0.0f);
            glVertex3f(120.0f, s32_I, 0.0f);
        }
        glEnd();
        DrawMeterUnit(-120, 120, 2, 0.5);
        DrawGridPoint(-120, 120, 2);

    }
    glEnd();
}

void InitEgoBox(float32_t f32_E, float32_t f32_N, float32_t f32_D)
{
    float32_t f32_Up = 5.f;
    float32_t f32_Down = -5.f;
    float32_t f32_Front = 5.f;
    float32_t f32_Back = -5.f;
    float32_t f32_Left = -10.f;
    float32_t f32_Right = 10.f;

    IMU_DATA_t st_IMU = st_DeadReckoningDataViewer.st_IMU;

    if(st_IMU.f32_Yaw_deg != 0 && st_IMU.f32_Pitch_deg != 0 && st_IMU.f32_Roll_deg != 0)
    {
        rotatePoint(f32_Left, f32_Back, f32_Up, st_IMU.f32_Roll_deg,st_IMU.f32_Pitch_deg, st_IMU.f32_Yaw_deg, SqaurePoint[0][0], SqaurePoint[0][1], SqaurePoint[0][2]);      // 0 꼭짓점
        rotatePoint(f32_Right, f32_Back, f32_Up, st_IMU.f32_Roll_deg,st_IMU.f32_Pitch_deg, st_IMU.f32_Yaw_deg, SqaurePoint[1][0], SqaurePoint[1][1], SqaurePoint[1][2]);     // 1 꼭짓점
        rotatePoint(f32_Left, f32_Front, f32_Up, st_IMU.f32_Roll_deg,st_IMU.f32_Pitch_deg, st_IMU.f32_Yaw_deg, SqaurePoint[2][0], SqaurePoint[2][1], SqaurePoint[2][2]);     // 2 꼭짓점
        rotatePoint(f32_Right, f32_Front, f32_Up, st_IMU.f32_Roll_deg,st_IMU.f32_Pitch_deg, st_IMU.f32_Yaw_deg, SqaurePoint[3][0], SqaurePoint[3][1], SqaurePoint[3][2]);    // 3 꼭짓점
        rotatePoint(f32_Left, f32_Back, f32_Down, st_IMU.f32_Roll_deg,st_IMU.f32_Pitch_deg, st_IMU.f32_Yaw_deg, SqaurePoint[4][0], SqaurePoint[4][1], SqaurePoint[4][2]);    // 4 꼭짓점
        rotatePoint(f32_Right, f32_Back, f32_Down, st_IMU.f32_Roll_deg,st_IMU.f32_Pitch_deg, st_IMU.f32_Yaw_deg, SqaurePoint[5][0], SqaurePoint[5][1], SqaurePoint[5][2]);   // 5 꼭짓점
        rotatePoint(f32_Left, f32_Front, f32_Down, st_IMU.f32_Roll_deg,st_IMU.f32_Pitch_deg, st_IMU.f32_Yaw_deg, SqaurePoint[6][0], SqaurePoint[6][1], SqaurePoint[6][2]);  // 6 꼭짓점
        rotatePoint(f32_Right, f32_Front, f32_Down, st_IMU.f32_Roll_deg,st_IMU.f32_Pitch_deg, st_IMU.f32_Yaw_deg, SqaurePoint[7][0], SqaurePoint[7][1], SqaurePoint[7][2]);   // 7 꼭짓점

    }

    // usleep(150000);
    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // glLoadIdentity();

    // 카메라 위치 및 시점 설정
    gluLookAt(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // 직육면체를 그리기 위해 꼭짓점 설정
    if(SqaurePoint[0][0] != 0.f)
    {

        glLineWidth(2.5f);
        glBegin(GL_LINE_LOOP);
        // 앞면
        glVertex3f(SqaurePoint[2][0] + f32_N,SqaurePoint[2][1] + f32_E, SqaurePoint[2][2] + f32_D);
        glVertex3f(SqaurePoint[3][0] + f32_N,SqaurePoint[3][1] + f32_E, SqaurePoint[3][2] + f32_D);
        glVertex3f(SqaurePoint[7][0] + f32_N,SqaurePoint[7][1] + f32_E, SqaurePoint[7][2] + f32_D);
        glVertex3f(SqaurePoint[6][0] + f32_N,SqaurePoint[6][1] + f32_E, SqaurePoint[6][2] + f32_D);
        glEnd();
        glBegin(GL_LINE_LOOP);
        // 뒷면
        glVertex3f(SqaurePoint[0][0] + f32_N,SqaurePoint[0][1] + f32_E, SqaurePoint[0][2] + f32_D);
        glVertex3f(SqaurePoint[1][0] + f32_N,SqaurePoint[1][1] + f32_E, SqaurePoint[1][2] + f32_D);
        glVertex3f(SqaurePoint[5][0] + f32_N,SqaurePoint[5][1] + f32_E, SqaurePoint[5][2] + f32_D);
        glVertex3f(SqaurePoint[4][0] + f32_N,SqaurePoint[4][1] + f32_E, SqaurePoint[4][2] + f32_D);
        glEnd();
        glBegin(GL_LINE_LOOP);
        // 왼쪽면
        glVertex3f(SqaurePoint[0][0] + f32_N,SqaurePoint[0][1] + f32_E, SqaurePoint[0][2] + f32_D);
        glVertex3f(SqaurePoint[2][0] + f32_N,SqaurePoint[2][1] + f32_E, SqaurePoint[2][2] + f32_D);
        glVertex3f(SqaurePoint[6][0] + f32_N,SqaurePoint[6][1] + f32_E, SqaurePoint[6][2] + f32_D);
        glVertex3f(SqaurePoint[4][0] + f32_N,SqaurePoint[4][1] + f32_E, SqaurePoint[4][2] + f32_D);
        glEnd();
        glBegin(GL_LINE_LOOP);
        // 오른쪽면
        glVertex3f(SqaurePoint[1][0] + f32_N,SqaurePoint[1][1] + f32_E, SqaurePoint[1][2] + f32_D);
        glVertex3f(SqaurePoint[3][0] + f32_N,SqaurePoint[3][1] + f32_E, SqaurePoint[3][2] + f32_D);
        glVertex3f(SqaurePoint[7][0] + f32_N,SqaurePoint[7][1] + f32_E, SqaurePoint[7][2] + f32_D);
        glVertex3f(SqaurePoint[5][0] + f32_N,SqaurePoint[5][1] + f32_E, SqaurePoint[5][2] + f32_D);
        glEnd();
        glBegin(GL_LINE_LOOP);
        // 윗면
        glVertex3f(SqaurePoint[0][0] + f32_N,SqaurePoint[0][1] + f32_E, SqaurePoint[0][2] + f32_D);
        glVertex3f(SqaurePoint[1][0] + f32_N,SqaurePoint[1][1] + f32_E, SqaurePoint[1][2] + f32_D);
        glVertex3f(SqaurePoint[3][0] + f32_N,SqaurePoint[3][1] + f32_E, SqaurePoint[3][2] + f32_D);
        glVertex3f(SqaurePoint[2][0] + f32_N,SqaurePoint[2][1] + f32_E, SqaurePoint[2][2] + f32_D);
        glEnd();
        glBegin(GL_LINE_LOOP);
        // 아랫면
        glVertex3f(SqaurePoint[4][0] + f32_N,SqaurePoint[4][1] + f32_E, SqaurePoint[4][2] + f32_D);
        glVertex3f(SqaurePoint[5][0] + f32_N,SqaurePoint[5][1] + f32_E, SqaurePoint[5][2] + f32_D);
        glVertex3f(SqaurePoint[7][0] + f32_N,SqaurePoint[7][1] + f32_E, SqaurePoint[7][2] + f32_D);
        glVertex3f(SqaurePoint[6][0] + f32_N,SqaurePoint[6][1] + f32_E, SqaurePoint[6][2] + f32_D);
        glEnd();
    }


    // glutSwapBuffers();

    // glBegin(GL_)
}

const char* GetVehicleStateString(VEHICLE_STATE_t state)
{
    switch (state)
    {
        case STATE_IDLE:
            return "IDLE";
        case STATE_IDLE_ACC_MODE:
            return "IDLE_ACC_MODE";
        case STATE_OVERTAKING:
            return "OVERTAKING";
        case STATE_NON_STABLE_MODE:
            return "NON_STABLE_MODE";
        case STATE_NON_STABLE_ACC_MODE:
            return "NON_STABLE_ACC_MODE";
        case STATE_STATIC:
            return "STATIC";
        case STATE_PITSTOP:
            return "PITSTOP";
        case STATE_EMERGENCY:
            return "EMERGENCY";
        default:
            return "UNKNOWN_STATE";
    }
}

const char* GetBankStateString(PLANNING_DATA_t *pst_PlanningData)
{
    if (pst_PlanningData->st_Planner.b_PlanningBank)
    {
        return "Bank";
    }
    else
    {
        return "No Bank";
    }
}

void DrawCarBody(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2)
{
    // 자동차 본체를 하늘색으로 설정
    glColor3f(0.529, 0.808, 0.980); // 하늘색 (Sky Blue)

    // 자동차 본체 그리기
    glBegin(GL_QUADS);

    // 앞면
    glVertex3f(f32_X1, f32_Y1, f32_Z1);
    glVertex3f(f32_X1, f32_Y2, f32_Z1);
    glVertex3f(f32_X1, f32_Y2, f32_Z2);
    glVertex3f(f32_X1, f32_Y1, f32_Z2);

    // 뒷면
    glVertex3f(f32_X2, f32_Y1, f32_Z1);
    glVertex3f(f32_X2, f32_Y2, f32_Z1);
    glVertex3f(f32_X2, f32_Y2, f32_Z2);
    glVertex3f(f32_X2, f32_Y1, f32_Z2);

    // 왼쪽면
    glVertex3f(f32_X1, f32_Y1, f32_Z1);
    glVertex3f(f32_X2, f32_Y1, f32_Z1);
    glVertex3f(f32_X2, f32_Y1, f32_Z2);
    glVertex3f(f32_X1, f32_Y1, f32_Z2);

    // 오른쪽면
    glVertex3f(f32_X1, f32_Y2, f32_Z1);
    glVertex3f(f32_X2, f32_Y2, f32_Z1);
    glVertex3f(f32_X2, f32_Y2, f32_Z2);
    glVertex3f(f32_X1, f32_Y2, f32_Z2);

    // 윗면
    glVertex3f(f32_X1, f32_Y1, f32_Z2);
    glVertex3f(f32_X1, f32_Y2, f32_Z2);
    glVertex3f(f32_X2, f32_Y2, f32_Z2);
    glVertex3f(f32_X2, f32_Y1, f32_Z2);

    // 바닥면
    glVertex3f(f32_X1, f32_Y1, f32_Z1);
    glVertex3f(f32_X1, f32_Y2, f32_Z1);
    glVertex3f(f32_X2, f32_Y2, f32_Z1);
    glVertex3f(f32_X2, f32_Y1, f32_Z1);

    glEnd();
}

// 창문 그리기 함수
void DrawWindows(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2)
{
    // 창문을 하얀색으로 설정
    glColor3f(1.0, 1.0, 1.0); // 하얀색

    // 창문 그리기
    glBegin(GL_QUADS);

    // 왼쪽 창문
    glVertex3f(f32_X1 - 0.1f, f32_Y1, f32_Z2 + 1.0f);
    glVertex3f(f32_X1 - 0.1f, f32_Y1, f32_Z2 + 1.8f);
    glVertex3f(f32_X1 - 1.6f, f32_Y1, f32_Z2 + 1.8f);
    glVertex3f(f32_X1 - 1.6f, f32_Y1, f32_Z2 + 1.0f);

    // 오른쪽 창문
    glVertex3f(f32_X1 - 0.1f, f32_Y2, f32_Z2 + 1.0f);
    glVertex3f(f32_X1 - 0.1f, f32_Y2, f32_Z2 + 1.8f);
    glVertex3f(f32_X1 - 1.6f, f32_Y2, f32_Z2 + 1.8f);
    glVertex3f(f32_X1 - 1.6f, f32_Y2, f32_Z2 + 1.0f);

    glEnd();
}

void DrawWheel(float32_t centerX, float32_t centerY, float32_t centerZ, float32_t radius, int segments)
{
    // 바퀴를 검정색으로 설정
    glColor3f(0.1, 0.1, 0.1); // 검정색

    // 바퀴의 앞면 그리기
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(centerX, centerY, centerZ); // 중심점

    for (int i = 0; i <= segments; i++)
    {
        float angle = 2.0f * PI * i / segments; // 각도 계산
        float x = centerX + radius * cos(angle);
        float z = centerZ + radius * sin(angle); // Y 대신 Z 좌표 사용
        glVertex3f(x, centerY, z); // 원의 점
    }
    glEnd();

    // 바퀴의 뒷면 그리기
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(centerX, centerY - 0.1f, centerZ); // 중심점

    for (int i = 0; i <= segments; i++)
    {
        float angle = 2.0f * PI * i / segments; // 각도 계산
        float x = centerX + radius * cos(angle);
        float z = centerZ + radius * sin(angle); // Y 대신 Z 좌표 사용
        glVertex3f(x, centerY - 0.1f, z); // 원의 점
    }
    glEnd();

    // 바퀴의 옆면 그리기
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; i++)
    {
        float angle = 2.0f * PI * i / segments; // 각도 계산
        float x = centerX + radius * cos(angle);
        float z = centerZ + radius * sin(angle); // Y 대신 Z 좌표 사용

        glVertex3f(x, centerY, z);          // 앞면의 점
        glVertex3f(x, centerY - 0.1f, z);   // 뒷면의 점
    }
    glEnd();
}

void DrawWheels(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2)
{
    // 바퀴의 반지름 및 세그먼트 수 설정
    float32_t radius = 0.5f;
    int segments = 20; // 다각형 세그먼트 수 (더 높을수록 더 원에 가까워짐)

    // 앞 바퀴 (좌)
    DrawWheel(f32_X1 - 0.3f, f32_Y1, 0.2f, radius, segments);
    // 앞 바퀴 (우)
    DrawWheel(f32_X1 - 0.3f, f32_Y2, 0.2f, radius, segments);
    // 뒷 바퀴 (좌)
    DrawWheel(f32_X2 + 0.3f, f32_Y1, 0.2f, radius, segments);
    // 뒷 바퀴 (우)
    DrawWheel(f32_X2 + 0.3f, f32_Y2, 0.2f, radius, segments);
}

// 자동차 전체 그리기
void DrawCar(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2)
{
    // 자동차 본체 그리기
    DrawCarBody(f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2, f32_Z2);

    // 창문 그리기
    DrawWindows(f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2, f32_Z2);

    // 바퀴 그리기
    DrawWheels(f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2);
}



void DrawBox(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2)
{
    glBegin(GL_LINE_LOOP);
    glVertex3f(f32_X1, f32_Y1, f32_Z1);
    glVertex3f(f32_X1, f32_Y2, f32_Z1);
    glVertex3f(f32_X2, f32_Y2, f32_Z1);
    glVertex3f(f32_X2, f32_Y1, f32_Z1);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3f(f32_X1, f32_Y1, f32_Z2);
    glVertex3f(f32_X1, f32_Y2, f32_Z2);
    glVertex3f(f32_X2, f32_Y2, f32_Z2);
    glVertex3f(f32_X2, f32_Y1, f32_Z2);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(f32_X1, f32_Y1, f32_Z1);
    glVertex3f(f32_X1, f32_Y1, f32_Z2);

    glVertex3f(f32_X2, f32_Y1, f32_Z1);
    glVertex3f(f32_X2, f32_Y1, f32_Z2);

    glVertex3f(f32_X1, f32_Y2, f32_Z1);
    glVertex3f(f32_X1, f32_Y2, f32_Z2);

    glVertex3f(f32_X2, f32_Y2, f32_Z1);
    glVertex3f(f32_X2, f32_Y2, f32_Z2);
    glEnd();
}




void DrawBox(float32_t f32_CenterX, float32_t f32_CenterY, float32_t f32_CenterZ, float32_t f32_Length, float32_t f32_Width, float32_t f32_Height, float32_t f32_Yaw_rad)
{
    float32_t arf32_Vertice[8][3];

    float32_t f32_HL = f32_Length / 2.f;
    float32_t f32_HW = f32_Width / 2.f;
    float32_t f32_HH = f32_Height / 2.f;

    float32_t arf32_LocalVertice[8][3] = {
        {-f32_HL, -f32_HW, -f32_HH},
        { f32_HL, -f32_HW, -f32_HH},
        { f32_HL,  f32_HW, -f32_HH},
        {-f32_HL,  f32_HW, -f32_HH},
        {-f32_HL, -f32_HW,  f32_HH},
        { f32_HL, -f32_HW,  f32_HH},
        { f32_HL,  f32_HW,  f32_HH},
        {-f32_HL,  f32_HW,  f32_HH},
    };

    float32_t f32_CosTheta = cosf(f32_Yaw_rad);
    float32_t f32_SinTheta = sinf(f32_Yaw_rad);

    float32_t f32_X, f32_Y, f32_Z;

    int32_t s32_I;

    for(s32_I = 0; s32_I < 8; s32_I++)
    {
        f32_X = arf32_LocalVertice[s32_I][0];
        f32_Y = arf32_LocalVertice[s32_I][1];
        f32_Z = arf32_LocalVertice[s32_I][2];

        arf32_Vertice[s32_I][0] = f32_CenterX + (f32_X * f32_CosTheta - f32_Y * f32_SinTheta);
        arf32_Vertice[s32_I][1] = f32_CenterY + (f32_X * f32_SinTheta + f32_Y * f32_CosTheta);
        arf32_Vertice[s32_I][2] = f32_CenterZ + f32_Z;
    }

    glBegin(GL_LINE_LOOP);
    glVertex3fv(arf32_Vertice[0]);
    glVertex3fv(arf32_Vertice[1]);
    glVertex3fv(arf32_Vertice[2]);
    glVertex3fv(arf32_Vertice[3]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3fv(arf32_Vertice[4]);
    glVertex3fv(arf32_Vertice[5]);
    glVertex3fv(arf32_Vertice[6]);
    glVertex3fv(arf32_Vertice[7]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3fv(arf32_Vertice[0]);
    glVertex3fv(arf32_Vertice[1]);
    glVertex3fv(arf32_Vertice[5]);
    glVertex3fv(arf32_Vertice[4]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3fv(arf32_Vertice[2]);
    glVertex3fv(arf32_Vertice[3]);
    glVertex3fv(arf32_Vertice[7]);
    glVertex3fv(arf32_Vertice[6]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3fv(arf32_Vertice[0]);
    glVertex3fv(arf32_Vertice[3]);
    glVertex3fv(arf32_Vertice[7]);
    glVertex3fv(arf32_Vertice[4]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3fv(arf32_Vertice[1]);
    glVertex3fv(arf32_Vertice[2]);
    glVertex3fv(arf32_Vertice[6]);
    glVertex3fv(arf32_Vertice[5]);
    glEnd();
}


bool CheckingVariableSpace(float64_t f64_XPos,float64_t f64_YPos, int32_t s32_ViewX, int32_t s32_ViewY, int32_t s32_Width, int32_t s32_Height)
{
    float64_t f64_MouseX = f64_XPos;
    float64_t f64_MouseY = f64_YPos;
    int32_t s32_OriginX = s32_ViewX;
    int32_t s32_OriginY = s32_ViewY;
    // printf("%f %f %d %d %d %d \n", f64_XPos, f64_YPos , s32_ViewX, s32_V/iewY, s32_Width, s32_Height);
    if(f64_MouseX > s32_OriginX && f64_MouseX < s32_OriginX + s32_Width
        && f64_MouseY > st_ViewerParam.s32_ScreenHeight - (s32_OriginY + s32_Height) && f64_MouseY < st_ViewerParam.s32_ScreenHeight - s32_OriginY)
    {
        return true;
    }
    else 
    {
        return false;
    }
}



void CursorPositionCallback(GLFWwindow* pst_Window, float64_t f64_XPos, float64_t f64_YPos)
{
    float64_t f64_DeltaXX = st_Pos.f64_X - f64_XPos;
    float64_t f64_DeltaYY = st_Pos.f64_Y - f64_YPos;
    int32_t s32_Temp;

    // printf("%lf %lf %d\n", f64_XPos, f64_YPos, st_ViewerParam.s32_CurrentWindow);

    if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_LOCAL)
    {
        if (st_ViewerParam.b_TransLocalMap)
        {

            float64_t f64_DeltaX_L = (f64_XPos - st_ViewerParam.f32_PrevX) * st_ViewerParam.f32_LocalZoomFactor * 0.05;
            float64_t f64_DeltaY_L = (f64_YPos - st_ViewerParam.f32_PrevY) * st_ViewerParam.f32_LocalZoomFactor * 0.05;

            st_Trans.f32_CentorX_L += f64_DeltaX_L;
            st_Trans.f32_CentorY_L -= f64_DeltaY_L; 
            st_ViewerParam.f32_PrevX = f64_XPos;
            st_ViewerParam.f32_PrevY = f64_YPos;
        }
        else if (st_ViewerParam.b_RotateLocalMap)
        {
            float64_t f64_DeltaX_L = (f64_XPos - st_ViewerParam.f32_PrevX) * 0.1;
            float64_t f64_DeltaY_L = (f64_YPos - st_ViewerParam.f32_PrevY) * 0.1;

            st_Trans.f32_AngleX_L += f64_DeltaX_L;
            st_Trans.f32_AngleY_L += f64_DeltaY_L;
            st_ViewerParam.f32_PrevX = f64_XPos;
            st_ViewerParam.f32_PrevY = f64_YPos;
        }
    }
    if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_GLOBAL)
    {
        if (st_ViewerParam.b_TransGlobalMap)
        {
            float64_t f64_DeltaX_G = (f64_XPos - st_ViewerParam.f32_PrevX) * st_ViewerParam.f32_GlobalZoomFactor;
            float64_t f64_DeltaY_G = (f64_YPos - st_ViewerParam.f32_PrevY) * st_ViewerParam.f32_GlobalZoomFactor;

            st_Trans.f32_CentorX_G += f64_DeltaX_G;
            st_Trans.f32_CentorY_G -= f64_DeltaY_G; 
            st_ViewerParam.f32_PrevX = f64_XPos;
            st_ViewerParam.f32_PrevY = f64_YPos;

        }
        else if (st_ViewerParam.b_RotateGlobalMap)
        {
            float64_t f64_DeltaX_G = (f64_XPos - st_ViewerParam.f32_PrevX) * 0.1;
            float64_t f64_DeltaY_G = (f64_YPos - st_ViewerParam.f32_PrevY) * 0.1;

            st_Trans.f32_AngleX_G += f64_DeltaX_G;
            st_Trans.f32_AngleY_G += f64_DeltaY_G;
            st_ViewerParam.f32_PrevX = f64_XPos;
            st_ViewerParam.f32_PrevY = f64_YPos;
        }
    }
    if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_FRAME)
    {
        if (st_ViewerParam.b_MoveFrame)
        {
            // // st_ViewerParam.s32_FrameBarX = f64_XPos / (st_ViewerParam.s32_FrameWidth * 0.0005);

            st_ViewerParam.s32_FrameBarX = f64_XPos * st_FileInfo.s32_CntFrame / st_ViewerParam.s32_FrameWidth;
            st_ViewerParam.s32_State |= c_STATE_MODE_STOP;
            s32_Temp = st_ViewerParam.s32_FrameBarX;

            if (s32_Temp < 0)
            {
                s32_FileFrameCnt = 0;
                st_ViewerParam.s32_FrameBarX = 0;
            }
            else if (s32_Temp >= st_FileInfo.s32_CntFrame)
            {
                s32_FileFrameCnt = st_FileInfo.s32_CntFrame - 1;
                st_ViewerParam.s32_FrameBarX = st_ViewerParam.s32_FrameWidth;
            }
            else
            {
                s32_FileFrameCnt = s32_Temp;
            }

            // printf("%lf, %d %d %d\n", f64_XPos, st_ViewerParam.s32_FrameWidth, st_FileInfo.s32_CntFrame, s32_FileFrameCnt);
        }
    }
}

void DrawModeStatus()
{   
    //1200, 70, 500, 990
    // printf("width : %d %d\n", st_ViewerParam.s32_ScreenWidth, st_ViewerParam.s32_ScreenHeight);
    float32_t f32_TrackingObjectSpeed;
    int32_t s32_I;
    glViewport(st_ViewerParam.s32_LocalX * 1.05f, st_ViewerParam.s32_ScreenHeight * 0.8, st_ViewerParam.s32_LocalWidth, st_ViewerParam.s32_LocalHeight * 0.2);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 100, 0, 100, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    int32_t ars_Time[3] = {0};

    char arc_BufferText[500] = {0};

     //-------------------------------------GPS Mode-------------------------------------//
    sprintf(arc_BufferText, "GPS Mode : %d", st_DeadReckoningDataViewer.st_GPS.s32_mode);
    DrawText(arc_BufferText,18, 95, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "East : %.2fm", st_PlanningDataViewer.st_EgoVehicleData.f32_X_global);
    DrawText(arc_BufferText,18, 85, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "North : %.2fm", st_PlanningDataViewer.st_EgoVehicleData.f32_Y_global);
    DrawText(arc_BufferText,18, 75, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Heading (NED) : %.1fdeg", st_DeadReckoningDataViewer.st_IMU.f32_Yaw_deg);
    DrawText(arc_BufferText,18, 65, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Heading (ENU) : %.1fdeg", -st_DeadReckoningDataViewer.st_IMU.f32_Yaw_deg + 90.f);
    DrawText(arc_BufferText,18, 55, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Velocity NED : %.2fkm/h,   %.2fkm/h,   %.2fkm/h", st_DeadReckoningDataViewer.st_IMU.arf32_VelocityNED_m_s[0] * 3.6f, st_DeadReckoningDataViewer.st_IMU.arf32_VelocityNED_m_s[1] * 3.6f, st_DeadReckoningDataViewer.st_IMU.arf32_VelocityNED_m_s[2] * 3.6f);
    DrawText(arc_BufferText,18, 45, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Velocity XYZ : %.2fkm/h,   %.2fkm/h,   %.2fkm/h", st_DeadReckoningDataViewer.st_IMU.arf32_VelocityXYZ_m_s[0] * 3.6f, st_DeadReckoningDataViewer.st_IMU.arf32_VelocityXYZ_m_s[1] * 3.6f, st_DeadReckoningDataViewer.st_IMU.arf32_VelocityXYZ_m_s[2] * 3.6f);
    DrawText(arc_BufferText,18, 35, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Roll : %.1fdeg", st_DeadReckoningDataViewer.st_IMU.f32_Roll_deg);
    DrawText(arc_BufferText,25, 85, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Pitch : %.1fdeg", st_DeadReckoningDataViewer.st_IMU.f32_Pitch_deg);
    DrawText(arc_BufferText,25, 75, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    



    //-------------------------------------LIDAR Mode-------------------------------------//
    // std::cout << "Status: " << st_SensorData.st_RawLIDAR.s_LIDAR_Status << std::endl;
    // std::cout << "Mode  : " << st_SensorData.st_RawLIDAR.s_LIDAR_Mode << std::endl;
    // std::cout << "Temp  : " << st_SensorData.st_RawLIDAR.s64_LIDAR_Temp << std::endl;
    sprintf(arc_BufferText, "Status : %s", st_SensorData.st_RawLIDAR.s_LIDAR_Status.c_str());
    DrawText(arc_BufferText,45, 95, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "LiDAR Mode : %s", st_SensorData.st_RawLIDAR.s_LIDAR_Mode.c_str());
    DrawText(arc_BufferText,45, 85, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "LiDAR Temp : %ld", st_SensorData.st_RawLIDAR.s64_LIDAR_Temp);
    DrawText(arc_BufferText,45, 75, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    //     for(int i = 0; i < st_HMCLidarDataViewer.s32_TrackingNum; i++)
    // {

    //     f32_TrackingObjectSpeed = getDistance2d(st_HMCLidarDataViewer.arst_Tracking[i].f32_VelocityX_m_s * 3.6f,
    //      st_HMCLidarDataViewer.arst_Tracking[i].f32_VelocityY_m_s * 3.6f, 0 , 0);

    //     sprintf(arc_BufferText, "Object Speed : %f", f32_TrackingObjectSpeed);
    //     DrawText(arc_BufferText,60, 65, 18);
    //     memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    //     sprintf(arc_BufferText, "Object PosX : %f", st_HMCLidarDataViewer.arst_Tracking[i].f32_X);
    //     DrawText(arc_BufferText,60, 55, 18);
    //     memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    //     sprintf(arc_BufferText, "Object PosY : %f", st_HMCLidarDataViewer.arst_Tracking[i].f32_Y);
    //     DrawText(arc_BufferText,60, 45, 18);
    //     memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    // }

    //-------------------------------------LIDAR Mode-------------------------------------//
    //-------------------------------------PLANNING Mode-------------------------------------//



    sprintf(arc_BufferText, "ACC Idx: %d", st_PlanningDataViewer.s32_AccObjIdx);
    DrawText(arc_BufferText,0, 95, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    // printf("Lat: %f, Lon: %f\n", st_CANDataViewer.f32_LAT_ACCEL, st_CANDataViewer.f32_Long_ACCEL );
    // printf("Brake: %f, Yaw Rate: %f\n", st_CANDataViewer.f32_BRK_CYLINDER, st_CANDataViewer.f32_YAW_RATE );
    // printf("User: %d\n", st_CANDataViewer.u8_EPS_ERR);
    // printf("Veh: %d\n", st_CANDataViewer.u8_EPS_SAS_ERR);
    ///////////////////////////////////////////

    if(st_PlanningDataViewer.st_Planner.b_AccMode == 1)
    {
        sprintf(arc_BufferText, "Safe Dist: %.1f", st_PlanningDataViewer.arst_ObjectData[st_PlanningDataViewer.s32_AccObjIdx].f32_SafetyDistance);
        DrawText(arc_BufferText,0, 85, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));

        sprintf(arc_BufferText, "Rel Dist: %.1f", st_PlanningDataViewer.arst_ObjectData[st_PlanningDataViewer.s32_AccObjIdx].f32_RelativeDistance);
        DrawText(arc_BufferText,0, 75, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));

        sprintf(arc_BufferText, "ABS Dist: %.1f", st_PlanningDataViewer.arst_ObjectData[st_PlanningDataViewer.s32_AccObjIdx].f32_AbsDistance);
        DrawText(arc_BufferText,0, 65, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));

        sprintf(arc_BufferText, "ABS: %d", st_CANDataViewer.u8_AEB_En);
        DrawText(arc_BufferText,0, 55, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else
    {
        sprintf(arc_BufferText, "Ref Near: %d", st_PlanningDataViewer.st_ReferenceLine2_global.s32_NearIdx);
        DrawText(arc_BufferText,0, 85, 12);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));

        sprintf(arc_BufferText, "PitStop Near: %d", st_PlanningDataViewer.st_ReferenceSELine2_global.s32_NearIdx);
        DrawText(arc_BufferText,0, 75, 12);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    if (st_PlanningDataViewer.st_Planner.u8_PitStopReady == 1)
    {
        sprintf(arc_BufferText, "PitStopReady:On");
        DrawText(arc_BufferText,0, 45, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else
    {
        sprintf(arc_BufferText, "PitStopReady:Off");
        DrawText(arc_BufferText,0, 45, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }

    if (st_PlanningDataViewer.st_Planner.u8_PitStopMode == 1)
    {
        sprintf(arc_BufferText, "PitStopMode:On");
        DrawText(arc_BufferText,0, 35, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else
    {
        sprintf(arc_BufferText, "PitStopMode:Off");
        DrawText(arc_BufferText,0, 35, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }



    ///////////////////////////////////////////




    //-------------------------------------Control Mode-------------------------------------//

    // if(st_ControlDataViewer.s32_Judgement == 0)
    // {
    //     sprintf(arc_BufferText, "Just Tracking");
    //     DrawText(arc_BufferText,72, 75, 12);
    //     memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    // }
    // else if(st_ControlDataViewer.s32_Judgement == 1)
    // {
    //     sprintf(arc_BufferText, "ACC ON");
    //     DrawText(arc_BufferText,72, 75, 12);
    //     memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    // }
    // else if(st_ControlDataViewer.s32_Judgement == 2)
    // {
    //     sprintf(arc_BufferText, "AEB");
    //     DrawText(arc_BufferText,72, 75, 12);
    //     memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    // }
}

void drawDashedLine(float32_t x1, float32_t y1, float32_t x2, float32_t y2) 
{
    glEnable(GL_LINE_STIPPLE);       
    glLineStipple(1, 0x00FF);        
    glBegin(GL_LINES);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glEnd();
    glDisable(GL_LINE_STIPPLE);      
}

void drawFilledCircle(float32_t f32_x, float32_t f32_y, float32_t f32_radius, int32_t s32_segments) 
{
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(f32_x, f32_y); 
    for (int32_t i = 0; i <= s32_segments; ++i) 
    {
        float32_t f32_angle = 2.0f * M_PI * float32_t(i) / float32_t(s32_segments);
        float32_t f32_dx = f32_radius * cosf(f32_angle);
        float32_t f32_dy = f32_radius * sinf(f32_angle);
        glVertex2f(f32_x + f32_dx, f32_y + f32_dy);
    }
    glEnd();
}

void drawRectangle(float32_t x, float32_t y, float32_t width, float32_t height) 
{
    glBegin(GL_LINE_STRIP);
    glVertex2f(x, y); 
    glVertex2f(x + width, y); 
    glEnd();

    glBegin(GL_LINE_STRIP);
    glVertex2f(x + width, y); 
    glVertex2f(x + width, y + height); 
    glEnd();

    glBegin(GL_LINE_STRIP);
    glVertex2f(x + width, y + height); 
    glVertex2f(x, y + height); 
    glEnd();

    glBegin(GL_LINE_STRIP);
    glVertex2f(x, y + height); 
    glVertex2f(x, y); 
    glEnd();
}

void drawFilledRectangle(float32_t x, float32_t y, float32_t width, float32_t height) 
{
    glBegin(GL_QUADS); // 사각형을 그리기 위해 GL_QUADS 시작
    glVertex2f(x, y); // 왼쪽 아래
    glVertex2f(x + width, y); // 오른쪽 아래
    glVertex2f(x + width, y + height); // 오른쪽 위
    glVertex2f(x, y + height); // 왼쪽 위
    glEnd(); // GL_QUADS 종료
}

void drawParaBola(float32_t p1x, float32_t p1y, float32_t p2x, float32_t p2y, float32_t p3x, float32_t p3y, int32_t segments) 
{
    float32_t a = ((p3y - p1y) * (p2x - p1x) - (p2y - p1y) * (p3x - p1x)) / ((p3x * p3x - p1x * p1x) * (p2x - p1x) - (p2x * p2x - p1x * p1x) * (p3x - p1x));
    float32_t b = (p2y - p1y - a * (p2x * p2x - p1x * p1x)) / (p2x - p1x);
    float32_t c = p1y - a * p1x * p1x - b * p1x;

    // p1x 부터 p3x 까지 이차함수를 그리기 시작
    glBegin(GL_LINE_STRIP);
    for (int32_t i = 0; i <= segments; ++i) {
        float32_t x = p1x + i * (p3x - p1x) / segments;
        float32_t y = a * x * x + b * x + c;
        glVertex2f(x, y);
    }
    glEnd();
}

void DrawControlBox()
{   
    Draw_GG_Diagram();
    DrawSteering();
    DrawThrrotleBrake();
    DrawControlCluster();
    DrawLapData();
}

void DrawLapData()
{
    glViewport(st_ViewerParam.s32_LocalX * 1.05f + st_ViewerParam.s32_LocalWidth * 0.7f,
               st_ViewerParam.s32_ScreenHeight * 0.05f, 
               st_ViewerParam.s32_LocalWidth * 0.25f,
               st_ViewerParam.s32_LocalHeight * 0.25f);

    float32_t f32_aspect = (float32_t)st_ViewerParam.s32_LocalWidth * 0.25f / (st_ViewerParam.s32_LocalHeight * 0.25f);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 100 * f32_aspect, 0, 100, -1, 1); 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    float32_t f32_Center_X = 0.5f * 100 * f32_aspect; 
    float32_t f32_Center_Y = 0.5f * 100;   
    char arc_BufferText[500] = {0};

    int32_t segments = 36; 
    float32_t angleStep = M_PI_2 / segments; 
    float32_t radius = 5.0f; 
    float32_t x = 5.f;
    float32_t y = 0.0f;
    float32_t width = 95.0f * f32_aspect;
    float32_t height = 80.0f;

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_STRIP);
    glVertex2f(x + width, y + height); 
    glVertex2f(x + radius, y + height); 
    glEnd();

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_STRIP);
    glVertex2f(x, y + height - radius); 
    glVertex2f(x, y); 
    glEnd();

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_STRIP);
    for (int32_t i = 0; i <= segments; i++) {
        float32_t angle = M_PI_2 + i * angleStep; 
        float32_t dx = radius * cos(angle);
        float32_t dy = radius * sin(angle);
        glVertex2f(x + radius + dx, y + height - radius + dy);
    }
    glEnd();

    sprintf(arc_BufferText, "Control Method: %d", st_ControlDataViewer.s32_ControlMethod);
    DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 1.4f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    if(st_ControlDataViewer.s32_V2XMode == 0)
    {
        sprintf(arc_BufferText, "V2X Mode : Stop");
        DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 1.25f, 12);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else if(st_ControlDataViewer.s32_V2XMode == 1)
    {
        sprintf(arc_BufferText, "V2X Mode : Steady State");
        DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 1.25f, 12);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else if(st_ControlDataViewer.s32_V2XMode == 2)
    {
        sprintf(arc_BufferText, "V2X Mode : Slow Down");
        DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 1.25f, 12);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else if(st_ControlDataViewer.s32_V2XMode == 3)
    {
        sprintf(arc_BufferText, "V2X Mode : Pit Stop");
        DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 1.25f, 12);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    }
    else if(st_ControlDataViewer.s32_V2XMode == 4)
    {
        sprintf(arc_BufferText, "V2X Mode : Test Mode");
        DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 1.25f, 12);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    drawDashedLine(10.f, 57.f, f32_aspect * 100, 57.f);

    sprintf(arc_BufferText, "LAP");
    DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 0.95f, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    sprintf(arc_BufferText, "Time");
    DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 0.8f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "%02d:%02d:%02d", st_DeadReckoningDataViewer.st_GPS.ars32_CurrentLapTime[0],
                                              st_DeadReckoningDataViewer.st_GPS.ars32_CurrentLapTime[1],
                                              st_DeadReckoningDataViewer.st_GPS.ars32_CurrentLapTime[2]);
    DrawText(arc_BufferText,f32_Center_X * 1.3f, f32_Center_Y * 0.8f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "LAP %d", st_DeadReckoningDataViewer.st_GPS.s32_LabCount);
    DrawText(arc_BufferText,f32_Center_X * 0.35f, f32_Center_Y * 0.6f, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    float32_t f32_Laplen = 0.f;
    if (InRange(0, 9, st_DeadReckoningDataViewer.st_GPS.s32_LabCount))
    {
        f32_Laplen = 18.f;
    }
    else if(InRange(10, 99, st_DeadReckoningDataViewer.st_GPS.s32_LabCount))
    {
        f32_Laplen = 21.f;
    }
    glLineWidth(1.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    drawRectangle(f32_Center_X * 0.3f, f32_Center_Y * 0.55f, f32_Laplen, 8.f);


    sprintf(arc_BufferText, "Best");
    DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 0.35f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    sprintf(arc_BufferText, "%02d:%02d:%02d", st_DeadReckoningDataViewer.st_GPS.ars32_BestLapTime[0], st_DeadReckoningDataViewer.st_GPS.ars32_BestLapTime[1], st_DeadReckoningDataViewer.st_GPS.ars32_BestLapTime[2]);
    DrawText(arc_BufferText,f32_Center_X * 1.3f, f32_Center_Y * 0.35f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Total");
    DrawText(arc_BufferText,f32_Center_X * 0.3f, f32_Center_Y * 0.15f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    sprintf(arc_BufferText, "%02d:%02d:%02d", st_DeadReckoningDataViewer.st_GPS.ars32_TotalTime[0], st_DeadReckoningDataViewer.st_GPS.ars32_TotalTime[1], st_DeadReckoningDataViewer.st_GPS.ars32_TotalTime[2]);
    DrawText(arc_BufferText,f32_Center_X * 1.3f, f32_Center_Y * 0.15f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

}

void DrawControlCluster()
{
    glViewport(st_ViewerParam.s32_LocalX * 1.05f + st_ViewerParam.s32_LocalWidth * 0.4f,
               st_ViewerParam.s32_ScreenHeight * 0.05f + st_ViewerParam.s32_LocalHeight * 0.1f, 
               st_ViewerParam.s32_LocalWidth * 0.3f,
               st_ViewerParam.s32_LocalHeight * 0.15f);

    float32_t f32_aspect = (float32_t)st_ViewerParam.s32_LocalWidth * 0.3f / (st_ViewerParam.s32_LocalHeight * 0.15f);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 100 * f32_aspect, 0, 100, -1, 1); 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    int32_t segments = 36; 
    float32_t angleStep = M_PI_2 / segments; 
    float32_t radius = 10.0f; 
    float32_t x = 0.0f;
    float32_t y = 0.0f;
    float32_t width = 100.0f * f32_aspect;
    float32_t height = 30.0f;

    float32_t f32_Center_X = 0.5f * 100 * f32_aspect; 
    float32_t f32_Center_Y = 0.5f * 100;   

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_STRIP);
    glVertex2f(x + width, y); 
    glVertex2f(x + width, y + height - radius); 
    glEnd();

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_STRIP);
    glVertex2f(x + width - radius, y + height); 
    glVertex2f(x + radius, y + height); 
    glEnd();

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_STRIP);
    glVertex2f(x, y + height - radius); 
    glVertex2f(x, y); 
    glEnd();

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_STRIP);
    for (int32_t i = 0; i <= segments; i++) 
    {
        float32_t angle = i * angleStep; 
        float32_t dx = radius * cos(angle);
        float32_t dy = radius * sin(angle);
        glVertex2f(x + width - radius + dx, y + height - radius + dy);
    }
    glEnd();

    glLineWidth(2.0f); 
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glBegin(GL_LINE_STRIP);
    for (int32_t i = 0; i <= segments; i++) {
        float32_t angle = M_PI_2 + i * angleStep; 
        float32_t dx = radius * cos(angle);
        float32_t dy = radius * sin(angle);
        glVertex2f(x + radius + dx, y + height - radius + dy);
    }
    glEnd();

    glColor3f(0.2f, 0.2f, 0.2f);
    glLineWidth(1.0f);
    drawDashedLine(f32_aspect * 100 * 0.33f, 0.f, f32_aspect * 100 * 0.33f, height);
    drawDashedLine(f32_aspect * 100 * 0.66f, 0.f, f32_aspect * 100 * 0.66f, height);

    char arc_BufferText[500] = {0};

    sprintf(arc_BufferText, "CAN Mode");
    DrawText(arc_BufferText, f32_Center_X * 0.12f , f32_Center_Y * 0.37f, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    if(st_CANDataViewer.u8_EPS_Control_Status == 0)
    {
        sprintf(arc_BufferText, "None");
        DrawText(arc_BufferText,f32_Center_X * 0.2f, f32_Center_Y * 0.1f, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else if(st_CANDataViewer.u8_EPS_Control_Status == 1)
    {
        sprintf(arc_BufferText, "Ready");
        DrawText(arc_BufferText,f32_Center_X * 0.2f, f32_Center_Y * 0.1f, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else if(st_CANDataViewer.u8_EPS_Control_Status == 2)
    {
        sprintf(arc_BufferText, "All On");
        DrawText(arc_BufferText,f32_Center_X * 0.2f, f32_Center_Y * 0.1f, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else if(st_CANDataViewer.u8_EPS_Control_Status == 3)
    {
        sprintf(arc_BufferText, "EStop");
        DrawText(arc_BufferText,f32_Center_X * 0.2f, f32_Center_Y * 0.1f, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }    
    else if(st_CANDataViewer.u8_EPS_Control_Status == 4)
    {
        sprintf(arc_BufferText, "Steer On");
        DrawText(arc_BufferText,f32_Center_X * 0.2f, f32_Center_Y * 0.1f, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }
    else
    {
        sprintf(arc_BufferText, "Error");
        DrawText(arc_BufferText,f32_Center_X * 0.2f, f32_Center_Y * 0.1f, 18);
        memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    }


    sprintf(arc_BufferText, "KM/H");
    DrawText(arc_BufferText, f32_Center_X * 0.89f , f32_Center_Y * 0.f , 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    float32_t f32_Pos_X = 0.f;
    if (InRange(0, 9, (int32_t)st_PlanningDataViewer.st_EgoVehicleData.f32_Velocity_kph_global))
    {
        f32_Pos_X = f32_Center_X * 0.96f;
    }
    else if (InRange(10, 99, (int32_t)st_PlanningDataViewer.st_EgoVehicleData.f32_Velocity_kph_global))
    {
        f32_Pos_X = f32_Center_X * 0.935f;
    }
    else if (InRange(100, 200, (int32_t)st_PlanningDataViewer.st_EgoVehicleData.f32_Velocity_kph_global))
    {
        f32_Pos_X = f32_Center_X * 0.89f;
    }

    sprintf(arc_BufferText, "%d", (int32_t)st_PlanningDataViewer.st_EgoVehicleData.f32_Velocity_kph_global);
    DrawText(arc_BufferText, f32_Pos_X , f32_Center_Y * 0.2f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "KM/H");
    DrawText(arc_BufferText, f32_Center_X * 1.55f , f32_Center_Y * 0.f , 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "%0.1f", st_ControlDataViewer.f32_TargetSpeed_kph);
    DrawText(arc_BufferText, f32_Center_X * 1.55f , f32_Center_Y * 0.2f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

}

void DrawThrrotleBrake()
{
    glViewport(st_ViewerParam.s32_LocalX * 1.05f + st_ViewerParam.s32_LocalWidth * 0.4f,
               st_ViewerParam.s32_ScreenHeight * 0.05f,
               st_ViewerParam.s32_LocalWidth * 0.3f,
               st_ViewerParam.s32_LocalHeight * 0.1f);

    float32_t f32_aspect = (float32_t)st_ViewerParam.s32_LocalWidth * 0.3f / (st_ViewerParam.s32_LocalHeight * 0.1f);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 100 * f32_aspect, 0, 100, -1, 1); 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    char arc_BufferText[500] = {0};

    // Draw Upper Lower Line
    glColor3f(0.7f, 0.7f, 0.7f); 
    glLineWidth(2.0f); 
    drawDashedLine(50.f, 13.f, f32_aspect * 100.f , 13.f);
    drawDashedLine(50.f, 83.f, f32_aspect * 100.f , 83.f);

    // Get Data
    int32_t s32_Throttle = (int32_t)(st_ControlDataViewer.f32_TargetThrottle * 20 / 2);
    int32_t s32_Brake = -(int32_t)(st_ControlDataViewer.f32_TargetThrottle * 20 / 4);
    GetModData(0, 20, s32_Throttle);
    GetModData(0, 20, s32_Brake);

    // Draw Throttle Box
    sprintf(arc_BufferText, "Throttle");
    DrawText(arc_BufferText, 0.f , 80.f, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    for (int32_t s32_i = 0; s32_i < 20; s32_i++)
    {
        if (s32_i <= s32_Throttle && st_ControlDataViewer.f32_TargetThrottle > 0)
        {
            float32_t f32_Green = s32_i / 20.f + 0.1f;
            glColor3f(0.f, f32_Green, 0.f);
        }
        else
        {
            glColor3f(0.2f, 0.2f, 0.2f);
        }

        drawFilledRectangle(2.f + s32_i * 5.f * f32_aspect, 53.f, 4.f * f32_aspect, 20.f); 
    }

    // Draw Brake Box
    sprintf(arc_BufferText, "Brake");
    DrawText(arc_BufferText, 0.f , 10.f, 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    for (int32_t s32_i = 0; s32_i < 20; s32_i++)
    {
        if (s32_i <= s32_Brake && st_ControlDataViewer.f32_TargetThrottle < 0)
        {
            float32_t f32_Red = s32_i / 20.f + 0.2f;
            glColor3f(f32_Red, 0.f, 0.f);
        }
        else
        {
            glColor3f(0.2f, 0.2f, 0.2f);
        }
        drawFilledRectangle(2.f + s32_i * 5.f * f32_aspect, 25.f, 4.f * f32_aspect, 20.f); 
    }
}

void DrawSteering()
{
    glViewport(st_ViewerParam.s32_LocalX * 1.05f + st_ViewerParam.s32_LocalWidth * 0.2,
               st_ViewerParam.s32_ScreenHeight * 0.05,
               st_ViewerParam.s32_LocalWidth * 0.2,
               st_ViewerParam.s32_LocalHeight * 0.25);

    float32_t f32_aspect = (float32_t)st_ViewerParam.s32_LocalWidth * 0.2 / (st_ViewerParam.s32_LocalHeight * 0.25);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 100 * f32_aspect, 0, 100, -1, 1); 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    char arc_BufferText[500] = {0};

    float32_t f32_Size = 30.f;
    float32_t f32_Center_X = 0.5f * 100 * f32_aspect; 
    float32_t f32_Center_Y = 0.5f * 100;    

    glLineWidth(5.0f); 
    glColor4f(0.2f, 0.2f, 0.2f, 0.5f);
    drawCircle(f32_Center_X, f32_Center_Y, f32_Size, 36, 1);

    glLineWidth(1.0f); 
    glColor4f(0.2f, 0.2f, 0.2f, 0.5f);
    drawDashedLine(f32_Center_X, f32_Center_Y - f32_Size, f32_Center_X, f32_Center_Y + f32_Size);
    drawDashedLine(f32_Center_X - f32_Size, f32_Center_Y, f32_Center_X + f32_Size, f32_Center_Y);

    glLineWidth(3.0f);   
    glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
    drawCircle(f32_Center_X, f32_Center_Y, 25.f, 48, 1);

    // Draw Steering Angle
    glPushMatrix();
    glTranslatef(f32_Center_X, f32_Center_Y, 0.0f); // 중심으로 이동
    glRotatef(st_CANDataViewer.f32_StrAng, 0.0f, 0.0f, 1.0f); // 각도에 따라 이미지 회전
    glTranslatef(-f32_Center_X, -f32_Center_Y, 0.0f); // 원래 위치로 복귀

    glColor4f(0.8f, 0.f, 0.f, 0.5f);
    drawFilledRectangle(f32_Center_X - 1.5f, f32_Center_Y + 23.5f, 4.f * f32_aspect, 2.5f); 
    glEnd();
    glPopMatrix();

    sprintf(arc_BufferText, "Steer Angle");
    DrawText(arc_BufferText, f32_Center_X * 0.67f , f32_Center_Y * 1.2f , 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "%0.1f", st_CANDataViewer.f32_StrAng / 12.9);
    DrawText(arc_BufferText, f32_Center_X * 0.87f , f32_Center_Y * 1.0f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));
    
    sprintf(arc_BufferText, "Target Angle");
    DrawText(arc_BufferText, f32_Center_X * 0.65f , f32_Center_Y * 0.8f , 12);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "%0.1f", -st_ControlDataViewer.f32_TargetSteer_deg);
    DrawText(arc_BufferText, f32_Center_X * 0.87f , f32_Center_Y * 0.6f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));
}

void Draw_GG_Diagram()
{
    glViewport(st_ViewerParam.s32_LocalX * 1.05f, 
               st_ViewerParam.s32_ScreenHeight * 0.05,
               st_ViewerParam.s32_LocalWidth * 0.2,
               st_ViewerParam.s32_LocalHeight * 0.25);

    float32_t f32_aspect = (float32_t)st_ViewerParam.s32_LocalWidth * 0.2 / (st_ViewerParam.s32_LocalHeight * 0.25);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 100 * f32_aspect, 0, 100, -1, 1); 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    char arc_BufferText[500] = {0};

    float32_t f32_Size = 30.f;
    float32_t f32_GGCenter_X = 0.5f * 100 * f32_aspect;  
    float32_t f32_GGCenter_Y = 0.5f * 100;    

    // Draw GG Center Line
    glColor3f(0.2f, 0.2f, 0.2f);
    glLineWidth(1.0f);
    drawDashedLine(f32_GGCenter_X, f32_GGCenter_Y - f32_Size, f32_GGCenter_X, f32_GGCenter_Y + f32_Size);
    drawDashedLine(f32_GGCenter_X - f32_Size, f32_GGCenter_Y, f32_GGCenter_X + f32_Size, f32_GGCenter_Y);

    // Draw GG Circle
    glColor3f(0.7f, 0.7f, 0.7f);
    glLineWidth(1.0f);
    drawCircle(f32_GGCenter_X, f32_GGCenter_Y, f32_Size, 36, 1);
    drawCircle(f32_GGCenter_X, f32_GGCenter_Y, f32_Size * 0.5, 36, 1);

    glColor3f(0.2f, 0.2f, 0.2f);
    drawCircle(f32_GGCenter_X, f32_GGCenter_Y, f32_Size * 0.75, 36, 1);
    drawCircle(f32_GGCenter_X, f32_GGCenter_Y, f32_Size * 0.25, 36, 1);

    // Draw GG Point
    float32_t f32_GG_X = 0;
    float32_t f32_GG_Y = 0;

    for (int32_t s32_i = 19; s32_i > 1; s32_i--)
    {
        glLineWidth(0.5f);
        if(s32_i < 11)
        {
            glColor3f(0.4f, 0.4f, 0.4f);
        }
        else
        {
            glColor3f(0.25f, 0.25f, 0.25f);
        }
        f32_GG_X = f32_GGCenter_X + (st_ControlDataViewer.st_GG_Diagram.arf32_AccelY[s32_i] * f32_Size);
        f32_GG_Y = f32_GGCenter_Y + (st_ControlDataViewer.st_GG_Diagram.arf32_AccelX[s32_i] * f32_Size);
        drawCircle(f32_GG_X, f32_GG_Y, 1.5f, 36, 1);
    }

    // Draw GG Max Line
    float32_t f32_GGMax_X = f32_GGCenter_X + (st_ControlDataViewer.st_GG_Diagram.f32_MaxAccelY * f32_Size);
    float32_t f32_GGMin_X = f32_GGCenter_X - (st_ControlDataViewer.st_GG_Diagram.f32_MaxAccelY * f32_Size);
    float32_t f32_GGMax_Y = f32_GGCenter_Y + (st_ControlDataViewer.st_GG_Diagram.f32_MaxAccelUpperX * f32_Size);
    float32_t f32_GGMin_Y = f32_GGCenter_Y + (st_ControlDataViewer.st_GG_Diagram.f32_MaxAccelLowerX * f32_Size);

    glLineWidth(1.0f);
    glColor3f(0.6f, 0.f, 0.6f);
    drawParaBola(f32_GGMin_X, f32_GGCenter_Y, f32_GGCenter_X, f32_GGMax_Y, f32_GGMax_X, f32_GGCenter_Y, 36);
    drawParaBola(f32_GGMin_X, f32_GGCenter_Y, f32_GGCenter_X, f32_GGMin_Y, f32_GGMax_X, f32_GGCenter_Y, 36);

    // Curret G-Force
    glColor3f(0.8f, 0.f, 0.f);
    f32_GG_X = f32_GGCenter_X + (st_ControlDataViewer.st_GG_Diagram.arf32_AccelY[0] * f32_Size);
    f32_GG_Y = f32_GGCenter_Y + (st_ControlDataViewer.st_GG_Diagram.arf32_AccelX[0] * f32_Size);
    drawFilledCircle(f32_GG_X, f32_GG_Y, 1.5f, 36);

    // G-Force Text
    float32_t f32_GForce = sqrtf(pow(st_ControlDataViewer.st_GG_Diagram.arf32_AccelX[0], 2) + pow(st_ControlDataViewer.st_GG_Diagram.arf32_AccelY[0], 2));
    sprintf(arc_BufferText, "%0.2fG", f32_GForce);
    DrawText(arc_BufferText, f32_GGCenter_X * 0.75f , f32_GGCenter_Y * 0.24f, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));
}


void LiDARIMU_Calibration(float32_t &f32_X, float32_t &f32_Y, float32_t &f32_Z, float32_t f32_Roll, float32_t f32_Pitch, float32_t f32_Yaw)
{
    float32_t RX, RY, RZ; 
    rotatePoint(f32_X, f32_Y, f32_Z, f32_Roll, f32_Pitch, f32_Yaw, RX, RY, RZ);
    f32_X = RX;
    f32_Y = RY;
    f32_Z = RZ;
}


void MouseButtonCallback(GLFWwindow* pst_Window, int32_t s32_Button, int32_t s32_Action, int32_t s32_Mods)
{
    int32_t s32_I = 0;
    float64_t f64_XPos, f64_YPos;
    glfwGetCursorPos(pst_Window, &f64_XPos, &f64_YPos);
    st_Pos.f64_X = f64_XPos;
    st_Pos.f64_Y = f64_YPos;
    uint64_t u64_TimeThresHold = 0;


    if (CheckingVariableSpace(f64_XPos, f64_YPos, st_ViewerParam.s32_GlobalX,st_ViewerParam.s32_GlobalY,
        st_ViewerParam.s32_GlobalWidth,  st_ViewerParam.s32_GlobalHeight))
    {
        st_ViewerParam.s32_CurrentWindow = c_VIEWER_GLOBAL;
    }
    else if (CheckingVariableSpace(f64_XPos, f64_YPos, st_ViewerParam.s32_LocalX,st_ViewerParam.s32_LocalY,
        st_ViewerParam.s32_LocalWidth,  st_ViewerParam.s32_LocalHeight))
    {
        st_ViewerParam.s32_CurrentWindow = c_VIEWER_LOCAL;
    }
    else if (CheckingVariableSpace(f64_XPos, f64_YPos, st_ViewerParam.s32_FrameX,st_ViewerParam.s32_FrameY,
        st_ViewerParam.s32_FrameWidth,  st_ViewerParam.s32_FrameHeight))
    {
        st_ViewerParam.s32_CurrentWindow = c_VIEWER_FRAME;
    }
    else if (CheckingVariableSpace(f64_XPos, f64_YPos, st_ViewerParam.s32_CameraX,st_ViewerParam.s32_CameraY,
        st_ViewerParam.s32_CameraWidth,  st_ViewerParam.s32_CameraHeight))
    {
        st_ViewerParam.s32_CurrentWindow = c_VIEWER_CAMERA;
    }
    else if (CheckingVariableSpace(f64_XPos, f64_YPos, st_ViewerParam.s32_MenuX,st_ViewerParam.s32_MenuY,
        st_ViewerParam.s32_MenuWidth,  st_ViewerParam.s32_MenuHeight))
    {
        st_ViewerParam.s32_CurrentWindow = c_VIEWER_MENU;
    }


    if (s32_Button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (s32_Action == GLFW_PRESS)
        { 
            if( st_Pos.f64_X >=  st_ViewerParam.s32_ScreenWidth * 0.015 &&
                st_Pos.f64_X <=  st_ViewerParam.s32_ScreenWidth * 0.085 &&
                st_Pos.f64_Y >= st_ViewerParam.s32_ScreenHeight * 0.630 &&
                st_Pos.f64_Y <= st_ViewerParam.s32_ScreenHeight * 0.650 )
                {
                    Go = true;
                }
             else if( st_Pos.f64_X >=  st_ViewerParam.s32_ScreenWidth * 0.085 &&
                st_Pos.f64_X <=  st_ViewerParam.s32_ScreenWidth * 0.155 &&
                st_Pos.f64_Y >= st_ViewerParam.s32_ScreenHeight * 0.630 &&
                st_Pos.f64_Y <= st_ViewerParam.s32_ScreenHeight * 0.650 )
                {
                    Stop = true;
                }
                else if( st_Pos.f64_X >=  st_ViewerParam.s32_ScreenWidth * 0.155 &&
                st_Pos.f64_X <=  st_ViewerParam.s32_ScreenWidth * 0.225 &&
                st_Pos.f64_Y >= st_ViewerParam.s32_ScreenHeight * 0.630 &&
                st_Pos.f64_Y <= st_ViewerParam.s32_ScreenHeight * 0.650 )
                {
                    SlowOn = true;

                }
                else if( st_Pos.f64_X >=  st_ViewerParam.s32_ScreenWidth * 0.225 &&
                st_Pos.f64_X <=  st_ViewerParam.s32_ScreenWidth * 0.295 &&
                st_Pos.f64_Y >= st_ViewerParam.s32_ScreenHeight * 0.630 &&
                st_Pos.f64_Y <= st_ViewerParam.s32_ScreenHeight * 0.650 )
                {
                    SlowOff = true;
                }
                else if( st_Pos.f64_X >=  st_ViewerParam.s32_ScreenWidth * 0.295 &&
                st_Pos.f64_X <=  st_ViewerParam.s32_ScreenWidth * 0.365 &&
                st_Pos.f64_Y >= st_ViewerParam.s32_ScreenHeight * 0.630 &&
                st_Pos.f64_Y <= st_ViewerParam.s32_ScreenHeight * 0.650 )
                {
                    PitStop = true;
                }
            if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_MENU)
            {
                if (st_ViewerParam.s32_CurrentMode == c_VIEWER_SIMUL_MODE)
                {
                    // Simulation Mode & Button Action
                    for (s32_I = 0; s32_I < c_VIEWER_BUTTON_NUM_SIMUL;s32_I++)
                    {
                        if (f64_XPos >= st_ViewerParam.s32_ScreenWidth * 0.9 && 
                            f64_XPos < st_ViewerParam.s32_ScreenWidth && 
                            f64_YPos > st_ViewerParam.s32_ScreenHeight * (0.03 * (s32_I))  && 
                            f64_YPos < st_ViewerParam.s32_ScreenHeight * (0.03 * (s32_I + 1)))
                        {
                            st_ViewerParam.s32_CurrentButton = s32_I;
                        }

                    }

                    if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_SIMUL_MODE)
                    {
                        st_ViewerParam.s32_CurrentMode = c_VIEWER_REAL_MODE;
                    }
                    else if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_SIMUL_LOAD_MAP)
                    {
                        st_ViewerParam.s32_State |= c_STATE_MODE_LOAD_MAP; 
                    }
                    else if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_SIMUL_RECORD_MAP)
                    {
                        st_ViewerParam.s32_State ^= c_STATE_MODE_RECORD_MAP;
                    }
                    else if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_SIMUL_SAVE_MAP)
                    {
                        st_ViewerParam.s32_State |= c_STATE_MODE_SAVE_MAP;
                    }
                    else if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_SIMUL_LOGGING)
                    {
                        if((st_ViewerParam.s32_State & c_STATE_MODE_LOGGING) == 0)
                        {
                            s32_FileNum += 1;
                            if(s32_FileFrameCnt == 0)
                            {
                                CreateFile();
                            }
                            else
                            {
                                s32_FileFrameCnt = 0;
                                CreateFile();
                            }
                        }

                        st_ViewerParam.s32_State ^= c_STATE_MODE_LOGGING; 
                    }
                    else if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_SIMUL_PLAY)
                    {
                        if (st_ViewerParam.s32_State & c_STATE_MODE_PLAY)
                        {
                            if (st_ViewerParam.s32_State & c_STATE_MODE_STOP)
                            {
                                st_ViewerParam.s32_State ^= c_STATE_MODE_STOP;
                            }
                        }
                        else
                        {
                            LoggingFirstRead();                        
                        }
                        
                        st_ViewerParam.s32_State ^= c_STATE_MODE_PLAY;
                    }

                    st_ViewerParam.s32_CurrentButton = c_VIEWER_BUTTON_NULL;

                    for (s32_I = 0; s32_I < s32_TotalFileNum;s32_I++)
                    {
                        if (f64_XPos >= st_ViewerParam.s32_ScreenWidth * 0.9 && 
                            f64_XPos < st_ViewerParam.s32_ScreenWidth && 
                            f64_YPos > (st_ViewerParam.s32_ScreenHeight * (0.02 * s32_I)) + st_ViewerParam.s32_ScreenHeight / 3.0267  &&
                            f64_YPos < (st_ViewerParam.s32_ScreenHeight * (0.02 * (s32_I + 1))) + st_ViewerParam.s32_ScreenHeight / 3.0267)
                        {
                            s32_SelectFileNum = s32_I;
                            break;
                        }
                    }

                }
                else if(st_ViewerParam.s32_CurrentMode == c_VIEWER_REAL_MODE)
                {
                    // Real Mode & Button Action
                    for (s32_I = 0; s32_I < c_VIEWER_BUTTON_NUM_REAL;s32_I++)
                    {
                        if (f64_XPos >= st_ViewerParam.s32_ScreenWidth * 0.9 && 
                            f64_XPos < st_ViewerParam.s32_ScreenWidth && 
                            f64_YPos > st_ViewerParam.s32_ScreenHeight * (0.03 * (s32_I ))  && 
                            f64_YPos < st_ViewerParam.s32_ScreenHeight * (0.03 * (s32_I + 1)))
                        {
                            st_ViewerParam.s32_CurrentButton = s32_I;
                        }
                    }


                    if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_REAL_MODE)
                    {
                        st_ViewerParam.s32_CurrentMode = c_VIEWER_SIMUL_MODE;
                    }
                    else if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_REAL_AUTOMODE_ON)
                    {
                        ;
                    }
                    else if (st_ViewerParam.s32_CurrentButton == c_VIEWER_BUTTON_REAL_LOGGING)
                    {
                        if((st_ViewerParam.s32_State & c_STATE_MODE_LOGGING) == 0)
                        {
                            s32_FileNum += 1;
                            CreateFile();
                        }

                        st_ViewerParam.s32_State ^= c_STATE_MODE_LOGGING;
                    }
                }

                st_ViewerParam.s32_CurrentButton = c_VIEWER_BUTTON_NULL;

                // Exit Button
                if (st_Pos.f64_X >= st_ViewerParam.s32_ScreenWidth*0.90 && 
                        st_Pos.f64_X <= st_ViewerParam.s32_ScreenWidth &&
                        st_Pos.f64_Y >= st_ViewerParam.s32_ScreenHeight * 0.955 && 
                        st_Pos.f64_Y <= st_ViewerParam.s32_ScreenHeight*0.985)
                {
                    glfwSetWindowShouldClose(pst_Window, GLFW_TRUE);
                    b_Running = false;
                }
                 
            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_LOCAL)
            {
                st_ViewerParam.b_TransLocalMap = true;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;
                u64_NextButtonActionTime = getMillisecond();
                s32_ActionButtonCount += 1;


            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_GLOBAL)
            {
                st_ViewerParam.b_TransGlobalMap = true;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;

            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_CAMERA)
            {
                ;
            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_FRAME)
            {
                st_ViewerParam.b_MoveFrame = true;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;
            }

        }
        else if (s32_Action == GLFW_RELEASE)
        {

            if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_LOCAL)
            {
                st_ViewerParam.b_TransLocalMap = true;
                st_Prev.f32_AngleX_L = st_Prev.f32_AngleX_L;
                st_Prev.f32_AngleY_L = st_Prev.f32_AngleY_L;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;
                u64_NextButtonActionTime = getMillisecond();
                u64_TimeThresHold = u64_NextButtonActionTime - u64_PrevButtonActionTime;
                if(u64_TimeThresHold < 250)
                {
                    st_Trans.f32_CentorX_L = 0;
                    st_Trans.f32_CentorY_L = 0;
                    st_Trans.f32_AngleX_L = 0;
                    st_Trans.f32_AngleY_L = 0;
                }
                u64_PrevButtonActionTime = u64_NextButtonActionTime;
            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_GLOBAL)
            {
                st_ViewerParam.b_TransGlobalMap = true;
                st_Prev.f32_AngleX_G = st_Prev.f32_AngleX_G;
                st_Prev.f32_AngleY_G = st_Prev.f32_AngleY_G;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;
                u64_NextButtonActionTime = getMillisecond();
                u64_TimeThresHold = u64_NextButtonActionTime - u64_PrevButtonActionTime;
                if(u64_TimeThresHold < 250)
                {
                    st_Trans.f32_CentorX_G = 0;
                    st_Trans.f32_CentorY_G = 0;
                    st_Trans.f32_AngleX_G = 0;
                    st_Trans.f32_AngleY_G = 0;
                }
                u64_PrevButtonActionTime = u64_NextButtonActionTime;
            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_CAMERA)
            {
                ;
            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_FRAME)
            {
                st_ViewerParam.b_MoveFrame = true;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;
            }

            st_ViewerParam.b_TransLocalMap = false;
            st_ViewerParam.b_TransGlobalMap = false;
            st_ViewerParam.b_MoveFrame = false;
            // st_ViewerParam.s32_CurrentWindow = c_VIEWER_NULL;
        } 
    }
    else if (s32_Button == GLFW_MOUSE_BUTTON_RIGHT )
    {
        if (s32_Action == GLFW_PRESS)
        {
            if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_LOCAL)
            {
                st_ViewerParam.b_RotateLocalMap = true;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;


            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_GLOBAL)
            {
                st_ViewerParam.b_RotateGlobalMap = true;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;

            }
        }
        else if (s32_Action == GLFW_RELEASE)
        {
            if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_LOCAL)
            {
                st_ViewerParam.b_RotateLocalMap = true;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;
            }
            else if (st_ViewerParam.s32_CurrentWindow == c_VIEWER_GLOBAL)
            {
                st_ViewerParam.b_RotateGlobalMap = true;
                st_ViewerParam.f32_PrevX = (float32_t)f64_XPos;
                st_ViewerParam.f32_PrevY = (float32_t)f64_YPos;
            }

            st_ViewerParam.b_RotateLocalMap = false;
            st_ViewerParam.b_RotateGlobalMap = false;

        }
    }
}

void ScrollCallback(GLFWwindow* pst_Window, float64_t f64_X_Offset, float64_t f64_Y_Offset)
{
    float32_t f32_ZoomSpeed = 0.1f;
    float64_t f64_XPos, f64_YPos;
    glfwGetCursorPos(pst_Window, &f64_XPos, &f64_YPos);

    if (CheckingVariableSpace(f64_XPos, f64_YPos, st_ViewerParam.s32_GlobalX,st_ViewerParam.s32_GlobalY,
        st_ViewerParam.s32_GlobalWidth,  st_ViewerParam.s32_GlobalHeight)) 
    {
        // Global 영역에서의 스크롤
        if (f64_Y_Offset > 0) 
        {
            st_ViewerParam.f32_GlobalZoomFactor = fminf(10.f, st_ViewerParam.f32_GlobalZoomFactor * 1.1f);
        } 
        else if (f64_Y_Offset < 0) 
        {
            st_ViewerParam.f32_GlobalZoomFactor = fmaxf(0.0001f, st_ViewerParam.f32_GlobalZoomFactor * 0.9f);
        }
    } 
    else if (CheckingVariableSpace(f64_XPos, f64_YPos, st_ViewerParam.s32_LocalX,st_ViewerParam.s32_LocalY,
        st_ViewerParam.s32_LocalWidth,  st_ViewerParam.s32_LocalHeight))
    {
        // Local 영역에서의 스크롤
        if (f64_Y_Offset > 0) 
        {
            st_ViewerParam.f32_LocalZoomFactor = fminf(10.f, st_ViewerParam.f32_LocalZoomFactor * 1.1f);
        } 
        else if (f64_Y_Offset < 0) 
        {
            st_ViewerParam.f32_LocalZoomFactor = fmaxf(0.01f, st_ViewerParam.f32_LocalZoomFactor * 0.9f);
        }
    }
}

void KeyboardCallback(GLFWwindow* pst_Window, int32_t s32_Key, int32_t s32_Scancode, int32_t s32_Action, int32_t Mods) 
{
    if ((s32_Key == GLFW_KEY_SPACE || s32_Key == GLFW_KEY_ENTER) && s32_Action == GLFW_RELEASE)
    {
        if(s32_SelectFileNum >= 0)
        {
            st_ViewerParam.s32_State ^= c_STATE_MODE_STOP;
            // if (st_ViewerParam.s32_State & c_STATE_MODE_STOP == 0)
            // {
            //     if(s32_FileFrameCnt == st_FileInfo.s32_CntFrame)
            //     {
            //         st_ViewerParam.s32_State = c_STATE_MODE_STOP;
            //         s32_FileFrameCnt = 0;
            //         st_FileInfo.s32_CntFrame = 0;
            //         st_ViewerParam.s32_FrameBarX = 0;
            //     }
            // }
        }
    }
    else if (s32_Key == GLFW_KEY_LEFT && (s32_Action == GLFW_RELEASE || s32_Action == GLFW_REPEAT))
    {
        if(s32_SelectFileNum >= 0)
        {
            st_ViewerParam.s32_State |= c_STATE_MODE_STOP;
            s32_FileFrameCnt = max(0, s32_FileFrameCnt - 1);
        }
    }
    else if (s32_Key == GLFW_KEY_RIGHT && (s32_Action == GLFW_RELEASE || s32_Action == GLFW_REPEAT))
    {
        if(s32_SelectFileNum >= 0)
        {
            st_ViewerParam.s32_State |= c_STATE_MODE_STOP;
            s32_FileFrameCnt = min(st_FileInfo.s32_CntFrame - 1, s32_FileFrameCnt + 1);
        }
    }
	else if (s32_Key == GLFW_KEY_G && s32_Action == GLFW_RELEASE) 
	{
		b_VisibleLidarGround = !b_VisibleLidarGround;
    }
	else if (s32_Key == GLFW_KEY_V && s32_Action == GLFW_RELEASE) 
	{
		b_VisibleLidarVoxel = !b_VisibleLidarVoxel;
    }
    else if(s32_Key == GLFW_KEY_J && s32_Action == GLFW_RELEASE)
    {
        b_VisibleLidarVoxelCoor = !b_VisibleLidarVoxelCoor;
    }
    else if(s32_Key == GLFW_KEY_I && s32_Action == GLFW_RELEASE)
    {
        b_VisibleLidarIntensity = !b_VisibleLidarIntensity;
    }
	else if (s32_Key == GLFW_KEY_C && s32_Action == GLFW_RELEASE) 
	{
		b_VisibleLidarCluster = !b_VisibleLidarCluster;
    }
    else if (s32_Key == GLFW_KEY_T && s32_Action == GLFW_RELEASE)
    {
        b_VisibleLidarTracking = !b_VisibleLidarTracking;
    }
    else if (s32_Key == GLFW_KEY_P && s32_Action == GLFW_RELEASE)
    {
        if(st_ViewerParam.s32_GlobalWindowState == c_STATE_GLOBALNONFIX)
        {
            st_Trans.f32_CentorX_G = 0.0f;
            st_Trans.f32_CentorY_G = 0.0f;
            st_ViewerParam.s32_GlobalWindowState = c_STATE_GLOBALFIX;
        }
        else if(st_ViewerParam.s32_GlobalWindowState == c_STATE_GLOBALFIX)
        {
            st_ViewerParam.s32_GlobalWindowState = c_STATE_GLOBALNONFIX;
        }
    }
    else if (s32_Key == GLFW_KEY_L && s32_Action == GLFW_RELEASE)
    {
        if(b_Start == c_STATE_PATHMAKEROFF)
        {
            b_Start = c_STATE_PATHMAKERON;
        }
        else if(b_Start == c_STATE_PATHMAKERON)
        {
            b_Start = c_STATE_PATHMAKEROFF;
        }
    }
    else if(s32_Key == GLFW_KEY_Q && s32_Action == GLFW_RELEASE)
    {
        if(s32_ImShowKey == 0)
        {
            s32_ImShowKey = 1;
        } 
        else
        {
            s32_ImShowKey = 0;
        } 
    }
    else if(s32_Key == GLFW_KEY_1 && s32_Action == GLFW_RELEASE)
    {
        if(st_PlanningDataViewer.s32_Round == 0)
        {   
            if (st_PlanningData.st_Planner.s32_TargetLane > 1)
            {
                st_PlanningData.st_Planner.s32_TargetLane--;
            }
        }
    }
    else if(s32_Key == GLFW_KEY_2 && s32_Action == GLFW_RELEASE)
    {
        if(st_PlanningDataViewer.s32_Round == 0)
        {
            if (st_PlanningData.st_Planner.s32_TargetLane < 17)
            {
                st_PlanningData.st_Planner.s32_TargetLane++;
            }
        }
    }
    else if(s32_Key == GLFW_KEY_SLASH && s32_Action == GLFW_RELEASE)
    {
        b_VisibleSingleAzimuth = !b_VisibleSingleAzimuth;
    }
    else if(s32_Key == GLFW_KEY_COMMA && (s32_Action == GLFW_RELEASE || s32_Action == GLFW_REPEAT))
    {
        s32_TargetAzimuthIndex = (s32_TargetAzimuthIndex + 1023) % 1024;
    }
    else if(s32_Key == GLFW_KEY_PERIOD && (s32_Action == GLFW_RELEASE  || s32_Action == GLFW_REPEAT))
    {
        s32_TargetAzimuthIndex = (s32_TargetAzimuthIndex + 1) % 1024;
    }
    else if(s32_Key == GLFW_KEY_LEFT_BRACKET && (s32_Action == GLFW_RELEASE  || s32_Action == GLFW_REPEAT))
    {
        s32_ICPIteration -= 1;
        s32_ICPIteration = max(0, s32_ICPIteration);
    }
    else if (s32_Key == GLFW_KEY_RIGHT_BRACKET && (s32_Action == GLFW_RELEASE  || s32_Action == GLFW_REPEAT))
    {
        s32_ICPIteration += 1;
        s32_ICPIteration = min(30, s32_ICPIteration);        
    }
    else if(s32_Key == GLFW_KEY_A && s32_Action == GLFW_RELEASE)
    {
        st_ViewerParam.b_AccToggle ^= true;
    }
    else if(s32_Key == GLFW_KEY_S && s32_Action == GLFW_RELEASE)
    {
        st_ViewerParam.b_SatToggle ^= true;        
    }
    else if(s32_Key == GLFW_KEY_D && s32_Action == GLFW_RELEASE)
    {
        st_ViewerParam.b_VertexToggle ^= true;        
    }
    else if(s32_Key == GLFW_KEY_F && s32_Action == GLFW_RELEASE)
    {
        st_ViewerParam.b_FrenetToggle ^= true;
    }

    else if(s32_Key == GLFW_KEY_INSERT && s32_Action == GLFW_RELEASE)
    {
        b_UpdateLocalMap = !b_UpdateLocalMap;
    }
    else if(s32_Key == GLFW_KEY_UP && s32_Action == GLFW_RELEASE)
    {  
        if (st_ControlDataViewer.s32_V2XMode == 4 && st_ControlData.f32_TargetSpeed_kph < st_ControlData.st_Param.f32_MAX_SPEED_kph)
        {
            st_ControlData.f32_TargetSpeed_kph += 5.0;
            st_ControlData.f32_TargetSpeed_m_s = st_ControlData.f32_TargetSpeed_kph / 3.6;
        }
    }
    else if(s32_Key == GLFW_KEY_DOWN && s32_Action == GLFW_RELEASE)
    {
        if (st_ControlDataViewer.s32_V2XMode == 4)
        {
            st_ControlData.f32_TargetSpeed_kph -= 5.0;
            st_ControlData.f32_TargetSpeed_m_s = st_ControlData.f32_TargetSpeed_kph / 3.6;
        }
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
 {
    st_ViewerParam.s32_ScreenHeight = height;
    st_ViewerParam.s32_ScreenWidth = width;
 }

void ListFilesInFolder(const string& s_Path) 
{
    s32_FileCnt = 0;

    if (!fs::exists(s_Path)) {
        // printf("%s를 찾을 수 없습니다.\n", s_Path.c_str());
        return;
    }

    std::vector<fs::path> files_in_directory;
    std::copy(fs::directory_iterator(s_Path), fs::directory_iterator(), std::back_inserter(files_in_directory));
    std::sort(files_in_directory.begin(), files_in_directory.end());

    st_LoggingFileList.clear(); // 기존 데이터를 모두 제거하고 새로운 파일 목록으로 대체
    
    for (const auto& file_path : files_in_directory) 
    {
        s32_FileCnt += 1;
        st_LoggingFileList.push_back(file_path.filename().string());
    }

}


void DrawMeterUnit(int32_t s32_Min, int32_t s32_Max, int32_t s32_Var, float32_t f32_UnitPos)
{
    int32_t s32_I = 0;
    char arc_ButtonText[100] = {0};

    for (s32_I = s32_Min; s32_I <= s32_Max; s32_I += s32_Var) 
    {
        sprintf(arc_ButtonText, "%d", s32_I);
        glColor3f(0.4f, 0.4f,0.f);
        DrawText(arc_ButtonText, s32_I, -f32_UnitPos, 10);
        DrawText(arc_ButtonText, f32_UnitPos, s32_I, 10);
    }
}

void DrawText(char* pc_Text, float32_t f32_X, float32_t f32_Y,int32_t s32_TextSize)
{
    glPushMatrix();
    char *pc_Temp = NULL;
    glColor3f(1.0f, 1.0f, 1.0f); // 글자 색상
    glRasterPos2f(f32_X, f32_Y); // 글자 위치 수정
    if(s32_TextSize == 10)
    {
        for (pc_Temp = pc_Text; *pc_Temp != '\0'; pc_Temp++)
        {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *pc_Temp);
        }
    }
    else if(s32_TextSize == 18)
    {
        for (pc_Temp = pc_Text; *pc_Temp != '\0'; pc_Temp++)
        {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *pc_Temp);        
        }
    }
    else if(s32_TextSize == 12)
    {
        for (pc_Temp = pc_Text; *pc_Temp != '\0'; pc_Temp++)
        {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *pc_Temp);        
        }
    }
    glEnd();
    glPopMatrix();

}


void DrawText3D(const char* text, float x, float y, float z)
{
    // 텍스트 위치를 설정합니다.
    glRasterPos3f(x, y, z);

    // 텍스트를 렌더링합니다.
    for (const char* c = text; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *c); // 헬베티카 18 크기로 렌더링
    }
}

void DrawNumber3D(int number, float x, float y, float z)
{
    char buffer[32];  // 숫자를 문자열로 변환하여 저장할 버퍼
    sprintf(buffer, "%d", number);  // 숫자를 문자열로 변환
    DrawText3D(buffer, x, y, z);  // 변환된 문자열을 텍스트 렌더링 함수로 전달
}

void DrawLabelWithNumber3D(const char* label, int number, float x, float y, float z)
{
    char buffer[64];  // 텍스트와 숫자를 결합하여 저장할 버퍼
    sprintf(buffer, "%s%03d", label, number);  // 문자열과 숫자를 결합하여 문자열 생성
    DrawText3D(buffer, x, y, z);  // 결합된 문자열을 텍스트 렌더링 함수로 전달
}

void DrawGridPoint(int32_t s32_Min, int32_t s32_Max, int32_t s32_Var)
{
    int32_t s32_I = 0;

    for (s32_I = s32_Min; s32_I <= s32_Max; s32_I += s32_Var) 
    {
        if(s32_I == 0)
        {
            glPointSize(50.f);
            glColor3f(1.f,0.f,0.f);
            glVertex2f(s32_I, 0);
            continue;
        }

        glPointSize(1.f);
        glBegin(GL_POINTS);
        glColor3f(1.f, 1.f, 1.f);
        glVertex2f(s32_I, 0);
        glVertex2f(0, s32_I);
	    glEnd();
    }
}

void DrawLine()
{
    glViewport(st_ViewerParam.s32_ScreenX, st_ViewerParam.s32_ScreenY, st_ViewerParam.s32_ScreenWidth, st_ViewerParam.s32_ScreenHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, st_ViewerParam.s32_ScreenWidth, 0.0, st_ViewerParam.s32_ScreenHeight, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glLineWidth(1.f);
    glBegin(GL_LINES);
    glColor3f(1.f, 1.f, 1.f);
    glVertex2f(st_ViewerParam.s32_ScreenWidth * 0.4, 0.05*st_ViewerParam.s32_ScreenHeight);
    glVertex2f(st_ViewerParam.s32_ScreenWidth * 0.4, st_ViewerParam.s32_ScreenHeight);

    glColor3f(1.f, 1.f, 1.f);
    glVertex2f(0.0f, st_ViewerParam.s32_ScreenHeight * 0.35);
    glVertex2f(st_ViewerParam.s32_ScreenWidth * 0.4, st_ViewerParam.s32_ScreenHeight * 0.35);

    glColor3f(1.f, 1.f, 1.f);
    glVertex2f(0.0f, st_ViewerParam.s32_ScreenHeight * 0.05);
    glVertex2f(st_ViewerParam.s32_ScreenWidth, st_ViewerParam.s32_ScreenHeight * 0.05);

    glColor3f(1.f, 1.f, 1.f);
    glVertex2f(st_ViewerParam.s32_ScreenWidth * 0.9, 0.0f);
    glVertex2f(st_ViewerParam.s32_ScreenWidth * 0.9, st_ViewerParam.s32_ScreenHeight);
    glEnd();
}

void DrawButton(float32_t f32_YPos, float32_t f32_Width, float32_t f32_Height, const char* pc_ButtonText, float32_t f32_CenterX)
{
    int32_t s32_MenuWidth = st_ViewerParam.s32_ScreenWidth * 0.1;
    int32_t s32_MenuHeight = st_ViewerParam.s32_ScreenHeight;

    if (f32_CenterX == -1.0f)
    {
        f32_CenterX = s32_MenuWidth / 2.0f;
    }

    float32_t f32_X = f32_CenterX - f32_Width / 2.0f;
    float32_t f32_Y = f32_YPos - f32_Height / 2.0f;

    glBegin(GL_QUADS);
    glColor3f(0.2f, 0.2f, 0.2f); // 버튼 배경 색상
    glVertex2f(f32_X, f32_Y);            // 좌상단
    glVertex2f(f32_X + f32_Width, f32_Y);    // 우상단
    glVertex2f(f32_X + f32_Width, f32_Y + f32_Height); // 우하단
    glVertex2f(f32_X, f32_Y + f32_Height);       // 좌하단
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3f(0.5f, 0.5f, 0.5f); // 테두리 색상
    glVertex2f(f32_X, f32_Y);
    glVertex2f(f32_X + f32_Width, f32_Y);
    glVertex2f(f32_X + f32_Width, f32_Y + f32_Height);
    glVertex2f(f32_X, f32_Y + f32_Height);
    glEnd();

    glColor3f(1.0f, 1.0f, 1.0f); // 글자 색상
    glRasterPos2f(f32_X + 50, f32_YPos - 3); // 글자 위치 수정

    int32_t s32_Length = strlen(pc_ButtonText);

    for (int i = 0; i < s32_Length; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, pc_ButtonText[i]);
    }
}


void DrawButtonColor(float32_t f32_YPos, float32_t f32_Width, float32_t f32_Height, const char* pc_ButtonText, float32_t f32_R, float32_t f32_G, float32_t f32_B, float32_t f32_CenterX)
{
    int32_t s32_MenuWidth = st_ViewerParam.s32_ScreenWidth * 0.1;
    int32_t s32_MenuHeight = st_ViewerParam.s32_ScreenHeight;
    if (f32_CenterX == -1.0f)
    {
        f32_CenterX = s32_MenuWidth / 2.0f;
    }

    float32_t f32_X = f32_CenterX - f32_Width / 2.0f;
    float32_t f32_Y = f32_YPos - f32_Height / 2.0f;
    
    
    glBegin(GL_QUADS);
    glColor3f(f32_R, f32_G, f32_B); // 버튼 배경 색상
    glVertex2f(f32_X, f32_Y);            // 좌상단
    glVertex2f(f32_X + f32_Width, f32_Y);    // 우상단
    glVertex2f(f32_X + f32_Width, f32_Y + f32_Height); // 우하단
    glVertex2f(f32_X, f32_Y + f32_Height);       // 좌하단
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3f(0.5f, 0.5f, 0.5f); // 테두리 색상
    glVertex2f(f32_X, f32_Y);
    glVertex2f(f32_X + f32_Width, f32_Y);
    glVertex2f(f32_X + f32_Width, f32_Y + f32_Height);
    glVertex2f(f32_X, f32_Y + f32_Height);
    glEnd();

    glColor3f(1.0f, 1.0f, 1.0f); // 글자 색상
    glRasterPos2f(f32_X + 50, f32_YPos-3 ); // 글자 위치 수정

    int32_t s32_Length = strlen(pc_ButtonText);
    
    for (int i = 0; i < s32_Length; i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, pc_ButtonText[i]);
    }
}

void DrawMenu()
{
    int32_t s32_I;
    char arc_BufferText[50] = {0};
    glViewport(st_ViewerParam.s32_MenuX, st_ViewerParam.s32_MenuY, st_ViewerParam.s32_MenuWidth, st_ViewerParam.s32_MenuHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, st_ViewerParam.s32_ScreenWidth*0.1, 0, st_ViewerParam.s32_ScreenHeight, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // DrawButton(0.05, 55.5 + 26, "CAMERA");
    // printf("current mode : %d current button : %d state : %d\n",st_ViewerParam.s32_CurrentMode, st_ViewerParam.s32_CurrentButton, st_ViewerParam.s32_State);
    if(st_ViewerParam.s32_CurrentMode == c_VIEWER_SIMUL_MODE)
    {
        ListFilesInFolder(s_LoggingPath);

        DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.985, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Simulation Mode", 0.5f, 0.5f, 0.f);
        DrawButton(st_ViewerParam.s32_ScreenHeight*0.955, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Load Map");

        if(st_ViewerParam.s32_State & c_STATE_MODE_LOAD_MAP)
        {
            DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.955, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Load Map", 0.5f, 0.4f, 0.35f);
        }

        DrawButton(st_ViewerParam.s32_ScreenHeight*0.925, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Update Map");

        if(st_ViewerParam.s32_State & c_STATE_MODE_RECORD_MAP)
        {
            DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.925, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Update Map", 0.3f, 0.4f, 0.8f);
        }


        DrawButton(st_ViewerParam.s32_ScreenHeight*0.895, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Save Map");

        if(st_ViewerParam.s32_State & c_STATE_MODE_SAVE_MAP)
        {
            DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.895, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "SAVE Map", 0.3f, 0.4f, 0.8f);
        }


        DrawButton(st_ViewerParam.s32_ScreenHeight*0.865, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Logging");
        
        if(st_ViewerParam.s32_State & c_STATE_MODE_LOGGING)
        {
            DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.865, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Logging", 0.9f, 0.6f, 0.1f);
        }

        DrawButton(st_ViewerParam.s32_ScreenHeight*0.835, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Play");
        
        if(st_ViewerParam.s32_State & c_STATE_MODE_PLAY)
        {
            DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.835, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Play", 0.3f, 0.7f, 0.7f);
        }

        for(s32_I = 0;s32_I < s32_FileCnt; s32_I++)
        {   
            DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.66 - st_ViewerParam.s32_ScreenHeight*0.02 * s32_I , st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.02, st_LoggingFileList[s32_I].c_str(), 0.1f, 0.1f, 0.1f);
        }

        if(s32_SelectFileNum >= 0)
        {
            DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.66 - st_ViewerParam.s32_ScreenHeight*0.02 * s32_SelectFileNum , st_ViewerParam.s32_ScreenWidth*0.1,st_ViewerParam.s32_ScreenHeight * 0.02, st_LoggingFileList[s32_SelectFileNum].c_str(), 0.5f, 0.5f, 0.5f);
        }


     
        s32_TotalFileNum = s32_FileCnt;
        s32_FileNum = s32_FileCnt;
        s32_FileCnt = 0;   


    }
    else if(st_ViewerParam.s32_CurrentMode == c_VIEWER_REAL_MODE)
    {
        DrawButtonColor(st_ViewerParam.s32_ScreenHeight*0.985, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Real Mode", 0.f, 0.5f, 0.5f);
        DrawButton(st_ViewerParam.s32_ScreenHeight*0.955, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Auto On");
        DrawButton(st_ViewerParam.s32_ScreenHeight*0.925, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Record");
    }
    DrawButton(st_ViewerParam.s32_ScreenHeight*0.03, st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight * 0.03, "Close");

    glColor3f(0.f, 0.3f, 0.f);
    glBegin(GL_POLYGON);
    glVertex2f(0, st_ViewerParam.s32_ScreenHeight*0.67);
    glVertex2f(0, st_ViewerParam.s32_ScreenHeight*0.817);
    glVertex2f(st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight*0.817);
    glVertex2f(st_ViewerParam.s32_ScreenWidth*0.1, st_ViewerParam.s32_ScreenHeight*0.67);
    glEnd();
    sprintf(arc_BufferText, "CAN : %lums", st_LogicHz.u64_CANHz);
    DrawText(arc_BufferText,st_ViewerParam.s32_ScreenWidth * 0.001, st_ViewerParam.s32_ScreenHeight*0.69, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "LiDAR : %lums", st_LogicHz.u64_LiDARHz);
    DrawText(arc_BufferText,st_ViewerParam.s32_ScreenWidth * 0.001, st_ViewerParam.s32_ScreenHeight*0.69 + 20, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Camera : %lums", st_LogicHz.u64_CameraHz);
    DrawText(arc_BufferText,st_ViewerParam.s32_ScreenWidth * 0.001, st_ViewerParam.s32_ScreenHeight*0.69 + 40, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "DeadReckoning : %lums", st_LogicHz.u64_DeadReckoningHz);
    DrawText(arc_BufferText,st_ViewerParam.s32_ScreenWidth * 0.001, st_ViewerParam.s32_ScreenHeight*0.69 + 60, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Planning : %.1fms", st_LogicHz.u64_PlanningHz * 0.001);
    DrawText(arc_BufferText,st_ViewerParam.s32_ScreenWidth * 0.001, st_ViewerParam.s32_ScreenHeight*0.69 + 80, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));

    sprintf(arc_BufferText, "Control : %lums", st_LogicHz.u64_ControlHz);
    DrawText(arc_BufferText,st_ViewerParam.s32_ScreenWidth * 0.001, st_ViewerParam.s32_ScreenHeight*0.69 + 100, 18);
    memset(&arc_BufferText, 0, sizeof(arc_BufferText));
}


void initializeWebcamTexture() {
    glGenTextures(1, &WebcamTexture);
    glBindTexture(GL_TEXTURE_2D, WebcamTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void updateWebcamTexture(const cv::Mat& m_Frame) {
    if (!m_Frame.empty())
    {
        glBindTexture(GL_TEXTURE_2D, WebcamTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_Frame.cols, m_Frame.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, m_Frame.ptr());
    }
}

void renderWebcamTexture() {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, WebcamTexture);

    // Set up orthographic projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glViewport(st_ViewerParam.s32_CameraX, st_ViewerParam.s32_CameraY, st_ViewerParam.s32_CameraWidth, st_ViewerParam.s32_CameraHeight);
    glOrtho(0.0, 640.0, 0.0, 1200.0, -1.0, 1.0);

    // Switch back to modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glBegin(GL_QUADS);
    // 좌표 시스템에 맞게 수정하고 상하 반전
    glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, 0.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex2f(640.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex2f(640.0f, 1200.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, 1200.0f);

    glEnd();

    // Reset the projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Reset the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_TEXTURE_2D);
    glDeleteTextures(1, &WebcamTexture);
}

void InitViewerVariable()
{   
    st_ViewerParam.s32_GlobalX = 0;                                                 // [글로벌] 0,370, 1200,690
    st_ViewerParam.s32_GlobalY = st_ViewerParam.s32_ScreenHeight * 0.35;                            // [로컬] 1200, 70, 500, 990
    st_ViewerParam.s32_GlobalWidth = st_ViewerParam.s32_ScreenWidth * 0.4;                          // [메뉴] 901, 70, 117.5 , 990
    st_ViewerParam.s32_GlobalHeight = st_ViewerParam.s32_ScreenHeight * 0.65;                       // [로스백] 901, 570, 117.5 , 490    
                                                                    // [프레임] 0, 0, 900, 70
    st_ViewerParam.s32_LocalX = st_ViewerParam.s32_ScreenWidth * 0.4;   //Local Map 가로             // [카메라] 0, 70, 1200, 300
    st_ViewerParam.s32_LocalY = st_ViewerParam.s32_ScreenHeight * 0.05;  //Local Map 세로
    st_ViewerParam.s32_LocalWidth = st_ViewerParam.s32_ScreenWidth * 0.5;  //
    st_ViewerParam.s32_LocalHeight = st_ViewerParam.s32_ScreenHeight * 0.95;

    st_ViewerParam.s32_MenuX = st_ViewerParam.s32_ScreenWidth * 0.900;
    st_ViewerParam.s32_MenuY = 0.0f;
    st_ViewerParam.s32_MenuWidth = st_ViewerParam.s32_ScreenWidth * 0.1;
    st_ViewerParam.s32_MenuHeight = st_ViewerParam.s32_ScreenHeight * 1;

    st_ViewerParam.s32_LoggingX = st_ViewerParam.s32_ScreenWidth * 0.900;
    st_ViewerParam.s32_LoggingY = st_ViewerParam.s32_ScreenHeight * 0.57;
    st_ViewerParam.s32_LoggingWidth = st_ViewerParam.s32_ScreenWidth * 0.1;
    st_ViewerParam.s32_LoggingHeight = st_ViewerParam.s32_ScreenHeight * 0.43;

    st_ViewerParam.s32_FrameX = 0;
    st_ViewerParam.s32_FrameY = 0;
    st_ViewerParam.s32_FrameWidth = st_ViewerParam.s32_ScreenWidth * 0.895;
    st_ViewerParam.s32_FrameHeight = st_ViewerParam.s32_ScreenHeight * 0.05;

    st_ViewerParam.s32_CameraX = 0;
    st_ViewerParam.s32_CameraY = st_ViewerParam.s32_ScreenHeight*0.05;
    st_ViewerParam.s32_CameraWidth = st_ViewerParam.s32_ScreenWidth*0.4;
    st_ViewerParam.s32_CameraHeight = st_ViewerParam.s32_ScreenHeight*0.3;
}


void DrawFrameBar() 
{
    float32_t f32_ScaleFactor;
    char arc_BufferText[50] = {0};
    char *pc_Temp = NULL;
    glViewport(st_ViewerParam.s32_FrameX, st_ViewerParam.s32_FrameY, st_ViewerParam.s32_FrameWidth, st_ViewerParam.s32_FrameHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 999, 0, 100, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    f32_ScaleFactor = 999/100.f;
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
    glColor3f(1.f, 1.f, 1.f);
    glVertex2f(1.f, 80.f);
    glVertex2f(999.f, 80.f); // X 좌표 변경
    glVertex2f(999.f, 20.f); // X 좌표 변경
    glVertex2f(1.f, 20.f);
    glEnd();

    if (st_FileInfo.s32_CntFrame > 0)
    {
        st_ViewerParam.s32_FrameBarX = (int32_t)((float32_t)s32_FileFrameCnt * 1000 / st_FileInfo.s32_CntFrame );
        // printf("%d %d %d %d\n", st_ViewerParam.s32_FrameBarX, st_ViewerParam.s32_FrameWidth, s32_FileFrameCnt, st_FileInfo.s32_CntFrame );
    }
    else
    {
        st_ViewerParam.s32_FrameBarX = 0;   
    }


    glBegin(GL_POLYGON);
    glColor3f(0.3f, 0.3f, 0.3f);
    glVertex2f(1.f, 80.f);
    glVertex2f(1.f, 20.f);
    glVertex2f(st_ViewerParam.s32_FrameBarX, 20.f);
    glVertex2f(st_ViewerParam.s32_FrameBarX, 80.f);
    glEnd();

    glBegin(GL_POLYGON);
    glColor3f(1.f, 1.f, 1.f);
    glVertex2f(st_ViewerParam.s32_FrameBarX - 0.3f * f32_ScaleFactor, 80.f);
    glVertex2f(st_ViewerParam.s32_FrameBarX - 0.3f * f32_ScaleFactor, 20.f);
    glVertex2f(st_ViewerParam.s32_FrameBarX + 0.1f * f32_ScaleFactor, 20.f);
    glVertex2f(st_ViewerParam.s32_FrameBarX + 0.1f * f32_ScaleFactor, 80.f);
    glEnd();

    sprintf(arc_BufferText, "%d/%d", s32_FileFrameCnt, st_FileInfo.s32_CntFrame);

    
    glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(15, 35);
    for (pc_Temp = arc_BufferText; *pc_Temp != '\0'; pc_Temp++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *pc_Temp);        
    }
    glEnd();

}

void DrawCamera()
{
    std::vector<char> st_EncodedImage;
    cv::Mat st_Image;


    if (st_CurrentData.st_RawCamera.s32_Num == 0)
    {
        return;
    }

    pthread_mutex_lock(&st_CurrentData.st_MutexCamera);
    
    st_EncodedImage.assign(st_CurrentData.st_RawCamera.arc_Buffer, 
                            st_CurrentData.st_RawCamera.arc_Buffer + st_CurrentData.st_RawCamera.s32_Num);

    st_Image = cv::imdecode(st_EncodedImage, cv::IMREAD_COLOR);
    
    pthread_mutex_unlock(&st_CurrentData.st_MutexCamera);

    DrawImageToTexture(st_Image);

    st_EncodedImage.clear();
}



void DrawImageToTexture(cv::Mat st_Image)
{
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, st_Image.cols, st_Image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, st_Image.ptr());

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, WebcamTexture);

    // Set up orthographic projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glViewport(st_ViewerParam.s32_CameraX, st_ViewerParam.s32_CameraY, st_ViewerParam.s32_CameraWidth, st_ViewerParam.s32_CameraHeight);
    glOrtho(0.0, 640.0, 0.0, 1200.0, -1.0, 1.0);

    // Switch back to modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glBegin(GL_QUADS);
    // 좌표 시스템에 맞게 수정하고 상하 반전
    glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, 0.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex2f(640.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex2f(640.0f, 1200.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, 1200.0f);

    glEnd();

    // Reset the projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Reset the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_TEXTURE_2D);
}



void DrawingMainViewer()
{
    DrawLine();
    DrawGlobalMap(&st_PlanningDataViewer);
    // DrawLocalMap();
    DrawHMCLocalMap(&st_HMCLidarDataViewer);
    DrawMenu();
    DrawCamera();
    DrawFrameBar();
}


void *Run_OpenGL(void *arg) {


    if(!glfwInit() || !glewInit()) {
        printf("Failed to initialize Viewer\n");
        exit(EXIT_FAILURE);
    }

    InitViewerVariable();

    pst_MainWindow = glfwCreateWindow(st_ViewerParam.s32_ScreenWidth, st_ViewerParam.s32_ScreenHeight, "Main Window", NULL, NULL);

    glfwMakeContextCurrent(pst_MainWindow);    

    glfwSetWindowPos(pst_MainWindow, st_ViewerParam.s32_ScreenX,st_ViewerParam.s32_ScreenY); //window 생성 위치를 바꿔줌 난 중앙에 나타나게 함
    glfwSetCursorPosCallback(pst_MainWindow, CursorPositionCallback);  //현재 내 커서의 위치를 파라미터에 반환함, 픽셀 단위이며 카메라 좌표계와 동일
    glfwSetMouseButtonCallback(pst_MainWindow, MouseButtonCallback);   //마우스가 눌렸는지 떨어졌는지 반환함 , 0이 왼쪽버튼 1이 오른쪽버튼
    glfwSetScrollCallback(pst_MainWindow, ScrollCallback);   //yoffset에 1(위로) -1(아래로) 반환
    glfwSetKeyCallback(pst_MainWindow, KeyboardCallback);   //키보드 눌렸을 때 key에 아스키코드로 무슨 키 눌렸는지 반환됨
    glfwMakeContextCurrent(pst_MainWindow);
    glfwSetFramebufferSizeCallback(pst_MainWindow, framebuffer_size_callback);
    initializeWebcamTexture();


    while (!glfwWindowShouldClose(pst_MainWindow) && b_Running) 
    {
        glClear(GL_COLOR_BUFFER_BIT);
        glfwMakeContextCurrent(pst_MainWindow);    
        InitViewerVariable();

        pthread_mutex_lock(&st_CopyMutex);
        // memcpy(&st_LidarDataViewer, &st_LidarDataTemporal, sizeof(LIDAR_DATA_t));
        memcpy(&st_HMCLidarDataViewer, &st_HMCLidarDataTemporal, sizeof(HMC_LIDAR_DATA_t));
        memcpy(&st_CameraDataViewer, &st_CameraDataTemporal, sizeof(CAMERA_DATA_t));
        memcpy(&st_DeadReckoningDataViewer, &st_DeadReckoningDataTemporal, sizeof(DEAD_RECKONING_DATA_t));
        memcpy(&st_PlanningDataViewer, &st_PlanningDataTemporal, sizeof(PLANNING_DATA_t));
        memcpy(&st_ControlDataViewer, &st_ControlDataTemporal, sizeof(CONTROL_DATA_t));
        memcpy(&st_CANDataViewer, &st_CANDataTemporal, sizeof(CAN_DATA_t));
        pthread_mutex_unlock(&st_CopyMutex);

        DrawingMainViewer();
        glfwSwapBuffers(pst_MainWindow);
        glfwPollEvents();

        usleep(1000 * 30);
    }

    b_Running = false;

    // Terminate GLFW
    glfwTerminate();
    pthread_exit(NULL); //1번 스레드 종료
}


