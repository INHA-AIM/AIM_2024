#ifndef VIEWER_H
#define VIEWER_H

#include "Global/global.h"
#include "LiDAR/lidar128.h"
#include "LiDAR/HMC_LiDAR.h"
#include "LiDAR/kernel.cuh"
#include "Camera/camera.h"
// #include "opencv2/opencv.hpp"       
// #include <sensor_msgs/CompressedImage.h>
#include <map>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLFW/glfw3.h>
#include <GL/freeglut.h>
#include <experimental/filesystem>  //파일 시스템 조작

namespace fs = experimental::filesystem; 

typedef struct _POS
{
    float64_t f64_X;
    float64_t f64_Y;
} POS_t;

typedef struct _EGO_POS
{
    float64_t f64_X;
    float64_t f64_Y;
    float64_t f64_Z;
} EGO_POS_t;

typedef struct _F32_INFO
{
    float32_t f32_CentorX_G;
    float32_t f32_CentorY_G;

    float32_t f32_CentorX_L;
    float32_t f32_CentorY_L;

    float32_t f32_AngleY_G;
    float32_t f32_AngleX_G;

    float32_t f32_AngleY_L;
    float32_t f32_AngleX_L;
} F32_INFO_t;

void DrawingMainViewer();
void DrawMenu();
void DrawFrameBar();
void DrawGlobalMap(PLANNING_DATA_t *pst_PlanningData, CAN_DATA_t *pst_CANData);
void DrawLocalMap();
void DrawHMCLocalMap(HMC_LIDAR_DATA_t* pst_LidarPointData);
void DrawLine();
void DrawCamera();
void DrawModeStatus();

void DrawControlBox();
void drawFilledCircle(float32_t f32_x, float32_t f32_y, float32_t f32_radius, int32_t s32_segments);
void drawDashedLine(float32_t x1, float32_t y1, float32_t x2, float32_t y2);
void drawRectangle(float32_t x, float32_t y, float32_t width, float32_t height);
void drawFilledRectangle(float32_t x, float32_t y, float32_t width, float32_t height);
void drawParaBola(float32_t p1x, float32_t p1y, float32_t p2x, float32_t p2y, float32_t p3x, float32_t p3y, int32_t segments);

void Draw_GG_Diagram();
void DrawSteering();
void DrawThrrotleBrake();
void DrawControlCluster();
void DrawLapData();

void DrawPosition(float32_t f32_E, float32_t f32_N, float32_t f32_Heading_rad);
void DrawOmni(HMC_LIDAR_DATA_t* pst_LidarData);
void DrawICP(HMC_LIDAR_DATA_t* pst_LidarData);
void drawCircle(float32_t x, float32_t y, float32_t radius, int segments, int mode);
void drawFlag(float32_t x, float32_t y, string color = "W");


void DrawButton(float32_t f32_YPos, float32_t f32_Width, float32_t f32_Height, const char* pc_ButtonText, float32_t f32_CenterX = -1.0f);
void DrawButtonColor(float32_t f32_YPos, float32_t f32_Width, float32_t f32_Height, const char* pc_ButtonText, float32_t f32_R, float32_t f32_G, float32_t f32_B, float32_t f32_CenterX = -1.0f);
void DrawMeterUnit(int32_t s32_Min, int32_t s32_Max, int32_t s32_Var, float32_t f32_UnitPos);
void DrawGridPoint(int32_t s32_Min, int32_t s32_Max, int32_t s32_Var);

void DrawText(char* pc_Text, float32_t f32_X, float32_t f32_Y, int32_t s32_TextSize);
void DrawText3D(const char* text, float x, float y, float z);
void DrawNumber3D(int number, float x, float y, float z);
void DrawLabelWithNumber3D(const char* label, int number, float x, float y, float z);

void DrawImageToTexture(cv::Mat st_Image);
void DrawBox(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2);
void DrawBox(float32_t f32_CenterX, float32_t f32_CenterY, float32_t f32_CenterZ, float32_t f32_Length, float32_t f32_Width, float32_t f32_Height, float32_t f32_Yaw_rad);
void DrawCarBody(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2);
void DrawWindows(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2);
void DrawWheels(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2);
void DrawCar(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2);

void InitGlobalMap(PLANNING_DATA_t *pst_PlanningData);
void InitLocalMap();
void InitEgoBox(float32_t f32_E, float32_t f32_N, float32_t f32_D);
const char* GetVehicleStateString(VEHICLE_STATE_t state);
const char* GetBankStateString(PLANNING_DATA_t *pst_PlanningData);
void LiDARIMU_Calibration(float32_t &f32_X, float32_t &f32_Y, float32_t &f32_Z, float32_t f32_Roll, float32_t f32_Pitch, float32_t f32_Yaw);


void framebuffer_size_callback(GLFWwindow* pst_Window, int width, int height);
void InitViewerVariable();


void CursorPositionCallback(GLFWwindow* pst_Window, float64_t f64_XPos, float64_t f64_YPos);
void MouseButtonCallback(GLFWwindow* pst_Window, int32_t s32_Bt, int32_t s32_Action, int32_t s32_Mods);
void ScrollCallback(GLFWwindow* pst_Window, float64_t f64_X_Offset, float64_t f64_Y_Offset);
void KeyboardCallback(GLFWwindow* pst_Window, int32_t s32_Key, int32_t s32_Scancode, int32_t s32_Action, int32_t s32_Mods);

void ListFilesInFolder(const string& s_Folder_path);
void executeCommand(const char* pc_Command);
void initializeWebcamTexture();
void updateWebcamTexture(const cv::Mat& m_Frame) ;
void renderWebcamTexture();
void Load_Path();
void Image_Callback(const sensor_msgs::CompressedImageConstPtr& msg);
void *Run_OpenGL(void *arg);

#endif
