#ifndef PARSER_H
#define PARSER_H

#include "Global/global.h"   
#include <vector>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include "opencv2/opencv.hpp"
#include <serial/serial.h>
#include <sensor_msgs/CompressedImage.h>
#include "morai_msgs/GPSMessage.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ArenaApi.h"
#include <Integration/Euler.h>
#include <Integration/Mode.h>
#include <curl/curl.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
typedef struct KalmanFilter 
{
    float64_t angle;
    float64_t bias;
    float64_t rate;
    float64_t P[2][2];
    float64_t Q_angle;
    float64_t Q_bias;
    float64_t R_measure;
} st_KF;
float64_t getAngle(st_KF& rst_KF, float64_t newAngle, float64_t newRate, float64_t f64_dT);

void LIDARVelodyne128Parser_UDP(const char *pc_Address, int32_t s32_Port);
void LIDARVelodyne128Parser_ROS();
void LIDAROuster128Parser_UDP(const char *pc_Address, int32_t s32_Port);
void *LIDARParserWrapper(void *p_Arg);
void* LiDARStatusWrapper(void* arg);
void LIDARFailSafe(std :: string s_LiDARHzMode);
size_t WriteCallback(void* pv_Contents, size_t u64_Size, size_t u64_NMEMB, string* ps_Response);

Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo>& deviceInfos);
void CameraParser_UDP(const char *pc_Address, int32_t s32_Port);
void CameraParser_Callback(const sensor_msgs::CompressedImageConstPtr& st_Msg);
void CameraParser_ROS(ros::NodeHandle *pst_NodeHandle);
void *CameraParserWrapper(void *p_Arg);


void GPS_IMU_Parser_Serial_AVANTE();
void GPSParser_MORAI_Callback(const sensor_msgs::NavSatFix::ConstPtr &st_Msg);
void GPSParser_MORAI(ros::NodeHandle *pst_NodeHandle);
void *GPSParserWrapper(void *p_Arg);


void IMUParser_MORAI_Callback(const sensor_msgs::Imu::ConstPtr &st_Msg);
void IMUParser_MORAI(ros::NodeHandle *pst_NodeHandle);
void *IMUParserWrapper(void *p_Arg);
void *OusterIMUParserWrapper(void *p_Arg);

void CreateDirectory(std::string& s_DirectoryName);
void MakeMap(GPS_DATA_t &st_GPSData, std::string& s_DirectoryName, std::string &FileName);
void MakeMapMove(std::string &s_DirectoryName, std::string &s_PathName);

#endif