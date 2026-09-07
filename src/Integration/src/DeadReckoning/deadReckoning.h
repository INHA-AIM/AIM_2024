#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include "Global/global.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <Integration/GNSSInfo.h>
#include <unsupported/Eigen/MatrixFunctions>




void EKF(RAW_GPSINS_DATA_t *pst_GPSINSData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void Initialization(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void get_GPSData(RAW_GPSINS_DATA_t *pst_GPSINSData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void get_IMUData(RAW_IMU_DATA_t *pst_IMUData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void InitialsettingEKFData(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void InitialsettingEKFObject(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
Eigen::MatrixXf skew(float32_t x, float32_t y, float32_t z);
void PredictState(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void IMUSetting(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
MatrixXf quat2DCM(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void DCM2eul_bn(DEAD_RECKONING_DATA_t *pst_DeadReckoningData, const Eigen::MatrixXf &matrix);
Eigen::MatrixXf reshape(Eigen::MatrixXf m, int32_t col, int32_t row);
void INS_ERROR_MODEL(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void updateBias(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void updateJacobianMatrix(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);

void INS_MECHANIZATION(DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void UpdataState(RAW_GPSINS_DATA_t *pst_GPSINSData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData);

void OdometricLocalization(DEAD_RECKONING_DATA_t *pst_DeadReckoningData, RAW_CAN_DATA_t *pst_CANData, HMC_LIDAR_DATA_t *pst_LidarData);
// void *INS_Kalman_Encoder(void *arg);
#endif // SUBANDPUB_H