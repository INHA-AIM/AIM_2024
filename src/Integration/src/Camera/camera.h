#ifndef CAMERA_H
#define CAMERA_H

#include "Global/global.h"
#include <sensor_msgs/CompressedImage.h>
// #include "Camera/inference.h"

void CameraProcessing(RAW_CAMERA_DATA_t *pst_RawData, CAMERA_DATA_t *pst_CameraData, PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData);
void OdomPathNearProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CAMERA_DATA_t *pst_CameraData);
void CalculatePerspectiveLaneCoef(const vector<Vec4i> st_Lines, LANE_COEFFICIENT_t& st_LaneCoefficient);
LANE_COEFFICIENT_t FitModelRANSAC(const Point& st_Pnt1, const Point& st_Pnt2, bool& b_Flag);
float32_t SlopeToDegree(float32_t f32_Slope);
void DrawLane(Mat& st_ResultImage, const Scalar st_Color, float32_t f32_Slope, float32_t f32_Intercept);
void RecalculateMedianSlope(float32_t& f32_MedianSlope, const vector<Vec4i> st_HoughLines);
LANE_COEFFICIENT_t FindSameLaneInfo(LANE_COEFFICIENT_t arst_LaneCoef[100], bool arb_Visited[100], int32_t s32_I, int32_t s32_SlopeCnt, const int32_t& s32_Width, const int32_t& s32_Height);
bool CheckSameLaneInfo(const LANE_COEFFICIENT_t& st_LaneCoef1, const LANE_COEFFICIENT_t& st_LaneCoef2, const int32_t& s32_Width, const int32_t& s32_Height);
int32_t CalculateXIntercept(const LANE_COEFFICIENT_t& st_LaneCoef, const int32_t& s32_Height);
Point CalculateCrossPoint(const LANE_COEFFICIENT_t& st_LaneCoef1, const LANE_COEFFICIENT_t& st_LaneCoef2);
void appendSlopeToFile(const std::string& filename, float32_t f32_Slope);
float32_t CalculateDegreeSlope(const Vec4i& st_Line);
float32_t ReCalculateSlope(const float32_t f32_Slope);
bool CheckNearWayPoint(const Point& st_LastPoint, const Point& st_CurrentPoint);
bool CheckValidVanishingPoint(const Point& st_Point, const int32_t& s32_Width, const int32_t& s32_Height);
bool FindNearLaneCoef(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_CurrentLaneCoef);
LANE_COEFFICIENT_t FindOptimalLane(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_BestLaneCoef
            , const LANE_COEFFICIENT_t& st_NewLaneCoef, const int32_t& s32_Height);
float32_t CalculateDifferenceDegreeSlope(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_CurrentLaneCoef);
void CheckSameHoughLineCluster(Mat& st_HoughLaneImage, vector<vector<Vec4i>> st_Lines, LANE_COEFFICIENT_t arst_LaneCoef[100], const int32_t& s32_SlopeCnt);
void CalcualteLaneCoefficient(LANE_COEFFICIENT_t arst_FinalLaneCoefLeft[100], LANE_COEFFICIENT_t arst_FinalLaneCoefRight[100], LANE_COEFFICIENT_t arst_LaneCoef[100]
    , bool arb_Visited[100], const int32_t& s32_SlopeCnt, LANE_STATUS_t& st_LaneStatus, const int32_t& s32_Width, const int32_t& s32_Height);
void CalculateOptimalLane(Mat& st_ResultImage,CAMERA_DATA_t* pst_CameraData, LANE_COEFFICIENT_t arst_FinalLaneCoefLeft[100], LANE_COEFFICIENT_t arst_FinalLaneCoefRight[100], LANE_COEFFICIENT_t arst_LaneCoef[100]
        , LANE_STATUS_t& st_LaneStatus, LANE_COEFFICIENT_t& st_MainLaneInfoLeft, LANE_COEFFICIENT_t& st_MainLaneInfoRight, const int32_t& s32_Width, const int32_t& s32_Height);
LANE_COEFFICIENT_t FindPitStopRightLane(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_BestLaneCoef
            , const LANE_COEFFICIENT_t& st_NewLaneCoef, const int32_t& s32_Height);
bool FindNearPitStopLane(const LANE_COEFFICIENT_t& st_LastLaneCoef1, const LANE_COEFFICIENT_t& st_LastLaneCoef2, const int32_t& s32_Height);
LANE_COEFFICIENT_t FindPitStopLeftLane(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_BestLaneCoef
            , const LANE_COEFFICIENT_t& st_NewLaneCoef, const int32_t& s32_Height);

#endif
