#include "camera.h"

extern Mat st_BGRImage;
extern SENSOR_DATA_t st_SensorData;
extern Mat st_IPMX;
extern Mat st_IPMY;
extern bool b_NoLaneLeft;
extern bool b_NoLaneRight;
extern CAMERA_LANEINFO_t st_LaneInfoLeftMain;
extern CAMERA_LANEINFO_t st_LaneInfoRightMain;
extern int32_t s32_ImShowKey;
extern Inference st_Inference;


void CameraProcessing(RAW_CAMERA_DATA_t *pst_RawData, CAMERA_DATA_t *pst_CameraData)
{
    // There is no Image
    if (pst_RawData->s32_Num == 0)
    { 
        return;
    }

    // Declare Variables
    vector<char> st_DecodedImage;
    vector<Detection> st_DetectedOutput;
    int32_t s32_DetectionCnt, s32_I;
    Detection st_DetectionInfo; 
    
    // Decode Image
    st_DecodedImage.assign(pst_RawData->arc_Buffer, pst_RawData->arc_Buffer + pst_RawData->s32_Num);
    st_BGRImage = imdecode(st_DecodedImage, IMREAD_COLOR);
    // ----------------------------------------------------------------------------------------------

    // Load Yolov8 Model
    st_DetectedOutput = st_Inference.runInference(st_BGRImage);
    s32_DetectionCnt = st_DetectedOutput.size();
    printf("Number of Detections:: %d\n",s32_DetectionCnt);


    for (s32_I = 0; s32_I < s32_DetectionCnt; s32_I++)
    {
        st_DetectionInfo = st_DetectedOutput[s32_I];
        if(st_DetectionInfo.className == "car" || st_DetectionInfo.className == "truck")
        {
            cv::Rect box = st_DetectionInfo.box;
            cv::Scalar color = st_DetectionInfo.color;

            cv::rectangle(st_BGRImage, box, color, 2);
            std::string classString = st_DetectionInfo.className + ' ' + std::to_string(st_DetectionInfo.confidence).substr(0, 4);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
            cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

            cv::rectangle(st_BGRImage, textBox, color, cv::FILLED);
            cv::putText(st_BGRImage, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);            
        }
    }

     // Show Processed Images    
    if(s32_ImShowKey == 1)
    {
        imshow("st_BGRImage",st_BGRImage);
        waitKey(1);
    }
    // -----------------------------------

}

void LoadParam(CAMERA_DATA_t *pst_CameraData)
{
    YAML::Node st_CameraParam = YAML::LoadFile("cameraParam.yaml");

    pst_CameraData->st_CameraParameter.s_IPMParameterX = st_CameraParam["IPMParameterX"].as<std::string>();
    pst_CameraData->st_CameraParameter.s_IPMParameterY = st_CameraParam["IPMParameterY"].as<std::string>();
    pst_CameraData->st_CameraParameter.s32_RemapHeight = st_CameraParam["RemapHeight"].as<int32_t>();
    pst_CameraData->st_CameraParameter.s32_RemapWidth  = st_CameraParam["RemapWidth"].as<int32_t>();

    // Kalman Object InitialLize
    pst_CameraData->s32_KalmanObjectNum = 0;
    pst_CameraData->f32_LastDistanceLeft = 0;
    pst_CameraData->f32_LastAngleLeft = 0;
    pst_CameraData->f32_LastDistanceRight = 0;
    pst_CameraData->f32_LastAngleRight = 0;
}  

LANE_COEFFICIENT_t FitModel(const Point& st_Point1, const Point& st_Point2, bool& b_Flag)
{
    LANE_COEFFICIENT_t st_TmpModel;
    
    if((st_Point2.x != st_Point1.x))
    {
        st_TmpModel.f64_Slope = float((st_Point2.y - st_Point1.y) / (st_Point2.x - st_Point1.x));
        st_TmpModel.f64_Intercept = int32_t(st_Point1.y - st_TmpModel.f64_Slope * st_Point1.x);
    }
    else
        b_Flag = false;

    return st_TmpModel;
}

void LoadMappingParam(CAMERA_DATA_t *pst_CameraData) 
{

    // Mat에서 원하는 DataType이 있기 때문에 s64_Value는 float형으로 설정해야 함
    float s64_Value;
    int32_t s32_Columns, s32_Rows;

    std::ifstream st_IPMParameters(pst_CameraData->st_CameraParameter.s_IPMParameterX);
    if (!st_IPMParameters.is_open()) {
        std::cerr << "Failed to open file: " << pst_CameraData->st_CameraParameter.s_IPMParameterX << std::endl;
        return;
    }
    st_IPMX.create(pst_CameraData->st_CameraParameter.s32_RemapHeight, pst_CameraData->st_CameraParameter.s32_RemapWidth, CV_32FC1);
    for (s32_Columns = 0; s32_Columns < pst_CameraData->st_CameraParameter.s32_RemapHeight; ++s32_Columns) {
        for (s32_Rows = 0; s32_Rows < pst_CameraData->st_CameraParameter.s32_RemapWidth; ++s32_Rows) {
            st_IPMParameters >> s64_Value;
            st_IPMX.at<float>(s32_Columns, s32_Rows) = s64_Value;
        }
    }
    st_IPMParameters.close();

    st_IPMParameters.open(pst_CameraData->st_CameraParameter.s_IPMParameterY);
    st_IPMY.create(pst_CameraData->st_CameraParameter.s32_RemapHeight, pst_CameraData->st_CameraParameter.s32_RemapWidth, CV_32FC1);
    for (s32_Columns = 0; s32_Columns < pst_CameraData->st_CameraParameter.s32_RemapHeight; ++s32_Columns) {
        for (s32_Rows = 0; s32_Rows <  pst_CameraData->st_CameraParameter.s32_RemapWidth; ++s32_Rows) {
            st_IPMParameters >> s64_Value;
            st_IPMY.at<float>(s32_Columns, s32_Rows) = s64_Value;
        }
    }
    st_IPMParameters.close();
}

// RANSAC 구현 및 Coefficient 추출 완료
// Data를 다루는 구조를 다시한번 생각 후 Kalman Filter까지 결합 진행