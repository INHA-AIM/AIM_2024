#include "camera.h"

extern Mat st_BGRImage;
extern SENSOR_DATA_t st_SensorData;
extern LANE_COEFFICIENT_t st_MainLaneInfoLeft;
extern LANE_COEFFICIENT_t st_MainLaneInfoRight;
extern int32_t s32_ImShowKey;
extern int32_t s32_CntWayPnt;
extern Point arst_LaneWayPnt[7];
extern int32_t s32_CntInvalidVanishingPnt;
extern bool b_ResetLaneWayPnt;
extern bool b_LastValidVanishingPnt;
extern bool b_FirstFrame;
extern LANE_STATUS_t st_LaneStatus;

void CameraProcessing(RAW_CAMERA_DATA_t *pst_RawData, CAMERA_DATA_t *pst_CameraData, PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
{
    // There is no Image
    if (pst_RawData->s32_Num == 0)
    { 
        return;
    }

    OdomPathNearProcessing(pst_PlanningData, pst_DeadReckoningData, pst_CameraData);

    // printf("b_NearPitStop: %d\n", pst_CameraData->b_NearPitStop);

    // Declare Variables
    int32_t s32_I, s32_J;
    std::vector<char> st_DecodedImage;

    Mat st_ProcessedImage, st_ResultImage, st_HoughLaneImage, st_TmpImage;

    // For Perspective Lane Detection
    vector<vector<Vec4i>> st_Lines(100);
    Vec4i st_Line;
    float32_t arf32_DegreeSlope[100], f32_DegreeSlope;
    int32_t ars32_XIntercept[100];
    bool b_ValidHoughLine[100];

    bool b_ThereIsHoughLane = false, b_SameCluster = false;
    bool b_NoMean = false;
    int32_t s32_XIntercept;
    int32_t s32_CntHoughLine = 0;

    LANE_COEFFICIENT_t arst_LaneCoef[100];

    bool arb_Visited[100] = {false};
    LANE_COEFFICIENT_t arst_FinalLaneCoefLeft[100];
    LANE_COEFFICIENT_t arst_FinalLaneCoefRight[100];

    int32_t s32_Width;
    int32_t s32_Height;

    Scalar st_Color;

    // -------------------------------------------------------------------

    // Decode Image
    st_DecodedImage.assign(pst_RawData->arc_Buffer, pst_RawData->arc_Buffer + pst_RawData->s32_Num);
    st_BGRImage = imdecode(st_DecodedImage, IMREAD_COLOR);

    s32_Width = st_BGRImage.cols;
    s32_Height = st_BGRImage.rows;

    st_BGRImage.copyTo(st_ResultImage);
    st_BGRImage.copyTo(st_HoughLaneImage);

    // Pre-Processing
    GaussianBlur(st_BGRImage, st_ProcessedImage, Size(3, 3), 0);

    cvtColor(st_ProcessedImage, st_ProcessedImage, COLOR_BGR2GRAY);
    Canny(st_ProcessedImage, st_TmpImage, 200, 300);

    // 허프 변환을 사용하여 차선 검출
    vector<Vec4i> st_HoughLines;
    HoughLinesP(st_TmpImage, st_HoughLines, 1, CV_PI / 360, 38, 26, 5);

    srand(time(0)); // 난수 초기화

    // 동일한 차선에 대한 Hough Line Clustering 및 Filtering
    for (s32_I = 0;s32_I < st_HoughLines.size();s32_I++)
    {
        st_Line = st_HoughLines[s32_I];
        f32_DegreeSlope = CalculateDegreeSlope(st_Line);

        // 제약조건 : 오르막길에서 감지되는 HoughLine 제거
        if(f32_DegreeSlope < 0 && st_Line[2] > 800)
            continue;

        if(f32_DegreeSlope > 0 && (st_Line[2] < 300) || (st_Line[3] < 115 && st_Line[1] < 115))
            continue;

        // 왼쪽 차선아 아닌데 가드라일 등 다른 것의 Edge가 왼쪽 차선으로 검출되는 경우
        if(f32_DegreeSlope < 0 && st_Line[0] > 700 )
            continue; 

        // 제약조건 : 수직, 수평 방향의 Hough Line, KIAPI 경사로 제거
        // if(abs(f32_DegreeSlope) < 20 || abs(f32_DegreeSlope) > 80 || ( st_Line[1] < 30 && st_Line[3] < 30 ))
        if(abs(f32_DegreeSlope) < 20 || abs(f32_DegreeSlope) > 80 || ( st_Line[1] < 130 && st_Line[3] < 130 ))
            continue;

        // printf("f32_DegreeSlope: %f, st_Line[0]: %d, st_Line[1]: %d, st_Line[2]: %d, st_Line[3]: %d\n"
        //     ,f32_DegreeSlope, st_Line[0], st_Line[1], st_Line[2], st_Line[3]);

        if(!b_ThereIsHoughLane)
        {
            arf32_DegreeSlope[s32_CntHoughLine] = f32_DegreeSlope;
            ars32_XIntercept[s32_CntHoughLine]  = CalculateXIntercept(FitModelRANSAC(Point(st_Line[0],st_Line[1]),Point(st_Line[2],st_Line[3]),b_NoMean), s32_Width);
            st_Lines[s32_CntHoughLine].push_back(st_Line);

            s32_CntHoughLine++;
            b_ThereIsHoughLane = true;
        }
        else
        {
            b_SameCluster = false;

            // 같은 차선에 대한 Hough Lane Clustering
            for (s32_J = 0;s32_J < s32_CntHoughLine;s32_J++)
            {
                if(abs(f32_DegreeSlope - arf32_DegreeSlope[s32_J]) < 3.5)     // Slope Threshold
                {
                    s32_XIntercept = CalculateXIntercept(FitModelRANSAC(Point(st_Line[0],st_Line[1]),Point(st_Line[2],st_Line[3]),b_NoMean), s32_Width);
                    
                    if(abs(s32_XIntercept - ars32_XIntercept[s32_J]) < 40)    // X Intercept Threshold      
                    {
                        // Clustering
                        st_Lines[s32_J].push_back(st_Line);
                        RecalculateMedianSlope(arf32_DegreeSlope[s32_J], st_Lines[s32_J]);
                        b_SameCluster = true;
                        break;
                    }
                }
            }

            if(!b_SameCluster)
            {
                // New Cluster
                arf32_DegreeSlope[s32_CntHoughLine] = f32_DegreeSlope;
                ars32_XIntercept[s32_CntHoughLine] = CalculateXIntercept(FitModelRANSAC(Point(st_Line[0],st_Line[1]),Point(st_Line[2],st_Line[3]),b_NoMean), s32_Width);
                st_Lines[s32_CntHoughLine].push_back(st_Line);
                s32_CntHoughLine++;           
            }
        }
    }

    CheckSameHoughLineCluster(st_HoughLaneImage, st_Lines, arst_LaneCoef, s32_CntHoughLine);
    CalcualteLaneCoefficient(arst_FinalLaneCoefLeft, arst_FinalLaneCoefRight, arst_LaneCoef, arb_Visited, s32_CntHoughLine, st_LaneStatus, s32_Width, s32_Height);
    
    CalculateOptimalLane(st_ResultImage, pst_CameraData, arst_FinalLaneCoefLeft, arst_FinalLaneCoefRight, arst_LaneCoef, st_LaneStatus, st_MainLaneInfoLeft, st_MainLaneInfoRight, s32_Width, s32_Height);

    // 실시간으로 검출한 차선 기반
    Point st_Point = CalculateCrossPoint(st_MainLaneInfoLeft,st_MainLaneInfoRight);

    if(b_FirstFrame)
    {
        pst_CameraData->s32_WayPntX = st_Point.x;
        pst_CameraData->s32_WayPntY = st_Point.y;
        pst_CameraData->st_LastLaneCoefLeft = st_MainLaneInfoLeft;
        pst_CameraData->st_LastLaneCoefRight = st_MainLaneInfoRight;
        b_FirstFrame = false;
    }

    // printf("CheckValidVanishingPoint : %d\n", CheckValidVanishingPoint(st_Point, s32_Width, s32_Height));
    // printf("CheckNearWayPoint        : %d\n", CheckNearWayPoint(Point(pst_CameraData->s32_WayPntX, pst_CameraData->s32_WayPntY),st_Point));
    // printf("s32_CntInvalidVanishingPnt           : %d\n", s32_CntInvalidVanishingPnt);

    if(CheckValidVanishingPoint(st_Point, s32_Width, s32_Height) 
        && CheckNearWayPoint(Point(pst_CameraData->s32_WayPntX, pst_CameraData->s32_WayPntY),st_Point))
    {
        b_LastValidVanishingPnt = true;
        s32_CntInvalidVanishingPnt = 0;

        pst_CameraData->st_LastLaneCoefLeft = st_MainLaneInfoLeft;
        pst_CameraData->st_LastLaneCoefRight = st_MainLaneInfoRight;
        // printf("Main Lane Update!!!!\n");
    }
    else
    {
        // st_Point 및 LastLaneCoef는 이전에 추출한 값으로 유지 
        st_Point = Point(pst_CameraData->s32_WayPntX, pst_CameraData->s32_WayPntY);

        // Count Invalid VanishingPoint Detection
        if(!b_LastValidVanishingPnt)
        {
            s32_CntInvalidVanishingPnt++;
        }
        else
        {
            b_LastValidVanishingPnt = false;
        }
        
        if(s32_CntInvalidVanishingPnt > 3)
        {
            s32_CntWayPnt = 0;
        }
    }

    // 3번 연속 지나간 경우 WayPoint가 현재 실시간 검출되는 Point를 바탕으로 계산되게 진행
    if(s32_CntInvalidVanishingPnt > 3)
    {
        b_LastValidVanishingPnt = true;
        s32_CntInvalidVanishingPnt = 0;

        st_Point = CalculateCrossPoint(st_MainLaneInfoLeft,st_MainLaneInfoRight);
        pst_CameraData->st_LastLaneCoefLeft = st_MainLaneInfoLeft;
        pst_CameraData->st_LastLaneCoefRight = st_MainLaneInfoRight;        
    }


    // //For Checking
    // if(CheckValidVanishingPoint(st_Point, s32_Width, s32_Height))
    //     printf("Valid Vanishing Point\n");
    // else
    //     printf("InValid Vanishing Point\n");

    // Draw Lane
    if(st_MainLaneInfoLeft.f32_Slope != 0 && st_MainLaneInfoLeft.f32_Intercept != 0)
    {
        st_Color = Scalar(rand() % 256, rand() % 256, rand() % 256);
        DrawLane(st_ResultImage, st_Color,st_MainLaneInfoLeft.f32_Slope,st_MainLaneInfoLeft.f32_Intercept);        
    }

    if(st_MainLaneInfoRight.f32_Slope != 0 && st_MainLaneInfoRight.f32_Intercept != 0)
    {
        st_Color = Scalar(rand() % 256, rand() % 256, rand() % 256);
        DrawLane(st_ResultImage, st_Color,st_MainLaneInfoRight.f32_Slope,st_MainLaneInfoRight.f32_Intercept);        
    }

    // 최종 Vanishing Point 추출

    Point st_TmpPoint;
    int32_t s32_Weight = 1;
    int32_t s32_TotalWeight = 0;

    if(s32_CntWayPnt<7)
    {
        if(CheckValidVanishingPoint(st_Point, s32_Width, s32_Height))
        {
            arst_LaneWayPnt[s32_CntWayPnt] = st_Point;

            if(s32_CntWayPnt == 0)
            {
                st_TmpPoint = st_Point;
            }
            else
            {
                st_TmpPoint.x = 0;
                st_TmpPoint.y = 0;

                for(s32_I=0;s32_I<s32_CntWayPnt;s32_I++)
                {
                    if(s32_CntWayPnt > 2)
                    {
                        if(s32_I==s32_CntWayPnt-2 || s32_I==s32_CntWayPnt-1)
                            s32_Weight+=5;                   
                    }


                    st_TmpPoint.x += arst_LaneWayPnt[s32_I].x * s32_Weight;
                    st_TmpPoint.y += arst_LaneWayPnt[s32_I].y * s32_Weight;
                    s32_TotalWeight += s32_Weight;
                }

                st_TmpPoint.x = static_cast<int32_t>(st_TmpPoint.x/s32_TotalWeight);
                st_TmpPoint.y = static_cast<int32_t>(st_TmpPoint.y/s32_TotalWeight);
            }

            pst_CameraData->s32_WayPntX = st_TmpPoint.x;
            pst_CameraData->s32_WayPntY = st_TmpPoint.y;
            
            cv::circle(st_HoughLaneImage, st_TmpPoint, 5, cv::Scalar(255, 255, 255), -1); 

            s32_CntWayPnt++;
        }
    }
    else
    {
        for(s32_I=1;s32_I<s32_CntWayPnt;s32_I++)
        {
            arst_LaneWayPnt[s32_I-1] = arst_LaneWayPnt[s32_I];
        }

        arst_LaneWayPnt[s32_CntWayPnt-1] = st_Point;

        st_TmpPoint.x = 0;
        st_TmpPoint.y = 0;

        for(s32_I=0;s32_I<s32_CntWayPnt;s32_I++)
        {
            if(s32_I==s32_CntWayPnt-2 || s32_I==s32_CntWayPnt-1)
                s32_Weight+=5;

            st_TmpPoint.x += arst_LaneWayPnt[s32_I].x * s32_Weight;
            st_TmpPoint.y += arst_LaneWayPnt[s32_I].y * s32_Weight;
            s32_TotalWeight += s32_Weight;
        }

        st_TmpPoint.x = static_cast<int32_t>(st_TmpPoint.x/s32_TotalWeight);
        st_TmpPoint.y = static_cast<int32_t>(st_TmpPoint.y/s32_TotalWeight);
        
        cv::circle(st_HoughLaneImage, st_TmpPoint, 5, cv::Scalar(255, 255, 255), -1); 

        pst_CameraData->s32_WayPntX = st_TmpPoint.x;
        pst_CameraData->s32_WayPntY = st_TmpPoint.y;
    }

    pst_CameraData->f32_DistLeft = abs(CalculateXIntercept(st_MainLaneInfoLeft, s32_Height)- 436);
    pst_CameraData->f32_DistRight = abs(CalculateXIntercept(st_MainLaneInfoRight, s32_Height)-436);

    Point pt1;

    if(!st_LaneStatus.b_NoLaneLeft && !st_LaneStatus.b_NoLaneRight)
    {
        pt1 = CalculateCrossPoint(st_MainLaneInfoLeft, st_MainLaneInfoRight);
        cv::circle(st_ResultImage, pt1, 5, cv::Scalar(0, 0, 255), -1); // 빨간색 원
    }
    else
    {
        cv::circle(st_ResultImage, CalculateCrossPoint(st_MainLaneInfoLeft,st_MainLaneInfoRight), 5, cv::Scalar(255, 100, 255), -1); // 빨간색 원    
    }

    // 최종 Output(소실점, LeftLane, Right Lane 추출)
    // appendSlopeToFile("/home/autonav/Slop.txt", st_KFState.f32_Angle);
    // appendSlopeToFile("/home/autonav/Distance.txt", st_KFState.f32_Distance);


    // printf("--------------------------------------------------------------------------\n");

    // Reset Parameter
    st_LaneStatus.b_NoLaneLeft       = true;
    st_LaneStatus.b_NoLaneRight      = true;
    st_LaneStatus.b_IsNotCornerLeft  = true;
    st_LaneStatus.b_IsNotCornerRight = true;

    st_LaneStatus.s32_CntValidLaneLeft  = 0;
    st_LaneStatus.s32_CntValidLaneRight = 0;
    // -----------------------------

    // Show Processed Images    
    static bool isResultImageWindowOpen = false;
    static bool isHoughLaneImageWindowOpen = false;

    if(s32_ImShowKey == 1)
    {
        imshow("st_ResultImage", st_ResultImage);
        imshow("st_HoughLaneImage", st_HoughLaneImage);
        waitKey(1);

        isResultImageWindowOpen = true;
        isHoughLaneImageWindowOpen = true;
    }
    else
    {
        if(isResultImageWindowOpen)
        {
            destroyWindow("st_ResultImage");
            isResultImageWindowOpen = false;
        }
        
        if(isHoughLaneImageWindowOpen)
        {
            destroyWindow("st_HoughLaneImage");
            isHoughLaneImageWindowOpen = false;
        }
    }
}

void OdomPathNearProcessing(PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData, CAMERA_DATA_t *pst_CameraData)
{
    VEHICLE_ODOMETRY_t *pst_Vehicle_Odometry = &pst_DeadReckoningData->st_Vehicle_Odometry;

    int32_t s32_NearIdx;
    float32_t f32_NearDist;


    // CalcNearestDistIdx(pst_Vehicle_Odometry->f32_PredictX, pst_Vehicle_Odometry->f32_PredictY                                // Odomatry EN
    CalcNearestDistIdx(pst_PlanningData->st_EgoVehicleData.f32_X_global, pst_PlanningData->st_EgoVehicleData.f32_Y_global       // Ego Vehicle EN
      ,pst_PlanningData->st_ReferenceLine2_global.arf32_X, pst_PlanningData->st_ReferenceLine2_global.arf32_Y
      ,pst_PlanningData->st_ReferenceLine2_global.s32_Num, f32_NearDist, s32_NearIdx);
    pst_CameraData->s32_Lane2Near = s32_NearIdx;

    // CalcNearestDistIdx(pst_Vehicle_Odometry->f32_PredictX, pst_Vehicle_Odometry->f32_PredictY                                // Odomatry EN
    CalcNearestDistIdx(pst_PlanningData->st_EgoVehicleData.f32_X_global, pst_PlanningData->st_EgoVehicleData.f32_Y_global       // Ego Vehicle EN
      ,pst_PlanningData->st_ReferenceSELine2_global.arf32_X, pst_PlanningData->st_ReferenceSELine2_global.arf32_Y
      ,pst_PlanningData->st_ReferenceSELine2_global.s32_Num, f32_NearDist, s32_NearIdx);
    pst_CameraData->s32_PitStopNear = s32_NearIdx;

    if( pst_CameraData->s32_PitStopNear > 160 && pst_CameraData->s32_PitStopNear < 230 )
    {
        pst_CameraData->b_NearPitStopR = true;

        if(pst_CameraData->s32_PitStopNear > 185)
        {
            pst_CameraData->b_NearPitStopL = true;
        }
        else
        {
            pst_CameraData->b_NearPitStopL = false;
        }

    }
    else
    {
        pst_CameraData->b_NearPitStopR = false;
        pst_CameraData->b_NearPitStopL = false;
    }
}

void CalculateOptimalLane(Mat& st_ResultImage, CAMERA_DATA_t* pst_CameraData, LANE_COEFFICIENT_t arst_FinalLaneCoefLeft[100], LANE_COEFFICIENT_t arst_FinalLaneCoefRight[100], LANE_COEFFICIENT_t arst_LaneCoef[100]
        , LANE_STATUS_t& st_LaneStatus, LANE_COEFFICIENT_t& st_MainLaneInfoLeft, LANE_COEFFICIENT_t& st_MainLaneInfoRight, const int32_t& s32_Width, const int32_t& s32_Height)
{

    int32_t s32_I;
    float32_t f32_DiffDegreeSlope = 0;
    Scalar st_Color;
    // LANE_COEFFICIENT_t st_FinalLaneCoefRight;
    // LANE_COEFFICIENT_t st_FinalLaneCoefLeft;


    // 최종적으로 소실점을 추출할 Lane 선택
    if(!st_LaneStatus.b_NoLaneLeft)
    {
        if(st_LaneStatus.b_IsNotCornerLeft) // 직선 -> Slope가 작은 차선이 Best
        {
            // printf("Straight\n");
 
            // printf("-----\nLast Left Lane X Intercept: %d\n",CalculateXIntercept(st_MainLaneInfoLeft, s32_Height));
            // printf("Last Left Lane Slope      : %f\n-----\n", SlopeToDegree(st_MainLaneInfoLeft.f32_Slope));

            st_MainLaneInfoLeft = arst_FinalLaneCoefLeft[0];
            f32_DiffDegreeSlope  = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft);

            st_Color = Scalar(rand() % 256, rand() % 256, rand() % 256);
            DrawLane(st_ResultImage, st_Color,arst_FinalLaneCoefLeft[0].f32_Slope,arst_FinalLaneCoefLeft[0].f32_Intercept);  
            
            // printf("Last Lane - Slope: %f, XIntercept: %d\n", SlopeToDegree(pst_CameraData->st_LastLaneCoefLeft.f32_Slope), CalculateXIntercept(pst_CameraData->st_LastLaneCoefLeft, s32_Height));
            // printf("%d's Lane - Slope: %f, XIntercept: %d\n", 0, SlopeToDegree(arst_FinalLaneCoefLeft[0].f32_Slope), CalculateXIntercept(arst_FinalLaneCoefLeft[0], s32_Height));

            for (s32_I = 1;s32_I < st_LaneStatus.s32_CntValidLaneLeft;s32_I++)
            {   

                st_Color = Scalar(rand() % 256, rand() % 256, rand() % 256);
                DrawLane(st_ResultImage, st_Color,arst_FinalLaneCoefLeft[s32_I].f32_Slope,arst_FinalLaneCoefLeft[s32_I].f32_Intercept);  

                // printf("st_MainLaneInfoLeft - Slope: %f, XIntercept: %d\n", SlopeToDegree(st_MainLaneInfoLeft.f32_Slope), CalculateXIntercept(st_MainLaneInfoLeft, s32_Height));
                // printf("%d's Lane - Slope: %f, XIntercept: %d\n", s32_I, SlopeToDegree(arst_FinalLaneCoefLeft[s32_I].f32_Slope), CalculateXIntercept(arst_FinalLaneCoefLeft[s32_I], s32_Height));


                if( pst_CameraData->b_PitStop && pst_CameraData->b_NearPitStopL )
                {
                    if(CalculateXIntercept(arst_FinalLaneCoefLeft[s32_I], s32_Height) < 0)
                    {
                        continue;
                    }


                    st_MainLaneInfoLeft = FindPitStopLeftLane(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft, arst_FinalLaneCoefLeft[s32_I], s32_Height);
                    f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);                            

                    continue;
                }


                if(CalculateXIntercept(arst_FinalLaneCoefLeft[s32_I], s32_Height) < 0)
                {
                    // printf("이런 상황입니다!\n");
                    st_MainLaneInfoLeft = FindOptimalLane(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft, arst_FinalLaneCoefLeft[s32_I], s32_Height);
                    f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft);
                }


                if(SlopeToDegree(st_MainLaneInfoLeft.f32_Slope) > SlopeToDegree(arst_FinalLaneCoefLeft[s32_I].f32_Slope))
                {
                    if(!FindNearLaneCoef(pst_CameraData->st_LastLaneCoefLeft,st_MainLaneInfoLeft))
                    {
                        st_MainLaneInfoLeft = FindOptimalLane(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft, arst_FinalLaneCoefLeft[s32_I], s32_Height);
                        f32_DiffDegreeSlope  = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft);
                    }
                }
                else if(FindNearLaneCoef(pst_CameraData->st_LastLaneCoefLeft,arst_FinalLaneCoefLeft[s32_I]))
                {
                    st_MainLaneInfoLeft = FindOptimalLane(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft, arst_FinalLaneCoefLeft[s32_I], s32_Height);
                    f32_DiffDegreeSlope  = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft);
                }

                // if(CalculateXIntercept(st_MainLaneInfoLeft, s32_Height) < 0)
                // {
                //     printf("이런 상황입니다!\n");
                //     st_MainLaneInfoLeft = FindOptimalLane(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft, arst_FinalLaneCoefLeft[s32_I], s32_Height);
                //     f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft);
                // }


            }
        }
        else                                // 곡선 -> Slope가 큰 차선이 Best
        {
            // printf("Corner\n");
            st_MainLaneInfoLeft = arst_FinalLaneCoefLeft[0];
            f32_DiffDegreeSlope  = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft);

            // printf("Last Lane - Slope: %f, XIntercept: %d\n", SlopeToDegree(pst_CameraData->st_LastLaneCoefLeft.f32_Slope), CalculateXIntercept(pst_CameraData->st_LastLaneCoefLeft, s32_Height));
            // printf("%d's Lane - Slope: %f, XIntercept: %d\n", 0, SlopeToDegree(arst_FinalLaneCoefLeft[0].f32_Slope), CalculateXIntercept(arst_FinalLaneCoefLeft[0], s32_Height));
            for (s32_I = 1;s32_I < st_LaneStatus.s32_CntValidLaneLeft;s32_I++)
            {   

                // printf("st_MainLaneInfoLeft - Slope: %f, XIntercept: %d\n", SlopeToDegree(st_MainLaneInfoLeft.f32_Slope), CalculateXIntercept(st_MainLaneInfoLeft, s32_Height));
                // printf("%d's Lane - Slope: %f, XIntercept: %d\n", s32_I, SlopeToDegree(arst_FinalLaneCoefLeft[s32_I].f32_Slope), CalculateXIntercept(arst_FinalLaneCoefLeft[s32_I], s32_Height));
                if(CalculateXIntercept(arst_FinalLaneCoefLeft[s32_I], s32_Height) < 0)
                    continue;

                if(SlopeToDegree(st_MainLaneInfoLeft.f32_Slope) < SlopeToDegree(arst_FinalLaneCoefLeft[s32_I].f32_Slope))
                {
                    st_MainLaneInfoLeft = FindOptimalLane(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft, arst_FinalLaneCoefLeft[s32_I], s32_Height);
                    f32_DiffDegreeSlope  = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft);
                }
                else if(FindNearLaneCoef(pst_CameraData->st_LastLaneCoefLeft, arst_FinalLaneCoefLeft[s32_I]))
                {
                    st_MainLaneInfoLeft = FindOptimalLane(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft ,arst_FinalLaneCoefLeft[s32_I], s32_Height);
                    f32_DiffDegreeSlope  = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefLeft, st_MainLaneInfoLeft);
                }
            }
        }  



        // printf("Final Left Lane X Intercept: %d\n",CalculateXIntercept(st_MainLaneInfoLeft, s32_Height));
        // printf("Final Left Lane Slope      : %f\n", SlopeToDegree(st_MainLaneInfoLeft.f32_Slope));
    }

    f32_DiffDegreeSlope = 0;

    // 최종적으로 소실점을 추출할 Lane 선택
    if(!st_LaneStatus.b_NoLaneRight)
    {

        if(st_LaneStatus.b_IsNotCornerLeft)   // 왼쪽이 더 정확함
        {
            // printf("Straight\n");

            // printf("----\nLast Right Lane X Intercept: %d\n",CalculateXIntercept(st_MainLaneInfoRight, s32_Height));
            // printf("Last Right Lane Slope      : %f\n", SlopeToDegree(st_MainLaneInfoRight.f32_Slope));

            st_MainLaneInfoRight = arst_FinalLaneCoefRight[0];
            f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);

            // printf("Last Lane - Slope: %f, XIntercept: %d\n", SlopeToDegree(pst_CameraData->st_LastLaneCoefRight.f32_Slope), CalculateXIntercept(pst_CameraData->st_LastLaneCoefRight, s32_Height));
            // printf("%d's Lane - Slope: %f, XIntercept: %d\n", 0, SlopeToDegree(arst_FinalLaneCoefRight[0].f32_Slope), CalculateXIntercept(arst_FinalLaneCoefRight[0], s32_Height));
            
            // st_Color = Scalar(rand() % 256, rand() % 256, rand() % 256);
            // DrawLane(st_ResultImage, st_Color,arst_FinalLaneCoefRight[0].f32_Slope,arst_FinalLaneCoefRight[0].f32_Intercept);  
            
            for(s32_I=1;s32_I<st_LaneStatus.s32_CntValidLaneRight;s32_I++)
            {   
                // 직선 구간

                // Draw Lane
                // st_Color = Scalar(rand() % 256, rand() % 256, rand() % 256);
                // DrawLane(st_ResultImage, st_Color,arst_FinalLaneCoefRight[s32_I].f32_Slope,arst_FinalLaneCoefRight[s32_I].f32_Intercept);  
                
                // printf("st_MainLaneInfoRight - Slope: %f, XIntercept: %d\n", SlopeToDegree(st_MainLaneInfoRight.f32_Slope), CalculateXIntercept(st_MainLaneInfoRight, s32_Height));
                // printf("%d's Lane - Slope: %f, XIntercept: %d\n", s32_I, SlopeToDegree(arst_FinalLaneCoefRight[s32_I].f32_Slope), CalculateXIntercept(arst_FinalLaneCoefRight[s32_I], s32_Height));

                // PitStop Zone Entrance 
                if( pst_CameraData->b_PitStop && pst_CameraData->b_NearPitStopR )
                {
                    // if(SlopeToDegree(st_MainLaneInfoRight.f32_Slope) < SlopeToDegree(arst_FinalLaneCoefRight[s32_I].f32_Slope))
                    // {
                    //     st_MainLaneInfoRight = arst_FinalLaneCoefRight[s32_I];
                    // }

                    // if(abs(SlopeToDegree(st_MainLaneInfoRight.f32_Slope)-SlopeToDegree(arst_FinalLaneCoefRight[s32_I].f32_Slope)) < 1)
                    // {
                    //     if(CalculateXIntercept(st_MainLaneInfoRight, s32_Height) < CalculateXIntercept(arst_FinalLaneCoefRight[s32_I], s32_Height))
                    //     {
                    //         st_MainLaneInfoRight = arst_FinalLaneCoefRight[s32_I];
                    //     }
                    // }

                    // if(SlopeToDegree(st_MainLaneInfoRight.f32_Slope) < SlopeToDegree(arst_FinalLaneCoefRight[s32_I].f32_Slope))
                    // {
                        // 새로운 차선의 기울기가 더 큰 경우
                    // printf("FindPitStopRightLane\n");
                    st_MainLaneInfoRight = FindPitStopRightLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                    f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);                            
                    // }

                    continue;
                }

                // 해당 차선이 Image Width보다 큰 경우 Invalid
                if(CalculateXIntercept(arst_FinalLaneCoefRight[s32_I], s32_Height) > s32_Width-1 )
                {
                    // st_MainLaneInfoRight = FindOptimalLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                    // f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);
                    continue;
                }

                if(SlopeToDegree(st_MainLaneInfoRight.f32_Slope) < SlopeToDegree(arst_FinalLaneCoefRight[s32_I].f32_Slope))
                {
                    if(!FindNearLaneCoef(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight))
                    {
                        st_MainLaneInfoRight = FindOptimalLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                        f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);                            
                    }
                    // else if(FindNearLaneCoef(pst_CameraData->st_LastLaneCoefRight, arst_FinalLaneCoefRight[s32_I])) -> 해당 조건문으로 테스트 필요 
                    else
                    {
                        st_MainLaneInfoRight = FindOptimalLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                        f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);
                    }
                }
                else
                {
                    if(FindNearLaneCoef(pst_CameraData->st_LastLaneCoefRight, arst_FinalLaneCoefRight[s32_I]))
                    {
                        st_MainLaneInfoRight = FindOptimalLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                        f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);
                    }
                }

                if(CalculateXIntercept(st_MainLaneInfoRight, s32_Height) > s32_Width-1)
                {
                    if(CalculateXIntercept(arst_FinalLaneCoefRight[s32_I], s32_Height) < s32_Width-1)
                    {
                        st_MainLaneInfoRight = arst_FinalLaneCoefRight[s32_I];    
                    }
                    else
                    {
                        st_MainLaneInfoRight = FindOptimalLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                        f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);
                    }
                }


            }
        }
        else
        {
            // 곡석 구간 

            st_MainLaneInfoRight = arst_FinalLaneCoefRight[0];
            f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);

            // printf("Last Lane - Slope: %f, XIntercept: %d\n", SlopeToDegree(pst_CameraData->st_LastLaneCoefRight.f32_Slope), CalculateXIntercept(pst_CameraData->st_LastLaneCoefRight, s32_Height));
            // printf("%d's Lane - Slope: %f, XIntercept: %d\n", 0, SlopeToDegree(arst_FinalLaneCoefRight[0].f32_Slope), CalculateXIntercept(arst_FinalLaneCoefRight[0], s32_Height));

            for(s32_I=1;s32_I<st_LaneStatus.s32_CntValidLaneRight;s32_I++)
            {   
  
                // printf("st_MainLaneInfoRight - Slope: %f, XIntercept: %d\n", SlopeToDegree(st_MainLaneInfoRight.f32_Slope), CalculateXIntercept(st_MainLaneInfoRight, s32_Height));
                // printf("%d's Lane - Slope: %f, XIntercept: %d\n", s32_I, SlopeToDegree(arst_FinalLaneCoefRight[s32_I].f32_Slope), CalculateXIntercept(arst_FinalLaneCoefRight[s32_I], s32_Height));


                if(CalculateXIntercept(arst_FinalLaneCoefRight[s32_I], s32_Height) > s32_Width-1 )
                    continue;

                if(SlopeToDegree(st_MainLaneInfoRight.f32_Slope) > SlopeToDegree(arst_FinalLaneCoefRight[s32_I].f32_Slope))
                {
                    if(!FindNearLaneCoef(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight))
                    {
                        st_MainLaneInfoRight = FindOptimalLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                        f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);
                    }
                }
                else
                {
                    st_MainLaneInfoRight = FindOptimalLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                    f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);
                }




                if(CalculateXIntercept(st_MainLaneInfoRight, s32_Height) > s32_Width-1)
                {
                    st_MainLaneInfoRight = FindOptimalLane(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight, arst_FinalLaneCoefRight[s32_I], s32_Height);
                    f32_DiffDegreeSlope = CalculateDifferenceDegreeSlope(pst_CameraData->st_LastLaneCoefRight, st_MainLaneInfoRight);
                }


            }
        }  


        // printf("----\nFinal Right Lane X Intercept: %d\n",CalculateXIntercept(st_MainLaneInfoRight, s32_Height));
        // printf("Final R ight Lane Slope      : %f\n", SlopeToDegree(st_MainLaneInfoRight.f32_Slope));
    }

    // printf("-------------------------------------------------------------------------\n");

}

void CalcualteLaneCoefficient(LANE_COEFFICIENT_t arst_FinalLaneCoefLeft[100], LANE_COEFFICIENT_t arst_FinalLaneCoefRight[100], LANE_COEFFICIENT_t arst_LaneCoef[100]
    , bool arb_Visited[100], const int32_t& s32_CntHoughLine, LANE_STATUS_t& st_LaneStatus, const int32_t& s32_Width, const int32_t& s32_Height)
{
    int32_t s32_I;
    LANE_COEFFICIENT_t st_TmpLaneCoef;
    Scalar st_Color;

    for (s32_I = 0;s32_I < s32_CntHoughLine;s32_I++)
    {
        if(!arb_Visited[s32_I])
        {
            st_Color = Scalar(rand() % 256, rand() % 256, rand() % 256);
            arb_Visited[s32_I] = true;

            // Lane 추출
            st_TmpLaneCoef = FindSameLaneInfo(arst_LaneCoef, arb_Visited, s32_I, s32_CntHoughLine, s32_Width, s32_Height);

            // 차량 기준 좌, 우측 Lane 구분
            if(SlopeToDegree(st_TmpLaneCoef.f32_Slope)<0)
            {
                arst_FinalLaneCoefLeft[st_LaneStatus.s32_CntValidLaneLeft] = st_TmpLaneCoef;
                st_LaneStatus.s32_CntValidLaneLeft++;

                // Corner 부분이 아님을 파악하는 제약조건
                if(SlopeToDegree(st_TmpLaneCoef.f32_Slope)<-36)
                {
                    st_LaneStatus.b_IsNotCornerLeft = true;
                }

                st_LaneStatus.b_NoLaneLeft = false;
            }
            else
            {
                arst_FinalLaneCoefRight[st_LaneStatus.s32_CntValidLaneRight] = st_TmpLaneCoef;
                st_LaneStatus.s32_CntValidLaneRight++;

                // Corner 부분임을 파악하는 제약조건
                if(SlopeToDegree(st_TmpLaneCoef.f32_Slope)>26)
                {
                    st_LaneStatus.b_IsNotCornerRight = true;
                }

                st_LaneStatus.b_NoLaneRight = false;
            }
        }
    }        
}

void CheckSameHoughLineCluster(Mat& st_HoughLaneImage, vector<vector<Vec4i>> st_Lines, LANE_COEFFICIENT_t arst_LaneCoef[100], const int32_t& s32_CntHoughLine)
{
    int32_t s32_I, s32_J;
    Scalar st_Color;

    // Check Same HoughLine Clusters
    for (s32_I = 0;s32_I < s32_CntHoughLine;s32_I++)
    {
        st_Color = Scalar(rand() % 256, rand() % 256, rand() % 256);

        for (s32_J = 0;s32_J < st_Lines[s32_I].size();s32_J++)
        {
            line(st_HoughLaneImage, Point(st_Lines[s32_I][s32_J][0], st_Lines[s32_I][s32_J][1]), Point(st_Lines[s32_I][s32_J][2], st_Lines[s32_I][s32_J][3]), st_Color, 2, LINE_AA);            
            // printf("HoughLane Info -  st_Line[0]: %d, st_Line[1]: %d, st_Line[2]: %d, st_Line[3]: %d]\n",
                // st_Lines[s32_I][s32_J][0], st_Lines[s32_I][s32_J][1], st_Lines[s32_I][s32_J][2], st_Lines[s32_I][s32_J][3]);

        }
        
        CalculatePerspectiveLaneCoef(st_Lines[s32_I], arst_LaneCoef[s32_I]);
    }
}

float32_t CalculateDifferenceDegreeSlope(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_CurrentLaneCoef)
{

    // printf("Last: %f, Current: %f\n", SlopeToDegree(st_LastLaneCoef.f32_Slope), SlopeToDegree(st_CurrentLaneCoef.f32_Slope));
    return abs(SlopeToDegree(st_LastLaneCoef.f32_Slope) - SlopeToDegree(st_CurrentLaneCoef.f32_Slope));
}

LANE_COEFFICIENT_t FindPitStopLeftLane(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_BestLaneCoef
            , const LANE_COEFFICIENT_t& st_NewLaneCoef, const int32_t& s32_Height)
{

    if(abs(SlopeToDegree(st_LastLaneCoef.f32_Slope) - SlopeToDegree(st_NewLaneCoef.f32_Slope)) > 10)
        return st_BestLaneCoef;
    if(abs(SlopeToDegree(st_LastLaneCoef.f32_Slope) - SlopeToDegree(st_BestLaneCoef.f32_Slope)) > 10)
        return st_NewLaneCoef;

    if(abs(SlopeToDegree(st_BestLaneCoef.f32_Slope) -SlopeToDegree(st_NewLaneCoef.f32_Slope)) < 2)
    {
        return (CalculateXIntercept(st_BestLaneCoef, s32_Height) < CalculateXIntercept(st_NewLaneCoef, s32_Height)) ? st_NewLaneCoef : st_BestLaneCoef;
    } 

    if(SlopeToDegree(st_BestLaneCoef.f32_Slope) >SlopeToDegree(st_NewLaneCoef.f32_Slope))
    {
        if(abs(SlopeToDegree(st_BestLaneCoef.f32_Slope) -SlopeToDegree(st_NewLaneCoef.f32_Slope)) < 2)       // 해당 조건문 불필요 
        {
            return (CalculateXIntercept(st_BestLaneCoef, s32_Height) < CalculateXIntercept(st_NewLaneCoef, s32_Height)) ? st_NewLaneCoef : st_BestLaneCoef;
        }
        else
        {
            return st_NewLaneCoef;
        }
    }
    else
    {
        return st_BestLaneCoef;
    }
}

LANE_COEFFICIENT_t FindPitStopRightLane(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_BestLaneCoef
            , const LANE_COEFFICIENT_t& st_NewLaneCoef, const int32_t& s32_Height)
{

    bool b_FlagBest, b_FlagCurrent;
    b_FlagBest    = FindNearPitStopLane(st_LastLaneCoef, st_BestLaneCoef, s32_Height);
    b_FlagCurrent = FindNearPitStopLane(st_LastLaneCoef, st_NewLaneCoef, s32_Height);

    // 한 차선에 대하여 고주로 및 PitStop 모두 검출되는 경우 -> 기울기가 큰 것으로 판단
    if((b_FlagBest && b_FlagCurrent) && abs(CalculateXIntercept(st_BestLaneCoef, s32_Height) - CalculateXIntercept(st_NewLaneCoef, s32_Height)) < 40)
    {
        if(st_BestLaneCoef.f32_Slope > st_NewLaneCoef.f32_Slope) 
        {
            return st_BestLaneCoef;
        }
        else
        {
            return st_NewLaneCoef;
        }
    }

    if((b_FlagBest && b_FlagCurrent) || (!b_FlagBest && !b_FlagCurrent))
    {
        // 둘 다 유사한 경우 기울기의 차이가 더 적은 차선을 BestLane으로 선택
        if(abs(CalculateDifferenceDegreeSlope(st_LastLaneCoef, st_BestLaneCoef)) 
            < abs(CalculateDifferenceDegreeSlope(st_LastLaneCoef, st_NewLaneCoef)))
        {
            return st_BestLaneCoef;
        }
        else
        {
            return st_NewLaneCoef;
        }
    }
    else if(b_FlagBest && !b_FlagCurrent)
    {
        return st_BestLaneCoef;
    }
    else
    {
        return st_NewLaneCoef;
    }

}

bool FindNearPitStopLane(const LANE_COEFFICIENT_t& st_LastLaneCoef1, const LANE_COEFFICIENT_t& st_LastLaneCoef2, const int32_t& s32_Height)
{
    if(abs(SlopeToDegree(st_LastLaneCoef1.f32_Slope)-SlopeToDegree(st_LastLaneCoef2.f32_Slope))<4)
    {
        if(abs(CalculateXIntercept(st_LastLaneCoef1, s32_Height) - CalculateXIntercept(st_LastLaneCoef2, s32_Height)) < 40)
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}


LANE_COEFFICIENT_t FindOptimalLane(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_BestLaneCoef
            , const LANE_COEFFICIENT_t& st_NewLaneCoef, const int32_t& s32_Height)
{
    // printf("st_FinalLaneCoef Diff: %f\n", CalculateDifferenceDegreeSlope(st_LastLaneCoef, st_BestLaneCoef));
    // printf("arst_FinalLaneCoefLeft Diff: %f\n", CalculateDifferenceDegreeSlope(st_LastLaneCoef, st_NewLaneCoef));

    // 둘 다 매우 유사한 경우,,,X 절편도 비교할 필요가 있음..
    if(FindNearLaneCoef(st_LastLaneCoef,st_BestLaneCoef) && FindNearLaneCoef(st_LastLaneCoef,st_NewLaneCoef))
    {
        // printf(" 둘 다 이전 차선과 유사한 경우 -> X Intercept 비교\n");

        // 오른쪽 예외처리
        if(CalculateXIntercept(st_BestLaneCoef, s32_Height) > 899 && CalculateXIntercept(st_NewLaneCoef, s32_Height) < 900)
        {
            // printf("최적의 차선은 Width 보다 큰 X Intercept를 가지기에 유효한 차선으로 강제 설정\n");
            return st_NewLaneCoef;
        }

        if(abs(CalculateXIntercept(st_LastLaneCoef, s32_Height) - CalculateXIntercept(st_BestLaneCoef, s32_Height)) 
            < abs(CalculateXIntercept(st_LastLaneCoef, s32_Height) - CalculateXIntercept(st_NewLaneCoef, s32_Height)))
        {
            return st_BestLaneCoef;
        }
        else
        {
            return st_NewLaneCoef;
        }
    }

    if(abs(CalculateDifferenceDegreeSlope(st_LastLaneCoef, st_BestLaneCoef)) 
        < abs(CalculateDifferenceDegreeSlope(st_LastLaneCoef, st_NewLaneCoef)))
    {
        return st_BestLaneCoef;
    }
    else
    {
        return st_NewLaneCoef;
    }
}


bool FindNearLaneCoef(const LANE_COEFFICIENT_t& st_LastLaneCoef, const LANE_COEFFICIENT_t& st_CurrentLaneCoef)
{
    // printf("Last Lane Slope: %f, Current Lane Slope: %f\n", SlopeToDegree(st_LastLaneCoef.f32_Slope),SlopeToDegree(st_CurrentLaneCoef.f32_Slope));

    if(abs(SlopeToDegree(st_LastLaneCoef.f32_Slope)-SlopeToDegree(st_CurrentLaneCoef.f32_Slope))<5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool CheckValidVanishingPoint(const Point& st_Point, const int32_t& s32_Width, const int32_t& s32_Height)
{
    int32_t s32_MinimumPointY = 50;

    if(st_Point.x < 0 || st_Point.x > s32_Width-1)
    {
        return false;
    }

    if(st_Point.y < 0 || st_Point.y > s32_Height-1)
    {
        return false;
    }

    if(st_Point.y < s32_MinimumPointY)
    {
        return false;
    }
    else
    {
        return true;
    }

}

bool CheckNearWayPoint(const Point& st_LastPoint, const Point& st_CurrentPoint)
{

    // printf("Befor   Pnt - x: %d, y: %d\n", st_LastPoint.x, st_LastPoint.y);
    // printf("Current Pnt - x: %d, y: %d\n", st_CurrentPoint.x, st_CurrentPoint.y);
    // printf("X Difference: %d\n", abs(st_LastPoint.x - st_CurrentPoint.x));
    // printf("Y Difference: %d\n", abs(st_LastPoint.y - st_CurrentPoint.y));
    // printf("-----------\n");
    if(st_LastPoint.x == 0 & st_LastPoint.y == 0)
        return true;

    if(abs(st_LastPoint.x - st_CurrentPoint.x) < 60)
    {
        if(abs(st_LastPoint.y - st_CurrentPoint.y) < 30)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

void appendSlopeToFile(const std::string& filename, float32_t f32_Slope) 
{
    std::ofstream outfile;
    outfile.open(filename, std::ios_base::app); // append 모드로 파일 열기
    if (!outfile) {
        std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
        return;
    }
    outfile << f32_Slope << std::endl;
    outfile.close();
}


LANE_COEFFICIENT_t FindSameLaneInfo(LANE_COEFFICIENT_t arst_LaneCoef[100], bool arb_Visited[100], int32_t s32_I, int32_t s32_CntHoughLine, const int32_t& s32_Width, const int32_t& s32_Height)
{
    LANE_COEFFICIENT_t st_TmpLaneCoef;
    st_TmpLaneCoef.f32_Slope = 0.0;
    st_TmpLaneCoef.f32_Intercept = 0.0;

    int32_t s32_J;
    vector<LANE_COEFFICIENT_t> arst_TmpLaneCoef;
    arst_TmpLaneCoef.push_back(arst_LaneCoef[s32_I]);

    for(s32_J = s32_I+1;s32_J<s32_CntHoughLine;s32_J++)
    {
        if(!arb_Visited[s32_J] && CheckSameLaneInfo(arst_LaneCoef[s32_I], arst_LaneCoef[s32_J], s32_Width, s32_Height))
        {
            arst_TmpLaneCoef.push_back(arst_LaneCoef[s32_J]);
            arb_Visited[s32_J] = true;
        }
    }

    for(s32_J=0;s32_J<arst_TmpLaneCoef.size();s32_J++)
    {
        st_TmpLaneCoef.f32_Slope += arst_TmpLaneCoef[s32_J].f32_Slope;
        st_TmpLaneCoef.f32_Intercept += arst_TmpLaneCoef[s32_J].f32_Intercept;
    }

    st_TmpLaneCoef.f32_Slope = st_TmpLaneCoef.f32_Slope/arst_TmpLaneCoef.size();
    st_TmpLaneCoef.f32_Intercept = st_TmpLaneCoef.f32_Intercept/arst_TmpLaneCoef.size();

    return st_TmpLaneCoef;    
}

Point CalculateCrossPoint(const LANE_COEFFICIENT_t& st_LaneCoef1, const LANE_COEFFICIENT_t& st_LaneCoef2)
{
    int32_t s32_X = static_cast<int32_t>(st_LaneCoef2.f32_Intercept - st_LaneCoef1.f32_Intercept) / (st_LaneCoef1.f32_Slope - st_LaneCoef2.f32_Slope);

    return Point(s32_X , static_cast<int32_t>(st_LaneCoef1.f32_Slope*s32_X+st_LaneCoef1.f32_Intercept));
}

bool CheckSameLaneInfo(const LANE_COEFFICIENT_t& st_LaneCoef1, const LANE_COEFFICIENT_t& st_LaneCoef2, const int32_t& s32_Width, const int32_t& s32_Height)
{

    if(CalculateXIntercept(st_LaneCoef1, s32_Height) < 0 || CalculateXIntercept(st_LaneCoef1, s32_Height)>s32_Width-1)
        return false;

    if(CalculateXIntercept(st_LaneCoef2, s32_Height) < 0 || CalculateXIntercept(st_LaneCoef2, s32_Height)>s32_Width-1)
        return false;

    // printf("%f - %f\n", SlopeToDegree(st_LaneCoef1.f32_Slope), SlopeToDegree(st_LaneCoef2.f32_Slope));

    return (abs(SlopeToDegree(st_LaneCoef1.f32_Slope)-SlopeToDegree(st_LaneCoef2.f32_Slope)) < 5 && 
                   abs(CalculateXIntercept(st_LaneCoef1, s32_Height)-CalculateXIntercept(st_LaneCoef2, s32_Height)) < 50);
}

int32_t CalculateXIntercept(const LANE_COEFFICIENT_t& st_LaneCoef, const int32_t& s32_Height)
{
    return static_cast<int32_t>(s32_Height - 1 - st_LaneCoef.f32_Intercept)/st_LaneCoef.f32_Slope;
}

void RecalculateMedianSlope(float32_t& f32_MedianSlope, const vector<Vec4i> st_HoughLines)
{
    int32_t s32_I;
    float32_t f32_Tmp = 0.0;

    for(s32_I=0;s32_I<st_HoughLines.size();s32_I++)
    {
        f32_Tmp += atan2(st_HoughLines[s32_I][3] - st_HoughLines[s32_I][1], st_HoughLines[s32_I][2] - st_HoughLines[s32_I][0]) * 180.0 / CV_PI;
    }

    f32_MedianSlope = static_cast<float32_t>(f32_Tmp/st_HoughLines.size());
}

void DrawLane(Mat& st_ResultImage, const Scalar st_Color, float32_t f32_Slope, float32_t f32_Intercept)
{

    if (isinf(f32_Slope) || isnan(f32_Intercept) || isinf(f32_Slope) || isnan(f32_Intercept))
    {
        printf("Invalid Lane\n");
        return;
    }

    Point pt1, pt2;
    pt1 = Point(0, f32_Intercept);
    pt2 = Point(st_ResultImage.cols-1, f32_Slope * (st_ResultImage.cols - 1) + f32_Intercept);

    line(st_ResultImage, pt1, pt2, st_Color, 2, LINE_AA);   
}

float32_t CalculateDegreeSlope(const Vec4i& st_Line)
{
    return atan2(st_Line[3] - st_Line[1], st_Line[2] - st_Line[0]) * 180.0 / CV_PI;
}

float32_t ReCalculateSlope(const float32_t f32_Slope)
{
    return f32_Slope * CV_PI / 180.0;
}

// RANSAC을 사용하여 차선의 기울기와 절편을 계산하는 함수
void CalculatePerspectiveLaneCoef(const vector<Vec4i> st_Lines, LANE_COEFFICIENT_t& st_LaneCoefficient)
{
    int32_t s32_BestInlierCount = 0, s32_I,s32_J,  s32_PosX, s32_Index;
    bool b_Flag = true;
    LANE_COEFFICIENT_t st_Temp, st_Tmp;
    int32_t ars32_Position[4] = {0};

    st_Temp.f32_Intercept = 0;
    st_Temp.f32_Slope = 0;


    if (st_Lines.size() == 1) 
    {
        // 라인이 하나인 경우 해당 라인 정보를 직접 사용
        Vec4i line = st_Lines[0];
        Point pt1(line[0], line[1]);
        Point pt2(line[2], line[3]);
        st_LaneCoefficient = FitModelRANSAC(pt1, pt2, b_Flag);
        return;
    }

    for (s32_I = 0;s32_I < st_Lines.size();s32_I++)
    {
        Point pt1(st_Lines[s32_I][0], st_Lines[s32_I][1]);
        Point pt2(st_Lines[s32_I][2], st_Lines[s32_I][3]);
        st_Tmp = FitModelRANSAC(pt1, pt2, b_Flag);

        st_Temp.f32_Slope += st_Tmp.f32_Slope;
        st_Temp.f32_Intercept += st_Tmp.f32_Intercept;
    }

    st_LaneCoefficient.f32_Slope = st_Temp.f32_Slope/st_Lines.size();
    st_LaneCoefficient.f32_Intercept = st_Temp.f32_Intercept/st_Lines.size();
}

// 기울기와 절편을 계산하는 함수
LANE_COEFFICIENT_t FitModelRANSAC(const Point& st_Pnt1, const Point& st_Pnt2, bool& b_Flag) 
{
    LANE_COEFFICIENT_t st_LaneCoef;
    // 두 점이 같지 않도록 보장해야 함
    if (st_Pnt1.x == st_Pnt2.x) 
    {
        b_Flag = false;
        return st_LaneCoef;
    }
    st_LaneCoef.f32_Slope = static_cast<float32_t>(st_Pnt2.y - st_Pnt1.y) / static_cast<float32_t>(st_Pnt2.x - st_Pnt1.x);
    st_LaneCoef.f32_Intercept = st_Pnt1.y - st_LaneCoef.f32_Slope * st_Pnt1.x;
    b_Flag = true;
    return st_LaneCoef;
}

// 기울기를 각도로 변환하는 함수
float32_t SlopeToDegree(float32_t f32_Slope) {
    return atan(f32_Slope) * 180.0 / CV_PI;
}