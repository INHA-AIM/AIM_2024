#include "cameraVO.h"

extern Mat st_BGRImage;
extern Mat st_IntrinsicParam;
extern SENSOR_DATA_t st_SensorData;
extern int32_t s32_ImShowKey;
extern VISUAL_ODOMETRY_t st_VisualOdometry;
extern BFMatcher st_BFMatcher;
extern bool b_FirstFrame;

void CameraProcessing(RAW_CAMERA_DATA_t *pst_RawData, CAMERA_DATA_t *pst_CameraData, PLANNING_DATA_t *pst_PlanningData, DEAD_RECKONING_DATA_t *pst_DeadReckoningData)
{
    // There is no Image
    if (pst_RawData->s32_Num == 0)
    { 
        return;
    }

    // Declare Variables
    int32_t s32_I, s32_J;
    std::vector<char> st_DecodedImage;

    int32_t s32_Width;
    int32_t s32_Height;

    Scalar st_Color;

    // For Visual Odometry
    vector<vector<DMatch>> st_Matches;
    vector<DMatch> st_OptimalMatches;
    vector<Point2f> st_FeatureLast;
    vector<Point2f> st_FeatureCurrent;
    float32_t f32_Ratio = 0.75f;
  	Ptr<ORB> pst_ORBDescriptor = ORB::create(1000);

    Mat st_MatchedImage;
    Mat st_EssentialMatrix;
    Mat st_RotationMatrix;
    Mat st_TranslationMatrix;
    Mat st_VOImage = Mat::zeros(400, 400, CV_8UC3);

    // -------------------------------------------------------------------

    // Decode Image
    st_DecodedImage.assign(pst_RawData->arc_Buffer, pst_RawData->arc_Buffer + pst_RawData->s32_Num);
    st_BGRImage = imdecode(st_DecodedImage, IMREAD_COLOR);

    s32_Width = st_BGRImage.cols;
    s32_Height = st_BGRImage.rows;

    if(b_FirstFrame)
    {
    	st_VisualOdometry.st_BGRImageLast = imdecode(st_DecodedImage, IMREAD_COLOR);;

    	// ORB Feature Detection
    	pst_ORBDescriptor->detectAndCompute(st_VisualOdometry.st_BGRImageLast, cv::noArray(), st_VisualOdometry.st_KeyPointLast, st_VisualOdometry.st_DescriptorLast);

    	// Visual Odometry Trajectory
    	st_VisualOdometry.st_Trajectory.push_back(Point2f(0,0));

    	// Set Param
    	b_FirstFrame = false;
    	st_VisualOdometry.st_RotationT = Mat::eye(3, 3, CV_64F);
    	st_VisualOdometry.st_TranslationT = Mat::eye(3, 1, CV_64F);
    }
    else
    {
    	st_VisualOdometry.st_BGRImageCurrent = imdecode(st_DecodedImage, IMREAD_COLOR);;

    	// ORB Feature Detection
    	pst_ORBDescriptor->detectAndCompute(st_VisualOdometry.st_BGRImageCurrent, cv::noArray(), st_VisualOdometry.st_KeyPointCurrent, st_VisualOdometry.st_DescriptorCurrent);

    	// Feature Matching
    	st_BFMatcher.knnMatch(st_VisualOdometry.st_DescriptorLast, st_VisualOdometry.st_DescriptorCurrent, st_Matches, 2);

    	for( s32_I = 0;s32_I < st_Matches.size();s32_I++ )
    	{
    		// 제약조건 -> Best Match만 추출
            if (st_Matches[s32_I][0].distance < st_Matches[s32_I][1].distance * f32_Ratio &&
            	abs(st_VisualOdometry.st_KeyPointLast[st_Matches[s32_I][0].queryIdx].pt.x - st_VisualOdometry.st_KeyPointCurrent[st_Matches[s32_I][0].trainIdx].pt.x) < 10 &&
            	abs(st_VisualOdometry.st_KeyPointLast[st_Matches[s32_I][0].queryIdx].pt.y - st_VisualOdometry.st_KeyPointCurrent[st_Matches[s32_I][0].trainIdx].pt.y) < 10 &&
            	st_Matches[s32_I][0].distance < 20)
            {
                st_OptimalMatches.push_back(st_Matches[s32_I][0]);
            }
    	}

    	drawMatches(st_VisualOdometry.st_BGRImageLast, st_VisualOdometry.st_KeyPointLast, st_VisualOdometry.st_BGRImageCurrent, st_VisualOdometry.st_KeyPointCurrent, st_OptimalMatches, st_MatchedImage);

    	// Camera Pose Estimation
    	for( s32_I=0;s32_I < st_OptimalMatches.size(); s32_I++)
    	{
    		st_FeatureLast.push_back(st_VisualOdometry.st_KeyPointLast[st_OptimalMatches[s32_I].queryIdx].pt);
    		st_FeatureCurrent.push_back(st_VisualOdometry.st_KeyPointCurrent[st_OptimalMatches[s32_I].trainIdx].pt);
    	}

    	if(st_FeatureLast.size() > 5 && st_FeatureCurrent.size() > 5)
    	{
    		// Essential Matrix
    		st_EssentialMatrix = findEssentialMat(st_FeatureCurrent, st_FeatureLast, st_IntrinsicParam, RANSAC);

    		// Rotation, Translation Matrix
    		recoverPose(st_EssentialMatrix, st_FeatureCurrent, st_FeatureLast, st_IntrinsicParam, st_RotationMatrix, st_TranslationMatrix);

    		// Update Pose
    		st_VisualOdometry.st_TranslationT = st_VisualOdometry.st_TranslationT + st_VisualOdometry.st_RotationT * st_TranslationMatrix;
    		st_VisualOdometry.st_RotationT 	  = st_RotationMatrix * st_VisualOdometry.st_RotationT;

    		// Update Trajectory
    		st_VisualOdometry.st_Trajectory.push_back(Point2f(st_VisualOdometry.st_TranslationT.at<float64_t>(0), st_VisualOdometry.st_TranslationT.at<float64_t>(2)));  

    		// Visualize Visual Odometry
    		for(s32_I = 1; s32_I<st_VisualOdometry.st_Trajectory.size();s32_I++)
    		{
                line(st_VOImage, Point(st_VisualOdometry.st_Trajectory[s32_I-1].x + 200, 200 - st_VisualOdometry.st_Trajectory[s32_I-1].y), Point(st_VisualOdometry.st_Trajectory[s32_I].x + 200, 200 - st_VisualOdometry.st_Trajectory[s32_I].y), Scalar(0, 255, 0), 2);
                circle(st_VOImage, Point(st_VisualOdometry.st_Trajectory[s32_I].x + 200, 200 - st_VisualOdometry.st_Trajectory[s32_I].y), 3, Scalar(0, 0, 255), -1);
    		}
    	}


    	// Frame Change
    	st_VisualOdometry.st_BGRImageLast 	= st_VisualOdometry.st_BGRImageCurrent;
    	st_VisualOdometry.st_DescriptorLast = st_VisualOdometry.st_DescriptorCurrent;
    	st_VisualOdometry.st_KeyPointLast 	= st_VisualOdometry.st_KeyPointCurrent;
    }


   	// Show Processed Images    
    if(s32_ImShowKey == 1)
    {
        imshow("st_BGRImage",st_BGRImage);
        imshow("st_MatchedImage",st_MatchedImage);
        imshow("st_VOImage", st_VOImage);
        waitKey(1);
    }
}
