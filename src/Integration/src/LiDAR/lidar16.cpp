// #include "lidar16.h"

// extern vector<POINT_t> vcl_Points;
// extern parameter P;



// void ROI(pcl::PointCloud<pcl::PointXYZI>& rawData){            //setting ROI
//     for(unsigned int j = 0; j<rawData.points.size(); j++){     //actual ROI setting
//         float32_t *x = &rawData.points[j].x, *y = &rawData.points[j].y, *z = &rawData.points[j].z;

//         if(*x > P.ROI_xMin && *x < P.ROI_xMax && *y > P.ROI_yMin && *y < P.ROI_yMax && *z > P.ROI_zMin && *z < P.ROI_zMax) continue;
//         *x = *y = *z = 0;
//     }
// }

// void RanSaC(pcl::PointCloud<pcl::PointXYZI>& rawData){
//     // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::copyPointCloud(rawData, cloud); 
//     //pcl::fromROSMsg (*rawData, cloud);

//     pcl::ModelCoefficients coefficients;

//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
//     pcl::PointCloud<pcl::PointXYZ> inlierPoints;
//     pcl::PointCloud<pcl::PointXYZ> inlierPoints_neg;
    
//     // Create the segmentation object

//     pcl::SACSegmentation<pcl::PointXYZ> seg;
    
//     // Optional    
//     seg.setOptimizeCoefficients (true);
    
//     // Mandatory
//     seg.setModelType (pcl::SACMODEL_PLANE); 
//     seg.setMethodType (pcl::SAC_RANSAC); 
//     seg.setDistanceThreshold (P.ransac_distanceThreshold);

//     seg.setInputCloud(cloud.makeShared()); 
//     seg.segment (*inliers, coefficients);   

//     // cout << "Model coefficients: (" << coefficients.values[0] << ")x + (" 
//     //                                 << coefficients.values[1] << ")y + ("
//     //                                 << coefficients.values[2] << ")z + (" 
//     //                                 << coefficients.values[3] << ") = 0" << endl;

//     pcl::copyPointCloud<pcl::PointXYZ>(cloud, *inliers, inlierPoints);  // cloud에서 inliers에 해당하는 점들만 inlierPoints로 복사


//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud (cloud.makeShared());
//     extract.setIndices (inliers);
//     extract.setNegative (true);//false
//     extract.filter (inlierPoints_neg);

//     pcl::copyPointCloud(inlierPoints_neg, rawData);
// }

// void Lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
//     pcl::PointCloud<pcl::PointXYZI> rawData;
//     pcl::fromROSMsg(*msg, rawData);

//     RanSaC(rawData);

//     vcl_Points.clear();
//     for (const auto& p : rawData.points) {
//         vcl_Points.emplace_back(p.x, p.y, p.z); 
//     }
    
// }

// // 