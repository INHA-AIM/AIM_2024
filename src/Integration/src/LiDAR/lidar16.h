// #ifndef LIDAR_16_H
// #define LIDAR_16_H

// #include "Global/global.h"
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/common.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/statistical_outlier_removal.h>  
// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/visualization/pcl_visualizer.h>

// typedef class _POINT 
// {
// public:
//     float32_t x;
//     float32_t y;
//     float32_t z;

//     _POINT(float32_t xCoord, float32_t yCoord, float32_t zCoord) : x(xCoord), y(yCoord), z(zCoord) {}
// } POINT_t;

// class parameter{
// public:
//     double REMOVE_FACTOR;
//     float voxel_size_x, voxel_size_y, voxel_size_z;
//     float ROI_xMin, ROI_xMax, ROI_yMin, ROI_yMax, ROI_zMin, ROI_zMax;
//     float clustering_offset;
//     int MinClusterSize, MaxClusterSize;
//     bool NoiseFiltering;
//     double ransac_distanceThreshold; //함수 : double형 parameter

//     parameter(){
//         this -> set_para();
//     }
//     void set_para(){
//         //jiwonFilter parameter
//         this -> REMOVE_FACTOR = 0.2;

//         //downsampling parameter
//         this -> voxel_size_x = 0.01f; 
//         this -> voxel_size_y = 0.01f; 
//         this -> voxel_size_z = 0.01f;

//         //ROI parameter
//         this -> ROI_xMin = 0;
//         this -> ROI_xMax = 8;
//         this -> ROI_yMin = -3;
//         this -> ROI_yMax = 3;
//         this -> ROI_zMin = -1.8;
//         this -> ROI_zMax = 1;

//         //clustering parameter
//         this -> clustering_offset = 0.3;
//         this -> MinClusterSize = 2;
//         this -> MaxClusterSize = 1000;

//         //ransac parameter
//         this -> ransac_distanceThreshold = 0.5;
        
//         //function on/off
//         this -> NoiseFiltering = 0;
//     }
// };

// void ROI(pcl::PointCloud<pcl::PointXYZI>& rawData);
// void RanSaC(pcl::PointCloud<pcl::PointXYZI>& rawData);
// void Lidar_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

// #endif
