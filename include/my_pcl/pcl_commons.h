
#ifndef PCL_BASICS_H
#define PCL_BASICS_H

#include <my_pcl/common_headers.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues

namespace my_pcl{
    
using namespace pcl;
// -- Print
void printCloudInfo(PointCloud<PointXYZRGB>::Ptr cloud);
void printCloudInfo(PointCloud<PointXYZ>::Ptr cloud);

// -- Set point color and pos
void setPointColor(pcl::PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b);
void setPointPos(pcl::PointXYZRGB &point, float x, float y, float z);
void setPointPos(pcl::PointXYZRGB &point, double x, double y, double z);
void setPointPos(pcl::PointXYZRGB &point, cv::Mat p);
void setPointPos(pcl::PointXYZ &point, float x, float y, float z);
void setPointPos(pcl::PointXYZ &point, double x, double y, double z);
void setPointPos(pcl::PointXYZ &point, cv::Mat p);

}

#endif