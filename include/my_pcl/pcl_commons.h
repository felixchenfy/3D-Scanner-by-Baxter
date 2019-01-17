
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
void printCloudSize(PointCloud<PointXYZRGB>::Ptr cloud);
void printCloudSize(PointCloud<PointXYZ>::Ptr cloud);

// -- Set point color and pos
void setPointColor(PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b);
void setCloudColor(PointCloud<PointXYZRGB>::Ptr cloud,  uint8_t r, uint8_t g, uint8_t b);
void setPointPos(PointXYZRGB &point, float x, float y, float z);
void setPointPos(PointXYZRGB &point, double x, double y, double z);
void setPointPos(PointXYZRGB &point, cv::Mat p);
void setPointPos(PointXYZ &point, float x, float y, float z);
void setPointPos(PointXYZ &point, double x, double y, double z);
void setPointPos(PointXYZ &point, cv::Mat p);

// -- Transformation
void rotateCloud(const PointCloud<PointXYZRGB>::Ptr src, PointCloud<PointXYZRGB>::Ptr &dst,
                 float T_dstFrame_to_srcFrame[4][4]);

}

#endif