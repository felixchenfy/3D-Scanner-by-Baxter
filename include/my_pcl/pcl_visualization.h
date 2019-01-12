
#ifndef PCL_VISUALIZATION_H
#define PCL_VISUALIZATION_H

#include "my_pcl/common_headers.h"
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace my_pcl
{

using namespace pcl;

// -- Initialize viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudRGBViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                        const string &viewer_name = "viewer_name",
                        const string &cloud_name = "cloud_name");

// // set the initial viewing angle (Copied from pcl website)
// void  setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

// // set the initial viewing angle (Overload by using pos and euler angles)
// void  setViewerPose (pcl::visualization::PCLVisualizer& viewer,
//     double x, double y, double z, double ea_x, double ea_y, double ea_z);


} // namespace my_pcl

#endif