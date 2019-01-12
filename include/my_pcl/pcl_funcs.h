
#ifndef PCL_FUNCS_H
#define PCL_FUNCS_H

#include <iostream>
#include <memory>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues

using namespace std;
using namespace pcl;
#define DEBUG_RESULT true

namespace my_pcl
{

void test_func();

// -- Input / Output
template <typename T>
typename PointCloud<T>::Ptr read_point_cloud(string filename);
void write_point_cloud(string filename, PointCloud<PointXYZRGB>::Ptr cloud);
void write_point_cloud(string filename, PointCloud<PointXYZ>::Ptr cloud);

// -- Initialize viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudRGBViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                        const string &viewer_name = "viewer_name",
                        const string &cloud_name = "cloud_name");

} // namespace my_pcl

#endif