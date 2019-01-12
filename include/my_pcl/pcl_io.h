
#ifndef PCL_FUNCS_H
#define PCL_FUNCS_H

#include <my_pcl/common_headers.h>



namespace my_pcl
{

using namespace pcl;

// -- Input / Output
bool read_point_cloud(string filename, PointCloud<PointXYZRGB>::Ptr &cloud);
bool read_point_cloud(string filename, PointCloud<PointXYZ>::Ptr &cloud);
PointCloud<PointXYZRGB>::Ptr read_point_cloud(string filename);
void write_point_cloud(string filename, PointCloud<PointXYZRGB>::Ptr cloud);
void write_point_cloud(string filename, PointCloud<PointXYZ>::Ptr cloud);


} // namespace my_pcl

#endif