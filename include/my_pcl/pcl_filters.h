/*
This script provides filtering functions including:
    PassThrough
    StatisticalOutlierRemoval
    VoxelGrid
Support for both PointXYZRGB and PointXYZ.
*/

#ifndef PCL_FILTERS_H
#define PCL_FILTERS_H

#include <my_pcl/common_headers.h>

namespace my_pcl{
using namespace pcl;


// -- Pass through: Input: axis, up bound, low bound. Output: only the points inside this range.
PointCloud<PointXYZRGB>::Ptr
filtByPassThrough(PointCloud<PointXYZRGB>::Ptr cloud, string axis_to_filt="z",
  float up_bound=1.0, float low_bound=0.0, bool flip_bound_direction=false);

PointCloud<PointXYZ>::Ptr
filtByPassThrough(PointCloud<PointXYZ>::Ptr cloud, string axis_to_filt="z",
  float up_bound=1.0, float low_bound=0.0, bool flip_bound_direction=false);


// -- Filter out noises by checking: whether point-to-point distance's mean and variance are larger than threshold. 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
filtByStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float mean_k = 50, float std_dev = 1.0, bool return_outliers = false);

pcl::PointCloud<pcl::PointXYZ>::Ptr
filtByStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float mean_k = 50, float std_dev = 1.0, bool return_outliers = false);

// -- Down-sampling point cloud by a voxel grid.
pcl::PointCloud<pcl::PointXYZ>::Ptr
filtByVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float x_grid_size=0.01, float y_grid_size=0.01, float z_grid_size=0.01);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
filtByVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float x_grid_size=0.01, float y_grid_size=0.01, float z_grid_size=0.01);
}

#endif