/*
This script provides filtering functions including:
    PassThrough
    StatisticalOutlierRemoval
    VoxelGrid
    detectPlane
    extractSubCloudByIndices

Data types:
* Default point cloud type: PointXYZRGB
    Some support for both PointXYZRGB and PointXYZ (But 
    (I'm not using template because of slow compile. I want linking only.)
    
* No double, only float.
    Because some pcl functions use float.

*/

#ifndef PCL_FILTERS_H
#define PCL_FILTERS_H

#include <my_pcl/common_headers.h>
#include <pcl/ModelCoefficients.h>

namespace my_pcl
{
using namespace pcl;

// -- PassThrough:
// Filter out points outside the range.
// Input: axis, up bound, low bound. Output: only the points inside this range.
PointCloud<PointXYZRGB>::Ptr
filtByPassThrough(const PointCloud<PointXYZRGB>::Ptr cloud, string axis_to_filt = "z",
                  float up_bound = 1.0, float low_bound = 0.0, bool flip_bound_direction = false);

PointCloud<PointXYZ>::Ptr
filtByPassThrough(const PointCloud<PointXYZ>::Ptr cloud, string axis_to_filt = "z",
                  float up_bound = 1.0, float low_bound = 0.0, bool flip_bound_direction = false);

// -- StatisticalOutlierRemoval:
// Filter out noises by checking: whether point-to-point distance's mean and variance are larger than threshold.
// http://pointclouds.org/documentation/tutorials/statistical_outlier.php
PointCloud<PointXYZRGB>::Ptr
filtByStatisticalOutlierRemoval(const PointCloud<PointXYZRGB>::Ptr cloud,
                                float mean_k = 50, float std_dev = 1.0, bool return_outliers = false);

PointCloud<PointXYZ>::Ptr
filtByStatisticalOutlierRemoval(const PointCloud<PointXYZ>::Ptr cloud,
                                float mean_k = 50, float std_dev = 1.0, bool return_outliers = false);

// -- VoxelGrid:
// Down-sampling point cloud by a voxel grid.
PointCloud<PointXYZ>::Ptr
filtByVoxelGrid(const PointCloud<PointXYZ>::Ptr cloud,
                float x_grid_size = 0.01, float y_grid_size = 0.01, float z_grid_size = 0.01);

PointCloud<PointXYZRGB>::Ptr
filtByVoxelGrid(const PointCloud<PointXYZRGB>::Ptr cloud,
                float x_grid_size = 0.01, float y_grid_size = 0.01, float z_grid_size = 0.01);

// -- Detect one plane in the point cloud. Return its params and indices.
// coefficients: ax+by+cz+d=0; Access them by: coefficients->values[0~3].
// inliers: the indices of points belong to the plane. Access them by: inliers->indices[i].
// http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php
bool detectPlane(
    const PointCloud<PointXYZRGB>::Ptr cloud,
    ModelCoefficients::Ptr &coefficients, PointIndices::Ptr &inliers,
    float distance_threshold = 0.01, int max_iterations = 50);
/*Example of usage{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    detectPlane(cloud, coefficients, inliers);
    cout << coefficients->values[0] << ", " << inliers->indices[0] << endl;
    printPlaneCoef(coefficients);
  }*/

void printPlaneCoef(const pcl::ModelCoefficients::Ptr);

// -- Extract sub point cloud by inliers (or its revert)
PointCloud<pcl::PointXYZRGB>::Ptr extractSubCloudByIndices(
    const PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointIndices::Ptr indices,
    bool invert_indices=false);

} // namespace my_pcl

#endif