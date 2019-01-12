
#include "my_pcl/pcl_filters.h"
#include <pcl/filters/passthrough.h> // PassThrough
#include <pcl/filters/statistical_outlier_removal.h> // StatisticalOutlierRemoval
#include <pcl/filters/voxel_grid.h> // VoxelGrid

namespace my_pcl
{

// -- Pass through: Input: axis, up bound, low bound. Output: only the points inside this range.

PointCloud<PointXYZRGB>::Ptr
filtByPassThrough(PointCloud<PointXYZRGB>::Ptr cloud, string axis_to_filt,
  float up_bound, float low_bound, bool flip_bound_direction)
{
  PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
  PassThrough<PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(axis_to_filt);
  pass.setFilterLimits(low_bound, up_bound);
  pass.setFilterLimitsNegative(flip_bound_direction); // If true, converting the range from [] to (]&[)
  pass.filter(*cloud_filtered);
  return cloud_filtered;
}

PointCloud<PointXYZ>::Ptr
filtByPassThrough(PointCloud<PointXYZ>::Ptr cloud, string axis_to_filt,
  float up_bound, float low_bound, bool flip_bound_direction)
{
  PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
  PassThrough<PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(axis_to_filt);
  pass.setFilterLimits(low_bound, up_bound);
  pass.setFilterLimitsNegative(flip_bound_direction); // If true, converting the range from [] to (]&[)
  pass.filter(*cloud_filtered);
  return cloud_filtered;
}

// -- Filter out noises by checking: whether point-to-point distance's mean and variance are larger than threshold. 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
filtByStatisticalOutlierRemoval(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float mean_k, float std_dev, bool return_outliers)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(std_dev);
  sor.setNegative(return_outliers);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
filtByStatisticalOutlierRemoval(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float mean_k, float std_dev, bool return_outliers)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(std_dev);
  sor.setNegative(return_outliers);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}




// -- Down-sampling point cloud by a voxel grid.
pcl::PointCloud<pcl::PointXYZ>::Ptr
filtByVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float x_grid_size, float y_grid_size, float z_grid_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize (x_grid_size,y_grid_size,z_grid_size);
  sor.filter (*cloud_filtered);
  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
filtByVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float x_grid_size, float y_grid_size, float z_grid_size)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize (x_grid_size,y_grid_size,z_grid_size);
  sor.filter (*cloud_filtered);
  return cloud_filtered;
}

} // namespace my_pcl