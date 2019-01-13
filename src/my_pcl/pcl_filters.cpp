
#include "my_pcl/pcl_filters.h"
#include <pcl/filters/passthrough.h>                 // PassThrough
#include <pcl/filters/statistical_outlier_removal.h> // StatisticalOutlierRemoval
#include <pcl/filters/voxel_grid.h>                  // VoxelGrid

// Planar segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h> // Extract sub cloud by indices 


namespace my_pcl
{

// -- Pass through: Input: axis, up bound, low bound. Output: only the points inside this range.

PointCloud<PointXYZRGB>::Ptr
filtByPassThrough(const PointCloud<PointXYZRGB>::Ptr cloud, string axis_to_filt,
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
filtByPassThrough(const PointCloud<PointXYZ>::Ptr cloud, string axis_to_filt,
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

// ------------------------------------------------------------------------------------
PointCloud<PointXYZRGB>::Ptr
filtByStatisticalOutlierRemoval(const PointCloud<PointXYZRGB>::Ptr cloud,
                                float mean_k, float std_dev, bool return_outliers)
{
  PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
  StatisticalOutlierRemoval<PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(std_dev);
  sor.setNegative(return_outliers);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}

PointCloud<PointXYZ>::Ptr
filtByStatisticalOutlierRemoval(const PointCloud<PointXYZ>::Ptr cloud,
                                float mean_k, float std_dev, bool return_outliers)
{
  PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
  StatisticalOutlierRemoval<PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(std_dev);
  sor.setNegative(return_outliers);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}

// ------------------------------------------------------------------------------------

PointCloud<PointXYZ>::Ptr
filtByVoxelGrid(const PointCloud<PointXYZ>::Ptr cloud,
                float x_grid_size, float y_grid_size, float z_grid_size)
{
  PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
  VoxelGrid<PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(x_grid_size, y_grid_size, z_grid_size);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}

PointCloud<PointXYZRGB>::Ptr
filtByVoxelGrid(const PointCloud<PointXYZRGB>::Ptr cloud,
                float x_grid_size, float y_grid_size, float z_grid_size)
{
  PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
  VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(x_grid_size, y_grid_size, z_grid_size);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}

// ------------------------------------------------------------------------------------

bool detectPlane(
    const PointCloud<PointXYZRGB>::Ptr cloud,
    ModelCoefficients::Ptr &coefficients, PointIndices::Ptr &inliers,
    float distance_threshold, int max_iterations)
{
  /* example of usage{
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    ModelCoefficients::Ptr coefficients;
    PointIndices::Ptr inliers;
    detectPlane(cloud, coefficients, inliers);
  }
  */
  coefficients.reset(new ModelCoefficients);
  inliers.reset(new PointIndices);
  // Create the segmentation object
  SACSegmentation<PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(SACMODEL_PLANE);
  seg.setMethodType(SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setMaxIterations(max_iterations);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return false;
  }
  if (0) printPlaneCoef(coefficients);

  if (0)
  {
    cerr << "Model inliers: number = " << inliers->indices.size() << endl;
    for (size_t i = 0; i < min(10, (int)inliers->indices.size()); ++i)
      cerr << inliers->indices[i] << " ";
    cerr << endl;
  }
  return true;
}
void printPlaneCoef(const pcl::ModelCoefficients::Ptr coefficients)
{
  cout << "Fitting plane ax+by+cz+d=0. The 4 parames are: " << coefficients->values[0] << ", "
       << coefficients->values[1] << ", "
       << coefficients->values[2] << ", "
       << coefficients->values[3] << endl;
}

// ------------------------------------------------------------------------------------
PointCloud<pcl::PointXYZRGB>::Ptr extractSubCloudByIndices(
    const PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointIndices::Ptr indices,
    bool invert_indices)
{
    PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud(new PointCloud<pcl::PointXYZRGB>);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(invert_indices);
    extract.filter(*sub_cloud);
    return sub_cloud;
}



} // namespace my_pcl