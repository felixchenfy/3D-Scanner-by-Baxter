/*
Functions include:
    removePlanes(==SACSegmentation+plane): remove planes until there are a few points left
    divideIntoClusters(==EuclideanClusterExtraction): divide a point cloud into different clusters
*/

#ifndef PCL_ADVANCED_H
#define PCL_ADVANCED_H

#include <my_pcl/common_headers.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace my_pcl
{

using namespace pcl;

// Given a cloud and a list of indices, return a list of point clouds corresponding to the indices.
vector<PointCloud<PointXYZRGB>::Ptr> extractSubCloudsByIndices(
    const PointCloud<PointXYZRGB>::Ptr cloud, const vector<PointIndices> &clusters_indices);

// Remove planes
int removePlanes(PointCloud<PointXYZRGB>::Ptr &cloud,
    float plane_distance_threshold = 0.01, int plane_max_iterations = 100,
    int stop_criteria_num_planes = -1, float stop_criteria_rest_points_ratio = 0.3,
    bool print_res=false);

// Do clustering using pcl::EuclideanClusterExtraction. Return the indices of each cluster.
vector<PointIndices> divideIntoClusters(const PointCloud<PointXYZRGB>::Ptr cloud,
    double cluster_tolerance = 0.02, int min_cluster_size = 100,int max_cluster_size = 20000);

} // namespace my_pcl

#endif