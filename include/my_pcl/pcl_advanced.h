/*
Functions include:
    divideIntoClusters(EuclideanClusterExtraction): divide a point cloud into different clusters
*/

#ifndef PCL_ADVANCED_H
#define PCL_ADVANCED_H

#include <my_pcl/common_headers.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace my_pcl{
    
using namespace pcl;

// Given a cloud and a vector of indices, return a vector of point clouds corresponding to the indices. 
vector<PointCloud<PointXYZRGB>::Ptr> extractSubCloudsByIndices(
    const PointCloud<PointXYZRGB>::Ptr cloud, const vector<PointIndices> &clusters_indices);

// Do clustering using pcl::EuclideanClusterExtraction. Return the indices of each cluster.
vector<PointIndices> divideIntoClusters( const PointCloud<PointXYZRGB>::Ptr cloud,
    double cluster_tolerance = 0.02, int min_cluster_size = 100, int max_cluster_size = 20000);

}

#endif