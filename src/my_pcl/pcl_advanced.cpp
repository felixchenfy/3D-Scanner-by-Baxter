
#include "my_pcl/pcl_advanced.h"
#include "my_pcl/pcl_filters.h"

namespace my_pcl
{

// Given a cloud and a vector of indices, return a vector of point clouds corresponding to the indices. 
vector<PointCloud<PointXYZRGB>::Ptr> extractSubCloudsByIndices(
    const PointCloud<PointXYZRGB>::Ptr cloud, const vector<PointIndices> &clusters_indices)
{
    vector<PointCloud<PointXYZRGB>::Ptr> cloud_clusters;
    for (std::vector<PointIndices>::const_iterator it = clusters_indices.begin(); it != clusters_indices.end(); ++it)
    {
        PointCloud<PointXYZRGB>::Ptr cloud_cluster(new PointCloud<PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_clusters.push_back(cloud_cluster);
    }
    return cloud_clusters;
}

// Do clustering using pcl::EuclideanClusterExtraction. Return the indices of each cluster.
vector<PointIndices> divideIntoClusters(const PointCloud<PointXYZRGB>::Ptr cloud,
                                  double cluster_tolerance, int min_cluster_size, int max_cluster_size)
{
    vector<PointIndices> clusters_indices; // Output

    // Creating the KdTree object for the search method of the extraction
    search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
    tree->setInputCloud(cloud);

    // Set extractor
    EuclideanClusterExtraction<PointXYZRGB> ec;
    ec.setClusterTolerance(0.02); // similar to mean distance between points inside a point cloud
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters_indices);
    return clusters_indices;
}


} // namespace my_pcl