/*
Test functions in "my_pcl/pcl_filters.h" and "my_pcl/pcl_advanced.h":
    filter, segment plane, divide cloud into clusters

Example of usage:
$ bin/pcl_test_filt_seg_clustering data_others/color_milk_and_2bottles.pcd

This script will generate several point cloud files (cloud_cluster_i.pcd), each is a separated object. 
Use pcl_viewer to view them:
$ pcl_viewer cloud_cluster_0.pcd cloud_cluster_1.pcd cloud_cluster_2.pcd cloud_cluster_3.pcd cloud_cluster_4.pcd 
(Or copy from the printed message of this script.)

// $ rosrun scan3d_by_baxter pcl_test_filt_seg_clustering /home/feiyu/baxterws/src/scan3d_by_baxter/data_others/color_milk_and_2bottles.pcd

*/

#include <iostream>
#include <stdio.h>
#include <string>
#include "my_pcl/pcl_io.h"
#include "my_pcl/pcl_commons.h"
#include "my_pcl/pcl_filters.h"
#include "my_pcl/pcl_advanced.h"

using namespace pcl;
using namespace my_pcl;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char **argv)
{
    // -- Load point cloud
    if(argc-1!=1){
        cout<<"my ERROR: please input an argument of the point cloud file name."<<endl;
        assert(0);
    }
    string filename = argv[1];
    PointCloudT::Ptr cloud = read_point_cloud(filename);
    PointCloudT::Ptr cloud_filtered;

    // -- filtByVoxelGrid
    float x_grid_size = 0.005, y_grid_size = 0.005, z_grid_size = 0.005;
    cloud_filtered = filtByVoxelGrid(cloud, x_grid_size, y_grid_size, z_grid_size);
    { // print
        cout << "Before filtByVoxelGrid, cloud size: " << cloud->width << "x" << cloud->height << "." << endl;
        cout << "After filtByVoxelGrid, cloud size: " << cloud_filtered->width << "x" << cloud_filtered->height << "." << endl;
        cout << endl;
    }
    cloud = cloud_filtered;

    // -- filtByStatisticalOutlierRemoval
    float mean_k = 50, std_dev = 1.0;
    cloud_filtered = filtByStatisticalOutlierRemoval(cloud, mean_k, std_dev);
    { // print
        cout << "Before filtByStatisticalOutlierRemoval, cloud size: " << cloud->width << "x" << cloud->height << "." << endl;
        cout << "After filtByStatisticalOutlierRemoval, cloud size: " << cloud_filtered->width << "x" << cloud_filtered->height << "." << endl;
        cout << endl;
    }
    cloud = cloud_filtered;

    // -- Write filtered cloud to file
    write_point_cloud("seg_res_downsampled.pcd", cloud);
    cout << "Saved " << cloud->points.size() << " data points to file." << endl
         << endl;

    // -- Remove the floor plane and table plane by:
    // detectPlane && extractSubCloudByIndices
    float plane_distance_threshold = 0.01;
    int plane_max_iterations = 100;
    int stop_criteria_num_planes = 0;
    float stop_criteria_rest_points_ratio = 0.3; // disabled
    int num_removed_planes = removePlanes(cloud,
        plane_distance_threshold, plane_max_iterations,
        stop_criteria_num_planes, stop_criteria_rest_points_ratio,
        true);

    // -- Clustering: Divide the remaining point cloud into different clusters (by pcl::EuclideanClusterExtraction)
    double cluster_tolerance = 0.05;
    int min_cluster_size = 100, max_cluster_size = 20000;
    cout<<"Cloud before clustering:";
    printCloudSize(cloud);
    vector<PointIndices> clusters_indices = divideIntoClusters(
        cloud, cluster_tolerance, min_cluster_size, max_cluster_size);

    // Extract indices into cloud clusters
    vector<PointCloud<PointXYZRGB>::Ptr> cloud_clusters =
        extractSubCloudsByIndices(cloud, clusters_indices);

    // -- Write to files
    string commands_for_pcl_viewer = "pcl_viewer ";
    for (int i = 0; i < cloud_clusters.size(); i++)
    {
        PointCloud<PointXYZRGB>::Ptr cloud_cluster = cloud_clusters[i];
        printf("%dth Cluster has %d points.\n", i, (int)cloud_cluster->points.size());
        string output_filename = "cloud_cluster_" + to_string(i) + ".pcd";
        write_point_cloud(output_filename, cloud_cluster);
        commands_for_pcl_viewer += output_filename + " ";
    }
    cout << "\nUse this command to debug:\n"
         << commands_for_pcl_viewer << endl << endl;

    // -- Return
    return (0);
}