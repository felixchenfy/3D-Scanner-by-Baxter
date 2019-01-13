// Example usage:
// $ bin/extract_plane data/gray_milk.pcd
// $ bin/extract_plane data/color_milk_and_2bottles.pcd

#include <iostream>
#include "my_pcl/pcl_io.h"
#include "my_pcl/pcl_commons.h"
#include "my_pcl/pcl_filters.h"

using namespace pcl;
using namespace my_pcl;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char **argv)
{
    // -- Load point cloud
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

    // Write
    write_point_cloud("point_cloud_downsampled.pcd", cloud);
    cout << "Saved " << cloud->points.size() << " data points to file." << endl << endl;

    // -- detectPlane
    ModelCoefficients::Ptr coefficients;
    PointIndices::Ptr inliers;
    float distance_threshold = 0.01;
    int max_iterations = 1000;
    bool res = detectPlane(cloud, coefficients, inliers,
                           distance_threshold, max_iterations);
    printPlaneCoef(coefficients);
    cout << endl;

    // -- extractSubCloudByIndices
    bool invert_indices = false;
    PointCloudT::Ptr plane = extractSubCloudByIndices(cloud, inliers, invert_indices);
    cloud = extractSubCloudByIndices(cloud, inliers, !invert_indices);
    {
        cout << "Detected plane: ";
        printCloudInfo(plane);
        write_point_cloud("point_cloud_plane.pcd", plane);

        cout << "The rest part: ";
        printCloudInfo(cloud);
        write_point_cloud("point_cloud_remained.pcd", cloud);
        cout << endl;
    }

    // -- Return
    return (0);
}