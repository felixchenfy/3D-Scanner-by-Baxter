
#include "my_pcl/pcl_funcs.h"

using namespace std;
using namespace my_pcl;

typedef pcl::PointXYZRGB PointT;
// typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char **argv)
{
    test_func();
    
    // -- Load point cloud
    string filename = argv[1];
    PointCloudT::Ptr cloud = read_point_cloud<PointT>(filename);

    // -- Test write point cloud
    string output_folder = "data_results/";
    write_point_cloud(output_folder + "tmp.pcd", cloud);
    // pcl::io::savePCDFileASCII(output_folder + "tmp.pcd", cloud);

    // -- Init viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer = initPointCloudRGBViewer(cloud);

    // -- Display point cloud in a loop
    while (1)
    {
        viewer->spinOnce(10);
        if (viewer->wasStopped())
            break;
    }
    return (0);
}
