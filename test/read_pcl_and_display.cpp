
#include "my_pcl/pcl_io.h"
#include "my_pcl/pcl_visualization.h"

using namespace std;
using namespace my_pcl;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char **argv)
{
    
    // -- Load point cloud
    string filename = argv[1];
    PointCloudT::Ptr cloud;
    read_point_cloud(filename, cloud);

    // -- Test write point cloud
    string output_folder = "data_results/";
    write_point_cloud(output_folder + "tmp.pcd", cloud);


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
