
#include "my_pcl/pcl_visualization.h"
#include "my_basics/eigen_funcs.h"


using namespace std;

#define DEBUG_RESULT true

namespace my_pcl
{

// initialize the viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudRGBViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                        const string &viewer_name,
                        const string &cloud_name)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer(viewer_name));

    // Add a RGB point cloud
    const int POINT_SIZE = 3;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_setting(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_setting, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);

    // Add Coordinate system
    viewer->addCoordinateSystem(1.0, "world frame");

    // Other properties
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    return (viewer);
}

} // namespace pcl_funcs