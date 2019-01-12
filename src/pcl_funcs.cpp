
#include "my_pcl/pcl_funcs.h"

namespace my_pcl
{

void test_func(){
    printf("dfasdf\n");
}

template <typename T>
typename PointCloud<T>::Ptr read_point_cloud(string filename)
{
    typename PointCloud<T>::Ptr cloud(new PointCloud<T>);
    int load_res = io::loadPCDFile<T>(filename, *cloud);
    if (load_res == -1) // load the file
    {
        string ERROR_MESSAGE = "Couldn't read file " + filename + "\n";
        PCL_ERROR(ERROR_MESSAGE.c_str());
        assert(0);
    }
    if (DEBUG_RESULT)
    {
        std::cout << "Loaded "
                  << cloud->width * cloud->height
                  << " data points from " + filename + " with the following fields: "
                  << std::endl;
    }
    return cloud;
}

void write_point_cloud(string filename, PointCloud<PointXYZRGB>::Ptr cloud)
{
    pcl::io::savePCDFileASCII(filename, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + filename << std::endl;
}
void write_point_cloud(string filename, PointCloud<PointXYZ>::Ptr cloud)
{
    pcl::io::savePCDFileASCII(filename, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + filename << std::endl;
}

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