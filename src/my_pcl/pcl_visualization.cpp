
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

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}
void setViewerPose(pcl::visualization::PCLVisualizer& viewer,
    double x, double y, double z, double ea_x, double ea_y, double ea_z){
    Eigen::Affine3d T = my_basics::getAffine3d(x, y, z, ea_x, ea_y, ea_z); 
    setViewerPose(viewer, T.cast<float>());
}


} // namespace pcl_funcs