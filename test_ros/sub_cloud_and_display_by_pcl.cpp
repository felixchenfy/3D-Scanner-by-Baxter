
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

// Init viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudRGBViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                        const string &viewer_name = "viewer_name", const string &cloud_name = "cloud_name");

// Set subscriber
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
bool flag_receive_cloud = false;
void subscriber_callback(const sensor_msgs::PointCloud2 &ros_cloud)
{
  flag_receive_cloud = true;
  pcl::fromROSMsg(ros_cloud, *pcl_cloud);
  ROS_INFO("Subscribed a point cloud from ros topic.");
}

// Main
int main(int argc, char **argv)
{
  // Init node
  string node_name = "sub_cloud_and_display_by_pcl";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  // Settings
  string topic_name_kinect_cloud;
  string viewer_name = "viewer_name";
  string viewer_cloud_name = "cloud_name";
  if (!nh.getParam("/topic_name_kinect_cloud", topic_name_kinect_cloud))
    topic_name_kinect_cloud = "kinect2/qhd/points";

  // Subscriber
  ros::Subscriber sub = nh.subscribe(topic_name_kinect_cloud, 1, subscriber_callback); // 1 is queue size

  // Init viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
      initPointCloudRGBViewer(pcl_cloud, viewer_name, viewer_cloud_name);

  // Loop, subscribe ros_cloud, and view
  while (ros::ok() && !viewer->wasStopped())
  {
    if (flag_receive_cloud)
    {
      flag_receive_cloud = false;
      viewer->updatePointCloud(pcl_cloud, viewer_cloud_name);
    }
    viewer->spinOnce(10);
    ros::spinOnce(); // In python, sub is running in different thread. In C++, same thread. So need this.
  }

  // Return
  ROS_INFO(("This node stops: " + node_name).c_str());
  return 0;
}

// Init viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudRGBViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                        const string &viewer_name, const string &cloud_name)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer(viewer_name));

  // Add a RGB point cloud
  const int POINT_SIZE = 3;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_setting(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_setting, cloud_name);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);

  // Add Coordinate system
  viewer->addCoordinateSystem(1.0, "world frame");

  // Other properties
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  return (viewer);
}