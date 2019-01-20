#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

#include "my_pcl/pcl_io.h"

using namespace std;

// Main
int main(int argc, char **argv)
{
    // Init node
    string node_name = "read_cloud_and_pub_by_pcl";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Settings
    string ros_cloud_frame_id = "base";
    string topic_name_kinect_cloud, file_folder, file_name;
    if (!nh.getParam("topic_name_kinect_cloud", topic_name_kinect_cloud))
        topic_name_kinect_cloud = "/camera/depth_registered/points";
    if (!nh.getParam("file_folder", file_folder))
        assert(0);
    if (!nh.getParam("file_name", file_name))
        assert(0);
    // { // These two are not working?!?!?!?! why
    //     nh.param<string>("file_folder", file_folder);
    //     nh.param("file_name", file_name);
    // }

    // Publisher
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name_kinect_cloud, 1);

    // Read file
    string filename_whole = file_folder + file_name;
    ROS_INFO(("Read point cloud from: " + filename_whole).c_str());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = my_pcl::read_point_cloud(filename_whole);

    // Convert file
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = ros_cloud_frame_id;

    // Publish
    ros::Rate loop_rate(2);
    int cnt = 0;
    while (ros::ok())
    {
        pub.publish(ros_cloud);
        ROS_INFO("Publish %dth cloud.\n", cnt++);
        loop_rate.sleep();
        // // ros::spinOnce();
    }

    // Return
    ROS_INFO("This node stops: read_cloud_and_pub_by_pcl"); 
    // ROS_INFO(("This node stops: " + node_name).c_str()); // This generates annoying warning.
    return 0;
}
