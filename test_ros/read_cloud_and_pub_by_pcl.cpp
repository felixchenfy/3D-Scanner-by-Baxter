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

    // Settings
    string topic_name = "/kinect2/qhd/points";
    string node_name = "read_cloud_and_pub_by_pcl";
    string filename = "/home/feiyu/baxterws/src/scan3d-by-baxter/data/cloud_cluster_0.pcd";
    string ros_cloud_frame_id = "odom";

    // Init node
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);

    // Read file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = my_pcl::read_point_cloud(filename);

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
    ROS_INFO(("This node stops: " + node_name).c_str());
    return 0;
}
