/*
Main function:
* subscribe to cloud_src, filter it, rotated, pub to rviz.
* seg plane, do clustering, pub the object to node3
*/

#include <iostream>
#include <string>
#include <stdio.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Pose.h"

#include "my_pcl/pcl_visualization.h"
#include "my_pcl/pcl_commons.h"
#include "my_pcl/pcl_filters.h"
#include "my_pcl/pcl_advanced.h"

using namespace std;
using namespace pcl;

// -- Vars
bool flag_receive_from_node1 = false;
bool flag_receive_kinect_cloud = false;
geometry_msgs::Pose camera_pose = geometry_msgs::Pose();
PointCloud<PointXYZRGB>::Ptr cloud_src(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_rotated(new PointCloud<PointXYZRGB>); // this pubs to rviz
PointCloud<PointXYZRGB>::Ptr cloud_segmented(new PointCloud<PointXYZRGB>); // this pubs to node3
const string PCL_VIEWER_NAME = "node2: point cloud rotated to the world frame";
const string PCL_VIEWER_CLOUD_NAME = "cloud_rotated";

// -- Functions

void callbackFromNode1(geometry_msgs::Pose pose)
{
    flag_receive_from_node1 = true;
    camera_pose = pose;
}
void callbackFromKinect(const sensor_msgs::PointCloud2 &ros_cloud)
{
    if (flag_receive_from_node1)
    {
        flag_receive_from_node1 = false;
        flag_receive_kinect_cloud = true;
        fromROSMsg(ros_cloud, *cloud_src);
    }
}
void pubPclCloudToTopic(
    ros::Publisher &pub,
    PointCloud<PointXYZRGB>::Ptr pcl_cloud)
{
    sensor_msgs::PointCloud2 ros_cloud_to_pub;
    pcl::toROSMsg(*pcl_cloud, ros_cloud_to_pub);
    ros_cloud_to_pub.header.frame_id = "odom";
    pub.publish(ros_cloud_to_pub);
}

// -- Main Loop:
void main_loop(boost::shared_ptr<visualization::PCLVisualizer> viewer,
          ros::Publisher &pub_to_node3, ros::Publisher &pub_to_rviz)
{
    int cnt_cloud=0;
    while (ros::ok() && !viewer->wasStopped())
    {
        if (flag_receive_kinect_cloud)
        {
            flag_receive_kinect_cloud = false;
            cnt_cloud++;

            // -- filtByVoxelGrid
            // float x_grid_size = 0.005, y_grid_size = 0.005, z_grid_size = 0.005;
            float x_grid_size = 0.02, y_grid_size = 0.02, z_grid_size = 0.02;
            cloud_src = my_pcl::filtByVoxelGrid(cloud_src, x_grid_size, y_grid_size, z_grid_size);

            // -- filtByStatisticalOutlierRemoval
            float mean_k = 50, std_dev = 1.0;
            cloud_src = my_pcl::filtByStatisticalOutlierRemoval(cloud_src, mean_k, std_dev);

            // -- rotate cloud, pub to rviz
            copyPointCloud(*cloud_src, *cloud_rotated);
            // cloud_rotated.points = cloud_src.points; // copy as rotate cloud
            pubPclCloudToTopic(pub_to_rviz, cloud_rotated);

            // -- remove plane, clustering, pub to node3
            copyPointCloud(*cloud_rotated, *cloud_segmented);
            // cloud_segmented.points = cloud_rotated.points; // copy as remove plane and clustering
            pubPclCloudToTopic(pub_to_node3, cloud_segmented);
        
            // -- Update viewer
            viewer->updatePointCloud(cloud_segmented, PCL_VIEWER_CLOUD_NAME);

            // -- Print info
            cout << endl;
            printf("------------------------------------------\n");
            printf("-------- Processing %dth cloud -----------\n", cnt_cloud);
            ROS_INFO("Subscribed a point cloud from ros topic.");

            cout << "cloud_src: ";
            my_pcl::printCloudSize(cloud_src);

            cout << "cloud_src: ";
            my_pcl::printCloudSize(cloud_rotated);

            cout << "cloud_src: ";
            my_pcl::printCloudSize(cloud_segmented);
            printf("------------------------------------------\n\n");
        }
        viewer->spinOnce(10);
        ros::spinOnce(); // In python, sub is running in different thread. In C++, same thread. So need this.
    }
}

// -- Main (Only for setting up variables. The main loop is at above.)
int main(int argc, char **argv)
{
    // Init node
    string node_name = "node2";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Settings: topic names
    string topic_n1_to_n2, topic_n2_to_n3, topic_name_kinect_cloud, topic_n2_to_rviz;
    if (!nh.getParam("topic_n1_to_n2", topic_n1_to_n2))
        assert(0);
    if (!nh.getParam("topic_n2_to_n3", topic_n2_to_n3))
        assert(0);
    if (!nh.getParam("topic_name_kinect_cloud", topic_name_kinect_cloud))
        assert(0);
    if (!nh.getParam("topic_n2_to_rviz", topic_n2_to_rviz))
        assert(0);

    // Settings: file names for saving point cloud
    string file_folder, file_name_cloud_rotated, file_name_cloud_segmented;
    if (!nh.getParam("file_folder", file_folder))
        assert(0);
    if (!nh.getParam("file_name_cloud_rotated", file_name_cloud_rotated))
        assert(0);
    if (!nh.getParam("file_name_cloud_segmented", file_name_cloud_segmented))
        assert(0);

    // Subscriber and Publisher
    ros::Subscriber sub_from_node1 = nh.subscribe(topic_n1_to_n2, 1, callbackFromNode1); // 1 is queue size
    ros::Subscriber sub_from_kinect = nh.subscribe(topic_name_kinect_cloud, 1, callbackFromKinect);
    ros::Publisher pub_to_node3 = nh.advertise<sensor_msgs::PointCloud2>(topic_n2_to_n3, 1);
    ros::Publisher pub_to_rviz = nh.advertise<sensor_msgs::PointCloud2>(topic_n2_to_rviz, 1);

    // Init viewer
    boost::shared_ptr<visualization::PCLVisualizer> viewer =
        my_pcl::initPointCloudRGBViewer(cloud_segmented,
            PCL_VIEWER_NAME, PCL_VIEWER_CLOUD_NAME,
            0.1); // unit length of the shown coordinate frame

    // Loop, subscribe ros_cloud, and view
    main_loop(viewer, pub_to_node3, pub_to_rviz);

    // Return
    ROS_INFO("Node2 stops");
    return 0;
}
