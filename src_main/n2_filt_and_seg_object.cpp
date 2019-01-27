/*
Main function:
* subscribe to cloud_src, filter it, rotated, pub to rviz.
* seg plane, do clustering, pub the object to node3
*/

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Pose.h"

#include "my_basics/basics.h"
#include "my_pcl/pcl_visualization.h"
#include "my_pcl/pcl_commons.h"
#include "my_pcl/pcl_filters.h"
#include "my_pcl/pcl_advanced.h"
#include "my_pcl/pcl_io.h"
#include "scan3d_by_baxter/T4x4.h" // my message

using namespace std;
using namespace pcl;

// ------------------------------------- ROS Params -------------------------------------

// Topic names
string topic_n1_to_n2, topic_n2_to_n3, topic_name_rgbd_cloud, topic_n2_to_rviz;

// Filenames for writing to file
string file_folder, file_name_cloud_src, file_name_cloud_segmented;
int file_name_index_width;

// Filename for reading chessboard's pose
string file_folder_config, file_name_T_baxter_to_chess;

// Filter: voxel filtering (filtByVoxelGrid)
float x_grid_size, y_grid_size, z_grid_size;

// Fitler: isolated points (filtByStatisticalOutlierRemoval)
float mean_k = 50, std_dev = 1.0;

// Filter: range filtering
bool flag_do_range_filt;
float x_range_radius, y_range_radius, z_range_low, z_range_up;
float chessboard_x, chessboard_y, chessboard_z;
float T_baxter_to_chess[4][4] = {0};

// Filter: plane segmentation
float plane_distance_threshold;
int plane_max_iterations;
int num_planes;
float ratio_of_rest_points = -1; // disabled

// Filter: divide cloud into clusters
bool flag_do_clustering;
double cluster_tolerance;
int min_cluster_size, max_cluster_size;

// ------------------------------------- Vars -------------------------------------

// Vars for workflow control
bool flag_receive_from_node1 = false;
bool flag_receive_kinect_cloud = false;

// Data contents
float T_baxter_to_depthcam[4][4];
PointCloud<PointXYZRGB>::Ptr cloud_src(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cloud_rotated(new PointCloud<PointXYZRGB>);   // this pubs to rviz
PointCloud<PointXYZRGB>::Ptr cloud_segmented(new PointCloud<PointXYZRGB>); // this pubs to node3

// ------------------------------------- Functions -------------------------------------
// -- Read params from ROS parameter server
void initAllROSParams();
#define NH_GET_PARAM(param_name, returned_val)                          \
    if (!nh.getParam(param_name, returned_val))                             \
    {                                                                \
        cout << "Error in reading ROS param named: " << param_name << endl; \
        assert(0);                                                   \
    }


// -- Input/Output and Sub/Publisher

void read_T_from_file(float T_16x1[16], string filename);
void subCallbackFromNode1(const scan3d_by_baxter::T4x4::ConstPtr &pose_message);
void subCallbackFromKinect(const sensor_msgs::PointCloud2 &ros_cloud);
void pubPclCloudToTopic(ros::Publisher &pub, PointCloud<PointXYZRGB>::Ptr pcl_cloud);

// -- Main processing functions
void process_to_get_cloud_rotated();
void process_to_get_cloud_segmented();
void print_cloud_processing_result(int cnt_cloud);

// -- Main Loop:
void main_loop(ros::Publisher &pub_to_node3, ros::Publisher &pub_to_rviz)
{
    int cnt_cloud = 0;
    while (ros::ok())
    {
        if (flag_receive_kinect_cloud)
        {
            flag_receive_kinect_cloud = false;
            cnt_cloud++;

            // Process cloud
            process_to_get_cloud_rotated();
            pubPclCloudToTopic(pub_to_rviz, cloud_rotated);

            process_to_get_cloud_segmented();
            pubPclCloudToTopic(pub_to_node3, cloud_segmented);

            // Save to file
            string suffix = my_basics::int2str(cnt_cloud, file_name_index_width) + ".pcd";

            string f0 = file_folder + file_name_cloud_src + suffix;
            my_pcl::write_point_cloud(f0, cloud_src);

            string f2 = file_folder + file_name_cloud_segmented + suffix;
            my_pcl::write_point_cloud(f2, cloud_segmented);

            // print
            print_cloud_processing_result(cnt_cloud); // Print info
        }
        ros::spinOnce(); // In python, sub is running in different thread. In C++, same thread. So need this.
        ros::Duration(0.01).sleep();
    }
}

// -- Main: set up variables, subscribers, and publishers.
int main(int argc, char **argv)
{
    // Init node
    string node_name = "node2";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    initAllROSParams();

    // Subscriber and Publisher
    ros::Subscriber sub_from_node1 = nh.subscribe(topic_n1_to_n2, 1, subCallbackFromNode1); // 1 is queue size
    ros::Subscriber sub_from_kinect = nh.subscribe(topic_name_rgbd_cloud, 1, subCallbackFromKinect);
    ros::Publisher pub_to_node3 = nh.advertise<sensor_msgs::PointCloud2>(topic_n2_to_n3, 1);
    ros::Publisher pub_to_rviz = nh.advertise<sensor_msgs::PointCloud2>(topic_n2_to_rviz, 1);

    // -- Loop, subscribe ros_cloud, and view
    main_loop(pub_to_node3, pub_to_rviz);

    // Return
    ROS_INFO("Node2 stops");
    return 0;
}

// ================================================================================
// =========================== Cloud Processing====================================
// ================================================================================

// -----------------------------------------------------
// -----------------------------------------------------
void process_to_get_cloud_rotated()
{
    // Func: Filtering; Rotate cloud to Baxter robot frame

    // -- filtByVoxelGrid
    pcl::copyPointCloud(*cloud_src, *cloud_rotated);
    cloud_rotated = my_pcl::filtByVoxelGrid(cloud_rotated, x_grid_size, y_grid_size, z_grid_size);

    // -- filtByStatisticalOutlierRemoval
    cloud_rotated = my_pcl::filtByStatisticalOutlierRemoval(cloud_rotated, mean_k, std_dev);

    // -- rotate cloud to Baxter's frame
    for (PointXYZRGB &p : cloud_rotated->points)
        my_basics::preTranslatePoint(T_baxter_to_depthcam, p.x, p.y, p.z);
}

// -----------------------------------------------------
// -----------------------------------------------------
void process_to_get_cloud_segmented()
{
    // Func:    Range filtering，
    //          Optional: Remove plane (table); Do clustering; Choose the largest one

    // -- filtByPassThrough (by range)
    copyPointCloud(*cloud_rotated, *cloud_segmented);
    if (flag_do_range_filt)
    {
        // cout<<"Debug: before range filter"<<endl;
        // my_pcl::printCloudSize(cloud_segmented);
        cloud_segmented = my_pcl::filtByPassThrough(
            cloud_segmented, "x", chessboard_x + x_range_radius, chessboard_x - x_range_radius);

        cloud_segmented = my_pcl::filtByPassThrough(
            cloud_segmented, "y", chessboard_y + y_range_radius, chessboard_y - y_range_radius);

        cloud_segmented = my_pcl::filtByPassThrough(
            cloud_segmented, "z", chessboard_z + z_range_up, chessboard_z + z_range_low);
        // cout<<"Debug: after range filter"<<endl;
        // my_pcl::printCloudSize(cloud_segmented);
    }

    // -- Remove planes
    int num_removed_planes = my_pcl::removePlanes(
        cloud_segmented,
        plane_distance_threshold, plane_max_iterations,
        num_planes, ratio_of_rest_points);

    // -- Clustering: Divide the remaining point cloud into different clusters
    if (flag_do_clustering)
    {
        vector<PointIndices> clusters_indices = my_pcl::divideIntoClusters(
            cloud_segmented, cluster_tolerance, min_cluster_size, max_cluster_size);

        // -- Extract indices into cloud clusters
        vector<PointCloud<PointXYZRGB>::Ptr> cloud_clusters =
            my_pcl::extractSubCloudsByIndices(cloud_segmented, clusters_indices);
        cloud_segmented = cloud_clusters[0];
    }

    // -- rotate cloud to Chessboard's frame, for better viewing in PCL viewer
    for (PointXYZRGB &p : cloud_segmented->points)
        my_basics::preTranslatePoint(T_baxter_to_chess, p.x, p.y, p.z);
}

// -----------------------------------------------------
// -----------------------------------------------------
void print_cloud_processing_result(int cnt_cloud)
{

    cout << endl;
    printf("------------------------------------------\n");
    printf("-------- Processing %dth cloud -----------\n", cnt_cloud);
    ROS_INFO("Subscribed a point cloud from ros topic.");

    cout << "camera pos:" << endl;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
            cout << T_baxter_to_depthcam[i][j] << " ";
        cout << endl;
    }
    cout << endl;

    cout << "cloud_src: ";
    my_pcl::printCloudSize(cloud_src);

    cout << "cloud_rotated: ";
    my_pcl::printCloudSize(cloud_rotated);

    cout << "cloud_segmented: ";
    my_pcl::printCloudSize(cloud_segmented);
    printf("------------------------------------------\n\n");
}

// -----------------------------------------------------
// -----------------------------------------------------
void read_T_from_file(float T_16x1[16], string filename)
{
    ifstream fin;
    fin.open(filename);
    float val;
    int cnt = 0;
    assert(fin.is_open()); // Fail to find the config file
    while (fin >> val)
        T_16x1[cnt++] = val;
    fin.close();
    return;
}

void subCallbackFromNode1(const scan3d_by_baxter::T4x4::ConstPtr &pose_message)
{
    flag_receive_from_node1 = true;
    const vector<float> &trans_mat_16x1 = pose_message->TransformationMatrix;
    for (int cnt = 0, i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            T_baxter_to_depthcam[i][j] = trans_mat_16x1[cnt++];
}
void subCallbackFromKinect(const sensor_msgs::PointCloud2 &ros_cloud)
{
    if (flag_receive_from_node1)
    {
        flag_receive_from_node1 = false;
        flag_receive_kinect_cloud = true;
        fromROSMsg(ros_cloud, *cloud_src);
    }
}
void pubPclCloudToTopic(ros::Publisher &pub, PointCloud<PointXYZRGB>::Ptr pcl_cloud)
{
    sensor_msgs::PointCloud2 ros_cloud_to_pub;
    pcl::toROSMsg(*pcl_cloud, ros_cloud_to_pub);
    ros_cloud_to_pub.header.frame_id = "base";
    pub.publish(ros_cloud_to_pub);
}


void initAllROSParams()
{
    {
        ros::NodeHandle nh;

        // Topic names
        NH_GET_PARAM("topic_n1_to_n2",topic_n1_to_n2)
        NH_GET_PARAM("topic_n2_to_n3",topic_n2_to_n3)
        NH_GET_PARAM("topic_name_rgbd_cloud",topic_name_rgbd_cloud)
        NH_GET_PARAM("topic_n2_to_rviz",topic_n2_to_rviz)

        // File names for saving point cloud
        NH_GET_PARAM("file_folder",file_folder)
        NH_GET_PARAM("file_name_cloud_src",file_name_cloud_src)
        NH_GET_PARAM("file_name_cloud_segmented",file_name_cloud_segmented)
        NH_GET_PARAM("file_name_index_width",file_name_index_width)


        // Filename for reading chessboard's pose
        NH_GET_PARAM("file_folder_config",file_folder_config)
        NH_GET_PARAM("file_name_T_baxter_to_chess",file_name_T_baxter_to_chess)
  
        float tmpT[16] = {0};
        read_T_from_file(tmpT, file_folder_config + file_name_T_baxter_to_chess);
        for (int cnt = 0, i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                T_baxter_to_chess[i][j] = tmpT[cnt++];
        chessboard_x = T_baxter_to_chess[0][3];
        chessboard_y = T_baxter_to_chess[1][3];
        chessboard_z = T_baxter_to_chess[2][3];

    }

    // ---------------------------- Filters ----------------------------
    {
        ros::NodeHandle nh("~");

        // -- filtByPassThrough
        NH_GET_PARAM("flag_do_range_filt",flag_do_range_filt)
        NH_GET_PARAM("x_range_radius",x_range_radius)
        NH_GET_PARAM("y_range_radius",y_range_radius)
        NH_GET_PARAM("z_range_low",z_range_low)
        NH_GET_PARAM("z_range_up",z_range_up)

        // -- filtByVoxelGrid
        NH_GET_PARAM("x_grid_size",x_grid_size)
        NH_GET_PARAM("y_grid_size",y_grid_size)
        NH_GET_PARAM("z_grid_size",z_grid_size)

        // -- Segment plane
        NH_GET_PARAM("plane_distance_threshold",plane_distance_threshold)
        NH_GET_PARAM("plane_max_iterations",plane_max_iterations)
        NH_GET_PARAM("num_planes",num_planes)

        // -- Clustering
        NH_GET_PARAM("flag_do_clustering",flag_do_clustering)
        NH_GET_PARAM("cluster_tolerance",cluster_tolerance)
        NH_GET_PARAM("min_cluster_size",min_cluster_size)
        NH_GET_PARAM("max_cluster_size",max_cluster_size)
    }

}
