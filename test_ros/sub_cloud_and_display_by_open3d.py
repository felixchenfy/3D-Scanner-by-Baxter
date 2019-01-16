#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Open3d.Visualizer() currently doesn't have a function of "destroy_geometry", 
  something similar to the "removePointCloud" in pcl.

I'm manually copy new_point_cloud's content to the one for visualization
{
    def copyOpen3dCloud(src, dst):
        dst.points=src.points
        dst.colors=src.colors
}

'''

# Include common
import open3d
import numpy as np
import sys, os
import copy
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# Include ros
import rospy
from sensor_msgs.msg import PointCloud2

# Include my lib
sys.path.append(PYTHON_FILE_PATH + "../src_ros")
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d

def copyOpen3dCloud(src, dst):
    dst.points=src.points
    dst.colors=src.colors

# Main
if __name__ == "__main__":

    # Params settings
    topic_name_kinect_cloud = rospy.get_param("/topic_name_kinect_cloud",
        "kinect2/qhd/points")
    node_name = "sub_cloud_and_display_by_open3d"
    
    # Set node
    rospy.init_node(node_name, anonymous=True)

    # Set subscriber
    global received_ros_cloud
    received_ros_cloud = None
    def callback(ros_cloud):
        global received_ros_cloud
        received_ros_cloud=ros_cloud
        rospy.loginfo("Received ROS PointCloud2 message.")
    rospy.Subscriber(topic_name_kinect_cloud, PointCloud2, callback)      
    
    # Set viewer
    open3d_cloud= open3d.PointCloud()
    vis = open3d.Visualizer()
    vis.create_window()
    vis.add_geometry(open3d_cloud)

    # Loop
    rate = rospy.Rate(100)
    cnt = 0
    while not rospy.is_shutdown():
        if received_ros_cloud is not None:
            cnt+=1
            idx_method=1
            if idx_method==1:
                tmp = convertCloudFromRosToOpen3d(received_ros_cloud)
                copyOpen3dCloud(tmp, open3d_cloud)
                vis.add_geometry(open3d_cloud) 
            if idx_method==2: # this is not working!!!
                open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
            if idx_method==3: # this is keep adding clouds to viewer
                open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
                vis.add_geometry(open3d_cloud)
            vis.update_geometry()
            print("Updating geometry for the {}th time".format(cnt))
            received_ros_cloud = None # clear
        vis.poll_events()
        vis.update_renderer()
        rate.sleep()
        # print("Updating viewer")

    # Return
    vis.destroy_window()
    rospy.loginfo("This node stops: " + node_name)
    
