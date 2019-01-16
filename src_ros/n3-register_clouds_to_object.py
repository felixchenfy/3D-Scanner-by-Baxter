#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
    rospy.init_node("node3")

    # -- Params settings
    topic_n2_to_n3 = rospy.get_param("topic_n2_to_n3")
    file_folder = rospy.get_param("file_folder")
    file_name_cloud_final = rospy.get_param("file_name_cloud_final")
    num_goalposes = rospy.get_param("num_goalposes") # DEBUG, NOT USED NOW

    # -- Set subscriber
    global received_ros_cloud
    received_ros_cloud = None
    def callback(ros_cloud):
        global received_ros_cloud
        received_ros_cloud=ros_cloud
    rospy.Subscriber(topic_n2_to_n3, PointCloud2, callback)      
    
    # -- Set viewer
    vis_cloud= open3d.PointCloud()
    vis = open3d.Visualizer()
    vis.create_window()
    vis.add_geometry(vis_cloud)

    # -- Set up point cloud registeration
    # DEBUG
    final_cloud= open3d.PointCloud()


    # -- Loop
    rate = rospy.Rate(100)
    cnt = 0
    while not rospy.is_shutdown():
        if received_ros_cloud is not None:
            cnt+=1
            rospy.loginfo("=========================================")
            rospy.loginfo("Node 3: Received the {}th segmented cloud.".format(cnt))

            # Convert file format
            new_cloud_piece = convertCloudFromRosToOpen3d(received_ros_cloud)

            # Register Point Cloud
            final_cloud = new_cloud_piece #DEBUG, change to registration

            # Update point cloud
            copyOpen3dCloud(final_cloud, vis_cloud)
            vis.add_geometry(vis_cloud) 
            vis.update_geometry()

            # clear
            received_ros_cloud = None

            # Save to file
            if cnt == num_goalposes:
                None # DEBUG, save cloud to file
        
        # Update viewer
        vis.poll_events()
        vis.update_renderer()

        # Sleep
        rate.sleep()

    # -- Node stops
    vis.destroy_window()
    rospy.loginfo("Node 3 stops")
    
