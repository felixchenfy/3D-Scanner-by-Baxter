#!/usr/bin/env python
# -*- coding: utf-8 -*-

# $ rosrun scan3d-by-baxter read_cloud_and_pub_by_open3d.py

# Include common
import open3d
import numpy as np
import sys, os
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# Include ros
import rospy
from sensor_msgs.msg import PointCloud2

# Include my lib
sys.path.append(PYTHON_FILE_PATH + "../src_ros")
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d

# Main
if __name__ == "__main__":

    # Params setting
    topic_name = "kinect2/qhd/points"
    cloud_filename = "../data_debug/cloud_cluster_0.pcd"
    node_name = "read_cloud_and_pub_by_open3d"
    
    # Set node
    rospy.init_node(node_name, anonymous=True)

    # Read file
    filename = PYTHON_FILE_PATH+cloud_filename
    open3d_cloud = open3d.read_point_cloud(filename)
    rospy.loginfo("Loading cloud file from: \n  " + filename)
    print(open3d_cloud)
    ros_cloud = convertCloudFromOpen3dToRos(open3d_cloud)

    # Publish
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
    cnt = 0
    while not rospy.is_shutdown():
        rospy.sleep(1)
        pub.publish(ros_cloud)
        rospy.loginfo("Publish {:d}th cloud...\n".format(cnt))
        cnt += 1
    rospy.loginfo("this node stops: "+node_name)
