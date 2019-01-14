#!/usr/bin/env python
# -*- coding: utf-8 -*-

open3d_ros_datatype_conversion

''' Main reference page:
* A very useful but very bad documented github repo:
    https://github.com/karaage0703/open3d_ros
    From it, I copied pieces of codes of converting open3d to ros, and then assembled to a function.
* PointCloud2 message type:
    http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
* function of sensor_msgs.point_cloud2.create_cloud()	
    http://docs.ros.org/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1point__cloud2.html#ad456fcf9391ad2ed2279df69572ca71d
* open3d: from numpy to open3d pointcloud
http://www.open3d.org/docs/tutorial/Basic/working_with_numpy.html#from-numpy-to-open3d-pointcloud
'''

''' Explanation of sensor_msgs::PointCloud2's contents: 
1. First using the following c++ code to generate a PointCloud2 message from pcl cloud
{
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_pcl_ptr；
  sensor_msgs::PointCloud2 cloud_ros;
  pcl::toROSMsg(*cloud_pcl_ptr, cloud_ros);
}
2. Publish cloud_ros, and view it by "rostopic echo", we get the following contents (see below).
3. We can see that it's using:
    1) 32 byte length, 2) "rgb" at 16th byte, 3) "rgb" datatype 7, which is float (4 bytes).
Despite this, we can still set things like this (And this is what this script is using):
    1) 16 byte length, 2) "rgb" at 12th byte, 3) "rgb" datatype 6, which is uint32.

---------------- contents after "rostopic echo" ---------------- 
header: 
  seq: 16
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ""
height: 1
width: 1706
fields: 
  - 
    name: "x"
    offset: 0
    datatype: 7
    count: 1
  - 
    name: "y"
    offset: 4
    datatype: 7
    count: 1
  - 
    name: "z"
    offset: 8
    datatype: 7
    count: 1
  - 
    name: "rgb"
    offset: 16
    datatype: 7
    count: 1
is_bigendian: False
point_step: 32
row_step: 54592
data: ...
'''

import open3d
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convert_cloud_from_Open3D_to_ROS(open3d_cloud, frame_id="odom"):
    # Change rgb color from "three float" to "one 24-byte int"
    # 0x00FFFFFF is white, 0x00000000 is black.
    colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
    colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2] 
    
    # Combine
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return sensor_msgs.point_cloud2.create_cloud(header, FIELDS ,
            np.c_[np.asarray(open3d_cloud.points), colors]) #column stack

def convert_cloud_from_ROS_to_Open3D(ros_cloud):
    # Change rgb color from "three float" to "one 24-byte int"
    # 0x00FFFFFF is white, 0x00000000 is black.
    colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
    colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2] 
    
    # Combine
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return sensor_msgs.point_cloud2.create_cloud(header, FIELDS ,
            np.c_[np.asarray(open3d_cloud.points), colors]) #column stack
# Example of usage
if __name__ == "__main__":
    rospy.init_node('test_pc_conversion_between_Open3D_and_ROS', anonymous=True)
    
    # Read point cloud from file
    filename="data/color_milk_and_2bottles.pcd"
    open3d_cloud = open3d.read_point_cloud(filename)
  
    # Set publisher
    topic_name="kinect2/qhd/points"
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
    
    # Set subscriber
    rospy.Subscriber('topic_name', PointCloud2, callback)      
    def callback(ros_cloud):
        None  

    # Convert open3d_cloud to ros_cloud, and publish
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if True: # Use the cloud converted from open3d
            rospy.loginfo("\n\nConverting cloud from Open3d to ROS PointCloud2 ...")
            ros_cloud = convert_cloud_from_Open3D_to_ROS(open3d_cloud)
        else: # Use a simple cloud with 3 points
            rospy.loginfo("\n\nConverting a 3-point cloud into ROS PointCloud2 ...")
            TEST_CLOUD_POINTS = [
                [1.0, 0.0, 0.0, 0xff0000],
                [0.0, 1.0, 0.0, 0x00ff00],
                [0.0, 0.0, 1.0, 0x0000ff],
            ]
            ros_cloud = sensor_msgs.point_cloud2.create_cloud(
                Header(frame_id="odom"), FIELDS , TEST_CLOUD_POINTS)
        pub.publish(ros_cloud)
        rospy.loginfo("Conversion and publish success ...")
        rate.sleep()
    
    # Receive ros_cloud, convert to open3d_cloud, and draw

    #    open3d.draw_geometries([open3d_cloud])

    

    