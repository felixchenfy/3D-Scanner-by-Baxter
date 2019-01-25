#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Include common
import numpy as np
import open3d
import sys, os, copy
from collections import deque
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

# Include ROS
import rospy
from sensor_msgs.msg import PointCloud2

# Include my lib
sys.path.append(PYTHON_FILE_PATH + "../src_python")
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
from lib_cloud_registration import drawTwoClouds, registerClouds, copyOpen3dCloud
from lib_geo_trans import rotx, roty, rotz

VIEW_RES_BY_OPEN3D=True # This is difficult to set orientation. And has some bug.
VIEW_RES_BY_RVIZ=~VIEW_RES_BY_OPEN3D

# ---------------------------- Two viewers (choose one) ----------------------------
class Open3DViewer(object):
    def __init__(self):
        self.vis_cloud = open3d.PointCloud()
        self.viewer = open3d.Visualizer()
        self.viewer.create_window()
        self.viewer.add_geometry(self.vis_cloud)

    def updateCloud(self, new_cloud):
        self.vis_cloud.points = copy.deepcopy(new_cloud.points)
        self.vis_cloud.colors = copy.deepcopy(new_cloud.colors)
        self.viewer.add_geometry(self.vis_cloud)
        self.viewer.update_geometry()

    def updateView(self):
        self.viewer.poll_events()
        self.viewer.update_renderer()
        print "NOTICE: OPEN3D VIEWER HAS BEEN UPDATED!!!"
    
    def destroy_window(self):
        self.viewer.destroy_window()

class RvizViewer(object):
    def __init__(self):
        topic_n3_to_rviz=rospy.get_param("topic_n3_to_rviz")
        self.pub = rospy.Publisher(topic_n3_to_rviz, PointCloud2, queue_size=10)
        self.updateView = lambda: None
        self.destroy_window = lambda: None

    def updateCloud(self, new_cloud):
        self.pub.publish(convertCloudFromOpen3dToRos(new_cloud))

def chooseViewer():
    if 0: 
        # This is 1) more difficult to set viewer angle. 
        # 2) cannot set window size. 
        # 3) Much Much Slower to display. Big latency. I don't know why.
        # Thus, Better not use this.
        return Open3DViewer() # open3d
    else:
        return RvizViewer() # rviz

# ---------------------------- One subscriber ----------------------------
class SubscriberOfCloud(object):
    def __init__(self):
        topic_n2_to_n3 = rospy.get_param("topic_n2_to_n3")
        rospy.Subscriber(topic_n2_to_n3, PointCloud2, self.sub_callback)
        self.cloud_buff = deque()

    def sub_callback(self, ros_cloud):
        open3d_cloud = convertCloudFromRosToOpen3d(ros_cloud)
        self.rotateCloudForBetterViewing(open3d_cloud)
        self.cloud_buff.append(open3d_cloud)
    
    def hasNewCloud(self):
        return len(self.cloud_buff)>0

    def popCloud(self):
        return self.cloud_buff.popleft()

    def rotateCloudForBetterViewing(self, cloud):
        T=rotx(np.pi, matrix_len=4)
        cloud.transform(T)

# ---------------------------- Main ----------------------------
if __name__ == "__main__":
    rospy.init_node("node3")

    # -- Output cloud file
    file_folder = rospy.get_param("file_folder") 
    file_name_cloud_final = rospy.get_param("file_name_cloud_final")

    # -- Subscribe to cloud + Visualize it
    cloud_subscriber = SubscriberOfCloud() # set subscriber
    viewer = chooseViewer() # set viewer

    # -- Loop
    rate = rospy.Rate(100)
    final_cloud = open3d.PointCloud()
    cnt = 0
    while not rospy.is_shutdown():
        if cloud_subscriber.hasNewCloud():
            
            cnt += 1
            rospy.loginfo("=========================================")
            rospy.loginfo("Node 3: Received the {}th segmented cloud.".format(cnt))


            # Register Point Cloud
            new_cloud_piece = cloud_subscriber.popCloud()
            if cnt==1:
                copyOpen3dCloud(src=new_cloud_piece, dst=final_cloud)
            else:
                final_cloud, transformation = registerClouds(
                    src=new_cloud_piece, target=final_cloud, 
                    radius_regi=0.010, radius_merge=0.005)

            # Update point cloud
            viewer.updateCloud(final_cloud)
                
            # Save to file for every update
            open3d.write_point_cloud(file_folder+file_name_cloud_final, final_cloud)


        # Update viewer
        viewer.updateView()

        # Sleep
        rate.sleep()

    # -- Save final cloud to file
    rospy.loginfo("!!!!! Node 3 ready to stop ...")
    
    # -- Node stops
    viewer.destroy_window()
    rospy.loginfo("!!!!! Node 3 stops.")
