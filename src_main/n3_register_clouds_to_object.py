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
from lib_cloud_registration import CloudRegister, resizeCloudXYZ, mergeClouds, createXYZAxis, getCloudSize, filtCloudByRange

from lib_geo_trans import rotx, roty, rotz

VIEW_RES_BY_OPEN3D=True # This is difficult to set orientation. And has some bug.
VIEW_RES_BY_RVIZ=~VIEW_RES_BY_OPEN3D
OBJECT_RANGE = 0.12 #The object is inside a region of x=(-r,r) && y=(-r,r)

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
        self.cloud_XYZaxis = createXYZAxis(coord_axis_length=0.1, num_points_in_axis=50)

    def updateCloud(self, new_cloud):
        try:
            new_cloud = mergeClouds(new_cloud, self.cloud_XYZaxis)
            new_cloud = resizeCloudXYZ(new_cloud, 5.0) # Resize cloud, so rviz has better view
            self.pub.publish(convertCloudFromOpen3dToRos(new_cloud))
        except:
            print "Node 3 fails to update cloud, due to the input is empty!\n"

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
        # self.rotateCloudForBetterViewing(open3d_cloud)
        self.cloud_buff.append(open3d_cloud)
    
    def hasNewCloud(self):
        return len(self.cloud_buff)>0

    def popCloud(self):
        return self.cloud_buff.popleft()

    # def rotateCloudForBetterViewing(self, cloud):
    #     T=rotx(np.pi, matrix_len=4)
    #     cloud.transform(T)

# ---------------------------- Main ----------------------------
if __name__ == "__main__":
    rospy.init_node("node3")
    num_goalposes = rospy.get_param("num_goalposes")

    # -- Set output filename
    file_folder = rospy.get_param("file_folder") 
    file_name_cloud_final = rospy.get_param("file_name_cloud_final")

    # -- Subscribe to cloud + Visualize it
    cloud_subscriber = SubscriberOfCloud() # set subscriber
    viewer = chooseViewer() # set viewer

    # -- Parameters
    radius_registration=rospy.get_param("~radius_registration") # 0.002
    radius_merge=rospy.get_param("~radius_merge")  # 0.001

    # -- Loop
    rate = rospy.Rate(100)
    cnt = 0
    cloud_register = CloudRegister(
        voxel_size_regi=0.005, global_regi_ratio=4.0, 
        voxel_size_output=0.002,
        USE_GLOBAL_REGI=True, USE_ICP=True, USE_COLORED_ICP=False)


    while not rospy.is_shutdown():
        if cnt<num_goalposes and cloud_subscriber.hasNewCloud():
            
            cnt += 1
            rospy.loginfo("=========================================")
            rospy.loginfo("Node 3: Received the {}th segmented cloud.".format(cnt))


            # Register Point Cloud
            new_cloud = cloud_subscriber.popCloud()
            if getCloudSize(new_cloud)==0:
                print "  The received cloud is empty. Not processing it."
                continue
            res_cloud = cloud_register.addCloud(new_cloud)
                
            # Update and save to file
            viewer.updateCloud(res_cloud)
            open3d.write_point_cloud(file_folder+file_name_cloud_final, res_cloud)
            
            if cnt==num_goalposes:
                rospy.loginfo("================== Cloud Registration Completes ====================")
                rospy.sleep(1.0)
                rospy.loginfo("=====================================================================")
                
                obj_range=OBJECT_RANGE
                open3d.write_point_cloud(file_folder+"finala_"+file_name_cloud_final, res_cloud)
                res_cloud = filtCloudByRange(res_cloud, xmin=-obj_range, xmax=obj_range, ymin=-obj_range, ymax=obj_range )

                viewer.updateCloud(res_cloud)
                open3d.write_point_cloud(file_folder+"finalb_"+file_name_cloud_final, res_cloud)

        # Update viewer
        viewer.updateView()

        # Sleep
        rate.sleep()

    # -- Save final cloud to file
    rospy.loginfo("!!!!! Node 3 ready to stop ...")
    
    # -- Node stops
    viewer.destroy_window()
    rospy.loginfo("!!!!! Node 3 stops.")
