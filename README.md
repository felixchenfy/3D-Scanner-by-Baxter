
测试一下XYZRGB的点云是否兼容XYZ。
读XYZ的，存起来，画出来看一下。

# reference

` Miaoding's 3d scanner project: https://github.com/zjudmd1015/Mini-3D-Scanner. It's really a good example for me to get started.
` PCL and Open3D. My codes for the algorithms are basically copied piece by piece from their official tutorial examples.
` https://github.com/karaage0703/open3d_ros
It's for datatype conversion of point cloud between open3d and ros-message. Super useful. I added some notes and stored it in xxxxxxxxxxxxxxxx.


# 简历
Title: Object 3d Scanning and Locating by Robot Arm with RGB-D Camera
Calibrate camera. Convert point cloud (RGB-D image) to robot coordinate through forward kinematics.
Filter, segmentation, and (clustering, or, manually thresholding) to acquire the object from a single point cloud. 
Register the object from different point clouds into one and estimate its true position.


# ROS file structure

0.5-sim-pub_cloud (py)
0.7-receive_cloud_and_display-ByPCL (This is for testing whether receives the point cloud)
0.7-receive_cloud_and_display-ByOpen3D (This is for testing whether receives the point cloud)
1-main-control_robot_and_workflow (robot move; take picture; take picture;)
2-receive_save_cloud-seg_object-save_pub (cpp)
3-register_clouds-save_pub (PointXYZ PINGxyz)
4-display-by_pcl
4-display-by_rviz
