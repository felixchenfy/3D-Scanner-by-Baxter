
测试一下XYZRGB的点云是否兼容XYZ。
读XYZ的，存起来，画出来看一下。

# 简历
Title: Object 3d Scanning and Locating by Robot Arm with RGB-D Camera
Calibrate camera. Convert point cloud (RGB-D image) to robot coordinate through forward kinematics.
Filter, segmentation, and (clustering, or, manually thresholding) to acquire the object from a single point cloud. 
Register the object from different point clouds into one and estimate its true position.


# ROS file structure

0.5-sim-pub_cloud
1-receive_cloud-save
1-receive_cloud-filt-seg-save-pub
2-register_clouds-save-pub (PointXYZ PINGxyz)
3-display
