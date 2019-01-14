
测试一下XYZRGB的点云是否兼容XYZ。
读XYZ的，存起来，画出来看一下。

# ------------------------------------------------------------------------------
# reference

* A 3d scanner project: https://github.com/zjudmd1015/Mini-3D-Scanner.  
It's a really good example for me to get started.

* PCL and Open3D.  
My codes for the algorithms are basically copied piece by piece from their official tutorial examples.

* https://github.com/karaage0703/open3d_ros  
It's for datatype conversion of point cloud between open3d and ros-message. Super useful.  
However, it's documentation and variables naming are pretty bad. I re-arranged the code and test cases and stored the file in xxxxxxxxxxxx.

# ------------------------------------------------------------------------------
# Dependencies and Install
Below are the dependencies. Some I give the installing commands I used. Others please refer to their official website.  
In short: OpenCV, Eigen, PCL, Open3D, and other ROS packages.

### OpenCV 4.0
I'm using 4.0, but >3.0 should be fine. I used it only for reading YAML and some cv::Mat operations.  
This is really nice tutorial for install OpenCV: [link](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/).

### [Eigen 3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
$ sudo apt-get install libeigen3-dev  
(Eigen only has header files. No ".so" or ".a".)

### pcl 1.7
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl  
sudo apt-get update  
sudo apt-get install libpcl-all  

### [pcl_ros](http://wiki.ros.org/pcl_ros)
$ sudo apt-get install ros-melodic-pcl-ros  
$ sudo apt-get install ros-melodic-pcl-conversions  
(It seems that the above two has already been installed on my ubuntu.)

### Open3D
Two official installation tutorials: [this](http://www.open3d.org/docs/getting_started.html) and [this](http://www.open3d.org/docs/compilation.html).

### Baxter's packages
Please follow the long tutorial on [Rethink Robotics' website](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter)

### Some other libs that might be usefll
* octomap  
$ sudo apt-get install doxygen # a denpendency of octomap  
$ sudo apt install octovis # for octomap visualization  
clone from https://github.com/OctoMap/octomap, and then "make install" it.  

# ------------------------------------------------------------------------------
# 简历
Title: Object 3d Scanning and Locating by Robot Arm with RGB-D Camera
Calibrate camera. Convert point cloud (RGB-D image) to robot coordinate through forward kinematics.
Filter, segmentation, and (clustering, or, manually thresholding) to acquire the object from a single point cloud. 
Register the object from different point clouds into one and estimate its true position.


# ------------------------------------------------------------------------------
# ROS file structure

0.5-sim-pub_cloud (py)
0.7-receive_cloud_and_display-ByPCL (This is for testing whether receives the point cloud)
0.7-receive_cloud_and_display-ByOpen3D (This is for testing whether receives the point cloud)
1-main-control_robot_and_workflow (robot move; take picture; take picture;)
2-receive_save_cloud-seg_object-save_pub (cpp)
3-register_clouds-save_pub (PointXYZ PINGxyz)
4-display-by_pcl
4-display-by_rviz
