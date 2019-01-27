
Object 3D Scan by Baxter Robot and Depth Camera
========================

This is a project of scanning object's 3D point cloud by using a depth camera mounted on Baxter Robot's limb.

[Video]  
[Scanned models]

I'm using PCL and Open3D for point cloud processing, and using ROS for controlling Baxter Robot to move its arm to different pose for taking depth images. 

This project can be useful if you are interested in: 3D Scanning, ROS (Python, c++), common filters in PCL, basic usage of Open3D, Sub/Pub and Display point cloud by PCL and Open3D, Calib Camera Pose by Chessboard.


<!-- TOC -->

- [Object 3D Scan by Baxter Robot and Depth Camera](#object-3d-scan-by-baxter-robot-and-depth-camera)
- [1. Procedures of 3D Scanning](#1-procedures-of-3d-scanning)
- [2. Workflow of the Program](#2-workflow-of-the-program)
  - [2.1. Overview](#21-overview)
  - [2.2. Node1: Move Baxter](#22-node1-move-baxter)
  - [2.3. Node2: Filter cloud](#23-node2-filter-cloud)
  - [2.4. Node3: Register clouds](#24-node3-register-clouds)
  - [2.5. Fake node1 for debug](#25-fake-node1-for-debug)
- [3. Algorithms](#3-algorithms)
  - [3.1. Calibrate poses of chessboard and depth_camera](#31-calibrate-poses-of-chessboard-and-depthcamera)
  - [3.2. Filter point cloud](#32-filter-point-cloud)
  - [3.3. Registration](#33-registration)
- [4. File Structure](#4-file-structure)
- [5. Dependencies and Installation](#5-dependencies-and-installation)
- [6. Problems to Solve](#6-problems-to-solve)
- [7. Reference](#7-reference)

<!-- /TOC -->

# 1. Procedures of 3D Scanning 

1. Place a depth camera on Baxter's lower forearm for collecting 3d point cloud.
2. Place a depth camera on Baxter's lower forearm for collecting 3d point cloud.
3. Before scanning, place a chessboard on the ground. Caliberate its pose and depth_camera's pose in the Baxter Robot's coordinate frame.
4. Place a flat board on the ground, and place the object on the board. I've drawn some colored patterns (English words, circles, cross, etc.) on the board to assist the registration of point cloud.
5. Move Baxter's limb to several positions, and take depth pictures of the object from different view angles.
6. Filter the point cloud, and register them into a single 3D model.
7. (TODO) Rotate the board and object for 180°, and do a second scan to get a more complete point cloud. It's due to the problem that Baxter's limb can only move around the object for about 200°, not 360°.

# 2. Workflow of the Program

## 2.1. Overview
I wrote **1 ROS node** for calibration and **3 ROS nodes** for 3D scanning.

Use ↓ to run [calib_camera_pose.py](src_main/calib_camera_pose/calib_camera_pose.py) for calibration:  
> $ roslaunch scan3d_by_baxter calib_camera_pose.launch  

Use ↓ to start the 3D scanner:  
> $ roslaunch scan3d_by_baxter main_3d_scanner.launch 

Details of calibration is described in the "Algorithm" section.

The workflow of the main 3 nodes are described below.

## 2.2. Node1: Move Baxter
file: [src_main/n1_move_baxter.py](src_main/n1_move_baxter.py)

The node controls Baxter to move to several positions in order to take pictures of object from different views.  After Baxter reaches the next goal pose, publish a message of depth_camera's pose to node 2.

Input: (a) [config/T_arm_to_depth.txt](config/T_arm_to_depth.txt); (b) Hard-coded Baxter poses.

Publish: Message to node 2.

## 2.3. Node2: Filter cloud

file: [src_main/n2_filt_and_seg_object.cpp](src_main/n2_filt_and_seg_object.cpp)

The node subscribes the (a) original point cloud, (b) rotated it to the Baxter's frame, (c) filter and segment the cloud to remove points that are noises or far away from the object. The result is then published to node 3.

Subscribe: (a) Message from node 1. (b) Point cloud from depth camera.

Publish: (b) Rotated cloud to rviz. (c) Segmented cloud to node 3.

Meanwhile, it also saves the (a) orignal cloud and (c) segmented cloud to the [data/](data/) folder. 

## 2.4. Node3: Register clouds
file: [src_main/n3_register_clouds_to_object.cpp](src_main/n3_register_clouds_to_object.cpp)

This node subsribes from node2 of the segmented cloud containing the object. Then it does a registration to obtain the complete 3D model of the object. Finally, the result is saved to file.

Subscribe: Segmented cloud from node2.

Publish: Registration result to rviz.

## 2.5. Fake node1 for debug
file: [src_main/n1_fake_data_publisher.cpp](src_main/n1_fake_data_publisher.cpp)

This is a replacement for "n1_move_baxter.py" and is used for debug. It reads the Baxter poses and point cloud from file, and publishes them to node 2. Thus, I can test previously saved data, and debug without connecting to any hardware.

# 3. Algorithms

## 3.1. Calibrate poses of chessboard and depth_camera

**Locate chessboard**:  

Detect chessboard corners in the image by Harris. Obtain and refine these corners' position (u,v). Since the corners are also in chessboard frame with a coord value of (x=i, y=j, z=0), we can solve this PnP problem and get the relative pose between chessboard and camera frame. (The above operations are achieved in Python using OpenCV library.)

Chessboard has a grid size of 7x9. Let 7 side be X-axis, and 9 be Y-axis. Now there are 4 possible chessboard frames. 

I do 2 things to obtain a unique chessboard frame: 
* Let Z-axis of chessboard pointing to the camera.  
* Manually draw a red circle in the middle of grid on one quadrant of the chessboard. Check which of the 2 quadrants has this red circle by thresholding HSV. Then, I can get a unique coordinate.

Also, I draw the obtained chessboard coordinate onto the 2D image and display it for easier debug.

**Transform Pose to Baxter Frame**:  
Poses of Baxter's frames can be read from '/tf' topic (forward kinematics).  
* For Baxter left hand camera, it already has a frame in the tf tree. 
* For the Depth camera, I placed it on the tf frame of '/left_lower_forearm'.  

With some geometry computation, I can know where the depth_camera and chessboard is in Baxter frame.

**Procedures of calibration:**  
Face the depth_camera and Baxter left hand camera towards the chessboard. The program reads the '/camera_info' topic, detect and locate the chessboard, and draw out the detected frame. If the detection is good, I'll press a key 'd' and obtain the calibration result, which will be saved to [/config](config) folder.

## 3.2. Filter point cloud

After acquiring the point cloud, I do following processes (by using PCL's library): 
* Filter by voxel grid and outlier removal.
* Rotate cloud to the Baxter's frame.
* Range filtering. Retrain only points that are near the chessboard.
* Segment out large plane (optional).
* Do a clustering and choose the largest object (optional).

## 3.3. Registration
* Register pieces of clouds together using ICP and colored-ICP (directly using Open3D's library function).


# 4. File Structure

[launch/](launch)  
Launch files, including scripts for starting **my 3D scanner** and **debug and test files**. All my ROS nodes have at least one corresponding launch file stored here.

[config/](config): configuration files: 
* ".rviz" settings
* calibration result of chessboard pose and depth_camera pose.  

(Configrations of algorithm params are set in launch file.)

[include/](include): c++ header files. Mainly wrapped up PCL filters.

[src_cpp/](src_cpp): c++ function definitions.

[src_python/](src_python): Python functions.  Test cases are also included in each script.

[src_main/](src_main)  
Main scripts for this 3D scan project.
* Scripts for 3 main ROS nodes.
* Calibration
* Other assistive nodes.

[test/](test): Testing cpp functions.

[test_ros/](test_ros): Testing ROS scripts, including:
* Read point cloud file and publish to ROS topic by both PCL/open3D.
* Subscribe ROS point cloud and display by both PCL/open3D.




# 5. Dependencies and Installation
Below are the dependencies. Some I give the installing commands I used. Others please refer to their official website.  
In short: Eigen, PCL, Open3D, OpenCV(Python) and other ROS packages.

-  Eigen 3  
http://eigen.tuxfamily.org/index.php?title=Main_Page  
$ sudo apt-get install libeigen3-dev   
(Eigen only has header files. No ".so" or ".a".)

-  pcl 1.7  
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl  
sudo apt-get update  
sudo apt-get install libpcl-all  

-  pcl_ros  
http://wiki.ros.org/pcl_ros  
$ sudo apt-get install ros-melodic-pcl-ros    
$ sudo apt-get install ros-melodic-pcl-conversions    
(It seems that the above two has already been installed on my ubuntu.)  

-  Open3D  
Two official installation tutorials: [this](http://www.open3d.org/docs/getting_started.html) and [this](http://www.open3d.org/docs/compilation.html).

-  Baxter's packages  
Please follow the long tutorial on [Rethink Robotics' website](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter)


-  OpenCV (C++)
OpenCV >= 3.0  
I used it for some cv::Mat operations.
This is really nice tutorial for install OpenCV 4.0: [link](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/).

-  Some other libs that might be usefll  
   * octomap  
   $ sudo apt-get install doxygen # a denpendency of octomap  
   $ sudo apt install octovis # for octomap visualization  
   clone from https://github.com/OctoMap/octomap, and then "make install" it.  

# 6. Problems to Solve
The current implementation has following problems:
1. Bottom of the object cannot be scanned.
2. Baxter can only move around the object for about 200 degrees.

To do:
1. Try solving the above problems.
2. Deal with the case when a person manually flip the object in order to scan the object parts that cannot be seen in current view. Maybe It requires tracking, and more advanced registration algorithm.
3. Replace Baxter trajectory from hard-coding to a more intellgent motion planner. For example, automatically facing the object and moving to directions that can see the unscaned parts of object.
4. Use a better depth camera with larger minimum scanning range and higher accuracy.

# 7. Reference

* A 3d scanner project from last cohort:  
 https://github.com/zjudmd1015/Mini-3D-Scanner.  
It's a really good example for me to get started. Algorithms I used for point cloud processing are almost the same as this.

* PCL and Open3D.  
My codes for the algorithms are basically copied piece by piece from their official tutorial examples.

* https://github.com/karaage0703/open3d_ros  
I referenced this page for point_cloud datatype  between open3d and ros. It's not so useful, and actually I rewrote the related functions.

