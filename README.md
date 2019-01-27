
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
- [3. Done and Need to be Done](#3-done-and-need-to-be-done)
- [4. Algorithms](#4-algorithms)
  - [4.1. Calibrate poses of chessboard and depth_camera](#41-calibrate-poses-of-chessboard-and-depthcamera)
  - [4.2. Filter point cloud](#42-filter-point-cloud)
  - [4.3. Registration](#43-registration)
- [5. File Structure](#5-file-structure)
- [6. Reference](#6-reference)
- [7. Dependencies and Installation](#7-dependencies-and-installation)

<!-- /TOC -->

# 1. Procedures of 3D Scanning 

1. Place a depth camera on Baxter's lower forearm for collecting 3d point cloud.
2. Before scanning, place a chessboard on the ground. Caliberate its pose and depth_camera's pose in the Baxter Robot's coordinate frame.
3. Place a flat board on the ground, and place the object on the board. I've drawn some colored patterns (English words, circles, cross, etc.) on the board to assist the registration of point cloud.
4. Move Baxter's limb to several positions, and take depth pictures of the object from different view angles.
5. Filter the point cloud, and register them into a single 3D model.
6. (TODO) Rotate the board and object for 180°, and do a second scan to get a more complete point cloud. It's due to the problem that Baxter's limb can only move around the object for about 200°, not 360°.

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

## 2.3. Node2: Filter cloud

## 2.4. Node3: Register clouds

## 2.5. Fake node1 for debug


# 3. Done and Need to be Done

I think I've completed most of the programs.  
What I need to do:   
* Figure out someway to place depth camera onto Baxter's arm. May be using tape at first, and change to 3d printed connector later?
* Record several feasible positions of Baxter Limb for taking pictures. I've examined it, it should be feasible.
* Test my program on real robot. Fix all bugs. And done!
* Complete this README.
* Scan some objects' 3d models for the next section of my project.

# 4. Algorithms

## 4.1. Calibrate poses of chessboard and depth_camera

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

## 4.2. Filter point cloud

After acquiring the point cloud, I do following processes (by using PCL's library): 
* Filter by voxel grid and outlier removal.
* Rotate cloud to the Baxter's frame.
* Range filtering. Retrain only points that are near the chessboard.
* Segment out large plane (optional).
* Do a clustering and choose the largest object (optional).

## 4.3. Registration
* Register pieces of clouds together using ICP and colored-ICP (directly using Open3D's library function).


# 5. File Structure

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
* Read point cloud file and publish to ROS topic by PCL/open3D.
* Subscribe ROS point cloud and display by PCL/open3D.



# 6. Reference

* A 3d scanner project from last cohort:  
 https://github.com/zjudmd1015/Mini-3D-Scanner.  
It's a really good example for me to get started. Algorithms I used for point cloud processing are almost the same as this.

* PCL and Open3D.  
My codes for the algorithms are basically copied piece by piece from their official tutorial examples.

* https://github.com/karaage0703/open3d_ros  
I referenced this page for point_cloud datatype  between open3d and ros. It's not so useful, and actually I rewrote the related functions.


# 7. Dependencies and Installation
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
