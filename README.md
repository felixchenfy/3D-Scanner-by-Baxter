
# Content: Scan Object 3D model using Baxter Robot.
This is part of my winter project.

Directory:
-   [Workflow](#workflow)
-   [Done and Need to be Done](#done-and-need-to-be-done)
-   [Algorithms](#algorithms)
-   [File Structure and Detailed
    Workflow](#file-structure-and-detailed-workflow)
-   [Files for Debugging](#files-for-debugging)
-   [Reference](#reference)
-   [Dependencies and Installation](#dependencies-and-installation)

# Workflow

1. Place a depth camera on Baxter's lower forearm to acquire 3d point cloud.
2. Place a chessboard on the table. Caliberate its pose and depth_camera's pose in the Baxter frame.
3. Move Baxter's limb to several positions, and take depth pictures of the object from different view angles.
4. Filter the point cloud, and register them into a single 3D model.

# Done and Need to be Done

I think I've completed most of the programs.  
What I need to do:   
* Figure out someway to place depth camera onto Baxter's arm. May be using tape at first, and change to 3d printed connector later?
* Record several feasible positions of Baxter Limb for taking pictures. I've examined it, it should be feasible.
* Test my program on real robot. Fix all bugs. And done!
* Complete this README.
* Scan some objects' 3d models for the next section of my project.

# Algorithms

## 1. Calibrate poses of chessboard and depth_camera

**Main technique**:  
Detect chessboard in the image, and solve for the relative pose between chessboard and camera.  

**How to implement:**  
First, face Baxter's left hand camera towards the chessboard, and locate the chessboard.  
Then, face depth_camera towards the chessboard, and locate the depth_camera itself. Since depth_camera is placed on Baxter's forearm, using forward kinematics (/tf) we can know what exact position is the depth_camera placed on the arm.

## 2. Filtering point cloud
After acquiring the point cloud, I do following processes (using PCL's library): 
* Filter by voxel grid and outlier removal.
* Rotate cloud to the Baxter's frame.
* Range filtering. Retrain only points that are near the chessboard.
* Segment out large plane (optional).
* Do a clustering and choose the largest object (optional).

## 3. Registration
* Register pieces of clouds together using ICP and colored-ICP (directly using Open3D's library function).


# File Structure and Detailed Workflow
TO DO

# Files for Debugging
TO DO

# Reference

* A 3d scanner project from last cohort:  
 https://github.com/zjudmd1015/Mini-3D-Scanner.  
It's a really good example for me to get started. Algorithms I used for point cloud processing are almost the same as this.

* PCL and Open3D.  
My codes for the algorithms are basically copied piece by piece from their official tutorial examples.

* https://github.com/karaage0703/open3d_ros  
I referenced this page for point_cloud datatype  between open3d and ros. It's not so useful, and actually I rewrote the related functions.


# Dependencies and Installation
Below are the dependencies. Some I give the installing commands I used. Others please refer to their official website.  
In short: Eigen, PCL, Open3D, and other ROS packages.


### - [Eigen 3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
$ sudo apt-get install libeigen3-dev  
(Eigen only has header files. No ".so" or ".a".)

### - pcl 1.7
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl  
sudo apt-get update  
sudo apt-get install libpcl-all  

### - [pcl_ros](http://wiki.ros.org/pcl_ros)
$ sudo apt-get install ros-melodic-pcl-ros  
$ sudo apt-get install ros-melodic-pcl-conversions  
(It seems that the above two has already been installed on my ubuntu.)

### - Open3D
Two official installation tutorials: [this](http://www.open3d.org/docs/getting_started.html) and [this](http://www.open3d.org/docs/compilation.html).

### - Baxter's packages
Please follow the long tutorial on [Rethink Robotics' website](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter)


### - OpenCV 4.0 (c++)
I used it to code scripts for reading YAML and some cv::Mat operations. (However, I didn't actually use it in this project. So I might delete OpenCV-related codes later.)  
This is really nice tutorial for install OpenCV: [link](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/).

### - Some other libs that might be usefll
* octomap  
$ sudo apt-get install doxygen # a denpendency of octomap  
$ sudo apt install octovis # for octomap visualization  
clone from https://github.com/OctoMap/octomap, and then "make install" it.  
