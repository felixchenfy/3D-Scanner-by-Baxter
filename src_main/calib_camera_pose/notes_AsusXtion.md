Xtion PRO LIVE
2013/11/05

# Camera specifications
https://www.asus.com/us/3D-Sensor/Xtion_PRO_LIVE/specifications/
58° H, 45° V, 70° D (Horizontal, Vertical, Diagonal)
Between 0.8m and 3.5m

# How to use
http://wiki.ros.org/openni2_launch

Run driver:
$ roslaunch openni2_launch openni2.launch

Config registration settings:  
$ rosrun rqt_reconfigure rqt_reconfigure  
Select camera/driver, set "depth_registration" to true, set "color_depth_synchronization" to true.  

Show:  
$ rviz  
Set fix frame to "camera_rgb_frame" (because I need to use rgb image to do PnP with the chessboard.)  
Set PointCloud2's topic to: "camera/depth_registered/points"


