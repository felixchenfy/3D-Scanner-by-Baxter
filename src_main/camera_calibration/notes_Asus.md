

http://wiki.ros.org/openni2_launch

Run driver:
$ roslaunch openni2_launch openni2.launch

Config registration settings:  
$ rosrun rqt_reconfigure rqt_reconfigure  
select camera/driver, set "depth_registration" to true, set "color_depth_synchronization" to true.  

Show:  
$ rviz  
Set fix frame to "camera_rgb_frame" (because I need to use rgb image to do PnP with the chessboard.)  
Set PointCloud2's topic to: "camera/depth_registered/points"


