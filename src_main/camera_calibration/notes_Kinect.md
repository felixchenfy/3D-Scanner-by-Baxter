
(I just found there is a Asus camera in the lab, and it is very easy to use! So this notes is not useful anymore.)

## Tools for using the Kinect One (Kinect v2) in ROS
https://github.com/code-iai/iai_kinect2

====== USE ======  
` How to get point clouds  
$ roslaunch kinect2_bridge kinect2_bridge.launch  

` How to view  
kinect_viewer or rostopic hz   


====== DEBUG ======  

`kinect2_bridge is not working / crashing, what is wrong?  

bug1: kinect2_bridge  
bug2: libfreenect2  

` Check libfreenect2  
use this tool: libfreenect2/build/bin/Protonect  

Before running kinect2_bridge please make sure Protonect is working and showing color, depth and ir images  


