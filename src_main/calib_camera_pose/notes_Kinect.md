
(I just found there is a Asus camera in the lab, and it is very easy to use! So this notes is not useful anymore.)  

# Camera specifications   
RGB：1920 x 1080 @ 30 / 15 FPS  
Depth：512 x 424 @ 30 FPS、16bit （mm）  
IR cam：512 x 484，30 Hz   
FOV: 70° x 60°   
Depth range: 0.5–4.5 meters   
RGB cam: 1080p 30 Hz (week light: 15 Hz)  

# Tools for using the Kinect One (Kinect v2) in ROS  
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


