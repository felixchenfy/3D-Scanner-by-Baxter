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

# Frames and their poses

$ rosrun tf tf_echo /camera_link /camera_rgb_frame
- Translation: [0.000, -0.045, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]

$ rosrun tf tf_echo /camera_link /camera_rgb_optical_frame
- Translation: [0.000, -0.045, 0.000]
- Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
            in RPY (radian) [-1.571, -0.000, -1.571]
            in RPY (degree) [-90.000, -0.000, -90.000]

$ rosrun tf tf_echo /camera_link /camera_depth_frame
At time 0.000
- Translation: [0.000, -0.020, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]

$ rosrun tf tf_echo /camera_link /camera_depth_optical_frame
At time 0.000
- Translation: [0.000, -0.020, 0.000]
- Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
            in RPY (radian) [-1.571, -0.000, -1.571]
            in RPY (degree) [-90.000, -0.000, -90.000]

$ rosrun tf tf_echo /camera_rgb_optical_frame /camera_depth_optical_frame
rosrun tf tf_echo /camera_rgb_optical_frame /camera_depth_optical_frame
At time 0.000
- Translation: [-0.025, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]


# Hardware

The connection of the wire is very loose on the camera side.
Push it tight, and use tape to fix it.


