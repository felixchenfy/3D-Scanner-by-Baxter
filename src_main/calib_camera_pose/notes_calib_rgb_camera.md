

# Open camera

Run the laptop's usb cam:
{ Method1: 
    $ rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv _camera_name:=tracker_camera
}
{ Method2: Use uvc_camera.
    $ rosrun uvc_camera uvc_camera_node _device:=/dev/video0 # remember to remap
}

See camera info:
$ rostopic echo -n 1 /usb_cam/camera_info

Before calibration, all params are zero.

# Calibrate
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

$ rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.0158 image:=/usb_cam/image_raw camera:=/usb_cam

Data (calibration data and images used for calibration) will be written to /tmp/calibrationdata.tar.gz.

# Load calibration result

Copy calibration result to some folder.

Run the camera driver with a url to the calib result:
$ rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.0158 image:=/usb_cam/image_raw camera:=/usb_cam camera_info_url:=/home/feiyu/tmp/my_laptop_cam_calib_result

Run image proc:
$ ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
