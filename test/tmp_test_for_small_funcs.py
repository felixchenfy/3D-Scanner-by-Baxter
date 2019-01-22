
import numpy as np


# -- Standard
import open3d
import numpy as np
import sys, os
import cv2
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

config_folder = PYTHON_FILE_PATH+"../config/"
T_arm_to_depth = np.loadtxt(config_folder+"T_arm_to_depth.txt")

print T_arm_to_depth