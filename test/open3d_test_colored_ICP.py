# examples/Python/Tutorial/Advanced/colored_pointcloud_registration.py

import numpy as np
import copy
from open3d import *
import time
import sys, os
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# Include my lib
print(PYTHON_FILE_PATH)
sys.path.append(PYTHON_FILE_PATH + "../src_python")
from lib_cloud_registration import drawTwoClouds, registerClouds

if __name__ == "__main__":

    # -- Load data
    src = read_point_cloud("../data_debug/ColoredICP_0.pcd")
    target = read_point_cloud("../data_debug/ColoredICP_1.pcd")

    # -- Draw initial alignment
    if 0:
        current_transformation = np.identity(4)
        draw_registration_result_original_color(
            src, target, current_transformation)

    # -- Register clouds
    result_cloud, transformation = registerClouds(src, target)
    
    # -- Print and plot
    # print(src)
    # print(target)
    print(result_cloud)
    print(transformation)
    time.sleep(0.3)
    draw_geometries([result_cloud])
