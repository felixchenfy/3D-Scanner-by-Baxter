# examples/Python/Tutorial/Advanced/colored_pointcloud_registration.py

import numpy as np
import copy
from open3d import *
import time

def draw_registration_result_original_color(src, target, transformation):
    src_temp = copy.deepcopy(src)
    src_temp.transform(transformation)
    draw_geometries([src_temp, target])
    
def combineTwoClouds(cloud1, cloud2):
    cloud1_points=np.asarray(cloud1.points)
    cloud2_points=np.asarray(cloud2.points)
    cloud1_colors=np.asarray(cloud1.colors)
    cloud2_colors=np.asarray(cloud2.colors)
    
    out_points=np.vstack((cloud1_points,cloud2_points))
    out_colors=np.vstack((cloud1_colors,cloud2_colors))
    
    result=PointCloud()
    result.points = Vector3dVector(out_points)
    result.colors = Vector3dVector(out_colors)
    
    radius = 0.01
    result = voxel_down_sample(result, radius)
    return result



if __name__ == "__main__":

    # -- Load data
    src = read_point_cloud("../data_debug/ColoredICP_0.pcd")
    target = read_point_cloud("../data_debug/ColoredICP_1.pcd")

    # draw initial alignment
    if 0:
        current_transformation = np.identity(4)
        draw_registration_result_original_color(
               src, target, current_transformation)
    
    # This function is mainly copied from here.
    # http://www.open3d.org/docs/tutorial/Advanced/colored_pointcloud_registration.html

    # -- Point to plane ICP
    current_transformation = np.identity(4);
    distance_threshold = 0.02
    result_icp = registration_icp(src, target, distance_threshold,
            current_transformation, TransformationEstimationPointToPlane())
    if 0:
        print("ICP result:")
        print(result_icp)
        draw_registration_result_original_color(src, target, result_icp.transformation)

    # -- Colored pointcloud registration
    voxel_radiuses = [ 0.04, 0.02, 0.01 ];
    max_iters = [ 50, 30, 14 ];
    current_transformation = np.identity(4)
    print("\nStart Colored point cloud registration")
    for ith_loop in range(3):
        # Set param in this loop
        max_iter = max_iters[ith_loop]
        radius = voxel_radiuses[ith_loop]
 
    
        # Downsample
        src_down = voxel_down_sample(src, radius)
        target_down = voxel_down_sample(target, radius)

        # Estimate normal
        estimate_normals(src_down, KDTreeSearchParamHybrid(
                radius = radius * 2, max_nn = 30))
        estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius = radius * 2, max_nn = 30))

        # Applying colored point cloud registration
        result_icp = registration_colored_icp(src_down, target_down,
                radius, current_transformation,
                ICPConvergenceCriteria(relative_fitness = 1e-6,
                relative_rmse = 1e-6, max_iteration = max_iter))
        current_transformation = result_icp.transformation
        
        if 1:
            print("\n{}th loop: radius={:.2f}, max_iter={}".format(
                    ith_loop, radius, max_iter))
            print(result_icp)
            
    # Transform src to dst's frame
    src_tmp = copy.deepcopy(src)
    src_tmp.transform(result_icp.transformation)
    
    # Combine the two
    result_cloud = combineTwoClouds(src_tmp, target)

    # Print and plot
    print(src)
    print(target)
    print(result_cloud)
    time.sleep(0.3)
    draw_geometries([result_cloud])
    
    
