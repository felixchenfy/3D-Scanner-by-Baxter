import numpy as np
import copy
from open3d import *


def drawTwoClouds(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    draw_geometries([source_temp, target])

def copyOpen3dCloud(src, dst):
    dst.points = src.points
    dst.colors = src.colors


def combineTwoClouds(cloud1, cloud2, radius_downsample=0.002):
    cloud1_points = np.asarray(cloud1.points)
    cloud2_points = np.asarray(cloud2.points)
    cloud1_colors = np.asarray(cloud1.colors)
    cloud2_colors = np.asarray(cloud2.colors)

    out_points = np.vstack((cloud1_points, cloud2_points))
    out_colors = np.vstack((cloud1_colors, cloud2_colors))

    result = PointCloud()
    result.points = Vector3dVector(out_points)
    result.colors = Vector3dVector(out_colors)

    result = voxel_down_sample(result, radius_downsample)
    return result


def registerClouds(src, target, radius_base=0.002):
    # -- Use colored ICP to register src onto dst, and return the combined cloud
    # This function is mainly copied from here.
    # http://www.open3d.org/docs/tutorial/Advanced/colored_pointcloud_registration.html

    # -- Params
    ICP_distance_threshold = radius_base*4
    voxel_radiuses = [radius_base*8, radius_base*2, radius_base]
    max_iters = [50, 30, 14]

    # -- Point to plane ICP
    current_transformation = np.identity(4)
    radius = radius_base
    src_down = voxel_down_sample(src, radius)
    target_down = voxel_down_sample(target, radius)
    estimate_normals(src, KDTreeSearchParamHybrid(
            radius=radius * 2, max_nn=30))
    estimate_normals(target, KDTreeSearchParamHybrid(
            radius=radius * 2, max_nn=30))
    result_icp = registration_icp(src, target, ICP_distance_threshold,
        current_transformation, TransformationEstimationPointToPlane())
        
    if 0:
        print("ICP result:")
        print(result_icp)
        drawTwoClouds(
            src, target, result_icp.transformation)

    # -- Colored pointcloud registration

    current_transformation = np.identity(4)
    print("\nStart Colored point cloud registration")
    for ith_loop in range(len(voxel_radiuses)):
        # Set param in this loop
        max_iter = max_iters[ith_loop]
        radius = voxel_radiuses[ith_loop]

        # Downsample
        src_down = voxel_down_sample(src, radius)
        target_down = voxel_down_sample(target, radius)

        # Estimate normal
        estimate_normals(src_down, KDTreeSearchParamHybrid(
            radius=radius * 2, max_nn=30))
        estimate_normals(target_down, KDTreeSearchParamHybrid(
            radius=radius * 2, max_nn=30))

        # Applying colored point cloud registration
        result_icp = registration_colored_icp(src_down, target_down,
                                              radius, current_transformation,
                                              ICPConvergenceCriteria(relative_fitness=1e-6,
                                                                     relative_rmse=1e-6, max_iteration=max_iter))
        current_transformation = result_icp.transformation

        if 1:
            print("  {}th loop: radius={:.4f}, max_iter={}".format(
                ith_loop, radius, max_iter))
            # print(current_transformation)
    print("  Colored ICP completes.\n\n")

    # Transform src to target's frame
    src_tmp = copy.deepcopy(src)
    src_tmp.transform(result_icp.transformation)

    # Combine the two
    print "src:",src_tmp
    print "target:",target
    result_cloud = combineTwoClouds(src_tmp, target, radius_base)
    print "result_cloud: ", result_cloud
    return result_cloud, result_icp.transformation



if __name__ == "__main__":
    import numpy as np
    import copy
    from open3d import *
    import time
    import sys, os

    # -- Settings
    filename_="../data_debug/from_real/segmented_0"
    num_files=6
    target = open3d.PointCloud()
    
    for ith_file in range(1, num_files+1):
        print "==================== {}th file ======================".format(ith_file)
        filename = filename_+str(ith_file)+".pcd"
        src = read_point_cloud(filename)
        if ith_file==1:
            copyOpen3dCloud(src, target)
            continue
        
        # -- Draw initial alignment
        if 0:
            current_transformation = np.identity(4)
            draw_registration_result_original_color(
                src, target, current_transformation)
    
        # -- Register clouds
        if 1: # Register by color ICP
            target, transformation = registerClouds(src, target)
        else: # By simply combine all together
            target = combineTwoClouds(src, target)
        
        # -- Print and plot
        print(target)
        time.sleep(0.3)
        draw_geometries([target])
