#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import copy
from open3d import *
import time
import os
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

# -------------- Basic operations on cloud ----------------

def my_sleep(t):
    if 1:
        time.sleep(t)

def clearCloud(cloud):
    cloud.points=Vector3dVector(np.ndarray((0,0)))
    cloud.colors=Vector3dVector(np.ndarray((0,0)))

def copyOpen3dCloud(src, dst):
    dst.points = copy.deepcopy(src.points)
    dst.colors = copy.deepcopy(src.colors)

def getCloudContents(cloud):
    return np.asarray(cloud.points), np.asarray(cloud.colors)

def formNewCloud(np_points, np_colors):
    cloud = PointCloud()
    cloud.points = Vector3dVector(np_points)
    cloud.colors = Vector3dVector(np_colors)
    return cloud

def mergeClouds(cloud1, cloud2, radius_downsample=None, T=None):
    if T is not None:
        cloud1_transed = copy.deepcopy(cloud1)
        cloud1_transed.transform(T)
    else:
        cloud1_transed = copy.deepcopy(cloud1)
    cloud1_points,cloud1_colors = getCloudContents(cloud1_transed)
    cloud2_points,cloud2_colors = getCloudContents(cloud2)

    out_points = np.vstack((cloud1_points, cloud2_points))
    out_colors = np.vstack((cloud1_colors, cloud2_colors))

    result = formNewCloud(out_points, out_colors)
    if radius_downsample is not None:
        result = voxel_down_sample(result, radius_downsample)
    return result

def resizeCloudXYZ(cloud, scale=1.0):
    cloud_points, cloud_colors = getCloudContents(cloud)
    cloud_points = cloud_points * scale
    new_cloud = formNewCloud(cloud_points, cloud_colors)
    return new_cloud

def moveCloudToCenter(cloud):
    cloud_points, cloud_colors = getCloudContents(cloud)
    cloud_points = cloud_points-np.mean(cloud_points,axis=0)
    new_cloud = formNewCloud(cloud_points, cloud_colors)
    return new_cloud

# -------------- Display ----------------
    
def getCloudSize(cloud):
    return np.asarray(cloud.points).shape[0]

def drawTwoClouds(c1, c2, T_applied_to_c1=None):
    c1_tmp = copy.deepcopy(c1)
    if T_applied_to_c1 is not None:
        c1_tmp.transform(T_applied_to_c1)
    draw_geometries([c1_tmp, c2])

color_map = {'r':[1.0, 0.0, 0.0],'g':[0.0, 1.0, 0.0], 'b':[0.0, 0.0, 1.0]}
def getColor(color):
    color_map[color]

def createXYZAxis(coord_axis_length=1.0, num_points_in_axis=10):
    xyz_offset=[0,0,0]
    xyz_color=['r','g','b']
    NUM_AXIS=3
    data_xyz=np.zeros((num_points_in_axis*NUM_AXIS,NUM_AXIS))
    data_rgb=np.zeros((num_points_in_axis*NUM_AXIS,NUM_AXIS))
    cnt_row=0
    for axis in range(3):
        color = color_map[xyz_color[axis]]
        offset = xyz_offset[axis]
        for cnt_points in range(1, 1+num_points_in_axis):
            data_xyz[cnt_row, axis]=offset+coord_axis_length*cnt_points/num_points_in_axis
            data_rgb[cnt_row,:]=color
            cnt_row+=1
    cloud_XYZaxis = formNewCloud(data_xyz, data_rgb)
    return cloud_XYZaxis


def drawCloudWithCoord(cloud, coord_axis_length=0.1, num_points_in_axis=150):
    cloud_XYZaxis = createXYZAxis(coord_axis_length, num_points_in_axis)
    cloud_with_axis = mergeClouds(cloud, cloud_XYZaxis)
    draw_geometries([cloud_with_axis])

# -------------- Filters ----------------

def filtCloudByRange(cloud, xmin=None, xmax=None, 
                        ymin=None, ymax=None, zmin=None, zmax=None):
    none2maxnum =  lambda val: +99999.9 if val is None else val
    none2minnum =  lambda val: -99999.9 if val is None else val
    xmax=none2maxnum(xmax)
    ymax=none2maxnum(ymax)
    zmax=none2maxnum(zmax)
    xmin=none2minnum(xmin)
    ymin=none2minnum(ymin)
    zmin=none2minnum(zmin)
    criteria = lambda x,y,z: \
        x>=xmin and x<=xmax and y>=ymin and y<=ymax and z>=zmin and z<=zmax
    return filtCloud(cloud, criteria)

def filtCloud(cloud, criteria):
    points, colors = getCloudContents(cloud)
    num_pts=points.shape[0]
    valid_indices=np.zeros(num_pts, np.int)
    cnt_valid=0
    for i in range(num_pts):
        x,y,z=points[i][0],points[i][1],points[i][2]
        if criteria(x,y,z):
            valid_indices[cnt_valid]=i
            cnt_valid+=1
    return formNewCloud(
        points[valid_indices[:cnt_valid],:],
        colors[valid_indices[:cnt_valid],:]
    )
    
# -------------- MAIN! REGISTRATION (Mainly copied from Open3D website) ----------------
 
def computeFeaturesForGlobalRegistration(pcd, voxel_size):
    # http://www.open3d.org/docs/tutorial/Advanced/global_registration.html#global-registration
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = compute_fpfh_feature(pcd_down,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    return pcd_down, pcd_fpfh

def execute_global_registration(
    source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    # http://www.open3d.org/docs/tutorial/Advanced/global_registration.html#global-registration
    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            distance_threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            RANSACConvergenceCriteria(4000000, 500))
    return result.transformation

def execute_fast_global_registration(source_down, target_down,
        source_fpfh, target_fpfh, voxel_size):
    # http://www.open3d.org/docs/tutorial/Advanced/fast_global_registration.html
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = registration_fast_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            FastGlobalRegistrationOption(
            maximum_correspondence_distance = distance_threshold))
    return result.transformation
    
def registerClouds_Global(src, dst, voxel_size=0.01, FAST_REGI=True):
    # http://www.open3d.org/docs/tutorial/Advanced/fast_global_registration.html
    src_down, src_fpfh = computeFeaturesForGlobalRegistration(src, voxel_size)
    dst_down, dst_fpfh = computeFeaturesForGlobalRegistration(dst, voxel_size)
    if FAST_REGI:
        T = execute_fast_global_registration(src_down, dst_down, src_fpfh, dst_fpfh, voxel_size)
    else:
        T = execute_global_registration(src_down, dst_down, src_fpfh, dst_fpfh, voxel_size)
    return T, src_down, dst_down

def registerClouds_Local(src, target, voxel_size=0.01, current_T=None, 
    ICP = True, COLORED_ICP = False, ICP_OPTION="Point2Point",
    DRAW_INIT_POSE = False, DRAW_ICP = False, DRAW_COLORED_ICP = False):
    
    # -- Use colored ICP to register src onto dst, and return the combined cloud
    # This function is mainly copied from here.
    # http://www.open3d.org/docs/tutorial/Advanced/colored_pointcloud_registration.html
    
    # -- Params
    ICP_distance_threshold = voxel_size*4
    voxel_radiuses = [voxel_size*2.0, voxel_size, voxel_size/2.0]
    max_iters = [50, 25, 10]
    if current_T is None:
        current_T = np.identity(4)

    if DRAW_INIT_POSE:
        tmp = mergeClouds(src, target, voxel_size)
        drawCloudWithCoord(tmp, coord_axis_length=0.1, num_points_in_axis=50)

    # -- Point to plane ICP
    if ICP:
        print("Running ICP ...")
        src_down = voxel_down_sample(src, voxel_size)
        target_down = voxel_down_sample(target, voxel_size)
        estimate_normals(src_down, KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30))
        estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30))
        
        if ICP_OPTION == "PointToPlane": # or "PointToPoint"
            result_trans = registration_icp(src_down, target_down, ICP_distance_threshold,
                current_T, 
                TransformationEstimationPointToPlane())
        else:
            result_trans = registration_icp(src_down, target_down, ICP_distance_threshold, 
                current_T,
                TransformationEstimationPointToPoint())
            
        current_T = result_trans.transformation
        if DRAW_ICP:
            print("-- Draw ICP result:")
            print(result_trans)
            my_sleep(1)
            drawTwoClouds(
                src, target, result_trans.transformation)

    # -- Colored pointcloud registration
    if COLORED_ICP:
        print "Running colored-ICP ...",
        for ith_loop in range(len(voxel_radiuses)):
            # Set param in this loop
            max_iter = max_iters[ith_loop]
            radius = voxel_radiuses[ith_loop]
            print " radius {:.4f}...".format(radius),
    
            # Downsample
            src_down = voxel_down_sample(src, radius)
            target_down = voxel_down_sample(target, radius)
    
            # Estimate normal
            estimate_normals(src_down, KDTreeSearchParamHybrid(
                radius=radius * 2, max_nn=30))
            estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius=radius * 2, max_nn=30))
    
            # Applying colored point cloud registration
            result_trans = registration_colored_icp(src_down, target_down,
                  radius, current_T,
                  ICPConvergenceCriteria(relative_fitness=1e-5,
                 relative_rmse=1e-5, max_iteration=max_iter))
            
            current_T = result_trans.transformation
        print "Complete!"
        if DRAW_COLORED_ICP:
            print("-- Draw Colored ICP result:")
            print(result_trans)
            my_sleep(1)
            drawTwoClouds(
                src, target, result_trans.transformation)
    print "Local registration completes.\n",    
    return current_T



class CloudRegister(object):
    def __init__(self, voxel_size_regi=0.005, global_regi_ratio=2.0, voxel_size_output=0.005, 
        USE_GLOBAL_REGI=True, USE_ICP=True, USE_COLORED_ICP=False):

        # copy params
        self.voxel_size_regi=voxel_size_regi # for registration
        self.global_regi_ratio=global_regi_ratio 
        self.voxel_size_output=voxel_size_output # for downsampling the res_cloud and output
        self.USE_GLOBAL_REGI = USE_GLOBAL_REGI
        self.USE_ICP = USE_ICP
        self.USE_COLORED_ICP = USE_COLORED_ICP
        
        # init vars
        self.res_cloud = open3d.PointCloud()
        self.new_cloud = open3d.PointCloud()
        self.prev_res_cloud = open3d.PointCloud()
        self.cnt_cloud=0

    def addCloud(self, new_cloud):
        self.new_cloud = copy.deepcopy(new_cloud)
        self.prev_res_cloud = copy.deepcopy(self.res_cloud)
        self.cnt_cloud+=1
        if self.cnt_cloud==1:
            self.res_cloud=copy.deepcopy(self.new_cloud)
        else:
            # compute transformation matrix to rotate new_cloud to the res_cloud frame
            T=np.identity(4)

            # Global regi
            if self.USE_GLOBAL_REGI:
                T, src_down, dst_down = registerClouds_Global(
                    self.new_cloud, self.res_cloud, self.voxel_size_regi * self.global_regi_ratio,
                    FAST_REGI=True)
            
            # Local regi
            if self.USE_ICP or self.USE_COLORED_ICP:
                T = registerClouds_Local(self.new_cloud, self.res_cloud, self.voxel_size_regi, 
                    T, ICP=self.USE_ICP, COLORED_ICP=self.USE_COLORED_ICP)

            # Merge
            self.res_cloud = mergeClouds(
                self.new_cloud, self.res_cloud, self.voxel_size_output, T)

        # self.res_cloud, 
        return self.res_cloud

    def drawRegisterInput(self):
        tmp = mergeClouds(self.new_cloud, self.prev_res_cloud, self.voxel_size_output)
        drawCloudWithCoord(tmp)
    
    def drawRegisterOutput(self):
        drawCloudWithCoord(self.res_cloud)

    def getResult(self):
        return self.res_cloud

# ====================================================================
# ============================ TESTS =================================
# ====================================================================

    
def test_registration():
  
    # -- Settings
    filename_=PYTHON_FILE_PATH+"../data/testing_log/03-07/volt_colorICP2/segmented_"
    cloud_register = CloudRegister(
        voxel_size_regi=0.01, global_regi_ratio=2.0, 
        voxel_size_output=0.002,
        USE_GLOBAL_REGI=False, USE_ICP=True, USE_COLORED_ICP=False)

    # -- Loop
    FILE_INDEX_BEGIN=1
    FILE_INDEX_END=9
    cnt = 0
    for file_index in range(FILE_INDEX_BEGIN, FILE_INDEX_END+1):
        print "==================== {}th file ======================".format(file_index)
        cnt+=1
        
        # Read point cloud
        filename = filename_+"{:02d}".format(file_index)+".pcd"
        new_cloud = read_point_cloud(filename)
        
        # Process cloud
        cloud_register.addCloud(new_cloud) 
        
        # Print and plot
        print(cloud_register.res_cloud)
        # my_sleep(0.3)
        # cloud_register.drawRegisterInput()
        # cloud_register.drawRegisterOutput()

    cloud_register.drawRegisterOutput()


if __name__ == "__main__":
    test_registration()

    if 0:
        filename="/home/feiyu/baxterws/src/winter_prj/scan3d_by_baxter/data/data/segmented_06.pcd"
        cloud_disp = read_point_cloud(filename)
        drawCloudWithCoord(cloud_disp)
    if 0:
        res_cloud = open3d.PointCloud()
        for i in range(1, 1+10):
            print i
            
            cloud_disp = read_point_cloud(PYTHON_FILE_PATH+"../data/data/driller_1/segmented_"+"{:02d}".format(i)+".pcd")
            # cloud_disp = read_point_cloud(PYTHON_FILE_PATH+"../data/data/driller_color_board/segmented_"+"{:02d}".format(i)+".pcd")
            # cloud_disp = read_point_cloud(PYTHON_FILE_PATH+"../data/data/driller_floor/segmented_"+"{:02d}".format(i)+".pcd")
             # drawCloudWithCoord(cloud_disp)
            res_cloud = mergeClouds(res_cloud, cloud_disp)
        drawCloudWithCoord(res_cloud)
