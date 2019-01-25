import numpy as np
import copy
from open3d import *
import time
import os
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

# -------------- Basic operations on cloud ----------------

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

def combineTwoClouds(cloud1, cloud2, radius_downsample=0.002):
    cloud1_points,cloud1_colors = getCloudContents(cloud1)
    cloud2_points,cloud2_colors = getCloudContents(cloud2)

    out_points = np.vstack((cloud1_points, cloud2_points))
    out_colors = np.vstack((cloud1_colors, cloud2_colors))

    result = formNewCloud(out_points, out_colors)
    result = voxel_down_sample(result, radius_downsample)
    return result


# -------------- Display ----------------
    
def getCloudSize(cloud):
    return np.asarray(cloud.points).shape[0]


def drawTwoClouds(c1, c2, T_applied_to_c1=None):
    c1_tmp = copy.deepcopy(c1)
    if T_applied_to_c1 is not None:
        c1_tmp.transform(T_applied_to_c1)
    draw_geometries([c1_tmp, c2])

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
    
# -------------- MAIN! REGISTRATION ----------------
 
def registerClouds(src, target, radius_regi=0.005, radius_merge=0.002):
    # -- Use colored ICP to register src onto dst, and return the combined cloud
    # This function is mainly copied from here.
    # http://www.open3d.org/docs/tutorial/Advanced/colored_pointcloud_registration.html
    ICP = True
    ICP_DRAW = False
    COLORED_ICP = True
    COLORED_ICP_DRAW = False    

    # -- Params
    ICP_distance_threshold = radius_regi*8
    voxel_radiuses = [radius_regi*8, radius_regi*2, radius_regi]
    max_iters = [50, 25, 10]

    # -- Vars
    current_transformation = np.identity(4)

    # -- Point to plane ICP
    if ICP:
        radius = radius_regi
        src_down = voxel_down_sample(src, radius)
        target_down = voxel_down_sample(target, radius)
        estimate_normals(src, KDTreeSearchParamHybrid(
                radius=radius * 2, max_nn=30))
        estimate_normals(target, KDTreeSearchParamHybrid(
                radius=radius * 2, max_nn=30))
        
        if 0: # point to plane
            result_trans = registration_icp(src, target, ICP_distance_threshold,
                current_transformation, 
                TransformationEstimationPointToPlane())
        else:
            
            result_trans = registration_icp(src, target, ICP_distance_threshold, 
                current_transformation,
                TransformationEstimationPointToPoint())
            
        current_transformation = result_trans.transformation
        if ICP_DRAW:
            print("-- Draw ICP result:")
            print(result_trans)
            sleep(1)
            drawTwoClouds(
                src, target, result_trans.transformation)

    # -- Colored pointcloud registration
    if COLORED_ICP:
        print("\n\n-- Start Colored point cloud registration")
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
            result_trans = registration_colored_icp(src_down, target_down,
                  radius, current_transformation,
                  ICPConvergenceCriteria(relative_fitness=1e-6,
                 relative_rmse=1e-6, max_iteration=max_iter))
            
            current_transformation = result_trans.transformation
    
            if 1:
                print("  {}th loop: radius={:.4f}, max_iter={}".format(
                    ith_loop, radius, max_iter))
                # print(current_transformation)
    
        if COLORED_ICP_DRAW:
            print("-- Draw Colored ICP result:")
            print(result_trans)
            sleep(1)
            drawTwoClouds(
                src, target, result_trans.transformation)
    

    # Transform src to target's frame
    src_tmp = copy.deepcopy(src)
    src_tmp.transform(result_trans.transformation)

    # Combine the two
    sleep(0.01)
    print("-- Colored ICP completes.\n\n")
    print "src:",src_tmp
    print "target:",target
    result_cloud = combineTwoClouds(src_tmp, target, radius_merge)
    print "result_cloud: ", result_cloud
    return result_cloud, result_trans.transformation



# ====================================================================
# ============================ TESTS =================================
# ====================================================================
    
def test_basics():
    c1 = read_point_cloud(PYTHON_FILE_PATH+"../data_debug/ColoredICP_1.pcd")
    c2 = read_point_cloud(PYTHON_FILE_PATH+"../data_debug/ColoredICP_2.pcd")
    t0 = time.time()
    
    # Display ori
#    drawTwoClouds(c1, c2) # draw[T*c1, c2]
    
    # Test
    clearCloud(c1)
    c3=combineTwoClouds(c1,c2,radius_downsample=0.05)    
    # draw_geometries([c3])
    
    c4 = filtCloudByRange(c3, xmin=0.9, xmax=1.8)
    draw_geometries([c4])

    # return
    t1= time.time()
    print "Time cost =:", t1-t0
def test_registration():
  
    # -- Settings
    filename_="../data_debug/from_real/segmented_0"
    target = open3d.PointCloud()

    # -- Loop
    FILE_INDEX_BEGIN=4
    FILE_INDEX_END=5
    cnt = 0
    for file_index in range(FILE_INDEX_BEGIN, FILE_INDEX_END+1):
        print "==================== {}th file ======================".format(file_index)
        cnt+=1
        
        # Read point cloud
        filename = filename_+str(file_index)+".pcd"
        src = read_point_cloud(filename)
        
        # Init cloud
        if cnt==1:
            copyOpen3dCloud(src, target)
            continue
        
        # Draw initial alignment
        if 0:
            current_transformation = np.identity(4)
            draw_registration_result_original_color(
                src, target, current_transformation)
    
        # Register clouds
        if 1: # Register by color ICP
            target, transformation = registerClouds(src, target)
            print "transformation:\n", transformation
        else: # By simply combine all together
            target = combineTwoClouds(src, target)
        
        # Print and plot
        print(target)
        time.sleep(0.3)
        # draw_geometries([target])


if __name__ == "__main__":
    test_basics()
    # test_registration()