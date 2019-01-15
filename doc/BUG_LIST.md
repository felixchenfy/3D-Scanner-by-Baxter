

# ============================================================

# PCL bugs
* headers
for point cloud, include followings:
    #include <pcl/point_types.h>
    #include <pcl/common/common_headers.h>
Though in tutorial, the 2nd is not needed.
But in my test, I need the common_headers for pcl::PointCloud.

* "vtk" bug or ‘Impl’ has not been declared
This is a bug related to "cv::" and "pcl::".  
Some data types are the same in pcl and cv.

# ============================================================
# ROS bugs
* ROS_INFO 
Not ros::ROS_INFO
Besides, ROS_INFO(str) where str should be a char* instead of string !!!

# ============================================================
# Open3D
For its vis=open3d.Visualizer() and its add_geometry, I cannot understand its low level principle.
When I use vis.add_geometry(cloud), and then cloud=createARandomTestCloud(), 
    and then update_geometry, the vis does not change.
Not solved yet. (Except using add_geometry again)

# ============================================================
# Others

* error: undefined reference to
I got the namespace spelled wrong. Stupid bug.

* error: does not name a type
I got the namespace brasket before the function, thus my "using namespace pcl" is not working.
And thus the PointCloud does not name a type.

* template cannot be splitted into .h and .cpp
Instead, define all template definitions in .hpp. Meanwhile, in .h, include this .hpp.
(Will this slow the compile, compared to linking only?)


* About Cmake and linking
In my A.cpp, I linked it with B.so.
In my B.so, I linked it with C.so.
Then in CMakeLists, I only need to:
    target_link_libraries( A B)
and there is no need for A to target_link C.



