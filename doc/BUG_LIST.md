
# ============================================================
# Programming bugs

* Error of calling dynamics libraries in ROS
ROS seems to
  1. put the .so files into another folder
  2. Or, linking the .so into another folder
which makes the linking error.

Even if I commented out the ROS related lines in CMakeLists and re-cmake, it still gives error.
So another reason that further causes this problem might be:
    ccache, which stores the old .so info, and not updating it.

* Smart pointer passes to function

Once it happens to be: Passing a smart pointer to a function, change the content it points to, after the function is finished, the content is still not changed!

However, the real problem is (suppose I passed ptrA into the function):
    I used ptrA = ptrB in side the function, which actually binding the ptrA to ptrB's content.
    However, since I was not using reference, ptrA is a copy of the ptrA outside the function.
    So ptrA outside the function was not changed!!!
Thus, in such cases, please use reference.

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

* private namespace in cpp

what():  Using ~ names with NodeHandle methods is not allowed.  If you want to use private names with the NodeHandle interface, construct a NodeHandle using a private name as its namespace.  e.g. ros::NodeHandle nh("~");  nh.getParam("my_private_name"); (name = [~plane_distance_threshold])

in short: use this:ros::NodeHandle nh("~");

* ROS_INFO 
Not ros::ROS_INFO
Besides, ROS_INFO(str) where str should be a char* instead of string !!!

# ============================================================
# Open3D

* For its vis=open3d.Visualizer() and its add_geometry, I cannot understand its low level principle.
When I use vis.add_geometry(cloud), and then cloud=createARandomTestCloud(), 
    and then update_geometry, the vis does not change.
Result: this bug is Partially solved, by manullay copying fields of a pointcloud to another.

# ============================================================
# Others

* error: undefined reference to (Stupid bug)
I got my namespace spelled wrong.

* error: does not name a type (Stupid bug)
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



