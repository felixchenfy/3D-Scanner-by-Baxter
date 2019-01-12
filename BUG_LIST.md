

* error: undefined reference to
I got the namespace wrong. Stupid bug.

* template cannot be splitted into .h and .cpp
Instead, define all template definitions in .hpp. Meanwhile, in .h, include this .hpp.
(Will this slow the compile, compared to linking only?)

* "vtk" bug or ‘Impl’ has not been declared
This is a bug related to "cv::" and "pcl::".  
Some data types are the same in pcl and cv.

* About Cmake and linking
In my A.cpp, I linked it with B.so.
In my B.so, I linked it with C.so.
Then in CMakeLists, I only need to:
    target_link_libraries( A B)
and there is no need for A to target_link C.


