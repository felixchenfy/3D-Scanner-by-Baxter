
add_executable( n2_filt_and_seg_object n2_filt_and_seg_object.cpp )
target_link_libraries( n2_filt_and_seg_object
    mylib_pcl mylib_basics
    ${catkin_LIBRARIES} 
)

