
add_executable( n2_filt_and_seg_object n2_filt_and_seg_object.cpp )
target_link_libraries( n2_filt_and_seg_object
    my_pcl my_basics
    ${catkin_LIBRARIES} 
)

