
add_executable( n2-filt_and_seg_object n2-filt_and_seg_object.cpp )
target_link_libraries( n2-filt_and_seg_object
    my_pcl my_basics
    ${catkin_LIBRARIES} 
)

