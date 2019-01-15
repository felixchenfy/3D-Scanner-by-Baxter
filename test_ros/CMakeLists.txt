

# add_executable(read_cloud_and_pub_by_pcl read_cloud_and_pub_by_pcl.cpp)
# target_link_libraries(read_cloud_and_pub_by_pcl ${catkin_LIBRARIES} my_pcl)

add_executable(sub_cloud_and_display_by_pcl sub_cloud_and_display_by_pcl.cpp)
target_link_libraries(sub_cloud_and_display_by_pcl ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS} )
