
add_executable( read_pcl_and_display read_pcl_and_display.cpp )
target_link_libraries( read_pcl_and_display
    my_pcl
)

add_executable( simple_test simple_test.cpp )
target_link_libraries( simple_test
    my_basics
)
