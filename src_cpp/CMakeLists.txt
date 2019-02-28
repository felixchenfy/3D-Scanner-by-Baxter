

add_library(mylib_pcl SHARED
    my_pcl/pcl_commons.cpp
    my_pcl/pcl_io.cpp
    my_pcl/pcl_visualization.cpp
    my_pcl/pcl_filters.cpp
    my_pcl/pcl_advanced.cpp
)

add_library(mylib_basics SHARED
    my_basics/basics.cpp
    my_basics/eigen_funcs.cpp
)

target_link_libraries( mylib_basics
    ${THIRD_PARTY_LIBS} 
)


target_link_libraries( mylib_pcl
    ${THIRD_PARTY_LIBS} 
    mylib_basics
)



