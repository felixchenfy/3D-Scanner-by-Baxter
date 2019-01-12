


add_library(my_pcl SHARED
    my_pcl/pcl_common.cpp
    my_pcl/pcl_io.cpp
    my_pcl/pcl_visualization.cpp
)

add_library(my_basics SHARED
    my_basics/basics.cpp
    my_basics/eigen_funcs.cpp
)

target_link_libraries( my_basics
    ${THIRD_PARTY_LIBS} 
)


target_link_libraries( my_pcl
    ${THIRD_PARTY_LIBS} 
    my_basics
)



