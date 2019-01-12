


add_library(my_pcl SHARED
    pcl_funcs.cpp
)

add_library(my_basics SHARED
    basics.cpp
)


target_link_libraries( my_pcl
    ${THIRD_PARTY_LIBS} 
)

target_link_libraries( my_basics
    ${THIRD_PARTY_LIBS} 
)


