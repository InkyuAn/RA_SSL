cmake_minimum_required(VERSION 2.8.3)
project(reflection_aware_ssl)

set(PACKAGE_DEPENDENCIES
    roscpp
    std_msgs
    pcl_ros
    pcl_conversions
    std_srvs
    octomap_ros
    octomap_msgs    
    visualization_msgs
    sensor_msgs    
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(PCL REQUIRED QUIET COMPONENTS
    common
    sample_consensus
    io
    segmentation
    filters
)

find_package(octomap REQUIRED)

add_definitions(-DOCTOMAP_NODEBUGOUT)

add_definitions(-std=c++11)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    DEPENDS octomap PCL
)


set(LINK_LIBS
    ${OCTOMAP_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}
    src/rayBasedSSL.cpp
    src/acousticRaytracing.cpp    
    src/soundPotentialField.cpp    
    src/particleFilter.cpp   
    src/SVD/singular_value_decomposition.cpp
)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(reflection_aware_ssl_node src/rayBasedSSL_node.cpp)
target_link_libraries(reflection_aware_ssl_node ${PROJECT_NAME} ${LINK_LIBS})

