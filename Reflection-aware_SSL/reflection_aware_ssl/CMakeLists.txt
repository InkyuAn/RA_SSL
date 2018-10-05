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
    #dynamic_reconfigure
    #nodelet
    visualization_msgs
    sensor_msgs
    #nav_msgs
    #geometry_msgs
    #kobuki_msgs
)

#include(cmake/pods.cmake)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(PCL REQUIRED QUIET COMPONENTS
    common
    sample_consensus
    io
    segmentation
    filters
)

find_package(octomap REQUIRED)

#find_package(PkgConfig REQUIRED)

add_definitions(-DOCTOMAP_NODEBUGOUT)

add_definitions(-std=c++11)
#add_definitions(-std=c++14)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

#generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)

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
    #src/gridKeyMaps.cpp
    src/soundPotentialField.cpp
    #src/rayBasedSSL_benchmark.cpp
    #src/rayBasedSSL_benchmark_test.cpp
    src/particleFilter.cpp
    #src/PathPlanner/Astar.cpp
    #src/PathPlanner/collision_checker.cpp
    #src/PathPlanner/navigation_AS_Project.cpp
    #src/PathPlanner/RRTS/rrts.cpp
    #src/PathPlanner/RRTS/kdtree.c
    #src/PathPlanner/RRTS/system_single_integrator.cpp
    #src/HoleFiling/holeFilling.cpp
    src/SVD/singular_value_decomposition.cpp
)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(reflection_aware_ssl_node src/rayBasedSSL_node.cpp)
target_link_libraries(reflection_aware_ssl_node ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(reflection_aware_ssl_benchmark_node src/rayBasedSSL_benchmark_node.cpp)
#target_link_libraries(reflection_aware_ssl_benchmark_node ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(raybased_ssl_benchmark_test_node src/rayBasedSSL_benchmark_test_node.cpp)
#target_link_libraries(raybased_ssl_benchmark_test_node ${PROJECT_NAME} ${LINK_LIBS})

add_executable(reflection_aware_ssl_benchmark_visualDetector_node src/rayBasedSSL_benchmark_visualDetector_node.cpp)
target_link_libraries(reflection_aware_ssl_benchmark_visualDetector_node ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(reflection_aware_ssl_benchmark_visualDetector_MS_node src/rayBasedSSL_benchmark_visualDetector_MS_node.cpp)
#target_link_libraries(reflection_aware_ssl_benchmark_visualDetector_MS_node ${PROJECT_NAME} ${LINK_LIBS})

