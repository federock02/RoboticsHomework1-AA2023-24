cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        dynamic_reconfigure
        tf
        tf2_ros
        tf2_geometry_msgs
)

## Generate dynamic reconfigure config file
generate_dynamic_reconfigure_options(
        cfg/lidar.cfg
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables for the nodes
add_executable(gps_to_odom src/gps_to_odom.cpp)
add_executable(odom_to_tf src/odom_to_tf.cpp)
add_executable(lidar_remap src/lidar_remap.cpp)

## Add dependencies and link libraries for each executable
add_dependencies(gps_to_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_to_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_remap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_remap ${PROJECT_NAME}_gencfg)

target_link_libraries(gps_to_odom ${catkin_LIBRARIES})
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})
target_link_libraries(lidar_remap ${catkin_LIBRARIES})
