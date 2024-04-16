# ###############################################################################
# Set minimum required version of cmake, project name and compile options
# ###############################################################################
cmake_minimum_required(VERSION 3.5)
project(robot_body_filter)

if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress all warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# ###############################################################################
# Find ament packages and libraries for ament and system dependencies
# ###############################################################################

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS

        # dynamic_reconfigure # may be superceeded by using ros2 parameters
        filters
        geometric_shapes
        geometry_msgs
        laser_geometry

        # instead of libpcl-all-dev?
        pcl_ros
        moveit_core

        # moveit_ros_perception

        # roscpp replaced by rclcpp
        rclcpp
        sensor_msgs
        std_srvs
        std_msgs
        tf2
        tf2_ros
        urdf
        visualization_msgs
        pcl_conversions
        fcl
        tf2_eigen
        tf2_sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
        find_package(${Dependency} REQUIRED)
endforeach()

# build
include_directories(
        include
        include/utils
        ${Dependency}_INCLUDE_DIRS
)

add_library(${PROJECT_NAME}_filter src/RobotBodyFilter.cpp)
ament_target_dependencies(${PROJECT_NAME}_filter ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(${PROJECT_NAME}_filter
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()