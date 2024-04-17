cmake_minimum_required(VERSION 3.7.2)
project(robot_body_filter)

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(DetectOptional)

if(${ROBOT_BODY_FILTER_HAVE_CXX_OPTIONAL})
  # add_compile_definitions would be nicer, but isn't available in Stretch
  add_compile_options(-DROBOT_BODY_FILTER_USE_CXX_OPTIONAL=1)
else()
  add_compile_options(-DROBOT_BODY_FILTER_USE_CXX_OPTIONAL=0)
endif()

set(THIS_PACKAGE_DEPS filters geometric_shapes laser_geometry moveit_core rclcpp sensor_msgs tf2 tf2_ros urdf visualization_msgs)
set(MESSAGE_DEPS geometry_msgs std_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPS})
        find_package(${Dependency} REQUIRED)
endforeach()

# build
include_directories(
        include
        include/utils
        ${Dependency}_INCLUDE_DIRS
)

add_library(${PROJECT_NAME}_filter src/RobotBodyFilter.cpp)
ament_target_dependencies(${PROJECT_NAME}_filter ${THIS_PACKAGE_DEPS})

target_include_directories(${PROJECT_NAME}_filter
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/OrientedBoundingBox.msg"
#   "msg/OrientedBoundingBoxStamped.msg"
#   DEPENDENCIES builtin_interfaces std_msgs geometry_msgs std_srvs tf2_msgs tf2_sensor_msgs tf2_ros tf2_eigen visualization_msgs
#  )

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()