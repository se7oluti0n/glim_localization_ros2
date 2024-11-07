cmake_minimum_required(VERSION 3.16)
project(glim_ros VERSION 1.0.4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(glim REQUIRED)
find_package(OCTOMAP REQUIRED)

if(BUILD_WITH_CUDA)
  add_definitions(-DBUILD_GTSAM_POINTS_GPU)
endif()

set(OLD_DISTRO "humble;galactic;foxy")
if($ENV{ROS_DISTRO} IN_LIST OLD_DISTRO)
  add_definitions(-DCV_BRIDGE_INCLUDE_H)
else()
  add_definitions(-DCV_BRIDGE_INCLUDE_HPP)
endif()

### glim_ros ###
ament_auto_add_library(glim_ros SHARED
  src/glim_ros/glim_ros.cpp
  # src/glim_ros/glim_localization.cpp
  src/glim_ros/glim_editor.cpp
)
target_include_directories(glim_ros PUBLIC
  include
)
target_link_libraries(glim_ros
  glim::glim
)
rclcpp_components_register_nodes(glim_ros "glim::GlimROS")

ament_auto_add_library(rviz_viewer SHARED
  src/glim_ros/rviz_viewer.cpp
  src/util/LatLong_UTMconversion.cpp
)

ament_auto_add_library(glim_octomap SHARED
  src/glim_ros/glim_octomap.cpp
)

target_include_directories(glim_octomap
  PUBLIC
    ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(glim_octomap
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

### glim_rosnode ###
ament_auto_add_executable(glim_rosnode
  src/glim_rosnode.cpp
)
target_link_libraries(glim_rosnode
  glim_ros
)

# ament_auto_add_executable(glim_localization
#   src/glim_roslocalization.cpp
# )
# target_link_libraries(glim_localization
#   glim_ros
# )

### glim_rosbag ###
ament_auto_add_executable(glim_rosbag
  src/glim_rosbag.cpp
)
target_link_libraries(glim_rosbag
  glim_ros
)

### validator_node ###
ament_auto_add_executable(validator_node
  src/validator_node.cpp
)
target_link_libraries(validator_node
  glim_ros
)

### validator_node ###
ament_auto_add_executable(evaluator_node
  src/evaluator_node.cpp
)


if(BUILD_WITH_VIEWER)
  ### offline_viewer ###
  ament_auto_add_executable(offline_viewer
    src/offline_viewer.cpp
  )
  target_include_directories(offline_viewer PUBLIC
    include
  )
  target_link_libraries(offline_viewer
    glim::interactive_viewer
    glim_ros
  )
endif()

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}/
)
ament_auto_package()
