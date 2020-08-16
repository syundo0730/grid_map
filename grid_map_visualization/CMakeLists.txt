cmake_minimum_required(VERSION 3.5)
project(grid_map_visualization)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

grid_map_package()

set(dependencies
  grid_map_core
  grid_map_ros
  grid_map_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/grid_map_visualization_node.cpp
# )

## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}
  src/grid_map_visualization_node.cpp
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/GridMapVisualization.cpp
  src/GridMapVisualizationHelpers.cpp
  src/visualizations/VisualizationBase.cpp
  src/visualizations/VisualizationFactory.cpp
  src/visualizations/PointCloudVisualization.cpp
  src/visualizations/FlatPointCloudVisualization.cpp
  src/visualizations/VectorVisualization.cpp
  src/visualizations/OccupancyGridVisualization.cpp
  src/visualizations/GridCellsVisualization.cpp
  src/visualizations/MapRegionVisualization.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()