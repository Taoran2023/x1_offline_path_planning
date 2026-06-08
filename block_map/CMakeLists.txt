cmake_minimum_required(VERSION 3.8)
project(block_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(stub_swarm_data REQUIRED)
find_package(stub_exp_comm_msgs REQUIRED)

set(DEPS
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  cv_bridge
  stub_swarm_data
  stub_exp_comm_msgs
)

# raycast (no ROS deps)
add_library(${PROJECT_NAME}_raycast src/raycast.cpp)
target_include_directories(${PROJECT_NAME}_raycast PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# color_manager
add_library(${PROJECT_NAME}_color src/color_manager.cpp)
target_include_directories(${PROJECT_NAME}_color PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_color ${DEPS})

# main grid library
add_library(${PROJECT_NAME}_grid
  src/block_map.cpp
  src/traversability.cpp
)
target_include_directories(${PROJECT_NAME}_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_grid ${DEPS})
target_link_libraries(${PROJECT_NAME}_grid
  ${PROJECT_NAME}_raycast
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS
  ${PROJECT_NAME}_raycast
  ${PROJECT_NAME}_color
  ${PROJECT_NAME}_grid
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS} Eigen3 PCL OpenCV)
ament_package()
