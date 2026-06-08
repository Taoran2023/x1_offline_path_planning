cmake_minimum_required(VERSION 3.8)
project(path_planning)

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
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(block_map REQUIRED)
find_package(lowres_map REQUIRED)
find_package(stub_swarm_data REQUIRED)
find_package(glog REQUIRED)

set(DEPS
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  block_map
  lowres_map
  stub_swarm_data
)

# viz library
add_library(viz src/viz.cpp)
target_include_directories(viz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(viz ${DEPS})
target_link_libraries(viz ${PCL_LIBRARIES} glog::glog)

# topo_planner library (Phase 2b+)
add_library(topo_planner src/topo_planner.cpp)
target_include_directories(topo_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(topo_planner ${DEPS})
target_link_libraries(topo_planner glog::glog)

# test_map executable
add_executable(test_map src/test_map.cpp)
target_include_directories(test_map PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(test_map ${DEPS})
target_link_libraries(test_map viz topo_planner ${PCL_LIBRARIES} glog::glog)

install(TARGETS viz topo_planner test_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
