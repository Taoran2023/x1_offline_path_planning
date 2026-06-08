cmake_minimum_required(VERSION 3.8)
project(stub_swarm_data)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(stub_exp_comm_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE geometry_msgs stub_exp_comm_msgs)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs stub_exp_comm_msgs)
ament_package()
