cmake_minimum_required(VERSION 2.8.3)
project(prcdmp_node)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  franka_control
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  std_msgs
  common_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

catkin_package(
  LIBRARIES prcdmp_node
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

INCLUDE_DIRECTORIES(include)

add_subdirectory(src)

add_library(prcdmp_node
  src/dmp_velocity_example_controller.cpp
  src/dmpstart_velocity_controller.cpp
  src/dmp_velocity_controller.cpp
  src/dmp_ct_velocity_controller.cpp
  src/dmpviap_controller.cpp
)

#temporary
add_executable(serviceDummy src/serviceDummy.cpp)
target_link_libraries(serviceDummy ${catkin_LIBRARIES})

add_executable(debugDummy src/debugDummy.cpp)
target_link_libraries(debugDummy ${catkin_LIBRARIES} ${Franka_LIBRARIES} DMP UTILS Eigen3::Eigen jsoncpp prcdmp_node)

add_dependencies(prcdmp_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(prcdmp_node PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  DMP UTILS jsoncpp
)

target_include_directories(prcdmp_node SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(prcdmp_node PUBLIC
  include
)

## Installation
install(TARGETS prcdmp_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES prcdmp_node_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(prcdmp_node FILES ${SOURCES} ${HEADERS})
  add_tidy_target(prcdmp_node
    FILES ${SOURCES}
    DEPENDS prcdmp_node
  )
endif()
