cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(podi_robot_human_coupled_planner)

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    geometry_msgs
    visualization_msgs
    std_msgs
    tf
    nav_msgs
    podi_nav_core
    podi_navigation_helpers
    pluginlib
    roscpp
)

include_directories(
  include

  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    visualization_msgs
    std_msgs
    tf
    nav_msgs
    podi_nav_core
    podi_navigation_helpers
    pluginlib
    roscpp
)

add_library(${PROJECT_NAME}
  src/node.cpp
  src/planner.cpp
  src/planner_core.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(planner
  src/plan_node.cpp
)

target_link_libraries(planner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES robot_human_coupled_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
