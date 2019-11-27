cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(podi_navigation_helpers)

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    geometry_msgs
    sensor_msgs
    roscpp
    message_generation
)

add_service_files(
  FILES
  PodiMakeNavPlan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
    sensor_msgs
    roscpp
    message_generation
)

add_library(${PROJECT_NAME}
  src/costmap_model.cpp
  src/grid.cpp
  src/couplingModel.cpp
  src/robot.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(coupling_model_node src/couplingModelNode.cpp)
target_link_libraries(coupling_model_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_grid src/testGrid.cpp)
target_link_libraries(test_grid ${PROJECT_NAME} ${catkin_LIBRARIES})

# add_executable(test_coupling_model src/testCouplingModel.cpp)
# target_link_libraries(test_coupling_model ${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
