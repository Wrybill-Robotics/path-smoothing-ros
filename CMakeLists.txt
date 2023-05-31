cmake_minimum_required(VERSION 2.8.3)
project(path_smoothing_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf
)

catkin_package(
 INCLUDE_DIRS
  include
 CATKIN_DEPENDS
  roscpp
  geometry_msgs
  nav_msgs
 LIBRARIES
  path_smoothing_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME} src/cubic_spline_interpolator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(demo src/demo.cpp)
add_executable(path_smoother src/path_smoother.cpp)
add_executable(demo_generator src/demo_generator.cpp)
add_executable(path_from_waypoints src/path_from_waypoints.cpp)
add_executable(path_from_markers src/path_from_markers.cpp)
add_executable(marker_demo src/marker_demo.cpp)
add_executable(waypoint_saver src/waypoint_saver.cpp)
add_executable(record_path src/record_path.cpp)


target_link_libraries(demo ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(path_smoother ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(demo_generator ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(path_from_waypoints ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(path_from_markers ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(marker_demo ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(waypoint_saver ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(record_path ${PROJECT_NAME} ${catkin_LIBRARIES})


