# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(green_fundamentals)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs 
  create_fundamentals 
  message_generation
)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
  FILES 
  Cell.msg
  Grid.msg
  Pose.msg
  Row.msg
  Position.msg
  Vector2f.msg
  LaserCoordinates.msg
  Obstacle.msg
)

#uncomment if you have defined services
add_service_files(
  FILES
  DetectGrid.srv
  DriveTo.srv
  DriveToWaypoints.srv
  ExecutePlan.srv
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES 
   std_msgs 
   sensor_msgs 
   nav_msgs 
   geometry_msgs
   create_fundamentals
)

catkin_package(
    DEPENDS python-serial
    CATKIN_DEPENDS roscpp rospy sensor_msgs nav_msgs geometry_msgs create_fundamentals message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry src/odometry/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
add_dependencies(odometry create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(grid_detection_server src/grid_detection/grid_detection_server.cpp src/classes/line.cpp)
target_link_libraries(grid_detection_server ${catkin_LIBRARIES})
add_dependencies(grid_detection_server create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(driving_server src/driving/driving_server.cpp)
target_link_libraries(driving_server ${catkin_LIBRARIES})
add_dependencies(driving_server create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(align src/align.cpp src/classes/driver.cpp src/classes/line.cpp)
target_link_libraries(align ${catkin_LIBRARIES})
add_dependencies(align create_fundamentals_generate_messages_cpp)# ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(execute_plan_server src/execute_plan/execute_plan_server.cpp)
target_link_libraries(execute_plan_server ${catkin_LIBRARIES})
add_dependencies(execute_plan_server create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(calculate_laser_coordinates src/obstacle_detection/calculate_laser_coordinates.cpp)
target_link_libraries(calculate_laser_coordinates ${catkin_LIBRARIES})
add_dependencies(calculate_laser_coordinates create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(obstacle_detection src/obstacle_detection/obstacle_detection.cpp)
target_link_libraries(obstacle_detection ${catkin_LIBRARIES})
add_dependencies(obstacle_detection create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Test files 
#add_executable(test-ransac src/test/test-ransac.cpp src/classes/line_detector.cpp src/classes/line.cpp src/classes/vector.cpp src/algorithms/ransac.cpp)
#target_link_libraries(test-ransac ${catkin_LIBRARIES} gtest gtest_main)

add_executable(grid_to_map src/occupancy_map.cpp)
target_link_libraries(grid_to_map ${catkin_LIBRARIES})
add_dependencies(grid_to_map create_fundamentals_generate_messages_cpp)

add_executable(mc_localization src/monte_carlo_localization.cpp)
target_link_libraries(mc_localization ${catkin_LIBRARIES})
add_dependencies(mc_localization create_fundamentals_generate_messages_cpp)