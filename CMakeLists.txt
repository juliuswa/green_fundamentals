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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(
  FILES 
  Cell.msg
  Grid.msg
  Pose.msg
  Row.msg
  Position.msg
  Obstacle.msg
)

add_service_files(
  FILES
  DriveTo.srv
  ExecutePlan.srv
)

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

add_executable(map_generator src/map_generator.cpp)
target_link_libraries(map_generator ${catkin_LIBRARIES})
add_dependencies(map_generator create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(localizer src/localizer.cpp)
target_link_libraries(localizer ${catkin_LIBRARIES})
add_dependencies(localizer create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(localizer2 src/localizer2.cpp)
target_link_libraries(localizer2 ${catkin_LIBRARIES})
add_dependencies(localizer2 create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(robot_mover src/robot_mover.cpp)
target_link_libraries(robot_mover ${catkin_LIBRARIES})
add_dependencies(robot_mover create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(obstacle_detector src/obstacle_detector.cpp)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES})
add_dependencies(obstacle_detector create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(local_planner src/local_planner.cpp)
target_link_libraries(local_planner ${catkin_LIBRARIES})
add_dependencies(local_planner create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(grid_detection_server src/grid_detection/grid_detection_server.cpp src/classes/line.cpp)
#target_link_libraries(grid_detection_server ${catkin_LIBRARIES})
#add_dependencies(grid_detection_server create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(driving_server src/driving/driving_server.cpp)
#target_link_libraries(driving_server ${catkin_LIBRARIES})
#add_dependencies(driving_server create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(align src/align.cpp src/classes/driver.cpp src/classes/line.cpp)
#target_link_libraries(align ${catkin_LIBRARIES})
#add_dependencies(align create_fundamentals_generate_messages_cpp)# ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(execute_plan_server src/execute_plan/execute_plan_server.cpp)
#target_link_libraries(execute_plan_server ${catkin_LIBRARIES})
#add_dependencies(execute_plan_server create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(calculate_laser_coordinates src/obstacle_detection/calculate_laser_coordinates.cpp)
#target_link_libraries(calculate_laser_coordinates ${catkin_LIBRARIES})
#add_dependencies(calculate_laser_coordinates create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(obstacle_detection src/obstacle_detection/obstacle_detection.cpp)
#target_link_libraries(obstacle_detection ${catkin_LIBRARIES})
#add_dependencies(obstacle_detection create_fundamentals_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Test files 
#add_executable(test-ransac src/test/test-ransac.cpp src/classes/line_detector.cpp src/classes/line.cpp src/classes/vector.cpp src/algorithms/ransac.cpp)
#target_link_libraries(test-ransac ${catkin_LIBRARIES} gtest gtest_main)