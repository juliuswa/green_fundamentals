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
  GoldRun.srv
  MoveToPosition.srv
  SetVideo.srv
  StartLocalization.srv
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
add_dependencies(map_generator create_fundamentals_generate_messages_cpp) #${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(localizer src/localizer.cpp)
target_link_libraries(localizer ${catkin_LIBRARIES})
add_dependencies(localizer create_fundamentals_generate_messages_cpp) #${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(robot_mover src/robot_mover.cpp)
target_link_libraries(robot_mover ${catkin_LIBRARIES})
add_dependencies(robot_mover create_fundamentals_generate_messages_cpp) #${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(obstacle_detector src/obstacle_detector.cpp)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES})
add_dependencies(obstacle_detector create_fundamentals_generate_messages_cpp) #${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})
add_dependencies(planner create_fundamentals_generate_messages_cpp) #${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(localizer2 src/localizer2.cpp)
target_link_libraries(localizer2 ${catkin_LIBRARIES})
add_dependencies(localizer2 create_fundamentals_generate_messages_cpp) #${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)