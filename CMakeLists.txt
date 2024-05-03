# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(green_fundamentals)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs create_fundamentals message_generation)

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
add_message_files(FILES Position.msg
)
#uncomment if you have defined services
#add_service_files(FILES
#)

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
   DEPENDENCIES std_msgs create_fundamentals
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS python-serial
    CATKIN_DEPENDS roscpp rospy sensor_msgs nav_msgs create_fundamentals message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(example_laser src/example_laser.cpp)
target_link_libraries(example_laser ${catkin_LIBRARIES})
add_dependencies(example_laser create_fundamentals_generate_messages_cpp)

add_executable(example_moving src/example_moving.cpp)
target_link_libraries(example_moving ${catkin_LIBRARIES})
add_dependencies(example_moving create_fundamentals_generate_messages_cpp)

add_executable(wanderer src/wanderer.cpp)
target_link_libraries(wanderer ${catkin_LIBRARIES})
add_dependencies(wanderer create_fundamentals_generate_messages_cpp)

add_executable(square_no_sensors src/square_no_sensors.cpp)
target_link_libraries(square_no_sensors ${catkin_LIBRARIES})
add_dependencies(square_no_sensors create_fundamentals_generate_messages_cpp)

add_executable(square_with_encoders src/square_with_encoders.cpp src/classes/driver.cpp)
target_link_libraries(square_with_encoders ${catkin_LIBRARIES})
add_dependencies(square_with_encoders create_fundamentals_generate_messages_cpp)

catkin_install_python(PROGRAMS src/move_to_position.py src/odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(align src/align.cpp src/class/driver.cpp src/classes/grid_detector.cpp)
target_link_libraries(align ${catkin_LIBRARIES})
add_dependencies(align create_fundamentals_generate_messages_cpp)