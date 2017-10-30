cmake_minimum_required(VERSION 2.8.3)
project(motion_example)


# Add support for c++11
add_definitions(-std=c++11)


# Find required packages
find_package(catkin REQUIRED COMPONENTS
  lwr_ros_client
  dynamic_reconfigure
  roscpp
  rospy
  serial
  std_msgs
)


# Generate dynamic reconfigure parameters
# generate_dynamic_reconfigure_options()

# find_package(Eigen REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

 catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS lwr_ros_client roscpp rospy serial std_msgs dynamic_reconfigure
    DEPENDS Eigen
 )

add_executable(moveToDesiredJoints src/MoveToDesiredJoints_node.cpp src/MoveToDesiredJoints.cpp)
target_link_libraries(moveToDesiredJoints ${catkin_LIBRARIES})