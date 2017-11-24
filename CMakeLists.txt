cmake_minimum_required(VERSION 2.8.3)
project(odom_tf_package)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs tf)



## Declare a catkin package
catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_tf src/odom_tf_node.cpp)
target_link_libraries(odom_tf ${catkin_LIBRARIES})

