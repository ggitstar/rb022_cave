cmake_minimum_required(VERSION 2.8.3)
project(rb022_cave)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES rb022_cave
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(rb022_main src/rb022_main.cpp)
 add_dependencies(rb022_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(rb022_main ${catkin_LIBRARIES} )


