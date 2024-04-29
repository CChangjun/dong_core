cmake_minimum_required(VERSION 3.0.2)
project(dong_core)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  turtlebot3_msgs
  serial
  message_generation
)

 add_message_files(
   FILES
   sensor.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs   
   nav_msgs   
   sensor_msgs   
   std_msgs   
   turtlebot3_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dong_core
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf turtlebot3_msgs serial
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(core_pub src/core_pub.cpp)
target_link_libraries(core_pub ${catkin_LIBRARIES})
#add_dependencies(core_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dong_sensor src/dong_sensor.cpp)
target_link_libraries(dong_sensor ${catkin_LIBRARIES})
#add_dependencies(dong_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

