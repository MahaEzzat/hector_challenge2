cmake_minimum_required(VERSION 2.8.3)
project(hector_challenge2)
find_package(catkin REQUIRED COMPONENTS actionlib geometry_msgs hector_quadrotor_interface roscpp tf2 tf2_geometry_msgs gazebo_msgs hector_uav_msgs)
  
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS roscpp
    DEPENDS
)

add_executable(${PROJECT_NAME} 
  src/hector_challenge2_node.cpp
  src/hector_challenge2.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(object
  src/object.cpp
)
add_dependencies(object ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object
  ${catkin_LIBRARIES}
)