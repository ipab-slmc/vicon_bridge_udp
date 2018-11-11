cmake_minimum_required(VERSION 2.8.3)
project(vicon_bridge_udp)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp tf
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/vicon_bridge_udp.cpp)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)