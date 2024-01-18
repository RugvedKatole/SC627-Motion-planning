cmake_minimum_required(VERSION 3.0.2)
project(tutorial2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

 add_message_files(
   FILES
   Num.msg
 )

 add_service_files(
   FILES
   AddTwoInts.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
install(TARGETS talker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
include_directories(${catkin_INCLUDE_DIRS})
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
install(TARGETS listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(${catkin_INCLUDE_DIRS})
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
install(TARGETS add_two_ints_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
