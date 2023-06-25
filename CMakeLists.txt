cmake_minimum_required(VERSION 2.8.3)
project(imu_uwb_fusion)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

 add_message_files(
   FILES
   UwbMsg.msg
 )

  generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES imu_uwb_fusion
  CATKIN_DEPENDS message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(imu_uwb_fusion src/imu_uwb_fusion.cpp)

add_executable(imu_uwb_fusion_node src/imu_uwb_fusion_node.cpp)
target_link_libraries(imu_uwb_fusion_node ${catkin_LIBRARIES} imu_uwb_fusion)

add_executable(moving_average_filter_node src/moving_average_filter.cpp)
target_link_libraries(moving_average_filter_node ${catkin_LIBRARIES})

#add_executable(uwb_plot_node src/uwb_plot.cpp)
#target_link_libraries(uwb_plot_node ${catkin_LIBRARIES})
