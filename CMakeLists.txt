cmake_minimum_required(VERSION 2.8.3)
project(swarm_bridge)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)  # 添加 PCL 依赖


set(ENABLE_TCP true) # requires zmq, zmqpp 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  # robot_msgs  # 添加 robot_msgs 作为依赖
  pcl_conversions
  pcl_ros
  nav_msgs
  mavros_msgs
  ruiyan_ros_sdk
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS traj_utils pcl_conversions pcl_ros
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}  # 包含 PCL 的头文件路径

)



if(ENABLE_TCP)
  add_executable(bridge_node_tcp
    src/bridge_node_tcp.cpp
  )
  add_executable(test_tcp_station
    src/test_tcp_station.cpp
  )
  add_executable(test_tcp_drone
    src/test_tcp_drone.cpp
  )
  add_executable(tcp_drone
    src/tcp_drone.cpp
  )
  add_executable(tcp_station
    src/tcp_station.cpp
  )
  target_link_libraries(test_tcp_station
    ${catkin_LIBRARIES}
    zmq
    zmqpp
  )
  target_link_libraries(bridge_node_tcp
    ${catkin_LIBRARIES}
    zmq
    zmqpp
  )
  target_link_libraries(test_tcp_drone
    ${catkin_LIBRARIES}
    zmq
    zmqpp
  )
  target_link_libraries(tcp_drone
    ${catkin_LIBRARIES}
    zmq
    zmqpp
  )
  target_link_libraries(tcp_station
    ${catkin_LIBRARIES}
    zmq
    zmqpp
  )
endif(ENABLE_TCP)
add_executable(point_cloud_expander src/point_cloud_expander.cpp)
target_link_libraries(point_cloud_expander ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(odometry_calculator src/odometry_calculator.cpp)
target_link_libraries(odometry_calculator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(drone_odom_modifier src/drone_odom_modifier.cpp)
target_link_libraries(drone_odom_modifier ${catkin_LIBRARIES})
# add_executable(traj2odom_node
#   src/traj2odom_node.cpp 
# )
# target_link_libraries(traj2odom_node 
#   ${catkin_LIBRARIES}
# )
