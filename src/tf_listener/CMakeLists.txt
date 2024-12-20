cmake_minimum_required(VERSION 3.0.2)
project(tf_listener)

## 找到catkin所需的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
)

## 添加 OpenCV 和 RealSense 包
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

## 声明catkin的包
catkin_package(
  CATKIN_DEPENDS roscpp tf2_ros geometry_msgs
  DEPENDS OpenCV realsense2
)

## 指定包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

## 生成可执行文件
add_executable(tf_listener_node src/tf_listener_node.cpp)

## 链接库
target_link_libraries(tf_listener_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${realsense2_LIBRARY}
)

