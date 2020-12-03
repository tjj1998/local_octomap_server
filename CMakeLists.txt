cmake_minimum_required( VERSION 2.8 )
project( local_octomap_server )
set(CMAKE_BUILD_TYPE Release)
# use sse3 instruction set
add_definitions(-msse3 -w)
# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++17" )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    octomap
    octomap_msgs
    pcl_conversions
    pcl_ros
    tf
    tf2)
add_service_files(
    FILES
    ClearOctomap.srv
)
generate_messages(
    DEPENDENCIES
    std_msgs
)
# eigen 
include_directories(/usr/include/eigen3)
# pcl 
find_package( PCL REQUIRED ) 
# glog
find_package( glog REQUIRED)
add_definitions( ${PCL_DEFINITIONS} )
# octomap 
find_package( octomap REQUIRED )
# 寻找OpenCV库 
find_package( OpenCV REQUIRED )
set(INC_DIR ./include)
set(CMAKE_CXX_FLAGS "-msse3")
# 添加头文件
include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${INC_DIR})
FILE(GLOB LOCAL_OCTOMAP_SERVER_SRC_FILES "src/*.cpp")
catkin_package()
add_executable( local_octomap_server_node Local_Octomap_Server.cpp ${LOCAL_OCTOMAP_SERVER_SRC_FILES})
# 链接库
target_link_libraries( local_octomap_server_node ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} glog)
