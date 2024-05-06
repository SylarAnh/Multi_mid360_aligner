cmake_minimum_required(VERSION 3.0.2)
project(mm_align)

SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings") # 关闭pcl烦人的警告

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  	     geometry_msgs
  	     nav_msgs
  	     sensor_msgs
  	     roscpp
  	     rospy
  	     std_msgs
  	     rosbag
  	     livox_ros_driver
  	     eigen_conversions
  	     pcl_conversions
  	     pcl_ros)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
)
link_directories(
		include
)
# generate_messages(DEPENDENCIES std_msgs nav_msgs sensor_msgs livox_ros_driver)
catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs livox_ros_driver
  	       pcl_conversions pcl_ros
	       eigen_conversions DEPENDS PCL INCLUDE_DIRS include)

add_executable(mm_lidarsAligner src/unionLidarsAligner.cpp)
target_link_libraries(mm_lidarsAligner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
