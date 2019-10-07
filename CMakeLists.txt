cmake_minimum_required(VERSION 2.8.3)
project(rasberry_data_collection)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  realsense2_camera
  ros_numpy
  message_generation
  dynamic_reconfigure
  camera_info_manager
)

# ==============================================
# Python scripts setup
# ==============================================
catkin_python_setup()

# ==============================================
# Service/Message files setup
# ==============================================
# add_service_files( FILES )

add_message_files(
  FILES
  WeatherBoard2.msg
  BME680.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs rasberry_data_collection)

# ==============================================
# Setup
# ==============================================
catkin_package(CATKIN_DEPENDS camera_info_manager std_msgs message_runtime)


# ==============================================
# Pico Zense setup
# ==============================================
find_package(OpenCV REQUIRED)
find_library(libpicozense_api NAMES libpicozense_api_x64_ubuntu_16.04.so PATHS src/pico_zense/lib/)
include_directories(include include/pico_zense ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(FrameViewer src/pico_zense/FrameViewer.cpp)
target_link_libraries(FrameViewer ${catkin_LIBRARIES} ${libpicozense_api} ${OpenCV_LIBRARIES})
add_dependencies(FrameViewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(FramePublisher src/pico_zense/FramePublisher.cpp)
target_link_libraries(FramePublisher ${catkin_LIBRARIES} ${libpicozense_api} ${OpenCV_LIBRARIES})
add_dependencies(FramePublisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# ==============================================
# Install
# ==============================================
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB python_scripts_DIR "scripts/*.py")
catkin_install_python(PROGRAMS ${python_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
