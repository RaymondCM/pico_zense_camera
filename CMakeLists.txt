cmake_minimum_required(VERSION 2.8.3)
project(pico_zense_camera)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  camera_info_manager
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

# ==============================================
# Python scripts setup
# ==============================================
catkin_python_setup()

# ==============================================
# Service/Message files setup
# ==============================================
# add_service_files( FILES )

#add_message_files(
#  FILES
#)
#

generate_messages(DEPENDENCIES std_msgs sensor_msgs pico_zense_camera)

generate_dynamic_reconfigure_options(
        cfg/pico_zense_dcam710.cfg
)

# ==============================================
# Setup
# ==============================================
catkin_package(CATKIN_DEPENDS camera_info_manager image_transport tf2 tf2_ros geometry_msgs std_msgs message_runtime roscpp rospy cv_bridge)

# ==============================================
# Pico Zense setup
# ==============================================
find_package(OpenCV REQUIRED)
find_library(libpicozense_api NAMES libpicozense_api_x64_ubuntu_16.04.so PATHS src/pico_zense_camera/lib/)
include_directories(include include/pico_zense_camera ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(FrameViewer src/pico_zense_camera/FrameViewer.cpp)
target_link_libraries(FrameViewer ${catkin_LIBRARIES} ${libpicozense_api} ${OpenCV_LIBRARIES})
add_dependencies(FrameViewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pico_zense_camera src/pico_zense_camera/pico_zense_driver.cpp
                                 src/pico_zense_camera/pico_zense_manager.cpp
                                 include/pico_zense_camera/pico_zense_manager.hpp)
target_link_libraries(pico_zense_camera ${catkin_LIBRARIES} ${libpicozense_api} ${OpenCV_LIBRARIES})
add_dependencies(pico_zense_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# ==============================================
# Install
# ==============================================
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB python_scripts_DIR "scripts/*.py")
catkin_install_python(PROGRAMS ${python_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
