#include <iostream>
#include "ros/ros.h"
#include <pico_zense_camera/pico_zense_manager.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pico_zense_manager");

    std::string camera_name = ros::param::param<std::string>("~camera_name", std::string("pico_zense"));
    int32_t device_index = ros::param::param<int32_t>("~device_index", 0);
    uint16_t depth_confidence_threshold = (uint16_t) ros::param::param<int32_t>("~depth_confidence_threshold", 100);
    double depth_range = ros::param::param<double>("~depth_range", 1.0);
    int rgb_width = ros::param::param<int>("~rgb_width", 1280);
    int rgb_height = ros::param::param<int>("~rgb_height", 720);

    PicoSenseManager manager = PicoSenseManager(device_index, camera_name, depth_confidence_threshold, depth_range,
                                                rgb_width, rgb_height);
    manager.run();
    return 0;
}