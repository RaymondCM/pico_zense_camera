#ifndef SRC_PICO_ZENSE_MANAGER_H
#define SRC_PICO_ZENSE_MANAGER_H


#include <csignal>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include "PicoZense_api.h"
#include <dynamic_reconfigure/server.h>
#include <pico_zense_camera/pico_zense_dcam710Config.h>

using namespace std;
using namespace cv;


class PicoSenseManager {
public:
    explicit PicoSenseManager(int32_t device_index = 0, const std::string &camera_name = "pico_zense",
                              uint16_t depth_threshold = 100, double depth_range = 1.0, int rgb_width = 1280,
                              int rgb_height = 720);

    void set_resolution(int width, int height);
    void set_range(double range);
    void set_distance_confidence(uint16_t threshold);
    void run();

private:
    static void sigsegv_handler(int sig);
    static void PsExceptionOnFail(PsReturnStatus status, const std::string &message_on_fail);
    void set_sensor_intrinsics();
    void dynamic_reconfigure_callback(pico_zense_camera::pico_zense_dcam710Config &config, uint32_t level);
    void initialise_dynamic_reconfigure_server();

    std::string camera_name_;
    ros::NodeHandle private_nh, colour_nh_, depth_nh_, aligned_nh_, imu_nh_;
    std::shared_ptr<image_transport::ImageTransport> colour_it_, depth_it_, aligned_it_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> colour_info_, depth_info_, aligned_info_;

    image_transport::CameraPublisher colour_pub_, depth_pub_, aligned_pub_;
    ros::Publisher imu_pub_;

    int32_t device_count_, device_index_, data_mode_ = PsDepthAndRGB_30;
    uint16_t depth_threshold_, slope_;
    double depth_range_;
    int rgb_width_, rgb_height_;

    PsCameraParameters depth_intrinsics_{}, colour_intrinsics_{};
    PsCameraExtrinsicParameters extrinsics_{};

    std::shared_ptr<dynamic_reconfigure::Server<pico_zense_camera::pico_zense_dcam710Config>> server;

    dynamic_reconfigure::Server<pico_zense_camera::pico_zense_dcam710Config>::CallbackType reconfigure_callback;
};


#endif //SRC_PICO_ZENSE_MANAGER_H
