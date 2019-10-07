#include <csignal>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include "PicoZense_api.h"

using namespace std;
using namespace cv;

class PicoSenseManager {
public:
    explicit PicoSenseManager(int32_t device_index = 0, const std::string &camera_name = "pico") :
            camera_name(camera_name),
            private_nh("~"),
            colour_nh(camera_name + "/colour"),
            depth_nh(camera_name + "/depth"),
            aligned_nh(camera_name + "/aligned_depth_to_colour"),
            colour_info(new camera_info_manager::CameraInfoManager(colour_nh)),
            depth_info(new camera_info_manager::CameraInfoManager(depth_nh)),
            aligned_info(new camera_info_manager::CameraInfoManager(aligned_nh)),
            colour_it(new image_transport::ImageTransport(colour_nh)),
            depth_it(new image_transport::ImageTransport(depth_nh)),
            aligned_it(new image_transport::ImageTransport(aligned_nh)) {
        signal(SIGSEGV, PicoSenseManager::sigsegv_handler);
        PsReturnStatus status;

        // Initialise the API
        PsExceptionOnFail(PsInitialize(), "PsInitialize failed!");

        // Get number of available devices
        PsExceptionOnFail(PsGetDeviceCount(&this->device_count), "PsGetDeviceCount failed!");
        ROS_INFO("Get device count: %d", this->device_count);

        // Verify device index selection
        this->device_index = device_index;
        if (this->device_index < 0 || this->device_index >= this->device_count)
            throw std::runtime_error(
                    "Device index outside of available devices range 0-" + std::to_string(this->device_count));

        //Set the Depth Range to Near through PsSetDepthRange interface
        PsExceptionOnFail(PsSetDepthRange(this->device_index, PsNearRange),
                          "Could not set device " + std::to_string(this->device_index) +
                          " to 0-1m depth range (PSNearRange)");
        ROS_INFO("Set device %d depth to PSNearRange (0-1m)", this->device_index);

        // Attempt to open the device
        PsExceptionOnFail(PsOpenDevice(this->device_index), "OpenDevice failed!");
        ROS_INFO("Successfully connected to device %d", this->device_index);

        //Set PixelFormat as PsPixelFormatRGB888 for regular display
        PsExceptionOnFail(PsSetColorPixelFormat(this->device_index, PsPixelFormatBGR888), "Could not set pixel format");
        ROS_INFO("Set Pixel Format to BGR888 for device %d", this->device_index);

        // Set to DepthAndRGB_30 mode
        PsExceptionOnFail(PsSetDataMode(this->device_index, (PsDataMode) this->data_mode),
                          "Set DataMode Failed failed!");
        ROS_INFO("Set capture mode to DepthAndRGB_30 for device %d", this->device_index);

        // Enable depth to rgb frame alignment in the API
        PsExceptionOnFail(PsSetMapperEnabledRGBToDepth(this->device_index, true),
                          "Could not enable depth to rgb alignment");
        ROS_INFO("Enabled depth to rgb alignment for device %d", this->device_index);

        // Set distortion correction for all sensors
        PsExceptionOnFail(PsSetDepthDistortionCorrectionEnabled(this->device_index, true),
                "Could not enable depth distortion correction");
        PsExceptionOnFail(PsSetRGBDistortionCorrectionEnabled(this->device_index, true),
                "Could not enable rgb distortion correction");
        PsExceptionOnFail(PsSetIrDistortionCorrectionEnabled(this->device_index, true),
                "Could not enable ir distortion correction");
        ROS_INFO("Enabled distortion correction for device %d", this->device_index);


        // Set synchronisation between rgb and depth
        PsExceptionOnFail(PsSetSynchronizeEnabled(this->device_index, true),
                "Could not enable sensor synchronisation between rgb and depth sensors");
        ROS_INFO("Enabled sensor synchronisation for device %d", this->device_index);

        // Set smoothing filter
        PsExceptionOnFail(PsSetFilter(this->device_index, PsSmoothingFilter, true),
                "Could not enable depth smoothing filter");
        ROS_INFO("Enabled smoothing filter for device %d", this->device_index);

        this->set_sensor_intrinsics();
    };

    static void sigsegv_handler(int sig) {
        signal(SIGSEGV, SIG_DFL);
        ROS_ERROR("Segmentation fault, stopping camera driver.");
        ros::shutdown();
    }

    void set_sensor_intrinsics() {
        std::string camera_frame(this->camera_name + "_frame"), colour_frame(this->camera_name + "_colour_frame"),
                aligned_frame(this->camera_name + "_aligned_depth_to_colour_frame"),
                depth_frame(this->camera_name + "_depth_frame");

        // Get camera parameters (extrinsic)
        PsExceptionOnFail(PsGetCameraExtrinsicParameters(this->device_index, &this->extrinsics),
                          "Could not get extrinsics!");

        // Setup tf broadcaster
        static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
        ros::Time now = ros::Time::now();

        // PsCameraExtrinsicParameters to ROS transform
        tf::Transform transform;
        tf::Matrix3x3 rotation_matrix(extrinsics.rotation[0], extrinsics.rotation[1], extrinsics.rotation[2],
                                      extrinsics.rotation[3], extrinsics.rotation[4], extrinsics.rotation[5],
                                      extrinsics.rotation[6], extrinsics.rotation[7], extrinsics.rotation[8]);
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

        // Publish static TFs
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = now;
        msg.transform.rotation.w = 1.0;

        // Camera base to Colour Frame
        msg.header.frame_id = camera_frame;
        msg.child_frame_id = colour_frame;
        tf_broadcaster.sendTransform(msg);

        // Colour Frame to Aligned Frame
        msg.header.frame_id = colour_frame;
        msg.child_frame_id = aligned_frame;
        tf_broadcaster.sendTransform(msg);

        // Colour Frame to Depth Frame
        msg.transform.translation.x = extrinsics.translation[0] / 1000;
        msg.transform.translation.y = extrinsics.translation[1] / 1000;
        msg.transform.translation.z = extrinsics.translation[2] / 1000;
        msg.transform.rotation = orientation;
        msg.header.frame_id = colour_frame;
        msg.child_frame_id = depth_frame;
        tf_broadcaster.sendTransform(msg);

        // Get camera parameters (intrinsic)
        PsExceptionOnFail(PsGetCameraParameters(this->device_index, PsDepthSensor, &this->depth_intrinsics),
                          "Could not get depth intrinsics!");
        PsExceptionOnFail(PsGetCameraParameters(this->device_index, PsRgbSensor, &this->colour_intrinsics),
                          "Could not get rgb intrinsics!");

        // Initialise camera info messages
        sensor_msgs::CameraInfo info_msg;
        info_msg.distortion_model = "plumb_bob";
        info_msg.header.frame_id = colour_frame;
        info_msg.D = {colour_intrinsics.k1, colour_intrinsics.k2, colour_intrinsics.p1, colour_intrinsics.p2,
                             colour_intrinsics.k3};
        info_msg.K = {colour_intrinsics.fx, 0, colour_intrinsics.cx, 0, colour_intrinsics.fy,
                             colour_intrinsics.cy, 0, 0, 1};
        info_msg.P = {colour_intrinsics.fx, 0, colour_intrinsics.cx, 0, 0, colour_intrinsics.fy,
                             colour_intrinsics.cy, 0, 0, 0, 1, 0};
        colour_info->setCameraInfo(info_msg);
        aligned_info->setCameraInfo(info_msg);

        info_msg.header.frame_id = depth_frame;
        info_msg.D = {depth_intrinsics.k1, depth_intrinsics.k2, depth_intrinsics.p1, depth_intrinsics.p2,
                            depth_intrinsics.k3};
        info_msg.K = {depth_intrinsics.fx, 0, depth_intrinsics.cx, 0, depth_intrinsics.fy, depth_intrinsics.cy, 0,
                            0, 1};
        info_msg.P = {depth_intrinsics.fx, 0, depth_intrinsics.cx, 0, 0, depth_intrinsics.fy, depth_intrinsics.cy,
                            0, 0, 0, 1, 0};
        depth_info->setCameraInfo(info_msg);

        ROS_INFO("Successfully received intrinsic and extrinsic parameters for device %d", this->device_index);
    }

    void run() {
        // Initialise ROS nodes
        ros::NodeHandle nh;

        sensor_msgs::CameraInfoPtr colour_ci(new sensor_msgs::CameraInfo(colour_info->getCameraInfo()));
        sensor_msgs::CameraInfoPtr depth_ci(new sensor_msgs::CameraInfo(depth_info->getCameraInfo()));
        sensor_msgs::CameraInfoPtr aligned_ci(new sensor_msgs::CameraInfo(aligned_info->getCameraInfo()));

        this->colour_pub = this->colour_it->advertiseCamera("image_raw", 30);
        this->depth_pub = this->depth_it->advertiseCamera("image_raw", 30);
        this->aligned_pub = this->aligned_it->advertiseCamera("image_raw", 30);

        // Containers for frames
        PsReturnStatus status;
        PsFrame depth_frame = {0};
        PsFrame colour_frame = {0};
        PsFrame aligned_depth_to_colour_frame = {0};
        cv::Mat colour_mat, depth_mat, aligned_depth_to_colour_mat;

        int cycle_id = 0;
        while (ros::ok()) {

            cout << "Cycle Count: " << cycle_id++ << endl;
            // Get next frame set
            // Test setup by getting a single frame
            status = PsReadNextFrame(this->device_index);
            if (status != PsRetOK) {
                ROS_WARN("Could not get next frame set from device %d", this->device_index);
                continue;
            }

            // Get frames
            PsGetFrame(this->device_index, PsRGBFrame, &colour_frame);
            PsGetFrame(this->device_index, PsDepthFrame, &depth_frame);
            PsGetFrame(this->device_index, PsMappedDepthFrame, &aligned_depth_to_colour_frame);

            // Ensure all frames have arrived
            if (colour_frame.pFrameData == NULL || depth_frame.pFrameData == NULL ||
                aligned_depth_to_colour_frame.pFrameData == NULL) {
                ROS_WARN("Could not get frames: rgb=%s, depth=%s, aligned_depth_to_rgb=%s",
                         colour_frame.pFrameData == NULL ? "failed" : "passed",
                         depth_frame.pFrameData == NULL ? "failed" : "passed",
                         aligned_depth_to_colour_frame.pFrameData == NULL ? "failed" : "passed");
                continue;
            }

            // Convert struct buffers to OpenCV mats for publishing with cv::bridge
            colour_mat = cv::Mat(colour_frame.height, colour_frame.width, CV_8UC3, colour_frame.pFrameData);
            depth_mat = cv::Mat(depth_frame.height, depth_frame.width, CV_16UC1, depth_frame.pFrameData);
            aligned_depth_to_colour_mat = cv::Mat(aligned_depth_to_colour_frame.height,
                                                  aligned_depth_to_colour_frame.width,
                                                  CV_16UC1, aligned_depth_to_colour_frame.pFrameData);

            // Set non static camera_info information (width, height and timestamp)
            ros::Time now = ros::Time::now();
            colour_ci->height = colour_frame.height;
            colour_ci->width = colour_frame.width;
            colour_ci->header.stamp = now;
            depth_ci->height = depth_frame.height;
            depth_ci->width = depth_frame.width;
            depth_ci->header.stamp = now;
            aligned_ci->height = aligned_depth_to_colour_frame.height;
            aligned_ci->width = aligned_depth_to_colour_frame.width;
            aligned_ci->header.stamp = now;

            // Publish messages and camera info
            sensor_msgs::ImagePtr colour_msg = cv_bridge::CvImage(colour_ci->header,
                                                                  sensor_msgs::image_encodings::TYPE_8UC3,
                                                                  colour_mat).toImageMsg();
            sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(depth_ci->header,
                                                                 sensor_msgs::image_encodings::TYPE_16UC1,
                                                                 depth_mat).toImageMsg();
            sensor_msgs::ImagePtr aligned_msg = cv_bridge::CvImage(aligned_ci->header,
                                                                   sensor_msgs::image_encodings::TYPE_16UC1,
                                                                   aligned_depth_to_colour_mat).toImageMsg();

            this->colour_pub.publish(colour_msg, colour_ci);
            this->depth_pub.publish(depth_msg, depth_ci);
            this->aligned_pub.publish(aligned_msg, aligned_ci);
        }

        status = PsCloseDevice(this->device_index);
        cout << "CloseDevice status: " << status << endl;
        status = PsShutdown();
        cout << "Shutdown status: " << status << endl;
    }

private:
    static void PsExceptionOnFail(PsReturnStatus status, const std::string &message_on_fail) {
        if (status == PsReturnStatus::PsRetOK)
            return;
        ROS_ERROR(message_on_fail.c_str());
        throw std::runtime_error(message_on_fail);
    }

    std::string camera_name;
    ros::NodeHandle private_nh, colour_nh, depth_nh, aligned_nh;
    std::shared_ptr<image_transport::ImageTransport> colour_it, depth_it, aligned_it;
    std::shared_ptr<camera_info_manager::CameraInfoManager> colour_info, depth_info, aligned_info;

    image_transport::CameraPublisher colour_pub, depth_pub, aligned_pub;

    int32_t device_count = 0, device_index = 0, data_mode = PsDepthAndRGB_30;
    PsCameraParameters depth_intrinsics{}, colour_intrinsics{};
    PsCameraExtrinsicParameters extrinsics{};
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pico_zense_manager");

    PicoSenseManager manager = PicoSenseManager();
    manager.run();
    return 0;
}
