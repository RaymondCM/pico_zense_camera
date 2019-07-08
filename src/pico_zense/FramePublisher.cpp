#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "PicoZense_api.h"

using namespace std;
using namespace cv;

static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t *pData, cv::Mat &dispImg) {
    dispImg = cv::Mat(height, width, CV_16UC1, pData);
    Point2d pointxy(width / 2, height / 2);
    int val = dispImg.at<ushort>(pointxy);
    char text[20];
#ifdef _WIN32
    sprintf_s(text, "%d", val);
#else
    snprintf(text, sizeof(text), "%d", val);
#endif
    dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
    applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
    int color;
    if (val > 2500)
        color = 0;
    else
        color = 4096;
    circle(dispImg, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
    putText(dispImg, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
}

class PicoSenseManager {
public:
    explicit PicoSenseManager(int32_t device_index = 0) {
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

        // Enable depth to rgb frame alignment in the API
        PsExceptionOnFail(PsSetMapperEnabledRGBToDepth(this->device_index, true),
                          "Could not enable depth to rgb alignment");

        // Get camera parameters (intrinsic and extrinsic)
        PsExceptionOnFail(PsGetCameraParameters(this->device_index, PsDepthSensor, &this->depth_intrinsics),
                          "Could not get depth intrinsics!");
        PsExceptionOnFail(PsGetCameraParameters(this->device_index, PsRgbSensor, &this->rgb_intrinsics),
                          "Could not get rgb intrinsics!");

        PsExceptionOnFail(PsGetCameraExtrinsicParameters(this->device_index, &this->extrinsics),
                          "Could not get extrinsics!");

        // Initialise camera info messages TODO: Change to camera_info_manager so parameters can be dynamically configured
        rgb_camera_info.D = {rgb_intrinsics.k1, rgb_intrinsics.k2, rgb_intrinsics.p1, rgb_intrinsics.p2, rgb_intrinsics.k3};
        rgb_camera_info.K = boost::array<double, 9>{rgb_intrinsics.fx, 0, rgb_intrinsics.cx, 0, rgb_intrinsics.fy, rgb_intrinsics.cy, 0, 0, 1};
        rgb_camera_info.P = boost::array<double, 12>{rgb_intrinsics.fx, 0, rgb_intrinsics.cx, 0, 0, rgb_intrinsics.fy, rgb_intrinsics.cy, 0, 0, 0, 1, 0};

        ROS_INFO("Successfully received intrinsic and extrinsic parameters for device %d", this->device_index);
    };

    void run() {
        // Initialise ROS nodes
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);

        image_transport::Publisher rgb_pub = it.advertise("/pico/color/image_raw", 30);
        image_transport::Publisher depth_pub = it.advertise("/pico/depth/image_raw", 30);
        image_transport::Publisher aligned_pub = it.advertise("/pico/aligned_depth_to_color/image_raw", 30);
//        image_transport::CameraPublisher rgb_pub = it.advertiseCamera("/pico/color/image_raw", 30);
//        image_transport::CameraPublisher depth_pub = it.advertiseCamera("/pico/depth/image_raw", 30);
//        image_transport::CameraPublisher aligned_pub = it.advertiseCamera("/pico/aligned_depth_to_color/image_raw", 30);

        // Containers for frames
        PsReturnStatus status;
        PsFrame depth_frame = {0};
        PsFrame rgb_frame = {0};
        PsFrame aligned_depth_to_rgb_frame = {0};
        cv::Mat rgb_mat, depth_mat, aligned_depth_to_rgb_mat;

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
            PsGetFrame(this->device_index, PsRGBFrame, &rgb_frame);
            PsGetFrame(this->device_index, PsDepthFrame, &depth_frame);
            PsGetFrame(this->device_index, PsMappedDepthFrame, &aligned_depth_to_rgb_frame);

            // Ensure all frames have arrived
            if (rgb_frame.pFrameData == NULL || depth_frame.pFrameData == NULL ||
                aligned_depth_to_rgb_frame.pFrameData == NULL) {
                ROS_WARN("Could not get frames: rgb=%s, depth=%s, aligned_depth_to_rgb=%s",
                         rgb_frame.pFrameData == NULL ? "failed" : "passed",
                         depth_frame.pFrameData == NULL ? "failed" : "passed",
                         aligned_depth_to_rgb_frame.pFrameData == NULL ? "failed" : "passed");
                continue;
            }

            // Convert struct buffers to OpenCV mats for publishing with cv::bridge
            rgb_mat = cv::Mat(rgb_frame.height, rgb_frame.width, CV_8UC3, rgb_frame.pFrameData);
            depth_mat = cv::Mat(depth_frame.height, depth_frame.width, CV_16UC1, depth_frame.pFrameData);
            aligned_depth_to_rgb_mat = cv::Mat(aligned_depth_to_rgb_frame.height, aligned_depth_to_rgb_frame.width,
                                               CV_16UC1, aligned_depth_to_rgb_frame.pFrameData);

            // Publish messages and camera info
            std_msgs::Header header{};
            ros::Time now = header.stamp;
            sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3, rgb_mat).toImageMsg();
            sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depth_mat).toImageMsg();
            sensor_msgs::ImagePtr aligned_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, aligned_depth_to_rgb_mat).toImageMsg();

            rgb_pub.publish(rgb_msg);
            depth_pub.publish(depth_msg);
            aligned_pub.publish(aligned_msg);
//            rgb_pub.publish(rgb_msg, rgb_camera_info, now);
//            depth_pub.publish(depth_msg, info, now);
//            aligned_pub.publish(aligned_msg, info, now);
        }
    }

private:
    static void PsExceptionOnFail(PsReturnStatus status, const std::string &message_on_fail) {
        if (status == PsReturnStatus::PsRetOK)
            return;
        ROS_ERROR(message_on_fail.c_str());
        throw std::runtime_error(message_on_fail);
    }

    int32_t device_count = 0, device_index = 0, data_mode = PsDepthAndRGB_30;
    PsCameraParameters depth_intrinsics{}, rgb_intrinsics{};
    sensor_msgs::CameraInfo rgb_camera_info, depth_camera_info, aligned_info;
    PsCameraExtrinsicParameters extrinsics{};
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pico_zense_manager");

    PicoSenseManager manager = PicoSenseManager();
    manager.run();
    return 0;
}
