#include "pico_zense_manager.hpp"

PicoSenseManager::PicoSenseManager(int32_t device_index, const string &camera_name, uint16_t depth_threshold,
                                   double depth_range, int rgb_width, int rgb_height) :
        private_nh("~"),
        colour_nh_(camera_name + "/colour"),
        depth_nh_(camera_name + "/depth"),
        aligned_nh_(camera_name + "/aligned_depth_to_colour"),
        camera_name_(camera_name),
        colour_info_(new camera_info_manager::CameraInfoManager(colour_nh_)),
        depth_info_(new camera_info_manager::CameraInfoManager(depth_nh_)),
        aligned_info_(new camera_info_manager::CameraInfoManager(aligned_nh_)),
        colour_it_(new image_transport::ImageTransport(colour_nh_)),
        depth_it_(new image_transport::ImageTransport(depth_nh_)),
        aligned_it_(new image_transport::ImageTransport(aligned_nh_)),
        device_count_(0),
        depth_threshold_(depth_threshold),
        depth_range_(depth_range),
        rgb_width_(rgb_width),
        rgb_height_(rgb_height),
        slope_(1450),
        server(new dynamic_reconfigure::Server<pico_zense_camera::pico_zense_dcam710Config>(private_nh)) {
    signal(SIGSEGV, PicoSenseManager::sigsegv_handler);

    // Initialise the API
    PsExceptionOnFail(PsInitialize(), "PsInitialize failed!");

    // Get number of available devices
    PsExceptionOnFail(PsGetDeviceCount(&this->device_count_), "PsGetDeviceCount failed!");
    ROS_INFO("Get device count: %d", this->device_count_);

    // Verify device index selection
    this->device_index_ = device_index;
    if (this->device_index_ < 0 || this->device_index_ >= this->device_count_)
        throw std::runtime_error(
                "Device index outside of available devices range 0-" + std::to_string(this->device_count_));

    // Set distance confidence
    this->set_distance_confidence(this->depth_threshold_);

    //Set the Depth Range to Near through PsSetDepthRange interface
    this->set_range(this->depth_range_);

    // Attempt to open the device
    PsExceptionOnFail(PsOpenDevice(this->device_index_), "OpenDevice failed!");
    ROS_INFO("Successfully connected to device %d", this->device_index_);

    //Set PixelFormat as PsPixelFormatRGB888 for regular display
    PsExceptionOnFail(PsSetColorPixelFormat(this->device_index_, PsPixelFormatBGR888),
                      "Could not set pixel format");
    ROS_INFO("Set Pixel Format to BGR888 for device %d", this->device_index_);

    // Set to DepthAndRGB_30 mode
    PsExceptionOnFail(PsSetDataMode(this->device_index_, (PsDataMode) this->data_mode_),
                      "Set DataMode Failed failed!");
    ROS_INFO("Set capture mode to DepthAndRGB_30 for device %d", this->device_index_);

    // Set default resolution
    this->set_resolution(this->rgb_width_, this->rgb_height_);

    // Enable depth to rgb frame alignment in the API
    PsExceptionOnFail(PsSetMapperEnabledRGBToDepth(this->device_index_, true),
                      "Could not enable depth to rgb alignment");
    ROS_INFO("Enabled depth to rgb alignment for device %d", this->device_index_);

    // Set synchronisation between rgb and depth
    PsExceptionOnFail(PsSetSynchronizeEnabled(this->device_index_, true),
                      "Could not enable sensor synchronisation between rgb and depth sensors");
    ROS_INFO("Enabled sensor synchronisation for device %d", this->device_index_);

    // Set distortion correction for all sensors
    PsExceptionOnFail(PsSetDepthDistortionCorrectionEnabled(this->device_index_, true),
                      "Could not enable depth distortion correction");
    PsExceptionOnFail(PsSetRGBDistortionCorrectionEnabled(this->device_index_, true),
                      "Could not enable rgb distortion correction");
    PsExceptionOnFail(PsSetIrDistortionCorrectionEnabled(this->device_index_, true),
                      "Could not enable ir distortion correction");
    ROS_INFO("Enabled distortion correction for device %d", this->device_index_);

    // Set smoothing filter
    PsExceptionOnFail(PsSetFilter(this->device_index_, PsSmoothingFilter, true),
                      "Could not enable depth smoothing filter");
    ROS_INFO("Enabled smoothing filter for device %d", this->device_index_);

    // Set intrinsics/extrinsics
    this->set_sensor_intrinsics();

    // Initialise dynamic reconfigure server
    this->initialise_dynamic_reconfigure_server();
}

void PicoSenseManager::set_resolution(int width, int height) {
    std::string message("Resolution " + to_string(width) + "x" + to_string(height));

    PsFrameMode frame_mode;
    frame_mode.fps = 30;
    frame_mode.pixelFormat = PsPixelFormatBGR888;

    int valid_resolutions[4][2]{{1920, 1080},
                                {1280, 720},
                                {640,  480},
                                {640,  360}};
    bool matched_resolution = false;

    for (auto &dims : valid_resolutions) {
        int v_width = dims[0], v_height = dims[1];
        if (width == v_width && height == v_height) {
            frame_mode.resolutionWidth = v_width;
            frame_mode.resolutionHeight = v_height;
            matched_resolution = true;
        }
    }

    if (!matched_resolution) {
        message += " not valid. Valid resolutions 1920x1080, 1280x720, 640x(480|360)";
        ROS_ERROR(message.c_str());
        throw std::runtime_error(message);
    }

    PsExceptionOnFail(PsSetFrameMode(this->device_index_, PsRGBFrame, &frame_mode),
                      "Could not set " + message + " for device.");
    this->rgb_width_ = frame_mode.resolutionWidth;
    this->rgb_height_ = frame_mode.resolutionHeight;

    message += " has been set for device " + to_string(this->device_index_);
    ROS_INFO(message.c_str());
}

void PicoSenseManager::set_range(double range) {
    if (range < 0 || range > 5.6) {
        string message("Please set a range between 0-15m");
        ROS_ERROR(message.c_str());
        throw std::runtime_error(message);
    }

    string range_identifiers[9]{"PsNearRange", "PsMidRange", "PsFarRange", "PsXNearRange", "PsXMidRange",
                                "PsXFarRange", "PsXXNearRange", "PsXXMidRange", "PsXXFarRange"};
    double ranges[9][2]{{-1,   1.45},
                        {1.45, 3.0},
                        {3.0,  4.4},
                        {4.4,  4.8},
                        {4.8,  5.6},
                        {5.6,  7.5},
                        {7.5,  9.6},
                        {9.6,  11.2},
                        {11.2, 15.0}};
    uint16_t slopes[9]{1450, 3000, 4400, 4800, 5600, 7500, 9600, 11200, 15000};

    PsDepthRange depth_range = static_cast<PsDepthRange>(0);
    uint16_t new_slope = slopes[0];
    string identifier = range_identifiers[0];

    for (int i = 0; i < 9; ++i) {
        double min_range = ranges[i][0], max_range = ranges[i][1];
        identifier = range_identifiers[i];
        if (range > min_range && range <= max_range) {
            depth_range = static_cast<PsDepthRange>(i);
            new_slope = slopes[i];
            break;
        }
    }

    // Set depth threshold
    this->slope_ = new_slope;
    PsExceptionOnFail(PsSetDepthRange(this->device_index_, depth_range), "Could not set depth range " + identifier);
    ROS_INFO("Set depth range %s at max distance %.2fm", identifier.c_str(), (double) this->slope_ / 1000.0);
}

void PicoSenseManager::set_distance_confidence(uint16_t threshold) {
    if (threshold < 0 || threshold > 100) {
        string message("Cannot set threshold greater that 100%");
        ROS_ERROR(message.c_str());
        throw std::runtime_error(message);
    }
    this->depth_threshold_ = threshold;
    PsExceptionOnFail(PsSetThreshold(this->device_index_, this->depth_threshold_), "Could not set depth threshold");
    ROS_INFO("Enabled depth confidence threshold at %d% (device %d)", this->depth_threshold_, this->device_index_);
}

void PicoSenseManager::run() {
    // Initialise ROS nodes
    sensor_msgs::CameraInfoPtr colour_ci(new sensor_msgs::CameraInfo(colour_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr depth_ci(new sensor_msgs::CameraInfo(depth_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr aligned_ci(new sensor_msgs::CameraInfo(aligned_info_->getCameraInfo()));

    this->colour_pub_ = this->colour_it_->advertiseCamera("image_raw", 30);
    this->depth_pub_ = this->depth_it_->advertiseCamera("image_raw", 30);
    this->aligned_pub_ = this->aligned_it_->advertiseCamera("image_raw", 30);
    auto aligned_depth_vis_pub = this->aligned_it_->advertise("colourmap_jet", 30);

    // Containers for frames
    PsReturnStatus status;
    PsFrame depth_frame = {0};
    PsFrame colour_frame = {0};
    PsFrame aligned_depth_to_colour_frame = {0};
    cv::Mat colour_mat, depth_mat, aligned_depth_to_colour_mat, aligned_depth_vis_mat;

    while (ros::ok()) {
        // Get next frame set
        status = PsReadNextFrame(this->device_index_);
        if (status != PsRetOK) {
            ROS_WARN("Could not get next frame set from device %d", this->device_index_);
            continue;
        }

        // Get frames
        PsGetFrame(this->device_index_, PsRGBFrame, &colour_frame);
        PsGetFrame(this->device_index_, PsDepthFrame, &depth_frame);
        PsGetFrame(this->device_index_, PsMappedDepthFrame, &aligned_depth_to_colour_frame);

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
        aligned_depth_vis_mat = cv::Mat(aligned_depth_to_colour_frame.height,
                                        aligned_depth_to_colour_frame.width,
                                        CV_16UC1, aligned_depth_to_colour_frame.pFrameData);
        aligned_depth_vis_mat.convertTo(aligned_depth_vis_mat, CV_8U, 255.0 / this->slope_);
        applyColorMap(aligned_depth_vis_mat, aligned_depth_vis_mat, cv::COLORMAP_JET);

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
        sensor_msgs::ImagePtr aligned_vis_msg = cv_bridge::CvImage(aligned_ci->header,
                                                                   sensor_msgs::image_encodings::TYPE_8UC3,
                                                                   aligned_depth_vis_mat).toImageMsg();
        this->colour_pub_.publish(colour_msg, colour_ci);
        this->depth_pub_.publish(depth_msg, depth_ci);
        this->aligned_pub_.publish(aligned_msg, aligned_ci);
        aligned_depth_vis_pub.publish(aligned_vis_msg);

        // Process any callbacks
        ros::spinOnce();
    }

    status = PsCloseDevice(this->device_index_);
    ROS_INFO("CloseDevice status: %d", status);
    status = PsShutdown();
    ROS_INFO("Shutdown status: %d", status);
}

void PicoSenseManager::sigsegv_handler(int sig) {
    signal(SIGSEGV, SIG_DFL);
    ROS_ERROR("Segmentation fault, stopping camera driver (%d).", sig);
    ros::shutdown();
}

void PicoSenseManager::PsExceptionOnFail(PsReturnStatus status, const std::string &message_on_fail) {
    if (status == PsReturnStatus::PsRetOK)
        return;
    ROS_ERROR(message_on_fail.c_str());
    throw std::runtime_error(message_on_fail);
}

void PicoSenseManager::set_sensor_intrinsics() {
    std::string camera_frame(this->camera_name_ + "_frame"), colour_frame(this->camera_name_ + "_colour_frame"),
            aligned_frame(this->camera_name_ + "_aligned_depth_to_colour_frame"),
            depth_frame(this->camera_name_ + "_depth_frame");

    // Get camera parameters (extrinsic)
    PsExceptionOnFail(PsGetCameraExtrinsicParameters(this->device_index_, &this->extrinsics_),
                      "Could not get extrinsics!");

    // Setup tf broadcaster
    static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    ros::Time now = ros::Time::now();

    // PsCameraExtrinsicParameters to ROS transform
    tf::Transform transform;
    tf::Matrix3x3 rotation_matrix(extrinsics_.rotation[0], extrinsics_.rotation[1], extrinsics_.rotation[2],
                                  extrinsics_.rotation[3], extrinsics_.rotation[4], extrinsics_.rotation[5],
                                  extrinsics_.rotation[6], extrinsics_.rotation[7], extrinsics_.rotation[8]);
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
    msg.transform.translation.x = extrinsics_.translation[0] / 1000;
    msg.transform.translation.y = extrinsics_.translation[1] / 1000;
    msg.transform.translation.z = extrinsics_.translation[2] / 1000;
    msg.transform.rotation = orientation;
    msg.header.frame_id = colour_frame;
    msg.child_frame_id = depth_frame;
    tf_broadcaster.sendTransform(msg);

    // Get camera parameters (intrinsic)
    PsExceptionOnFail(PsGetCameraParameters(this->device_index_, PsDepthSensor, &this->depth_intrinsics_),
                      "Could not get depth intrinsics!");
    PsExceptionOnFail(PsGetCameraParameters(this->device_index_, PsRgbSensor, &this->colour_intrinsics_),
                      "Could not get rgb intrinsics!");

    // Initialise camera info messages
    sensor_msgs::CameraInfo info_msg;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = colour_frame;
    info_msg.D = {colour_intrinsics_.k1, colour_intrinsics_.k2, colour_intrinsics_.p1, colour_intrinsics_.p2,
                  colour_intrinsics_.k3};
    info_msg.K = {colour_intrinsics_.fx, 0, colour_intrinsics_.cx, 0, colour_intrinsics_.fy,
                  colour_intrinsics_.cy, 0, 0, 1};
    info_msg.P = {colour_intrinsics_.fx, 0, colour_intrinsics_.cx, 0, 0, colour_intrinsics_.fy,
                  colour_intrinsics_.cy, 0, 0, 0, 1, 0};
    colour_info_->setCameraInfo(info_msg);
    aligned_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth_frame;
    info_msg.D = {depth_intrinsics_.k1, depth_intrinsics_.k2, depth_intrinsics_.p1, depth_intrinsics_.p2,
                  depth_intrinsics_.k3};
    info_msg.K = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx, 0, depth_intrinsics_.fy, depth_intrinsics_.cy, 0,
                  0, 1};
    info_msg.P = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx, 0, 0, depth_intrinsics_.fy, depth_intrinsics_.cy,
                  0, 0, 0, 1, 0};
    depth_info_->setCameraInfo(info_msg);

    ROS_INFO("Successfully received intrinsic and extrinsic parameters for device %d", this->device_index_);
}

void
PicoSenseManager::dynamic_reconfigure_callback(pico_zense_camera::pico_zense_dcam710Config &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: Depth Confidence threshold: %d% Depth range: %.2f RGB Resolution: %s",
             config.depth_confidence_threshold, config.depth_range, config.rgb_resolution.c_str());


    // Split resolution string ("WidthxHeight") into ints
    std::string segment;
    std::stringstream test(config.rgb_resolution);
    std::vector<std::string> token_seg_list;
    while(std::getline(test, segment, 'x'))
        token_seg_list.push_back(segment);
    if(token_seg_list.size() == 2) {
        int width = std::stoi(token_seg_list[0]), height = std::stoi(token_seg_list[1]);
        if(this->rgb_width_ != width || this->rgb_height_ != height)
            this->set_resolution(width, height);
    }

    if (config.depth_range != this->depth_range_)
        this->set_range(config.depth_range);
    if (config.depth_confidence_threshold != this->depth_threshold_)
        this->set_distance_confidence(config.depth_confidence_threshold);
}

void PicoSenseManager::initialise_dynamic_reconfigure_server() {
    pico_zense_camera::pico_zense_dcam710Config config;
    config.rgb_resolution = to_string(this->rgb_width_) + "x" + to_string(this->rgb_height_);
    config.depth_range = this->depth_range_;
    config.depth_confidence_threshold = this->depth_threshold_;
    this->server->updateConfig(config);
    this->reconfigure_callback = boost::bind(&PicoSenseManager::dynamic_reconfigure_callback, this, _1, _2);
    this->server->setCallback(reconfigure_callback);
}
