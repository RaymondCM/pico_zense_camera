from __future__ import absolute_import, division, print_function

from threading import Lock, Thread, Event
from timeit import default_timer as timer

import message_filters
import rospy
from dynamic_reconfigure.client import Client as DynClient
from realsense2_camera.msg import Extrinsics
from sensor_msgs.msg import Image, CameraInfo

try:
    import typing
except ImportError:
    pass


class TopicInfo:
    def __init__(self, topic_name, topic_type):
        self.topic = topic_name
        self.type = topic_type

    def as_list(self):
        return [self.topic, self.type]


class RealSenseTopics:
    def __init__(self, camera_name=None):
        self.rgb = TopicInfo("/color/image_raw", Image)
        self.rgb_intrinsics = TopicInfo("/color/camera_info", CameraInfo)
        self.depth = TopicInfo("/depth/image_rect_raw", Image)
        self.depth_intrinsics = TopicInfo("/depth/camera_info", CameraInfo)
        self.aligned_depth_to_rgb = TopicInfo("/aligned_depth_to_color/image_raw", Image)
        self.aligned_depth_to_rgb_intrinsics = TopicInfo("/aligned_depth_to_color/camera_info", CameraInfo)
        self.aligned_depth_to_ir_left = TopicInfo("/aligned_depth_to_infra1/image_raw", Image)
        self.aligned_depth_to_ir_left_intrinsics = TopicInfo("/aligned_depth_to_infra1/camera_info", CameraInfo)
        self.ir_left = TopicInfo("/infra1/image_rect_raw", Image)
        self.ir_left_intrinsics = TopicInfo("/infra1/camera_info", CameraInfo)
        self.ir_right = TopicInfo("/infra2/image_rect_raw", Image)
        self.ir_right_intrinsics = TopicInfo("/infra2/camera_info", CameraInfo)
        self.depth_to_color_extrinsics = TopicInfo("/extrinsics/depth_to_color", Extrinsics)
        self.depth_to_ir_left_extrinsics = TopicInfo("/extrinsics/depth_to_infra1", Extrinsics)
        self.depth_to_ir_right_extrinsics = TopicInfo("/extrinsics/depth_to_infra2", Extrinsics)

        if camera_name:
            for k, v in self.__dict__.items():
                if isinstance(v, TopicInfo):
                    v.topic = "/{}{}".format(camera_name, v.topic)


class RealSenseSensorData:
    def __init__(self, rgb=None, rgb_intrinsics=None, depth_to_color_extrinsics=None, depth=None, depth_intrinsics=None,
                 aligned_depth_to_rgb=None, aligned_depth_to_rgb_intrinsics=None, aligned_depth_to_ir_left=None,
                 aligned_depth_to_ir_left_intrinsics=None, ir_left=None, ir_left_intrinsics=None, ir_right=None,
                 ir_right_intrinsics=None, depth_to_ir_left_extrinsics=None, depth_to_ir_right_extrinsics=None):
        self.rgb = rgb
        self.rgb_intrinsics = rgb_intrinsics
        self.depth_to_color_extrinsics = depth_to_color_extrinsics

        self.depth = depth
        self.depth_intrinsics = depth_intrinsics

        self.aligned_depth_to_rgb = aligned_depth_to_rgb
        self.aligned_depth_to_rgb_intrinsics = aligned_depth_to_rgb_intrinsics
        self.aligned_depth_to_ir_left = aligned_depth_to_ir_left
        self.aligned_depth_to_ir_left_intrinsics = aligned_depth_to_ir_left_intrinsics

        self.ir_left = ir_left
        self.ir_left_intrinsics = ir_left_intrinsics
        self.ir_right = ir_right
        self.ir_right_intrinsics = ir_right_intrinsics
        self.depth_to_ir_left_extrinsics = depth_to_ir_left_extrinsics
        self.depth_to_ir_right_extrinsics = depth_to_ir_right_extrinsics


class RealSenseSubscriber:
    def __init__(self, camera_name, serial="", callback=None, rgb=True, depth=True, aligned_depth=True, ir=True,
                 sync_queue_size=10, sync_thresh=0.5, timeout=60):
        # type: (str, str, typing.Callable, bool, bool, bool, bool, int, float, float) -> None
        self.__sync_queue_size = sync_queue_size
        self.__sync_thresh = sync_thresh
        self.__timeout = timeout
        self.__callback = callback if callable(callback) else None
        self.camera_name = camera_name
        self.camera_serial = serial

        # Create data class to store most recent version of the data
        self.latest_data = RealSenseSensorData()

        # Create lookup of all topics
        self.camera_topics = RealSenseTopics(self.camera_name)

        # Validate requested sensor topics exist
        rgb, depth, aligned_depth, ir = self.__wait_for_enabled_sensors(self.camera_name, rgb, depth, aligned_depth, ir,
                                                                        self.__timeout)

        # Configure subscribers and get one time messages such as realsense-ros Extrinsics
        self.__time_sync_arg_keys = []
        subscribers = []

        if rgb:
            self.__time_sync_arg_keys.append("rgb")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.rgb.as_list()))
            self.__time_sync_arg_keys.append("rgb_intrinsics")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.rgb_intrinsics.as_list()))
            if depth:
                self.latest_data.depth_to_color_extrinsics = self.__get_single_message(
                    *self.camera_topics.depth_to_color_extrinsics.as_list())
        if depth:
            self.__time_sync_arg_keys.append("depth")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.depth.as_list()))
            self.__time_sync_arg_keys.append("depth_intrinsics")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.depth_intrinsics.as_list()))
        if aligned_depth and rgb:
            self.__time_sync_arg_keys.append("aligned_depth_to_rgb")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.aligned_depth_to_rgb.as_list()))
            self.__time_sync_arg_keys.append("aligned_depth_to_rgb_intrinsics")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.aligned_depth_to_rgb_intrinsics.as_list()))
            if ir:
                self.__time_sync_arg_keys.append("aligned_depth_to_ir_left")
                subscribers.append(message_filters.Subscriber(*self.camera_topics.aligned_depth_to_ir_left.as_list()))
                self.__time_sync_arg_keys.append("aligned_depth_to_ir_left_intrinsics")
                subscribers.append(
                    message_filters.Subscriber(*self.camera_topics.aligned_depth_to_ir_left_intrinsics.as_list()))
        if ir:
            self.__time_sync_arg_keys.append("ir_left")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.ir_left.as_list()))
            self.__time_sync_arg_keys.append("ir_left_intrinsics")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.ir_left_intrinsics.as_list()))
            self.__time_sync_arg_keys.append("ir_right")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.ir_right.as_list()))
            self.__time_sync_arg_keys.append("ir_right_intrinsics")
            subscribers.append(message_filters.Subscriber(*self.camera_topics.ir_right_intrinsics.as_list()))
            if depth:
                self.latest_data.depth_to_ir_left_extrinsics = self.__get_single_message(
                    *self.camera_topics.depth_to_ir_left_extrinsics.as_list())
                self.latest_data.depth_to_ir_right_extrinsics = self.__get_single_message(
                    *self.camera_topics.depth_to_ir_right_extrinsics.as_list())

        self.rw_lock = Lock()
        self.__data_event = Event()

        self.subscribers = subscribers
        self.time_syncroniser = message_filters.ApproximateTimeSynchronizer(self.subscribers, self.__sync_queue_size,
                                                                            self.__sync_thresh)
        self.time_syncroniser.registerCallback(self.__save_subscriber_result)

    @staticmethod
    def __get_single_message(topic, message_type, timeout=5.0):
        # type: (str, typing.Any, float) -> typing.Union[None, typing.Any]
        try:
            message = rospy.wait_for_message(topic, message_type, timeout=timeout)
            return message
        except rospy.ROSException as e:
            raise rospy.ROSException("Timeout exceeded waiting for {}\n\t{}".format(topic, e))

    @staticmethod
    def __get_relevant_published_topics(namespace):
        rs_topics = RealSenseTopics()  # List of valid message topics
        all_topics = [v[0] for v in rospy.get_published_topics(namespace)]
        valid_topics = [cls.topic for k, cls in rs_topics.__dict__.items() if isinstance(cls, TopicInfo)]
        filtered_topics = [k for k in all_topics if any(vt for vt in valid_topics if vt in k)]
        return filtered_topics

    def __wait_for_enabled_sensors(self, camera_name, rgb_enabled, depth_enabled, aligned_depth_enabled, ir_enabled, timeout):
        # type: (str, bool, bool, bool, bool, float) -> (bool, bool, bool, bool)
        expected_topics_state = [rgb_enabled, depth_enabled, aligned_depth_enabled, ir_enabled]
        actual_topic_state = []

        topic_list = RealSenseSubscriber.__get_relevant_published_topics(camera_name)
        start, rate = timer(), rospy.Rate(10)

        # Wait for timeout seconds for at least one relevant topic to exist and sensors to be set to correct state
        while not len(topic_list) or actual_topic_state != expected_topics_state:
            actual_topic_state = list(self.__get_enabled_sensors(camera_name, *expected_topics_state))
            topic_list = RealSenseSubscriber.__get_relevant_published_topics(camera_name)
            if timer() - start > timeout:
                msg = "No topics available in namespace '{}', timeout reached:\n\t".format(camera_name) + ", ".join(
                    k for v, k in zip(actual_topic_state, ["RGB", "Depth", "Aligned Depth", "IR"]) if not v) + \
                      " sensor topics not published."
                raise rospy.ROSException(msg)
            rate.sleep()

        rgb_enabled, depth_enabled, aligned_depth_enabled, ir_enabled = actual_topic_state
        return rgb_enabled, depth_enabled, aligned_depth_enabled, ir_enabled

    def __get_enabled_sensors(self, camera_name, rgb, depth, aligned_depth, ir):
        # type: (str, bool, bool, bool, bool) -> (bool, bool, bool, bool)
        # For each enabled sensor verify that the sensor topics exist currently
        topic_list = RealSenseSubscriber.__get_relevant_published_topics(camera_name)

        if rgb:
            rgb = self.camera_topics.rgb.topic in topic_list and self.camera_topics.rgb_intrinsics.topic in topic_list
            if depth:
                rgb = rgb and self.camera_topics.depth_to_color_extrinsics.topic in topic_list

        if depth:
            depth = self.camera_topics.depth.topic in topic_list and self.camera_topics.depth_intrinsics.topic in topic_list

        if aligned_depth:
            aligned_depth = self.camera_topics.aligned_depth_to_rgb.topic in topic_list and \
                            self.camera_topics.aligned_depth_to_rgb_intrinsics.topic in topic_list
            if ir:
                aligned_depth = aligned_depth and self.camera_topics.aligned_depth_to_ir_left.topic in topic_list and \
                                self.camera_topics.aligned_depth_to_ir_left_intrinsics.topic in topic_list

        if ir:
            ir = self.camera_topics.ir_left.topic in topic_list and \
                 self.camera_topics.ir_left_intrinsics.topic in topic_list and \
                 self.camera_topics.ir_right.topic in topic_list and \
                 self.camera_topics.ir_right_intrinsics.topic in topic_list
            if depth:
                ir = ir and self.camera_topics.depth_to_ir_left_extrinsics.topic in topic_list and \
                     self.camera_topics.depth_to_ir_right_extrinsics.topic in topic_list

        return rgb, depth, aligned_depth, ir

    def __save_subscriber_result(self, *args):
        if len(self.__time_sync_arg_keys) != len(args):
            raise rospy.ROSException("RealSenseSubscriber synchronisation callback is incorrectly configured")
        with self.rw_lock:
            for k, v in zip(self.__time_sync_arg_keys, args):
                # if isinstance(v, Image):
                #     v = ros_numpy.numpify(v)
                # elif isinstance(v, CameraInfo):
                #     v = {"distortion_model": v.distortion_model, "distortion_parameters": list(v.D),
                #          "intrinsic_matrix": list(v.K), "rectification_matrix": list(v.R),
                #          "projection_matrix": list(v.P)}
                setattr(self.latest_data, k, v)
            if self.__callback:
                Thread(target=self.__callback, args=(self.latest_data,)).start()
            self.__data_event.set()

    def get_latest_data(self):
        # type: () -> RealSenseSensorData
        with self.rw_lock:
            return self.latest_data

    def wait_for_message(self, timeout=5.0):
        # type: (float) -> RealSenseSensorData
        self.__data_event.clear()
        if not self.__data_event.wait(timeout):
            raise rospy.ROSException("Timeout reached waiting for camera '{}'".format(self.camera_name))
        with self.rw_lock:
            return self.latest_data


class RealSenseConfigurator:
    __timeout = 30

    def __init__(self, camera_name):
        self.__camera_name = camera_name

        # Connect reconfigure clients
        self.__rgb_module = DynClient(self.__camera_name + "/rgb_camera", timeout=self.__timeout)
        self.__depth_module = DynClient(self.__camera_name + "/stereo_module", timeout=self.__timeout)

    def set_rgb_config_callback(self, callback):
        # type: (typing.Callable) -> None
        self.__rgb_module.set_config_callback(callback)

    def set_depth_config_callback(self, callback):
        # type: (typing.Callable) -> None
        self.__depth_module.set_config_callback(callback)

    def update_rgb_parameter(self, name, value):
        # type: (str, typing.Any) -> None
        self.__update_parameter(self.__rgb_module, name, value)

    def update_depth_parameter(self, name, value):
        # type: (str, typing.Any) -> None
        self.__update_parameter(self.__depth_module, name, value)

    def get_rgb_parameter(self, name):
        # type: (str) -> typing.Union[typing.Any, None]
        return self.__get_parameter(self.__rgb_module, name)

    def get_depth_parameter(self, name):
        # type: (str) -> typing.Union[typing.Any, None]
        return self.__get_parameter(self.__depth_module, name)

    def get_rgb_parameter_info(self, name):
        # type: (str) -> typing.Union[dict, None]
        return self.__get_parameter_info(self.__rgb_module, name)

    def get_depth_parameter_info(self, name):
        # type: (str) -> typing.Union[dict, None]
        return self.__get_parameter_info(self.__depth_module, name)

    def __get_description(self, module, name, value=None):
        # type: (DynClient, str, typing.Optional[typing.Any]) -> typing.Union[dict, None]
        # Wait for module parameter_description and types to become available if not available
        if not module.param_description or not module._param_types:
            start_time = timer()
            while not module.param_description or not module._param_types:
                if timer() - start_time >= self.__timeout:
                    raise rospy.ROSException("Timeout reached waiting for parameter description")

        if name not in module._param_types:
            _available_parameters = ", ".join(module._param_types.keys())
            rospy.logerr("Parameter '{}' not in available parameters '{}'".format(name, _available_parameters))
            return None

        if value is not None:
            _param_type = module._param_types[name]
            if not isinstance(value, _param_type):
                rospy.logerr("Parameter '{}' is '{}' when it should be '{}'".format(name, type(value), _param_type))
                return None

        _param_descriptions = {d['name']: d for d in module.param_description}
        return _param_descriptions

    def __update_parameter(self, module, name, value):
        # type: (DynClient, str, typing.Any) -> None
        _param_descriptions = self.__get_description(module, name, value)
        if not _param_descriptions:
            return

        _min_v, _max_v = _param_descriptions[name]['min'], _param_descriptions[name]['max']
        if not isinstance(module._param_types[name], bool) and _max_v >= value <= _min_v:
            rospy.logerr("Cannot set parameter to '{}' valid range is '{}'-'{}'".format(value, _min_v, _max_v))
            return

        module.update_configuration({name: value})

    def __get_parameter(self, module, name):
        # type: (DynClient, str) -> typing.Union[typing.Any, None]
        _param_descriptions = self.__get_description(module, name)
        if not _param_descriptions:
            return None

        if not hasattr(module.config, name):
            start_time = timer()
            while not hasattr(module.config, name):
                if timer() - start_time >= self.__timeout:
                    rospy.logerr("Parameter '{}' does not exist in current config".format(name))
                    return None

        return getattr(module.config, name)

    def __get_parameter_info(self, module, name):
        # type: (DynClient, str) -> typing.Union[dict, None]
        _param_descriptions = self.__get_description(module, name)
        if not _param_descriptions:
            return None
        return _param_descriptions[name]

    def set_laser_power(self, value):
        self.update_depth_parameter('laser_power', value)

    def get_laser_power(self):
        return self.get_depth_parameter('laser_power')

    def get_laser_power_info(self):
        return self.get_depth_parameter_info('laser_power')

    def set_output_trigger_enabled(self, value):
        self.update_depth_parameter('output_trigger_enabled', value)

    def get_output_trigger_enabled(self):
        return self.get_depth_parameter('output_trigger_enabled')

    def get_output_trigger_enabled_info(self):
        return self.get_depth_parameter_info('output_trigger_enabled')

    def set_emitter_enabled(self, value):
        self.update_depth_parameter('emitter_enabled', value)

    def get_emitter_enabled(self):
        return self.get_depth_parameter('emitter_enabled')

    def get_emitter_enabled_info(self):
        return self.get_depth_parameter_info('emitter_enabled')

    def set_enable_depth_auto_exposure(self, value):
        self.update_depth_parameter('enable_auto_exposure', value)

    def get_enable_depth_auto_exposure(self):
        return self.get_depth_parameter('enable_auto_exposure')

    def get_enable_depth_auto_exposure_info(self):
        return self.get_depth_parameter_info('enable_auto_exposure')

    def set_global_depth_time_enabled(self, value):
        self.update_depth_parameter('global_time_enabled', value)

    def get_global_depth_time_enabled(self):
        return self.get_depth_parameter('global_time_enabled')

    def get_global_depth_time_enabled_info(self):
        return self.get_depth_parameter_info('global_time_enabled')

    def set_inter_cam_sync_mode(self, value):
        self.update_depth_parameter('inter_cam_sync_mode', value)

    def get_inter_cam_sync_mode(self):
        return self.get_depth_parameter('inter_cam_sync_mode')

    def get_inter_cam_sync_mode_info(self):
        return self.get_depth_parameter_info('inter_cam_sync_mode')

    def set_visual_preset(self, value):
        self.update_depth_parameter('visual_preset', value)

    def get_visual_preset(self):
        return self.get_depth_parameter('visual_preset')

    def get_visual_preset_info(self):
        return self.get_depth_parameter_info('visual_preset')

    def set_error_polling_enabled(self, value):
        self.update_depth_parameter('error_polling_enabled', value)

    def get_error_polling_enabled(self):
        return self.get_depth_parameter('error_polling_enabled')

    def get_error_polling_enabled_info(self):
        return self.get_depth_parameter_info('error_polling_enabled')

    def set_depth_gain(self, value):
        self.update_depth_parameter('gain', value)

    def get_depth_gain(self):
        return self.get_depth_parameter('gain')

    def get_depth_gain_info(self):
        return self.get_depth_parameter_info('gain')

    def set_depth_frames_queue_size(self, value):
        self.update_depth_parameter('frames_queue_size', value)

    def get_depth_frames_queue_size(self):
        return self.get_depth_parameter('frames_queue_size')

    def get_depth_frames_queue_size_info(self):
        return self.get_depth_parameter_info('frames_queue_size')

    def set_emitter_on_off(self, value):
        self.update_depth_parameter('emitter_on_off', value)

    def get_emitter_on_off(self):
        return self.get_depth_parameter('emitter_on_off')

    def get_emitter_on_off_info(self):
        return self.get_depth_parameter_info('emitter_on_off')

    def set_depth_exposure(self, value):
        self.update_depth_parameter('exposure', value)

    def get_depth_exposure(self):
        return self.get_depth_parameter('exposure')

    def get_depth_exposure_info(self):
        return self.get_depth_parameter_info('exposure')

    def set_gamma(self, value):
        self.update_rgb_parameter('gamma', value)

    def get_gamma(self):
        return self.get_rgb_parameter('gamma')

    def get_gamma_info(self):
        return self.get_rgb_parameter_info('gamma')

    def set_saturation(self, value):
        self.update_rgb_parameter('saturation', value)

    def get_saturation(self):
        return self.get_rgb_parameter('saturation')

    def get_saturation_info(self):
        return self.get_rgb_parameter_info('saturation')

    def set_brightness(self, value):
        self.update_rgb_parameter('brightness', value)

    def get_brightness(self):
        return self.get_rgb_parameter('brightness')

    def get_brightness_info(self):
        return self.get_rgb_parameter_info('brightness')

    def set_enable_rgb_auto_exposure(self, value):
        self.update_rgb_parameter('enable_auto_exposure', value)

    def get_enable_rgb_auto_exposure(self):
        return self.get_rgb_parameter('enable_auto_exposure')

    def get_enable_rgb_auto_exposure_info(self):
        return self.get_rgb_parameter_info('enable_auto_exposure')

    def set_sharpness(self, value):
        self.update_rgb_parameter('sharpness', value)

    def get_sharpness(self):
        return self.get_rgb_parameter('sharpness')

    def get_sharpness_info(self):
        return self.get_rgb_parameter_info('sharpness')

    def set_frames_queue_size(self, value):
        self.update_rgb_parameter('frames_queue_size', value)

    def get_frames_queue_size(self):
        return self.get_rgb_parameter('frames_queue_size')

    def get_frames_queue_size_info(self):
        return self.get_rgb_parameter_info('frames_queue_size')

    def set_global_rgb_time_enabled(self, value):
        self.update_rgb_parameter('global_time_enabled', value)

    def get_global_rgb_time_enabled(self):
        return self.get_rgb_parameter('global_time_enabled')

    def get_global_rgb_time_enabled_info(self):
        return self.get_rgb_parameter_info('global_time_enabled')

    def set_hue(self, value):
        self.update_rgb_parameter('hue', value)

    def get_hue(self):
        return self.get_rgb_parameter('hue')

    def get_hue_info(self):
        return self.get_rgb_parameter_info('hue')

    def set_white_balance(self, value):
        self.update_rgb_parameter('white_balance', value)

    def get_white_balance(self):
        return self.get_rgb_parameter('white_balance')

    def get_white_balance_info(self):
        return self.get_rgb_parameter_info('white_balance')

    def set_power_line_frequency(self, value):
        self.update_rgb_parameter('power_line_frequency', value)

    def get_power_line_frequency(self):
        return self.get_rgb_parameter('power_line_frequency')

    def get_power_line_frequency_info(self):
        return self.get_rgb_parameter_info('power_line_frequency')

    def set_backlight_compensation(self, value):
        self.update_rgb_parameter('backlight_compensation', value)

    def get_backlight_compensation(self):
        return self.get_rgb_parameter('backlight_compensation')

    def get_backlight_compensation_info(self):
        return self.get_rgb_parameter_info('backlight_compensation')

    def set_rgb_gain(self, value):
        self.update_rgb_parameter('gain', value)

    def get_rgb_gain(self):
        return self.get_rgb_parameter('gain')

    def get_rgb_gain_info(self):
        return self.get_rgb_parameter_info('gain')

    def set_rgb_auto_exposure_priority(self, value):
        self.update_rgb_parameter('auto_exposure_priority', value)

    def get_rgb_auto_exposure_priority(self):
        return self.get_rgb_parameter('auto_exposure_priority')

    def get_rgb_auto_exposure_priority_info(self):
        return self.get_rgb_parameter_info('auto_exposure_priority')

    def set_rgb_exposure(self, value):
        self.update_rgb_parameter('exposure', value)

    def get_rgb_exposure(self):
        return self.get_rgb_parameter('exposure')

    def get_rgb_exposure_info(self):
        return self.get_rgb_parameter_info('exposure')

    def set_rgb_contrast(self, value):
        self.update_rgb_parameter('contrast', value)

    def get_rgb_contrast(self):
        return self.get_rgb_parameter('contrast')

    def get_rgb_contrast_info(self):
        return self.get_rgb_parameter_info('contrast')

    def set_enable_rgb_auto_white_balance(self, value):
        self.update_rgb_parameter('enable_auto_white_balance', value)

    def get_enable_rgb_auto_white_balance(self):
        return self.get_rgb_parameter('enable_auto_white_balance')

    def get_enable_rgb_auto_white_balance_info(self):
        return self.get_rgb_parameter_info('enable_auto_white_balance')
