"""
This python file defines all the types specified in 'docs/database.md'.
"""
from __future__ import absolute_import, division, print_function

import datetime

import geometry_msgs.msg
import numpy as np
import rasberry_data_collection.msg
import realsense2_camera.msg
import ros_numpy
import rospy
import sensor_msgs.msg
import std_msgs.msg

from rasberry_data_collection_pkg.camera import RealSenseSensorData

try:
    import typing
except ImportError:
    pass


class Type:
    numeric = (int, float, long)
    sequence = (list, tuple, np.ndarray)
    text = (str,)
    binary = (bytes, bytearray, memoryview)
    sets = (set, frozenset)
    maps = (dict,)
    time = (datetime.datetime,)
    any = numeric + sequence + text + binary + sets + maps + time

    def __init__(self):
        pass


def ros_time_to_datetime(ros_time):
    return datetime.datetime.utcfromtimestamp(ros_time.to_sec())


class Document(object):
    def __init__(self):
        pass

    def to_dict(self, cls=None):
        # type: (typing.Union[Document, None]) -> Type.maps
        if cls is None:
            cls = self
        assert isinstance(cls, Document)

        # Recursively convert the documents class hierarchy into a dict (for dumping to a db)
        cls_dict = {}
        for obj_key, obj_val in cls.__dict__.items():
            if isinstance(obj_val, Document):
                obj_val = obj_val.to_dict()
            if isinstance(obj_val, Type.any):
                cls_dict[obj_key] = obj_val

        return cls_dict


class RiseholmeDefault(Document):
    def __init__(self, context, camera_data, environment, localisation, doc_datetime=None, ros_ref_time=None):
        # type: (Type.text, RealSenseSensor, EnvironmentMeta, ThorvaldLocalisation, Type.time, Type.time) -> None
        super(RiseholmeDefault, self).__init__()
        self.context = context
        self.camera_data = camera_data
        self.environment = environment
        self.localisation = localisation
        if doc_datetime is None:
            doc_datetime = datetime.datetime.now()
        self.doc_datetime = doc_datetime
        if ros_ref_time is None:
            ros_ref_time = ros_time_to_datetime(rospy.Time.now())
        self.ros_ref_time = ros_ref_time


class RealSenseSensor(Document):
    def __init__(self, camera_name, camera_serial, rgb, depth, aligned_depth_to_rgb,
                 aligned_depth_to_ir_left, ir_left, ir_right, depth_to_rgb_transform, depth_to_ir_left_transform,
                 depth_to_ir_right_transform):
        # type: (Type.text, Type.text, SensorImage, SensorImage, SensorImage, SensorImage, SensorImage, SensorImage, ExtrinsicsMeta, ExtrinsicsMeta, ExtrinsicsMeta) -> None
        super(RealSenseSensor, self).__init__()
        assert isinstance(camera_name, Type.text)
        assert isinstance(camera_serial, Type.text)
        assert isinstance(rgb, SensorImage)
        assert isinstance(depth, SensorImage)
        assert isinstance(aligned_depth_to_rgb, SensorImage)
        assert isinstance(aligned_depth_to_ir_left, SensorImage)
        assert isinstance(ir_left, SensorImage)
        assert isinstance(ir_right, SensorImage)
        assert isinstance(depth_to_rgb_transform, ExtrinsicsMeta)
        assert isinstance(depth_to_ir_left_transform, ExtrinsicsMeta)
        assert isinstance(depth_to_ir_right_transform, ExtrinsicsMeta)

        self.camera_name = camera_name
        self.camera_serial = camera_serial
        self.rgb = rgb
        self.depth = depth
        self.aligned_depth_to_rgb = aligned_depth_to_rgb
        self.aligned_depth_to_ir_left = aligned_depth_to_ir_left
        self.ir_left = ir_left
        self.ir_right = ir_right
        self.depth_to_rgb_transform = depth_to_rgb_transform
        self.depth_to_ir_left_transform = depth_to_ir_left_transform
        self.depth_to_ir_right_transform = depth_to_ir_right_transform

    @staticmethod
    def from_real_sense_sensor_data_class(name, serial, data):
        # type: (str, str, RealSenseSensorData) -> RealSenseSensor
        assert isinstance(name, Type.text)
        assert isinstance(serial, Type.text)
        assert isinstance(data, RealSenseSensorData)

        return RealSenseSensor(camera_name=name, camera_serial=serial,
                               rgb=SensorImage.from_image_msg(data.rgb, data.rgb_intrinsics),
                               depth=SensorImage.from_image_msg(data.depth, data.depth_intrinsics),
                               aligned_depth_to_rgb=SensorImage.from_image_msg(
                                   data.aligned_depth_to_rgb, data.aligned_depth_to_rgb_intrinsics),
                               aligned_depth_to_ir_left=SensorImage.from_image_msg(
                                   data.aligned_depth_to_ir_left, data.aligned_depth_to_ir_left_intrinsics),
                               ir_left=SensorImage.from_image_msg(data.ir_left, data.ir_left_intrinsics),
                               ir_right=SensorImage.from_image_msg(data.ir_right, data.ir_right_intrinsics),
                               depth_to_rgb_transform=ExtrinsicsMeta.from_extrinsics_msg(
                                   data.depth_to_color_extrinsics),
                               depth_to_ir_left_transform=ExtrinsicsMeta.from_extrinsics_msg(
                                   data.depth_to_ir_left_extrinsics),
                               depth_to_ir_right_transform=ExtrinsicsMeta.from_extrinsics_msg(
                                   data.depth_to_ir_right_extrinsics))


class SensorImage(Document):
    def __init__(self, data, ros_time, frame_id, height, width, intrinsics):
        # type: (Type.sequence, Type.time, Type.text, Type.numeric, Type.numeric, IntrinsicsMeta) -> None
        super(SensorImage, self).__init__()
        assert isinstance(data, Type.sequence)
        assert isinstance(ros_time, Type.time)
        assert isinstance(frame_id, Type.text)
        assert isinstance(height, Type.numeric)
        assert isinstance(width, Type.numeric)
        assert isinstance(intrinsics, IntrinsicsMeta)

        self.data = data
        self.ros_time = ros_time
        self.frame_id = frame_id
        self.height = height
        self.width = width
        self.intrinsics = intrinsics

    @staticmethod
    def from_image_msg(image, intrinsics):
        # type: (sensor_msgs.msg.Image, sensor_msgs.msg.CameraInfo) -> SensorImage
        assert isinstance(image, sensor_msgs.msg.Image)
        assert isinstance(intrinsics, sensor_msgs.msg.CameraInfo)
        return SensorImage(data=ros_numpy.numpify(image), ros_time=ros_time_to_datetime(image.header.stamp),
                           frame_id=image.header.frame_id, height=image.height, width=image.width,
                           intrinsics=IntrinsicsMeta.from_camera_info_msg(intrinsics))


class IntrinsicsMeta(Document):
    def __init__(self, distortion_model, distortion_parameters, intrinsic_matrix, rectification_matrix,
                 projection_matrix):
        # type: (Type.text, Type.sequence, Type.sequence, Type.sequence, Type.sequence) -> None
        super(IntrinsicsMeta, self).__init__()
        assert isinstance(distortion_model, Type.text)
        assert isinstance(distortion_parameters, Type.sequence)
        assert isinstance(intrinsic_matrix, Type.sequence)
        assert isinstance(rectification_matrix, Type.sequence)
        assert isinstance(projection_matrix, Type.sequence)

        self.distortion_model = distortion_model
        self.distortion_parameters = distortion_parameters
        self.intrinsic_matrix = intrinsic_matrix
        self.rectification_matrix = rectification_matrix
        self.projection_matrix = projection_matrix

    @staticmethod
    def from_camera_info_msg(intrinsics):
        # type: (sensor_msgs.msg.CameraInfo) -> IntrinsicsMeta
        assert isinstance(intrinsics, sensor_msgs.msg.CameraInfo)
        return IntrinsicsMeta(distortion_model=intrinsics.distortion_model, distortion_parameters=intrinsics.D,
                              intrinsic_matrix=intrinsics.K, rectification_matrix=intrinsics.R,
                              projection_matrix=intrinsics.P)


class ExtrinsicsMeta(Document):
    def __init__(self, rotation, translation):
        # type: (Type.sequence, Type.sequence) -> None
        super(ExtrinsicsMeta, self).__init__()
        assert isinstance(rotation, Type.sequence)
        assert isinstance(translation, Type.sequence)
        self.rotation = rotation
        self.translation = translation

    @staticmethod
    def from_extrinsics_msg(extrinsics):
        # type: (realsense2_camera.msg.Extrinsics) -> ExtrinsicsMeta
        assert isinstance(extrinsics, realsense2_camera.msg.Extrinsics)
        return ExtrinsicsMeta(rotation=extrinsics.rotation, translation=extrinsics.translation)


class ThorvaldLocalisation(Document):
    def __init__(self, current_node, current_edge, closest_node, laser_scan, robot_pose, amcl):
        # type: (Type.text, Type.text, Type.text, LaserScan, Pose, CovariancePose) -> None
        super(ThorvaldLocalisation, self).__init__()
        assert isinstance(current_node, Type.text)
        assert isinstance(current_edge, Type.text)
        assert isinstance(closest_node, Type.text)
        assert isinstance(laser_scan, LaserScan)
        assert isinstance(robot_pose, Pose)
        assert isinstance(amcl, CovariancePose)
        self.current_node = current_node
        self.current_edge = current_edge
        self.closest_node = closest_node
        self.laser_scan = laser_scan
        self.robot_pose = robot_pose
        self.amcl = amcl

    @staticmethod
    def from_ros_now(current_node_topic="/current_node", current_edge_topic="/current_edge",
                     closest_node_topic="/closest_node", laser_scan_topic="/scan", robot_pose_topic="/robot_pose",
                     amcl_topic="/amcl_pose", timeout=5):
        # type: (str, str, str, str, str, str, typing.Union[int, float]) -> ThorvaldLocalisation
        assert isinstance(current_node_topic, Type.text)
        assert isinstance(current_edge_topic, Type.text)
        assert isinstance(closest_node_topic, Type.text)
        assert isinstance(laser_scan_topic, Type.text)
        assert isinstance(robot_pose_topic, Type.text)
        assert isinstance(amcl_topic, Type.text)
        assert isinstance(timeout, (int, float))

        current_node_msg = rospy.wait_for_message(current_node_topic, std_msgs.msg.String, timeout=timeout)
        closest_node_msg = rospy.wait_for_message(closest_node_topic, std_msgs.msg.String, timeout=timeout)

        # Closest edge topic can be empty when at node
        try:
            current_edge_msg = rospy.wait_for_message(current_edge_topic, std_msgs.msg.String, timeout=timeout)
        except rospy.ROSException:
            current_edge_msg = std_msgs.msg.String(data="None")

        laser_scan_msg = rospy.wait_for_message(laser_scan_topic, sensor_msgs.msg.LaserScan, timeout=timeout)
        robot_pose_msg = rospy.wait_for_message(robot_pose_topic, geometry_msgs.msg.Pose, timeout=timeout)
        amcl_msg = rospy.wait_for_message(amcl_topic, geometry_msgs.msg.PoseWithCovarianceStamped, timeout=timeout)

        return ThorvaldLocalisation(current_node=current_node_msg.data, current_edge=current_edge_msg.data,
                                    closest_node=closest_node_msg.data,
                                    laser_scan=LaserScan.from_laser_scan_msg(laser_scan_msg),
                                    robot_pose=Pose.from_pose_msg(robot_pose_msg),
                                    amcl=CovariancePose.from_pose_with_covariance_stamped_msg(amcl_msg))


class LaserScan(Document):
    def __init__(self, acquisition_time, frame_id, angle_min, angle_max, angle_increment, time_increment, scan_time,
                 range_min, range_max, ranges, intensities):
        # type: (Type.time, Type.text, Type.numeric, Type.numeric, Type.numeric, Type.numeric, Type.numeric, Type.numeric, Type.numeric, Type.sequence, Type.sequence)-> None
        super(LaserScan, self).__init__()
        assert isinstance(acquisition_time, Type.time)
        assert isinstance(frame_id, Type.text)
        assert isinstance(angle_min, Type.numeric)
        assert isinstance(angle_max, Type.numeric)
        assert isinstance(angle_increment, Type.numeric)
        assert isinstance(time_increment, Type.numeric)
        assert isinstance(scan_time, Type.numeric)
        assert isinstance(range_min, Type.numeric)
        assert isinstance(range_max, Type.numeric)
        assert isinstance(ranges, Type.sequence)
        assert isinstance(intensities, Type.sequence)
        self.acquisition_time = acquisition_time
        self.frame_id = frame_id
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.time_increment = time_increment
        self.scan_time = scan_time
        self.range_min = range_min
        self.range_max = range_max
        self.ranges = ranges
        self.intensities = intensities

    @staticmethod
    def from_laser_scan_msg(laser_scan):
        # type: (sensor_msgs.msg.LaserScan) -> LaserScan
        return LaserScan(acquisition_time=ros_time_to_datetime(laser_scan.header.stamp),
                         frame_id=laser_scan.header.frame_id, angle_min=laser_scan.angle_min,
                         angle_max=laser_scan.angle_max, angle_increment=laser_scan.angle_increment,
                         time_increment=laser_scan.time_increment, scan_time=laser_scan.scan_time,
                         range_min=laser_scan.range_min, range_max=laser_scan.range_max, ranges=laser_scan.ranges,
                         intensities=laser_scan.intensities)


class CovariancePose(Document):
    def __init__(self, pose, covariance):
        # type: (Pose, Type.sequence)-> None
        super(CovariancePose, self).__init__()
        assert isinstance(pose, Pose)
        assert isinstance(covariance, Type.sequence)
        self.pose = pose
        self.covariance = covariance

    @staticmethod
    def from_pose_with_covariance_stamped_msg(pose_with_covariance_stamped):
        # type: (geometry_msgs.msg.PoseWithCovarianceStamped) -> CovariancePose
        assert isinstance(pose_with_covariance_stamped, geometry_msgs.msg.PoseWithCovarianceStamped)
        return CovariancePose.from_pose_with_covariance_msg(pose_with_covariance_stamped.pose)

    @staticmethod
    def from_pose_with_covariance_msg(pose_with_covariance):
        # type: (geometry_msgs.msg.PoseWithCovariance) -> CovariancePose
        assert isinstance(pose_with_covariance, geometry_msgs.msg.PoseWithCovariance)
        return CovariancePose(pose=Pose(position=Position.from_point_msg(pose_with_covariance.pose.position),
                                        orientation=Orientation.from_quaternion_msg(
                                            pose_with_covariance.pose.orientation)),
                              covariance=pose_with_covariance.covariance)


class Pose(Document):
    def __init__(self, position, orientation):
        # type: (Position, Orientation)-> None
        super(Pose, self).__init__()
        assert isinstance(position, Position)
        assert isinstance(orientation, Orientation)
        self.position = position
        self.orientation = orientation

    @staticmethod
    def from_pose_msg(pose):
        # type: (geometry_msgs.msg.Pose) -> Pose
        assert isinstance(pose, geometry_msgs.msg.Pose)
        return Pose(position=Position.from_point_msg(pose.position),
                    orientation=Orientation.from_quaternion_msg(pose.orientation))


class Position(Document):
    def __init__(self, x, y, z):
        # type: (Type.numeric, Type.numeric, Type.numeric)-> None
        super(Position, self).__init__()
        assert isinstance(x, Type.numeric)
        assert isinstance(y, Type.numeric)
        assert isinstance(z, Type.numeric)
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_point_msg(point):
        # type: (geometry_msgs.msg.Point) -> Position
        assert isinstance(point, geometry_msgs.msg.Point)
        return Position(x=point.x, y=point.y, z=point.z)


class Orientation(Document):
    def __init__(self, x, y, z, w):
        # type: (Type.numeric, Type.numeric, Type.numeric, Type.numeric)-> None
        super(Orientation, self).__init__()
        assert isinstance(x, Type.numeric)
        assert isinstance(y, Type.numeric)
        assert isinstance(z, Type.numeric)
        assert isinstance(w, Type.numeric)
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    @staticmethod
    def from_quaternion_msg(quaternion):
        # type: (geometry_msgs.msg.Quaternion) -> Orientation
        assert isinstance(quaternion, geometry_msgs.msg.Quaternion)
        return Orientation(x=quaternion.x, y=quaternion.y, z=quaternion.z, w=quaternion.w)


class EnvironmentMeta(Document):
    def __init__(self, temperature, pressure, humidity, air_quality):
        # type: (Type.numeric, Type.numeric, Type.numeric, Type.numeric, Type.numeric) -> None
        super(EnvironmentMeta, self).__init__()
        assert isinstance(temperature, Type.numeric)
        assert isinstance(pressure, Type.numeric)
        assert isinstance(humidity, Type.numeric)
        assert isinstance(air_quality, Type.numeric)

        self.temperature = temperature
        self.pressure = pressure
        self.humidity = humidity
        self.air_quality = air_quality

    @staticmethod
    def from_bme680_msg(topic=None, msg=None, timeout=5):
        # type: (Type.text, rasberry_data_collection.msg.BME680, int) -> EnvironmentMeta
        assert isinstance(topic, Type.text) or topic is None
        assert isinstance(msg, rasberry_data_collection.msg.BME680) or msg is None
        assert isinstance(timeout, Type.numeric)
        assert (msg is None) ^ (topic is None)  # Only pass one parameter with a value

        if msg is None:
            msg = rospy.wait_for_message(topic, rasberry_data_collection.msg.BME680, timeout=timeout)

        return EnvironmentMeta(temperature=msg.temperature, pressure=msg.pressure, humidity=msg.humidity,
                               air_quality=msg.air_quality)
