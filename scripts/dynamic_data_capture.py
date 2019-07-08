#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import datetime
import yaml

import actionlib
import rospy
import tf
import topological_navigation.msg

from database_manager.documents import RealSenseSensor, ExtrinsicsMeta, EnvironmentMeta, ThorvaldLocalisation, \
    RiseholmeDefault, ros_time_to_datetime
from rasberry_data_collection_pkg.camera import RealSenseSubscriber


class ThrovaldNavigator:
    def __init__(self):
        rospy.on_shutdown(self.__on_shutdown)
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        self.client.wait_for_server()

    def navigate_to_way_point(self, goal):
        nav_goal = topological_navigation.msg.GotoNodeGoal()

        print("\n\t- Navigation goal '{}' status: Requested".format(goal), end='')

        nav_goal.target = goal
        print(", Sent", end='')
        self.client.send_goal(nav_goal)

        print(", Waiting", end='')
        self.client.wait_for_result()

        print(", Fetching", end='')
        success = self.client.get_result()

        print(", Done.\n\t- Action Success: {}".format(success), end='')
        return success

    def __on_shutdown(self):
        self.client.cancel_all_goals()


def load_yaml_file(file_path):
    with open(file_path, 'r') as file_handle:
        try:
            contents = yaml.safe_load(file_handle)
        except yaml.YAMLError as exc:
            raise IOError(exc)
    return contents


def inter_row():
    rospy.init_node("inter_row_data_capture_node", anonymous=True)

    # Get camera parameters
    # TODO: Make these settings independent for each camera rather than shared
    rgb = rospy.get_param('~rgb_enabled', True)
    depth = rospy.get_param('~depth_enabled', True)
    aligned_depth = rospy.get_param('~aligned_depth_enabled', True)
    ir = rospy.get_param('~ir_enabled', True)
    sync_queue_size = rospy.get_param('~sync_queue_size', 10)
    sync_thresh = rospy.get_param('~sync_thresh', 0.5)
    timeout = rospy.get_param('~timeout', 60)
    warmup = rospy.get_param('~warmup', 0)
    max_retries_per_node = rospy.get_param('~max_retries_per_node', 6)
    environment_topic = rospy.get_param('~environment_topic', "/pi0_bme680")

    # Get scenario files
    scenario_goal_file = rospy.get_param('~scenario_file', "scenarios/inter_row_robot14_rowb1.yaml")

    # Load data capture plan
    scenario = load_yaml_file(scenario_goal_file)
    context = scenario["context"]
    cameras = scenario["cameras"]
    data_capture_home = scenario["data_capture_home"]
    data_capture_plan = scenario["data_capture_plan"]

    # Allow time for cameras to initialise
    rospy.sleep(warmup)

    # Log initialisation parameters
    rospy.loginfo("Initialising Cameras: {}".format(", ".join([c["name"] for c in cameras])))
    camera_subscribers = [RealSenseSubscriber(camera_info["name"], serial=camera_info["serial"], rgb=rgb, depth=depth,
                                              aligned_depth=aligned_depth, ir=ir, sync_queue_size=sync_queue_size,
                                              sync_thresh=sync_thresh, timeout=timeout) for camera_info in cameras]

    rospy.loginfo("Capture Plan: {} -> {}".format(data_capture_home, ", ".join(data_capture_plan)))

    # Allow 3 seconds stabilisation (auto-exposure etc)
    # rospy.sleep(3)

    navigator = ThrovaldNavigator()
    tf_listener = tf.TransformListener()

    # Send robot to home position
    print("Returning robot to home goal '{}':".format(data_capture_home), end='')
    if False and not navigator.navigate_to_way_point(data_capture_home):
        raise rospy.ROSException("Could not reach home goal '{}'".format(data_capture_home))

    for goal in data_capture_plan:
        attempt_count = 0
        print("\nStarting data collation plan for node '{}'".format(goal), end='')
        while not rospy.is_shutdown():
            try:
                print("\n\t- Attempt: {}".format(attempt_count), end='')

                # Go to node
                if not navigator.navigate_to_way_point(goal):
                    raise rospy.ROSException("Could not reach goal '{}'".format(data_capture_home))

                # Get data
                documents_to_store = []
                timestamp = datetime.datetime.now()
                ros_time = rospy.Time.now()
                ros_timestamp = ros_time_to_datetime(ros_time)

                print("\n\t- Time '{:%d/%m/%Y %H:%M:%S}' and "
                      "ros time '{:%d/%m/%Y %H:%M:%S}'".format(timestamp, ros_timestamp), end='')

                # Convert data from ros to defined structure in docs/database.md
                # TODO: Get actual environment data
                print("\n\t- Retrieving environment data", end='')
                environment_document = EnvironmentMeta.from_bme680_msg(environment_topic)
                print("\n\t- Retrieving localisation data", end='')
                localisation_document = ThorvaldLocalisation.from_ros_now()

                # Get data from cameras
                for camera_subscriber in camera_subscribers:
                    camera_name = camera_subscriber.camera_name
                    print("\n\t- Getting data from '{}' subscriber: Requested Transform".format(camera_name), end='')

                    # TODO: Validate this transform is correct
                    position, orientation = tf_listener.lookupTransform("base_link", camera_name + "_link", ros_time)
                    tran, rot = tf_listener.lookupTransform("base_link", camera_name + "_color_optical_frame", ros_time)
                    camera_to_robot_transform = ExtrinsicsMeta(rot, tran)
                    print(", Transform Received, Requesting Camera Data", end='')

                    camera_data = camera_subscriber.wait_for_message()
                    print(", Done".format(camera_name), end='')

                    image_document = RealSenseSensor.from_real_sense_sensor_data_class(camera_name,
                                                                                       camera_subscriber.camera_serial,
                                                                                       camera_data,
                                                                                       position, orientation,
                                                                                       camera_to_robot_transform)

                    # Create database entry
                    document = RiseholmeDefault(context=context, camera_data=image_document,
                                                environment=environment_document, localisation=localisation_document,
                                                doc_datetime=timestamp, ros_ref_time=ros_timestamp)
                    document_json = document.to_dict()
                    documents_to_store.append(document_json)

                # TODO: Save documents_to_store to database
                print("\n\t- Saving documents to database: {}".format([list(e.keys()) for e in documents_to_store]))
                break
            except (rospy.ROSException, tf.ExtrapolationException) as e:
                rospy.logerr(e)

            attempt_count += 1
            if attempt_count == max_retries_per_node:
                rospy.logerr("Reached max number of retries for node '{}', Skipping.".format(goal))
                break

    # Send robot to home position
    print("Returning robot to home goal '{}':".format(data_capture_home), end='')
    if not navigator.navigate_to_way_point(data_capture_home):
        raise rospy.ROSException("Could not reach home goal '{}'".format(data_capture_home))


if __name__ == '__main__':
    try:
        inter_row()
    except rospy.ROSInterruptException:
        pass
