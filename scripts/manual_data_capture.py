#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import datetime
import os
import pickle
import yaml
from PIL import Image as PILImage

import ros_numpy
import rospy
import tf
from sensor_msgs.msg import Image, CameraInfo

from database_manager.documents import ros_time_to_datetime
from rasberry_data_collection_pkg.camera import RealSenseSubscriber


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
    save_location = rospy.get_param('~save_location', "data")
    stabilise_time = rospy.get_param('~stabilise_time', 1)

    # Get scenario files
    scenario_goal_file = rospy.get_param('~scenario_file', "scenarios/inter_row_robot14_rowb1.yaml")

    # Load data capture plan
    scenario = load_yaml_file(scenario_goal_file)
    context = scenario["context"]
    cameras = scenario["cameras"]

    # Save folder
    save_folder = os.path.join(save_location, context, datetime.datetime.utcnow().strftime('%Y-%m-%d'))

    # Allow time for cameras to initialise
    rospy.loginfo("Allowing cameras to warm-up (initialise) for {} seconds".format(warmup))
    rospy.sleep(warmup)

    # Log initialisation parameters
    rospy.loginfo("Initialising Cameras: {}".format(", ".join([c["name"] for c in cameras])))
    camera_subscribers = [RealSenseSubscriber(camera_info["name"], serial=camera_info["serial"], rgb=rgb, depth=depth,
                                              aligned_depth=aligned_depth, ir=ir, sync_queue_size=sync_queue_size,
                                              sync_thresh=sync_thresh, timeout=timeout) for camera_info in cameras]

    # Allow 3 seconds stabilisation (auto-exposure etc)
    rospy.loginfo("Allowing cameras to warm-up (stabilise) for {} seconds".format(stabilise_time))
    rospy.sleep(stabilise_time)

    while not rospy.is_shutdown():
        try:
            raw_input('\n\nPress any key to save the data!\n')
            print("\n\t- Key Pressed, waiting for robot to stabilise: Waiting", end='')
            rospy.sleep(stabilise_time)
            print(", Stabilised", end='')

            # Get data
            documents_to_store = []
            timestamp = datetime.datetime.now()
            ros_time = rospy.Time.now()
            ros_timestamp = ros_time_to_datetime(ros_time)

            print("\n\t- Time '{:%d/%m/%Y %H:%M:%S}' and "
                  "ros time '{:%d/%m/%Y %H:%M:%S}'".format(timestamp, ros_timestamp), end='')
            unique_file_name = os.path.join(save_folder, datetime.datetime.utcnow().strftime('%H:%M:%S.%f')[:-3])

            # Get data from cameras
            for camera_subscriber in camera_subscribers:
                camera_name = camera_subscriber.camera_name
                print("\n\t- Getting data from '{}' subscriber\n\t\t- Requesting Camera Data".format(camera_name), end='')

                data = camera_subscriber.wait_for_message()
                print(", Done".format(camera_name), end='')

                camera_unique_file_name = os.path.join(unique_file_name, camera_name)
                data_to_be_saved = {k: getattr(data, k) for k in dir(data) if isinstance(getattr(data, k), (Image, CameraInfo))}

                for data_name, data in data_to_be_saved.items():
                    if not os.path.isdir(camera_unique_file_name):
                        os.makedirs(camera_unique_file_name)

                    image_file_name = os.path.join(camera_unique_file_name, "{}.png".format(data_name))
                    data_file_name = os.path.join(camera_unique_file_name, "{}.pkl".format(data_name))

                    if isinstance(data, Image):
                        # Save Image
                        image = ros_numpy.numpify(data)
                        if "16" in str(image.dtype):
                            array_buffer = image.tobytes()
                            im = PILImage.new("I", image.T.shape)
                            im.frombytes(array_buffer, 'raw', "I;16")
                        else:
                            im = PILImage.fromarray(image)

                        im.save(image_file_name)
                        print("\n\t\t- Saving: {}".format(image_file_name), end='')

                        # Remove data field to save space since image has been saved anyway (replace with file name)
                        data.data = image_file_name

                    with open(data_file_name, 'wb') as fh:
                        pickle.dump(data, fh)

            print("\n\t- Saving documents to folder: {}".format(save_folder), end='')
            for idx, doc in enumerate(documents_to_store):
                print(str(idx) + " ", end='')

        except (rospy.ROSException, tf.ExtrapolationException) as e:
            rospy.logerr(e)


if __name__ == '__main__':
    try:
        inter_row()
    except rospy.ROSInterruptException:
        pass
