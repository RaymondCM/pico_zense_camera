#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import os
import pickle
import sys
from PIL import Image as PILImage
from datetime import datetime

import ros_numpy
import rospy
from sensor_msgs.msg import Image, CameraInfo

from rasberry_data_collection_pkg.camera import RealSenseSubscriber


def periodic_capture_example():
    rospy.init_node("periodic_capture_example", anonymous=True)
    camera_name = rospy.get_param('~camera_name', "ip_camera")
    period = rospy.get_param('~period', 1800)
    rgb = rospy.get_param('~rgb_enabled', True)
    depth = rospy.get_param('~depth_enabled', False)
    aligned_depth = rospy.get_param('~aligned_depth_enabled', False)
    ir = rospy.get_param('~ir_enabled', False)
    sync_queue_size = rospy.get_param('~sync_queue_size', 10)
    sync_thresh = rospy.get_param('~sync_thresh', 0.5)
    timeout = rospy.get_param('~timeout', 60)

    rospy.loginfo("Initialising node with camera_name:={} and a period:={}".format(camera_name, period))

    subscriber = RealSenseSubscriber(camera_name, rgb=rgb, depth=depth, aligned_depth=aligned_depth, ir=ir,
                                     sync_queue_size=sync_queue_size, sync_thresh=sync_thresh, timeout=timeout)

    rate = rospy.Rate(1.0 / period)

    save_data_folder = 'periodic_saved_data/' + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] + '/'
    save_data_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), save_data_folder))

    if not os.path.isdir(save_data_folder):
        os.makedirs(save_data_folder)

    while not rospy.is_shutdown():
        # Get data
        data = subscriber.wait_for_message()

        # Save data
        unique_file_name = os.path.join(save_data_folder, datetime.utcnow().strftime('%H:%M:%S.%f')[:-3])
        data_to_be_saved = {k: getattr(data, k) for k in dir(data) if isinstance(getattr(data, k), (Image, CameraInfo))}

        save_message = "Saved Files:"
        for data_name, data in data_to_be_saved.items():
            image_file_name = "{}_{}.png".format(unique_file_name, data_name)
            data_file_name = "{}_{}.pkl".format(unique_file_name, data_name)

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
                save_message += "\n\t" + image_file_name

                # Remove data field to save space since image has been saved anyway (replace with file name)
                data.data = image_file_name

            # Save JSON
            with open(data_file_name, 'wb') as fh:
                pickle.dump(data, fh)

            save_message += "\n\t" + data_file_name

        rospy.loginfo(save_message)
        rate.sleep()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            periodic_capture_example()
        except rospy.ROSInterruptException:
            break
