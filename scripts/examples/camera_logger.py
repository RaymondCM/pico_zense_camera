from __future__ import absolute_import, division, print_function

import ros_numpy
import rospy
from rasberry_data_collection_pkg.camera import RealSenseSubscriber
from rasberry_data_collection_pkg.display import NonBlockingDisplay
import cv2


def camera_logger_example():
    rospy.init_node("camera_logger_example", anonymous=True)
    camera_name = rospy.get_param('~camera_name', "rasberry_data_collection_camera1")

    display = NonBlockingDisplay("Camera Reconfigure RGB", rate=30)
    subscriber = RealSenseSubscriber(camera_name)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        data = subscriber.wait_for_message()
        display.set_frame(ros_numpy.numpify(data.rgb))
        rate.sleep()


if __name__ == '__main__':
    camera_logger_example()
