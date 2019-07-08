#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import os.path
import rospkg
import subprocess
import yaml

import rospy


class RealSensePublisher:
    def __init__(self, **kwargs):
        self.args = ["{}:={}".format(k, v) for k, v in kwargs.items()]
        self.proc = subprocess.Popen(["roslaunch", "rasberry_data_collection", "camera_publisher.launch"] + self.args)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.proc.terminate()
        self.proc.wait()


def load_yaml_file(file_path):
    with open(file_path, 'r') as file_handle:
        try:
            contents = yaml.safe_load(file_handle)
        except yaml.YAMLError as exc:
            raise IOError(exc)
    return contents


def dynamic_camera_launcher():
    rospy.init_node("dynamic_camera_launcher", anonymous=True)

    # Get the package path
    pkg_root = rospkg.RosPack().get_path("rasberry_data_collection")

    # Get the scenario file
    scenario = load_yaml_file(rospy.get_param('~scenario_file', "scenarios/inter_row_robot14_rowb1.yaml"))

    # Get all the passed parameters
    node_name = rospy.get_name()
    parameters = {p.split('/')[-1].replace("dyn_", ""): rospy.get_param(p) for p in rospy.get_param_names() if
                  p.startswith(node_name) and "dyn_" in p}

    # ROS launch all of the cameras in the YAML file with this nodes parameters
    publishers = []
    try:
        for camera_meta in scenario["cameras"]:
            camera_parameters = parameters.copy()

            # Specify some smart default parameters
            if "json_file_path" not in parameters:
                camera_parameters["json_file_path"] = os.path.join(pkg_root, "config", camera_meta["serial"] + ".json")

            publishers.append(RealSensePublisher(camera_name=camera_meta["name"], serial_no=camera_meta["serial"],
                                                 **camera_parameters))
        rospy.spin()
    except rospy.ROSException:
        for p in publishers:
            p.shutdown()


if __name__ == '__main__':
    try:
        dynamic_camera_launcher()
    except rospy.ROSInterruptException:
        pass
