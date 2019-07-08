#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import os
import pickle
import sys
from PIL import Image as PILImage
from datetime import datetime

import cv2
import ros_numpy
import rospy

from rasberry_data_collection.msg import WeatherBoard2
from sensor_msgs.msg import Image, CameraInfo

from rasberry_data_collection_pkg.camera import RealSenseSubscriber


def display_saved_ip_camera_data():
    # rospy.init_node("ip_camera_saved_data_display", anonymous=True)
    cv2.namedWindow("IP camera saved data", cv2.WINDOW_GUI_EXPANDED)
    save_data_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])),
                                                    'ip_cam_saved_data/'))

    if not os.path.isdir(save_data_folder):
        raise ValueError("Folder '{}' does not exist".format(save_data_folder))

    data_lookup = {k: {} for k in os.listdir(save_data_folder)}
    data_lookup = {k: os.listdir(os.path.join(save_data_folder, k)) for k, v in data_lookup.items()}

    all_files = []
    for date, files in data_lookup.items():
        for f in files:
            datetime_object = datetime.strptime("{} {}".format(date, f.split('_')[0]), '%Y-%m-%d %H:%M:%S.%f')
            all_files.append([datetime_object, os.path.abspath(os.path.join(save_data_folder, date, f))])

    all_files = sorted(all_files, key=lambda x: x[0])
    rgb_files = [f for f in all_files if "rgb.png" in f[1] and "aligned" not in f[1]]
    weather_files = [f for f in all_files if "weather.pkl" in f[1]]

    for idx, (rgb_meta, weather_meta) in enumerate(zip(rgb_files, weather_files)):
        rgb_date, weather_date = rgb_meta[0], weather_meta[0]
        rgb_file, weather_file = rgb_meta[1], weather_meta[1]

        try:
            with open(weather_file, 'rb') as fh:
                weather = pickle.load(fh)
                weather.header = weather_date
                info = str(weather)
        except Exception as e:
            info = str(e)
        info_array = info.split('\n')

        rgb_image = cv2.imread(rgb_file)

        # Write info to image
        font, font_scale = cv2.FONT_HERSHEY_DUPLEX, 0.7
        x0, y0, dy = 20, rgb_image.shape[0] - 20, cv2.getTextSize(info, font, font_scale, 2)[1] + 20

        for i, line in enumerate(info_array):
            y = y0 - i * dy
            cv2.putText(rgb_image, line, (x0, y), font, font_scale, [0, 0, 0], thickness=2)
            cv2.putText(rgb_image, line, (x0, y), font, font_scale, [255, 255, 255], thickness=1)

        cv2.imshow("IP camera saved data", rgb_image)
        # RUN ffmpeg -r 12 -pattern_type glob -i '*.jpg' -i r_%04d.jpg -s 1280x720 -vcodec libx264 -crf 18 -preset slow timelapse.mp4
        # from inside the tlapse folder
        if not os.path.isdir("tlapse"):
            os.makedirs("tlapse")
        cv2.imwrite("tlapse/r_{:04d}.jpg".format(idx), rgb_image)
        if cv2.waitKey(1) == ord('q'):
            sys.exit(0)

    sys.exit(0)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            display_saved_ip_camera_data()
        except rospy.ROSInterruptException:
            break
