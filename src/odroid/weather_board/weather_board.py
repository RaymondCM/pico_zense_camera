#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy
from rasberry_data_collection.msg import WeatherBoard2

from odroid.weather_board.drivers import SI1132, BME280


class WeatherBoardPublisher:
    def __init__(self, ic2_device, topic_name="odroid/weather_board2", rate=10):
        self.__rate = rospy.Rate(rate)
        self.si1132 = SI1132(ic2_device)
        self.bme280 = BME280(ic2_device, 0x03, 0x02, 0x02, 0x02)
        self.publisher = rospy.Publisher(topic_name, WeatherBoard2, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            sea_level = 1024.25
            uv_index = (self.si1132.read_uv() / 100.0)  # UV Index
            visible_lux = int(self.si1132.read_visible())  # Lux
            ir_lux = int(self.si1132.read_ir())  # Lux
            temperature = self.bme280.read_temperature()  # Celsius
            humidity = self.bme280.read_humidity()  # %
            pressure = self.bme280.read_pressure() / 100  # hPA
            altitude = 44330.0 * (1.0 - pow(pressure / sea_level, 0.1903))  # meters

            message = WeatherBoard2(uv_index=uv_index, visible_lux=visible_lux, ir_lux=ir_lux, temperature=temperature,
                                    humidity=humidity, pressure=pressure, altitude=altitude)
            self.publisher.publish(message)
            self.__rate.sleep()


def __weather_board():
    rospy.init_node("weather_board_publisher", anonymous=True)
    ic2_device = rospy.get_param('~ic2_device', "/dev/ic2-5")
    publisher = rospy.get_param('~publisher', "/odroid/weather_board2")

    wb_pub = WeatherBoardPublisher(ic2_device, publisher)
    wb_pub.run()


if __name__ == '__main__':
    __weather_board()
