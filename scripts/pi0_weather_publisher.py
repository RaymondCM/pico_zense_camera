#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy
from SimpleDataTransport import DataSender
from SimpleDataTransport.local import ConnectionError
from rasberry_data_collection.msg import BME680


class Pi0WeatherPublisher:
    def __init__(self, topic="/pi0_bme680", host="0.0.0.0", port=5000, endpoint="/api/weather", poll_rate=2):
        self.__host = host
        self.__port = port
        self.__endpoint = endpoint
        self.__poll_rate = rospy.Rate(poll_rate)
        self.__publisher = rospy.Publisher(topic, BME680, queue_size=1)
        self.__valid_fields = ["pressure", "air_quality", "temperature", "humidity"]

    def __run(self):
        while not rospy.is_shutdown():
            self.__poll_rate.sleep()

            try:
                response = DataSender({}, self.__host, self.__port, endpoint=self.__endpoint)
                response = {k: v for k, v in response.items() if k in self.__valid_fields}
            except ConnectionError:
                continue

            self.__publisher.publish(BME680(**response))

    def spin(self):
        self.__run()


def pi0_weather_publisher():
    rospy.init_node("pi0_weather_publisher", anonymous=True)
    topic = rospy.get_param('~topic', "/pi0_bme680")
    host = rospy.get_param('~host', "0.0.0.0")
    port = rospy.get_param('~port', 5000)
    endpoint = rospy.get_param('~endpoint', "/api/weather")
    poll_rate = rospy.get_param('~poll_rate', 2)

    pi0_publisher = Pi0WeatherPublisher(topic, host, port, endpoint, poll_rate)
    pi0_publisher.spin()


if __name__ == '__main__':
    try:
        pi0_weather_publisher()
    except rospy.ROSInterruptException:
        pass
