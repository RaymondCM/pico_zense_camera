from threading import Thread, Lock, Event
import cv2
import ros_numpy
import rospy
import numpy as np
from sensor_msgs.msg import Image


class NonBlockingDisplay:
    def __init__(self, window_name="OpenCVDisplay", rate=20, image_topic=None, key_callback=None, delay_start=False):
        self.thread = None
        self.rw_lock = Lock()
        self.has_started = Event()

        self.frame = None
        self.window_name = window_name
        self.window_opened = Event()
        self.wait_key_delay = int((1.0 / rate) * 1000)

        self.key_callback = key_callback

        self.__update_sub = None
        self.__image_topic = image_topic

        if not delay_start:
            self.start()

    def start(self):
        if self.has_started.is_set():
            rospy.loginfo("Thread already alive.")
            return None
        self.has_started.set()
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        if self.__image_topic:
            self.__update_sub = rospy.Subscriber(self.__image_topic, Image, self.__set_frame_callback, queue_size=1)

        while self.has_started.is_set():
            with self.rw_lock:
                if self.frame is None:
                    continue
                frame = self.frame.copy()

            if not self.window_opened.is_set():
                self.window_opened.set()
                cv2.namedWindow(self.window_name, cv2.WINDOW_GUI_EXPANDED)

            if len(frame.shape) == 3 and frame.shape[2] == 3:
                frame = frame[..., ::-1]
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(self.wait_key_delay)

            if self.key_callback:
                Thread(target=self.key_callback, args=(key,)).start()

    def __set_frame_callback(self, image_message):
        self.set_frame(ros_numpy.numpify(msg=image_message))

    def set_frame(self, frame):
        with self.rw_lock:
            self.frame = frame.copy()

    def stop(self):
        self.has_started.clear()
        if self.thread.is_alive():
            self.thread.join()

    def __exit__(self, exc_type, exc_value, traceback):
        cv2.destroyAllWindows()
