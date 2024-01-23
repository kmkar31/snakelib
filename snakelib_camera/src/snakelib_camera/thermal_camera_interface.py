#!/usr/bin/env python3

import socket
import struct
from functools import partial

import numpy as np
import rospy
from sensor_msgs.msg import Image

from snakelib_camera.camera_interface import CameraStreamerBase
from snakelib_camera.camera_utils import to_img_msg
from snakelib_camera.streaming_utils import (toggle_cam_stream,
                                             toggle_led_stream)


class ThermalCameraStreamer(CameraStreamerBase):
    """Driver node that publishes the camera data from the snake head.
    """

    def __init__(self):
        super().__init__()

        # Initialize running attributes.
        self.thermal_img = self._thermal_img = None

        # Initialize camera parameters.
        self.port = rospy.get_param(f'port')
        self.ip = rospy.get_param(f'ip')

        self.min_temp = rospy.get_param(f'min_temp')
        self.max_temp = rospy.get_param(f'max_temp')

        self.img_rows = rospy.get_param(f'thermal_img_rows')
        self.img_cols = rospy.get_param(f'thermal_img_cols')
        self.byte_size = rospy.get_param(f'thermal_byte_size')
        self.img_size = (self.img_rows * self.img_cols) * self.byte_size

        self.thermal_cam_pub = rospy.Publisher('thermal_cam/image_raw', Image, queue_size=1)

        self.loop_rate = rospy.get_param("thermal_camera_frequency", 30.0) # FPS

        self.cam_names = ['thermal']

        rospy.loginfo(f'Thermal cam status: {self.thermal}')

    def _thermal_reader(self):
        image_np = np.zeros((self.img_rows, self.img_cols))

        image = self.socket.recv(self.img_size)
        iter_obj = struct.iter_unpack('f', image)

        try:
            for i in range(0, self.img_rows):
                for j in range(0, self.img_cols):
                    image_np[i][j] = next(iter_obj)[0]

        except StopIteration:
            return None

        thermal_np_img = image_np.reshape(self.img_rows, self.img_cols).astype(np.float32)
        thermal_img = np.zeros((self.img_rows, self.img_cols, 3), dtype = "uint8")
        thermal_greyscale = np.zeros((self.img_rows, self.img_cols), dtype="uint8")

        for y in range(len(thermal_np_img[0])):
            for x in range(len(thermal_np_img)):
                #rgb value (use for raw visualization)
                thermal_img[x][y] = self._to_rgb(thermal_np_img[x][y], self.min_temp, self.max_temp)
                #pixel value (use for overlay)
                pixel = (thermal_np_img[x][y] - self.min_temp) * (255/(self.max_temp - self.min_temp)) + 0.5
                pixel = min(255, max(0, pixel))
                thermal_greyscale[x][y] = pixel
        thermal_img = np.rot90(thermal_img)
        thermal_greyscale = np.rot90(thermal_greyscale)
        return thermal_img

    def _to_rgb(self, raw_value, min_temp, max_temp):
        r"""Converts raw thermal sensor readings to RGB values

        Args:
            raw_values (np.ndarray): Raw sensor readings
            min_temp (float): Minimum temperature parameter for conversion
            max_temp (float): Maximum temperature parameter for conversion
        """
        value = np.clip(np.nan_to_num(raw_value), self.min_temp, self.max_temp)
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # [BLUE, GREEN, RED]

        i_f = float(value - min_temp) / float(max_temp - min_temp) * (len(colors) - 1)
        i, f = int(i_f // 1), i_f % 1  # Split into whole & fractional parts.
        if f < self.EPSILON:
            return colors[i]
        else:
            (r1, g1, b1), (r2, g2, b2) = colors[i], colors[i+1]
            r_value, g_value, b_value = int(r1 + f*(r2-r1)), int(g1 + f*(g2-g1)), int(b1 + f*(b2-b1))
            return [r_value, g_value, b_value]

    @property
    def thermal(self):
        r"""Checks for the latest set status of thermal streaming and toggles stream if required."""
        status = rospy.get_param(f'stream_thermal', False)
        if not hasattr(self, f'_thermal_status'):
            setattr(self, f'_thermal_status', not status)
        if status and not getattr(self, f'_thermal_status'):
            success = toggle_cam_stream(f'thermal', True)
            setattr(self, f'_thermal_status', True)
            rospy.sleep(0.2) # Allow some time for the stream to start.
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.socket.connect((self.ip, self.port))
            except:
                setattr(self, f'_thermal_status', False)
        if not status and getattr(self, f'_thermal_status'):
            success = toggle_cam_stream(f'thermal', False)
            setattr(self, f'_thermal_status', False)
            if hasattr(self, f'socket'):
                self.socket.close()
                delattr(self, f'socket')
        return self._thermal_status
