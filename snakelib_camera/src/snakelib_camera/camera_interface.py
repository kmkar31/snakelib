#!/usr/bin/env python3

import sys
import time

import cv2
import numpy as np
import rospy

from snakelib_camera.camera_utils import to_img_msg
from snakelib_camera.streaming_utils import (toggle_cam_stream,
                                             toggle_led_stream)
from snakelib_hebi.hebi_interface import HEBIROSWrapper


class CameraStreamerBase(HEBIROSWrapper):
    """Base class for defining shared methods across different cameras.
    """

    _VALID_CAM_NAMES = ['pinhole', 'fisheye', 'thermal', 'led']
    _VALID_CAM_STREAMS = ['pinhole', 'fisheye', 'thermal', 'led']
    EPSILON = sys.float_info.epsilon

    def __init__(self):
        self.to_numpy_img = lambda x: np.frombuffer(x.data, dtype=np.uint8).reshape(x.height, x.width, -1)

        # Set the camera names
        self.cam_names = []

    def run_loop(self):
        r"""Publishes the required processed images.
        """
        self.update_feedback()

        for cam_name in self.cam_names:
            self._publish_stream(cam_name)

    def _publish_stream(self, cam_name):
        """Publishes the specified cam stream if cam img exists and is valid and cam stream is requested.

        Args:
            cam_name (str): Name of the camera stream to publish.
        """
        if cam_name not in self._VALID_CAM_STREAMS:
            rospy.logwarn_throttle(10, f'Invalid cam name: {cam_name}. Nothing published.')
        if getattr(self, cam_name) or cam_name is 'processed_overlay':
            if hasattr(self, f'{cam_name}_img'):
                img = getattr(self, f'{cam_name}_img')
                pub = getattr(self, f'{cam_name}_cam_pub')
                if img is not None: pub.publish(img)

    def _image_callback(self, msg, cam_name):
        img = self.to_numpy_img(msg)
        setattr(self, f"_{cam_name}_img", img)

    def update_feedback(self):
        r"""Populates respective attributes with updated sensor readings.
        """
        # Get raw camera feed from the snake head.
        feedback = self.get_feedback()

        # Convert the processed images to ROS Image messages.
        t_now = time.time()

        for k, v in feedback.items():
            setattr(self, k, to_img_msg(v, t=t_now))

    def get_feedback(self):
        r"""Returns raw feed from the snake head cameras.

        Returns:
            feedback (dict): Dictionary containing the feedback from the snake head cameras.
                - rgb_img (list): RGB camera image.
                - thermal_img (list): Thermal camera image.
        """
        feedback = {f'{cam_name}_img': self._safe_img_read(cam_name) for cam_name in self.cam_names}

        return feedback

    def _safe_img_read(self, cam_name):
        r"""Safely tries to read http camera feed. Returns None if feed doesn't exist or invalid cam name provided.

        Args:
            cam_name (str): Attribute name of the camera stream to read.

        Returns:
            img (np.ndarray): Numpy array image
        """
        img = None

        if not getattr(self, f'_{cam_name}_status'):
            return img

        if cam_name not in self._VALID_CAM_NAMES:
            rospy.logwarn_throttle(10, f'Invalid cam name: {cam_name}')
            return None
        else:
            img = getattr(self, f'_{cam_name}_reader')()
            if img is None:
                rospy.logwarn(f'problem with {cam_name}. Attempting restart.')
                setattr(self, f'_{cam_name}_status', False)
        return img

    def _update_usb_cam_status(self, cam_name, address):
        r"""Checks the latest user-set status of the camera and toggles the camera stream if required.

        Args:
            cam_name (str): Name of the camera stream to publish.
            address (str): Http address of the camera stream.
        """
        status = rospy.get_param(f'stream_{cam_name}', False)
        if not hasattr(self, f'_{cam_name}_status'):
            setattr(self, f'_{cam_name}_status', not status)
        if status and not getattr(self, f'_{cam_name}_status'):
            success = toggle_cam_stream(f'{cam_name}', True)
            setattr(self, f'_{cam_name}_status', True)
            rospy.sleep(0.2) # Allow some time for the stream to start.
            try:
                cap = cv2.VideoCapture(f'{address}')
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
                setattr(self, f'{cam_name}_cap', cap)
            except:
                setattr(self, f'_{cam_name}_status', False)

        if not status and getattr(self, f'_{cam_name}_status'):
            success = toggle_cam_stream(f'{cam_name}', False)
            setattr(self, f'_{cam_name}_status', False)
            if hasattr(self, f'{cam_name}_cap'):
                getattr(self, f'{cam_name}_cap').release()
                delattr(self, f'{cam_name}_cap')
        return getattr(self, f'_{cam_name}_status')

    @property
    def led(self):
        r"""Checks the latest user-set status of the LED stream and toggles the LED stream if required."""
        status = rospy.get_param("led", False)
        if not hasattr(self, f'_led_status'):
            setattr(self, '_led_status', not status)
        if status and not self._led_status:
            success = toggle_led_stream('led', True)
            self._led_status = True
        if not status and self._led_status:
            success = toggle_led_stream('led', False)
            self._led_status = False
        return getattr(self, '_led_status')
