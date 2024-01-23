#!/usr/bin/env python3

import time
from collections import deque
from functools import partial

import cv2
import numpy as np
import rospy
from imutils import rotate as _rotate_img
from sensor_msgs.msg import Image

from snakelib_camera.camera_interface import CameraStreamerBase
from snakelib_camera.camera_utils import module_to_head, to_img_msg
from snakelib_msgs.msg import HebiSensors


class ImageProcessor(CameraStreamerBase):
    """Driver node that publishes the camera data from the snake head.
    """
    def __init__(self):
        super().__init__()
        self._VALID_CAM_NAMES.extend(['processed_pinhole', 'processed_fisheye', 'processed_thermal'])
        self._VALID_CAM_STREAMS.extend(['processed_pinhole', 'processed_fisheye', 'processed_thermal', 'processed_overlay'])

        # Initialize running attributes.
        self._head_roll = 0.0
        self._head_roll_history = deque([], rospy.get_param('head_roll_history_len', 10))
        self._pinhole_img = self._thermal_img = self._fisheye_img = None # Raw images buffer
        self._processed_pinhole_img = self._processed_thermal_img = self._processed_fisheye_img = None # Raw images buffer

        self._smooth_rotate = rospy.get_param(f'smooth_rotate_image', False) and self.rotate
        self.max_delta = rospy.get_param(f'max_delta', 20)

        self.raw_cam_names = ['fisheye', 'pinhole', 'thermal']
        self.cam_names = [f'processed_{cam_name}' for cam_name in self.raw_cam_names]

        # Initialize required subscribers and publishers.
        rospy.Subscriber('/hebi_sensors/data', HebiSensors, self._hebi_sensor_callback)
        for cam_name, raw_cam_name in zip(self.cam_names, self.raw_cam_names):
            rospy.Subscriber(f'/{raw_cam_name}_cam/image_raw', Image, partial(self._image_callback, cam_name=raw_cam_name))
            publisher = rospy.Publisher(f'{cam_name}_cam/image_raw', Image, queue_size=1)
            setattr(self, f'{cam_name}_cam_pub', publisher)

        self.cam_names.append('processed_overlay')
        self.processed_overlay_cam_pub = rospy.Publisher(f'processed_overlay_cam/image_raw', Image, queue_size=1)

        # Set loop rate as camera FPS.
        self.loop_rate = rospy.get_param('image_processing_frequency', 30.0) # FPS

    def update_feedback(self):
        r"""Populates respective attributes with updated sensor readings.
        """
        # Get raw camera feed from the snake head.
        raw_feedback = self.get_feedback()

        background_img_type = f"{rospy.get_param('overlay_background', 'pinhole')}_img"
        if self.processed_overlay:
            raw_feedback.update({
                'overlay_img': self.get_overlay_img(raw_feedback)
            })
        else:
            background_img = raw_feedback.get(background_img_type, None)
            raw_feedback.update({
                'overlay_img': background_img
            })

        # More image processing can be added here in the future.
        processed_feedback = self.rotate_dict_imgs(raw_feedback) if self.rotate else raw_feedback

        time_now = time.time()
        for img_name, img in processed_feedback.items():
            # print(img_name, img is None)
            img_msg = to_img_msg(img, t=time_now)
            setattr(self, f'processed_{img_name}', img_msg)

    def get_feedback(self):
        r"""Returns raw feed images.

        Returns:
            feedback (dict): Dictionary containing the raw images published by individual camera nodes.
        """
        feedback = {
            f'{cam_name}_img': getattr(self, f'_{cam_name}_img') for cam_name in self.raw_cam_names
        }
        return feedback


    def get_overlay_img(self, feedback):
        r"""Returns an overlay of specified images.

        Args:
            feedback (dict): Dictionary containing the raw feedback from the snake head cameras.

        Returns:
            overlayed_img (np.ndarray): overlayed image.
        """
        alpha = rospy.get_param('foreground_alpha', 0.8)
        foreground_img_type = 'thermal' # rospy.get_param('overlay_foreground', False)
        background_img_type = rospy.get_param('overlay_background', False)

        foreground_img = feedback.get(f'{foreground_img_type}_img', None)
        background_img = feedback.get(f'{background_img_type}_img', None)

        if foreground_img is None:
            rospy.logwarn_throttle(10, f'Problem with foreground img: {foreground_img_type}')
            return None
        if background_img is None:
            rospy.logwarn_throttle(10, f'Problem with background img: {background_img_type}')
            return None

        og_h, og_w, _ = background_img.shape

        aspect_ratio = foreground_img.shape[1] / foreground_img.shape[0]
        new_h = og_w / aspect_ratio

        foreground_img = cv2.resize(foreground_img, (og_w, int(new_h)))

        #crop image from middle so that dimensions are equal to thermal image
        middle_val = og_h / 2.0
        middle_thermal = new_h / 2.0
        lower_val = int(middle_thermal - middle_val)
        upper_val = int(middle_val + middle_thermal)
        if foreground_img.ndim == 3:
            cropped_img = cv2.flip(foreground_img[lower_val:upper_val, :, :], 1)
        else:
            # Warning: received image is converted from greyscale to RGB which won't be accurate.
            cropped_img = cv2.flip(foreground_img[lower_val:upper_val, :], 1)
            channel = np.expand_dims(cropped_img, axis=-1)
            cropped_img = np.concatenate([channel, channel, channel], axis=-1)

        return cv2.addWeighted(cropped_img, alpha , background_img, 1-alpha, 0)

    def rotate_dict_imgs(self, img_dict, angle=None):
        r"""Rotates all images in the given dict

        Args:
            img_dict (Dict['str': np.ndarray]): Dict of images
            angle (optional): angle to rotate images by
        """
        processed_imgs = {
            k: self.rotate_single_img(v, angle=angle) for k, v in img_dict.items()
        }
        return processed_imgs

    def rotate_single_img(self, img, angle=None):
        r"""Wrapper around cv2.rotate to handle None inputs. Rotates by `head_roll` if `angle` is not provided.

        Args:
            img (np.ndarray): input image
            angle (optional): angle to rotate an image by
        """
        if img is None:
            return None
        rotation_angle = angle if angle is not None else self._head_roll
        return _rotate_img(img, rotation_angle)

    def _hebi_sensor_callback(self, msg):
        r"""Convert raw sensor readings to smooth head roll.

        Args:
            msg (HebiSensors): New received HEBI sensor message
        """
        updated_value = (180/np.pi) * module_to_head(msg.lin_acc)
        if self._smooth_rotate:
            delta = updated_value - self._head_roll

            # Reverse deltas on encountering flipping
            sign = -1 if abs(delta) > 180 else 1

            # Limit deltas and find the new head roll
            clipped_delta = min(max(delta, -self.max_delta), self.max_delta)
            updated_value = self._head_roll + sign * clipped_delta

            # Wrap values in [-180, 180] deg
            if abs(updated_value) > 180:
                updated_value -= np.sign(updated_value) * 360

            # Smoothen out using running average of last `n` samples
            self._head_roll_history.append(abs(updated_value))
            self._head_roll = np.sign(updated_value) * np.mean(self._head_roll_history)
        else:
            self._head_roll = updated_value

    @property
    def processed_overlay(self):
        r"""Flag to decide if overlay streaming is required"""
        return rospy.get_param('stream_overlay', False)

    @property
    def rotate(self):
        r"""Flag to decide if camera rotation is required"""
        return rospy.get_param('rotate_image', False)

    @property
    def processed_fisheye(self):
        r"""Flag to decide if processing is required for fisheye image"""
        return rospy.get_param('process_fisheye', False)

    @property
    def processed_pinhole(self):
        r"""Flag to decide if processing is required for pinhole image"""
        return rospy.get_param('process_pinhole', False)

    @property
    def processed_thermal(self):
        r"""Flag to decide if processing is required for thermal image"""
        return rospy.get_param('process_thermal', False)
