"""

"""
import math

import numpy as np
import rospy
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion


def module_to_head(data):
    r"""Returns the roll of the module preceeding the head module (we use this since head does not have an IMU.

    Args:
        data (Vec3List): List of linear accelerations for all modules
    """
    accel_x, accel_y, _ = [data.x[0], data.y[0], data.z[0]]
    original_rot_angle = math.atan2(accel_x, -accel_y)
    return original_rot_angle


def to_img_msg(img, t=0):
    """Converts a numpy array image to a ROS Image message.

    Args:
        img (np.ndarray): numpy array image.
        t (float): time stamp

    Returns:
        ros_image (sensor_msgs.msg.Image): ROS Image message

    """
    if img is None:
        return None

    if len(img.shape) == 3:
        ros_image = Image(encoding="bgr8")

        # Add meta data to the message.
        ros_image.header.stamp = rospy.Time.from_sec(t)
        # ros_image.header.frame_id = id
        ros_image.height, ros_image.width, _ = img.shape
        ros_image.step = ros_image.width
    elif len(img.shape) == 2:
        ros_image = Image(encoding="mono8")

        # Add meta data to the message.
        ros_image.header.stamp = rospy.Time.from_sec(t)
        # ros_image.header.frame_id = id
        ros_image.height, ros_image.width = img.shape
        ros_image.step = ros_image.width

        # Idk why this (the succeeding three lines) is here.
        img -= 40
        img *= 6
        img = np.maximum(np.minimum(img, 255), 0).astype(np.uint8)
    # Add image data to the message.
    ros_image.data = img.ravel().tobytes()

    return ros_image
