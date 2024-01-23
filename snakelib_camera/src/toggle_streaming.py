#!/usr/bin/env python

from __future__ import print_function

import rospy

from snakelib_camera.srv import ToggleStream, ToggleStreamResponse
from snakelib_camera.streaming_utils import execute_script

PASSWORD = "fa"
RPI_IP = "192.168.8.132"
CODE_DIR = "/root/reu-snake-head-vision/mlx90641-driver-master/scripts/"


def toggle_stream_fn(req):
    # Execute the appropriate remote server (snake head) command here
    mode = 'launch' if req.on else 'stop'
    script_name = f'{mode}_background_stream'
    script_args = f'{req.stream_name}_sender'
    password = PASSWORD
    rpi_ip = RPI_IP
    code_dir = CODE_DIR
    success = execute_script(password, rpi_ip, code_dir, script_name, script_args)
    return ToggleStreamResponse(success)

def toggle_led_fn(req):
    # Execute the appropriate remote server (snake head) command here
    mode = 'on' if req.on else 'off'
    script_name = f'led_{mode}'
    script_args = f''
    password = PASSWORD
    rpi_ip = RPI_IP
    code_dir = CODE_DIR
    success = execute_script(password, rpi_ip, code_dir, script_name, script_args)
    return ToggleStreamResponse(success)

def toggle_streaming():
    rospy.init_node('toggle_streaming')
    stream_toggle_service = rospy.Service('toggle_stream', ToggleStream, toggle_stream_fn)
    led_toggle_service = rospy.Service('toggle_led', ToggleStream, toggle_led_fn)
    rospy.loginfo('Camera stream toggle available.')
    rospy.spin()


if __name__ == "__main__":
    toggle_streaming()
