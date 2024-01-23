"""Utility functions to start/stop streaming from the RPI (snake head) server.
"""
import os

import rospy
from sensor_msgs.msg import Image

from snakelib_camera.srv import ToggleStream


def execute_script(password, ip, base_dir, script_name, script_args=None):
	"""SSH into the RPI, trigger the relevant scripts to toggle requested streaming.

	Args:
	    password (str): password for the head module
	    ip (str): IP address for the head module
	    base_dir (str): base directory for the script to be executed
	    script_name (str): name of the script to be executed
	    script_args (optional): arguments required by the script
	"""
	if not '.sh' in script_name: script_name += '.sh'

	# Bypass prompt for fingerprint addition and strick host key checking
	login = f"sshpass -p {password} ssh -o 'StrictHostKeyChecking no' root@{ip}"
	pos_login_execution = f"cd {base_dir} && bash {script_name} {script_args}"

	cmd = f"{login} \"{pos_login_execution}\""
	os.system(cmd)


def toggle_cam_stream(stream_name, on, max_attempts=2):
    """Try to toggle camera stream (at snake head server) `max_attempts` times before moving on.

    Args:
        stream_name (str): Name of the service to be toggled
	on (str): Status of the stream to update ('on' / 'off')
	max_attempts (int): Maximum number of attempts to toggle stream before giving up

    """
    rospy.wait_for_service('toggle_stream')
    n_attempt = 0
    success = False
    mode = 'on' if on else 'off'
    while not success and (n_attempt < max_attempts):
        n_attempt += 1
        try:
            func = rospy.ServiceProxy(f'toggle_stream', ToggleStream)
            response = func(stream_name, on)
            # if response.success:
            #     rospy.loginfo(f'Status ({stream_name}): {on}')
            # rospy.logwarn(f'Failed attempt [{n_attempt}/{max_attempts}] to turn {stream_name}: {mode}')
            rospy.loginfo(f'Status ({stream_name}): {mode}')
            return True
        except rospy.ServiceException as e:
            rospy.logwarn(f'[{e}] Failed attempt [{n_attempt}/{max_attempts}] to turn {stream_name}: {mode}')

    return False

def toggle_led_stream(stream_name, on, max_attempts=2):
    """Try to toggle led stream (at snake head server) `max_attempts` times before moving on.

    Args:
        stream_name (str): Name of the service to be toggled
	on (str): Status of the stream to update ('on' / 'off')
	max_attempts (int): Maximum number of attempts to toggle stream before giving up

    """
    rospy.wait_for_service('toggle_led')
    n_attempt = 0
    success = False
    mode = 'on' if on else 'off'
    while not success and (n_attempt < max_attempts):
        n_attempt += 1
        try:
            func = rospy.ServiceProxy('toggle_led', ToggleStream)
            response = func(stream_name, on)
            # if response.success:
            #     rospy.loginfo(f'Status ({stream_name}): {on}')
            # rospy.logwarn(f'Failed attempt [{n_attempt}/{max_attempts}] to turn {stream_name}: {mode}')
            rospy.loginfo(f'Status ({stream_name}): {mode}')
            return True
        except rospy.ServiceException as e:
            rospy.logwarn(f'[{e}] Failed attempt [{n_attempt}/{max_attempts}] to turn {stream_name}: {mode}')

    return False
