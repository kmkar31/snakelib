import numpy as np
import rospy
from sensor_msgs.msg import JointState
from snakelib_msgs.msg import HebiSensors, SnakeCommand

from snakelib_control.gaitlib_controller import GaitlibController
from snakelib_control.go_to_position_controller import GoToPositionController
from snakelib_control.watchdog import Watchdog

import time

class CommandManager:
    """Manages commands and dispatches different controllers for running the robot

    This module listens for "commands" that tell the robot what behavior and what
    parameters to use for that behavior. For example, it will listen for changes on a
    SnakeCommand topic and then update a GaitlibController to change the open-loop gait
    and gait parameters.

    Every "command" has a corresponding "controller" that executes that command. When
    the command changes, this class takes care of switching between these controllers.
    """

    def __init__(self):
        """Initializes the CommandManager"""

        self._snake_type = rospy.get_param(f"snake_type", "REU")

        self._snake_param = rospy.get_param(f"{self._snake_type}", {})
        self._module_names = self._snake_param.get("module_names", [])

        self._snake_command = SnakeCommand()  # the last received snake command
        self._loop_rate = rospy.get_param(
            "commmand_manager_frequency", 100.0
        )  # Hz, loop rate in Hz for the run() function

        # Rate in Hz at which sensor data must be received before watchdog is timed out
        self._sensors_watchdog_rate = rospy.get_param("sensors_watchdog_rate", 5.0)

        # Store the home configuration
        self._home_joint_state = JointState()
        self._home_joint_state.name = self._module_names
        self._home_joint_state.position = self._snake_param.get(
            "home", np.zeros(len(self._module_names))
        )

        # Subscriber that listens for SnakeCommand messages
        self._snake_command_sub = rospy.Subscriber(
            "snake/command", SnakeCommand, self.snake_command_cb
        )

        # Subscribes to the current joint state from hardware or simulation
        self._joint_state_sub = rospy.Subscriber(
            "snake/joint_states", JointState, self.joint_state_cb
        )

        self._joint_state_sub = rospy.Subscriber(
            "/hebi_sensors/data", HebiSensors, self.hebi_sensor_cb
        )

        # Publishes the desired joint state provided by the current running controller
        self._joint_state_pub = rospy.Publisher(
            "snake/joint_commands", JointState, queue_size=10
        )

        """Sensor watchdog prevents a command from being sent until the current joint
        state is updated, so initialization of _current_joint_state does not matter, but
        for additional safety we set it to the home joint state.
        """
        self._sensors_watchdog = Watchdog(
            1.0 / self._sensors_watchdog_rate * pow(10, 9)
        )

        self._elapsed_snake_time = 0

        # TODO: Try to move create arc in GaitlibController
        # Pole climb arc parameters
        self._pole_climb_params = self._snake_param.get("gait_params", {}).get("pole_climb", {})
        self._arc_beta = self._pole_climb_params.get("arc_beta", 0.3)
        self._imu_dir = self._pole_climb_params.get("imu_dir", 1)

        rospy.init_node("command_manager", anonymous=True)

        # TODO: watchdog likely doesn't work so using the following hack.
        rospy.wait_for_message("snake/joint_states", JointState)

        num_modules = len(self._module_names)
        self._prev_cmd = self._desired_joint_state = JointState(position=[],
                                                                velocity=[np.nan]*num_modules,
                                                                effort=[np.nan]*num_modules)

        self._prev_cmd.name = self._current_joint_state.name
        self._prev_cmd.position = self._current_joint_state.position

        self._desired_joint_state.name = self._current_joint_state.name
        self._desired_joint_state.position = self._current_joint_state.position

        # Default to holding position in home joint state
        self._controller = GoToPositionController(
            self._snake_type, self._home_joint_state.position, self._current_joint_state
        )

    def snake_command_cb(self, msg):
        """Callback function for SnakeCommand messages

        This only updates the SnakeCommand variable stored in the Manager class. It
        does not invoke any further function calls that modify the active controller,
        because this is taken care of inside run().

        Args:
            msg: SnakeCommand message received by the subscriber.
        """

        self._snake_command = msg
        #print("Setting _snake_command.")


    def joint_state_cb(self, msg):
        """Joint state callback function.

        Args:
            msg: JointState message received by the subscriber.
        """

        self._sensors_watchdog.trigger()
        self._current_joint_state = msg

    def hebi_sensor_cb(self, msg):
        """HEBI sensor callback function.

        Args:
            msg: HebiSensor message received by the subscriber.
        """

        self._hebi_sensor_msg = msg

    def _get_head_acc(self):
        """Fetches latest acceleration readings form HebiSensor msg.

        Returns:
            [x, y]: accelerations for the head module.
        """

        if not hasattr(self, '_hebi_sensor_msg'):
            return [0, -9.8]
        acc = self._hebi_sensor_msg.lin_acc

        return [acc.x[0], acc.y[0]]
    

    # TODO: Probably move this to GaitlibController
    def _create_arc(self, pole_direction, beta, imu_dir):
        """Joint angles for creating an arc in given direction.
        Args:
            pole_direction: Direction of the arc
            beta: Beta offset for arc
            imu_dir: IMU direction flip based on snake_type"""

        # Find head start orientation
        head_acc_x, head_acc_y = self._get_head_acc()

        # Choose joints that have most of their motion along ground.
        module_offset = imu_dir if abs(head_acc_x) > abs(head_acc_y) else (not imu_dir)

        inverted_module = -1
        if abs(head_acc_x) > abs(head_acc_y):
            if head_acc_x > 0:
                inverted_module *= -1
        else:
            if head_acc_y > 0:
                inverted_module *= -1

        # Create arc in the given direction.
        arc_angles = np.zeros(len(self._module_names))
        # Account for flipping in hardware
        module_flip = np.ones(len(arc_angles[module_offset::2]))
        module_flip[1::2] *= -1
        arc_angles[module_offset::2] = inverted_module * pole_direction * beta * module_flip

        self._pole_climb_start_angles = arc_angles
        self._pole_direction_initialized = True

    def start_head_look(self):
        """Runs initialization for starting head look.
        Head look handling
            If starting head_look:
                - Save current joint angles
                - Only move `n` modules closest to the head module
            If exiting head_look:
                - Send saved joint angles
        """

        self._joint_angles_before_head_look = self._prev_cmd.position
        self._head_acc = self._get_head_acc()
        self._started_head_look = True

        # Don't update accelerations when moving the head to avoid
        # singularities.
        self._exiting_head_look = False

    def _kill_head_look(self):
        """Clear head look specific attributes (flags)"""

        if hasattr(self, '_joint_angles_before_head_look'):
            delattr(self, '_joint_angles_before_head_look')
            delattr(self, '_started_head_look')
            delattr(self, '_exiting_head_look')
            self._exiting_head_look = False

    def _kill_pole_direction(self):
        """Clear pole climb specific attributes (flags)"""

        if self.pole_climb_initialized:
            delattr(self, '_pole_direction_initialized')

    def manage_last_command(self):
        """Read the last received command and update controller based on that command."""

        self._cmd_name = cmd_name = self._snake_command.command_name

        if cmd_name == 'head_look' or cmd_name == 'head_look_ik':
            if self._just_started_head_look:
                self.start_head_look()
            self._n_exiting_modules = 6 if 'ik' in cmd_name else 2

        if cmd_name != 'pole_direction':
            self._kill_pole_direction()

        # Hold position
        if cmd_name in ["head_look_exit", "home", "pole_direction"]:

            if cmd_name == "pole_direction" and not self.pole_climb_initialized:
                self._create_arc(self._snake_command.param_value[2], self._arc_beta, self._imu_dir)

            if cmd_name == 'head_look_exit':
                if hasattr(self, '_started_head_look'):
                    self._exiting_head_look = True

            # Set hold (fixed) position depending on the command
            if cmd_name == "home":
                target_position = self._home_joint_state.position
                self._kill_head_look()
            elif (not self._just_started_head_look) and self._exiting_head_look:
                target_position = self._joint_angles_before_head_look
            elif cmd_name == "pole_direction":
                target_position = self._pole_climb_start_angles
            else:
                target_position = self._prev_cmd.position

            if not isinstance(self._controller, GoToPositionController):
                if isinstance(self._controller, GaitlibController):
                    self._elapsed_snake_time = self._controller.snake_time
                self._controller = GoToPositionController(
                    self._snake_type,
                    target_position,
                    self._current_joint_state,
                )
            else:
                self._controller.set_go_to_joint_angles(
                    target_position, self._prev_cmd
                )

        # Process the last received gaitlib command
        elif GaitlibController.is_gaitlib_command(self._snake_command):

            transition_override = False
            if (not self._just_started_head_look) and self._exiting_head_look:
                transition_override = True
                self._kill_head_look()

            # Check if a new Gaitlib controller needs to be created
            if not isinstance(self._controller, GaitlibController):
                #print("Creating new GaitlibController.")
                self._controller = GaitlibController(
                    self._snake_type,
                    self._current_joint_state,
                    self._elapsed_snake_time,
                )
                #print("Created new Gaitlib controller.")
            if cmd_name == 'head_look':
                # Pass head accelerations as a gait param.
                self._snake_command.param_name = list(self._snake_command.param_name)
                self._snake_command.param_value = list(self._snake_command.param_value)
                self._snake_command.param_name.append('head_acc')
                self._snake_command.param_value.append(self._head_acc)

            self._controller.update_gait(self._snake_command, self._prev_cmd, transition_override=transition_override)
            
            #print("Finished update_gait.")

        elif cmd_name != 'hold_position':
            rospy.logwarn("Invalid SnakeCommand: %s", self._snake_command)

    def get_next_cmd(self):
        """Get next joint commands from the controller/hold position."""
        exiting_head_look = self._exiting_head_look if hasattr(self, '_exiting_head_look') else False
        n_exiting_modules = self._n_exiting_modules if hasattr(self, '_n_exiting_modules') else 0
        if self._cmd_name != 'hold_position' or exiting_head_look:
            if isinstance(self._controller, GoToPositionController) and exiting_head_look:
                self._desired_joint_state = self._controller.update(self._current_joint_state,\
                                                                    exiting_head_look=exiting_head_look, 
                                                                    n_exiting_modules=n_exiting_modules)

            else:
                self._desired_joint_state = self._controller.update(self._current_joint_state)
        else:
            self._controller._last_time = self._current_joint_state.header.stamp

        return self._desired_joint_state

    def run(self):
        """Main function that runs in a loop to run the robot."""

        rate = rospy.Rate(self._loop_rate)

        rospy.loginfo("Running CommandManager")
        while not rospy.is_shutdown():
            '''
            if not self._sensors_watchdog.timed_out():
                self.manage_last_command()

                desired_joint_state = self.get_next_cmd()

                self._joint_state_pub.publish(desired_joint_state)

                # Keep track of last command for hold_position
                self._prev_cmd = desired_joint_state

            else:
                rospy.logwarn_throttle(
                    0.25, "Sensor watchdog time exceeded. Joint commands not sent."
                )
            '''
   
            self.manage_last_command()
            
            desired_joint_state = self.get_next_cmd()
            
            self._joint_state_pub.publish(desired_joint_state)          

            # Keep track of last command for hold_position
            self._prev_cmd = desired_joint_state
            rate.sleep()

    @property
    def pole_climb_initialized(self):
        return hasattr(self, '_pole_direction_initialized')

    @property
    def _just_started_head_look(self):
        return not hasattr(self, '_joint_angles_before_head_look')
