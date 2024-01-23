import hebi
import time
import rospy


class HEBIROSWrapper:
    r"""Base class to establish interfacing with HEBI modules using the HEBI API."""

    def __init__(self):
        # Get robot specific configs.
        self.robot_name = rospy.get_param(f"snake_type", "REU")
        robot_configs = rospy.get_param(f"{self.robot_name}", {})

        # Get the module names.
        self.module_names = robot_configs.get("module_names", [])
        self.max_torque = robot_configs.get("max_torque", 7)  # Nm

        lookup_success = False
        n_lookup_tries = 0
        while (lookup_success is False) and n_lookup_tries  < 5:
            lookup_success = self.lookup_modules()
            n_lookup_tries += 1
            rospy.loginfo(f'Look try: {n_lookup_tries}, success: {lookup_success}')
        if not lookup_success:
            raise Exception(f"Robot not detected or robot initialized not complete.")
        

        self.num_modules = self.robot.size

        self.feedback_frequency = rospy.get_param(
            "hebi_feedback_frequency", 100.0
        )  # Hz
        self.control_frequency = rospy.get_param("hebi_control_frequency", 100.0)  # Hz
        self.robot.command_lifetime = rospy.get_param(
            "hebi_command_lifetime", 1000.0
        )  # ms

        # Issue a warning should the loop frequence be too low.
        if min(self.feedback_frequency, self.control_frequency) < (
            1000 / self.robot.command_lifetime
        ):
            rospy.logwarn_throttle(
                f"Send commands faster than command lifetime ({self.robot.command_lifetime} ms) to avoid missing \
                    commands (uncontrolled intervals). \n Control freq: {self.control_frequency} Hz, Feedback freq: \
                    {self.feedback_frequency} Hz",
            )

    def lookup_modules(self):
        # Find online modules.
        self.lookup = hebi.Lookup()
        rospy.sleep(
            2.0
        )  # Allow some time for the lookup procedure to find online modules.

        rospy.loginfo(f"--Looking up modules--")
        msg = ''
        for entry in self.lookup.entrylist:
            msg += f'{entry.family} - {entry.name} ||'
        rospy.loginfo(msg)

        # Initialize robot handle.
        self.robot = self.lookup.get_group_from_names("*", self.module_names)
        lookup_success = self.robot is not None
        return lookup_success

    def start(self):
        r"""Run loop until ros is not shutdown."""
        rate = rospy.Rate(self.loop_rate)

        while not rospy.is_shutdown():
            self.run_loop()
            rate.sleep()

    def run_loop(self):
        pass
