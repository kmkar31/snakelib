import numpy as np
import pybullet as p
import pybullet_data
import rospkg
from time import sleep


class SnakeBullet:
    """Class which handles the PyBullet environment for snake.

    Attributes:
        type (str) : Type of snake. Can be 'SEA','REU' or 'RSNAKE'
        num_of_modules : Number of modules for the snake. Defaults to 16.
        max_torque (int): Maximum joint torque applied. Defaults to 7.
        terrain (str) : Environment terrain. Can be 'flat' or 'pole'
        tracking_cam (bool) : Set if the camera should follow robot or not
        motor_idx ((16,1) np array): Motor index.
        robot_id : PyBullet generated ID for robot URDF.

    Methods:
        step(signal,sleep_time=0.01): Send position command to the snake sim and wait for some seconds.
        get_feedback(): Return Position, Velocity and Effort feedback from sim snake.
        load_terrain(): Load the user specified terrain into sim.
        load_urdf(): Load appropriate(SEA, REU or RSNAKE) URDF as specified by user.
        set_dynamics(dynamics): Set simulation friction dynamics.
    """

    def __init__(
        self, snake_type, num_robots=1, num_of_modules=16, terrain="flat", max_torque=7
    ):
        """Initialize PyBullet sim, load URDF and terrain.

        Args:
            snake_type (str) : Type of snake. Can be 'SEA', 'REU' or 'RSNAKE'
            num_of_modules (int, optional) : Number of modules for the snake. Defaults to 16.
            terrain (str, optional) : Environment terrain. Defaults to 'flat'.
            max_torque (int, optional) : Maximum joint torque applied. Defaults to 7.
        """
        self.type = snake_type
        self.num_of_modules = num_of_modules
        self.max_torque = max_torque

        self.terrain = terrain
        self.tracking_cam = False
        self.num_robots = num_robots
        self.robot_id = [0.0] * num_robots

        p.connect(p.GUI)
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(enableFileCaching=0)

        self.motor_idx = np.zeros(self.num_of_modules)

        self.load_terrain()
        self.robot_id = self.load_urdf()

        # For debugging new URDFs
        # print(self.motor_idx)
        # print(p.getNumJoints(self.robot_id))

        p.setGravity(0, 0, -9.81)  # everything should fall down
        # p.setTimeStep(0.01)       # this slows everything down, but let's be accurate...
        # p.setRealTimeSimulation(0)  # we want to be faster than real time :)

    def __del__(self):
        p.disconnect()

    def step(self, signal, sleep_time=0.01):
        """Send position command to the snake sim and wait for sleep_time seconds.

        Args:
            signal (list): List(len = number_of_modules) of joint command angles in radians.
            sleep_time (float, optional): Sleep time. Defaults to 0.01.
        """
        forces = (
            np.ones([self.motor_idx.shape[0]]) * self.max_torque
        )  # Max Applied Torque (Nm)

        for i in range(self.num_robots):
            if self.num_robots == 1:
                p.setJointMotorControlArray(
                    self.robot_id[i],
                    self.motor_idx.tolist(),
                    p.POSITION_CONTROL,
                    targetPositions=signal,
                    forces=forces.tolist(),
                )
            else:
                p.setJointMotorControlArray(
                    self.robot_id[i],
                    self.motor_idx.tolist(),
                    p.POSITION_CONTROL,
                    targetPositions=signal[i].tolist(),
                    forces=forces.tolist(),
                )
            p.stepSimulation()
        if self.tracking_cam:
            self.update_cam_view()
        sleep(sleep_time)

    def get_feedback(self):
        """Return Position, Velocity and Effort feedback from sim snake.

        Returns:
            position(list),velocity(list),effort(list): Returns a list of position, velocity and effort feedback.
        """

        position = np.array([0.0] * self.num_of_modules)
        velocity = np.array([0.0] * self.num_of_modules)
        effort = np.array([0.0] * self.num_of_modules)
        for idx, motorNo in enumerate(self.motor_idx):
            jointP, jointV, jointF, _ = p.getJointState(self.robotID, motorNo)
            position[idx] = jointP
            velocity[idx] = jointV
            effort[idx] = jointF[5]  # Mz

        return position.tolist(), velocity.tolist(), effort.tolist()

    def load_terrain(self):
        """Load the user specified terrain into pybullet sim."""

        plane = p.loadURDF("plane.urdf")
        if self.terrain == "pole":
            pole = p.loadURDF("/models/environments/pole.urdf", useFixedBase=1)

    def load_urdf(self):
        """Load appropriate(SEA, REU or RSNAKE) URDF as specified by user

        Returns:
            robot: PyBullet URDF ID
        """

        robot = [0.0] * self.num_robots
        rp = rospkg.RosPack()

        for i in range(self.num_robots):
            if self.type == "SEA":
                robot[i] = p.loadURDF(
                    rp.get_path("snakelib_description") + "/sea_snake/sea_snake.urdf",
                    [0 + i * 1, 0, 0],
                    useFixedBase=0,
                    flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                )
                self.motor_idx = np.arange(3, 3 * (self.num_of_modules + 1), 3)
                self.robot_id[i] = robot
            elif self.type == "REU":
                robot[i] = p.loadURDF(
                    rp.get_path("snakelib_description") + "/reu_snake/reu_snake.urdf",
                    [0 + i * 1, 0, 0],
                    useFixedBase=0,
                    flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                )
                self.motor_idx = np.arange(0, 3 * (self.num_of_modules), 3)
            elif self.type == "RSNAKE":
                robot[i] = p.loadURDF(
                    rp.get_path("snakelib_description") + "/rsnake/rsnake.urdf",
                    [0 + i * 1, 0, 0],
                    useFixedBase=0,
                    flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                )
                self.motor_idx = np.arange(0, self.num_of_modules)
            else:
                robot[i] = None
        return robot

    def update_cam_view(self, dist=2.0, yaw=0, pitch=-89):
        """Track robot by changing camera view"""
        pos_sum = np.zeros(3)
        for i in range(self.num_robots):
            pos, *_ = p.getLinkState(
                self.robot_id[i],
                self.motor_idx.tolist()[int(self.num_of_modules / 2) - 1],
            )
            pos_sum += pos
        p.resetDebugVisualizerCamera(dist, yaw, pitch, pos_sum / self.num_robots)
        return 0

    def set_dynamics(self, dynamics):
        """Set simulation friction dynamics.

        Args:
            dynamics (str): Set as 'xFriction' (for Lateral Undulation type gaits),
                            'yFriction' (for Sidewinding type gaits),
                            'Friction' (for Linear Progression type gaits)
        """

        if dynamics == "xFriction":
            # Set these Dynamics for Lateral Undulation
            anistropicFriction = [1, 0.1, 0.01]
            # For base link
            p.changeDynamics(
                self.robotID,
                -1,
                lateralFriction=2,
                anisotropicFriction=anistropicFriction,
            )
            # For other links
            for i in range(p.getNumJoints(self.robotID)):
                p.getJointInfo(self.robotID, i)
                p.changeDynamics(
                    self.robotID,
                    i,
                    lateralFriction=2,
                    anisotropicFriction=anistropicFriction,
                )
                p.enableJointForceTorqueSensor(self.robotID, i, 1)

        elif dynamics == "yFriction":
            # Set these Dynamics for Sidewinding
            anistropicFriction = [0.1, 1.2, 1.2]
            lateralFriction = 0.2  # 0.2
            # For base link
            p.changeDynamics(
                self.robotID,
                -1,
                lateralFriction=lateralFriction,
                anisotropicFriction=anistropicFriction,
            )
            # For other links
            for i in range(p.getNumJoints(self.robotID)):
                p.getJointInfo(self.robotID, i)
                p.changeDynamics(
                    self.robotID,
                    i,
                    lateralFriction=lateralFriction,
                    anisotropicFriction=anistropicFriction,
                )
                p.enableJointForceTorqueSensor(self.robotID, i, 1)

        elif dynamics == "Friction":
            # for linear progression
            spinningFriction = 0
            lateralFriction = 1.2
            rollingFriction = 0
            # For base link
            p.changeDynamics(
                self.robotID,
                -1,
                lateralFriction=lateralFriction,
                spinningFriction=spinningFriction,
                rollingFriction=rollingFriction,
            )
            # For other links
            for i in range(p.getNumJoints(self.robotID)):
                p.getJointInfo(self.robotID, i)
                p.changeDynamics(
                    self.robotID,
                    i,
                    lateralFriction=lateralFriction,
                    spinningFriction=spinningFriction,
                    rollingFriction=rollingFriction,
                )
                p.enableJointForceTorqueSensor(self.robotID, i, 1)
        else:
            pass
