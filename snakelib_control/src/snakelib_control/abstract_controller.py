from abc import ABCMeta, abstractmethod
import rospy
from sensor_msgs.msg import JointState

""" Defines the interface for all controllers (or behaviors) for the snake robots. 

Every controller takes as input the current robot joint state, and outputs a new set of 
joint states. Open-loop gaits will still operate as "controllers", but they will ignore
the current joint state. 

Note that the controller does not store the current joint state, because CommandManager, 
or some other higher-level process, should subscribe to and store the current joint state.

To avoid memory leaks, controllers should NOT have any subscribers or publishers. 
"""


class AbstractController(metaclass=ABCMeta):
    def __init__(self, snake_type):
        """Initialization the AbstractController.

        Args:
            snake_type: string denoting the type of snake, e.g. "REU"
        """
        self._snake_type = snake_type
        self._snake_param = rospy.get_param(f"{self._snake_type}", {})
        self._module_names = self._snake_param.get("module_names", [])

    @property
    @abstractmethod
    def controller_name(self):
        """String denoting the current controller being executed."""
        return None

    @abstractmethod
    def update(self, current_joint_state):
        """Returns a new desired joint state given a current joint state."""
        pass
