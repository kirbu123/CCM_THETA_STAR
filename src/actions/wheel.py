import copy
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)

from src.config import Config


class DifferentialController4(DifferentialController):
    def forward(self, command: np.ndarray) -> ArticulationAction:
        rs = super().forward(command)
        rs.joint_velocities = np.tile(rs.joint_velocities, 2)
        return rs


class CmdVelDiffController(BaseController):
    def __init__(self, name: str, cfg: Config, data_timeout_sec: float = 1.0) -> None:
        super().__init__(name)
        self._sub: rospy.Subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, self._subscriber_callback, queue_size=5
        )
        self._data: Twist = Twist()
        self._mult_vel = 4
        self._data_time = time.time()
        self._data_timeout = data_timeout_sec
        self._data_timeout_timer = rospy.Timer(
            rospy.Duration(nsecs=int(self._data_timeout * 10**9)),
            self._data_timeout_callback,
        )
        self._dif_controller = DifferentialController4(
            f"{name}_diff", cfg.wheel_radius, cfg.wheel_base
        )

    def forward(self) -> ArticulationAction:
        rot = self._mult_vel * self.data.angular.z
        forward = self._mult_vel * self.data.linear.x
        print("PARAMS: " + str([forward, rot]))
        action = self._dif_controller.forward([forward, rot])
        return action

    def _data_timeout_callback(self, args):
        if time.time() - self._data_time > self._data_timeout:
            self.data = Twist()

    def _subscriber_callback(self, data: Twist):
        self.data = data

    @property
    def data(self) -> Twist:
        return copy.copy(self._data)

    @data.setter
    def data(self, val: Twist):
        self._data = val
        self._data_time = time.time()
