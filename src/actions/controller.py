import numpy as np

from src.config import get_config
from src.actions.wheel import CmdVelDiffController

import rospy

from nav_msgs.msg import Odometry

import math

import tf

from std_msgs.msg import Float32MultiArray, MultiArrayDimension

cfg = get_config()

if cfg.mode == "online":
    from communication_msgs.msg import (
        MoveToFeedback,
        MoveToGoal,
        MoveToResult,
        PickupObjectGoal,
        PickupObjectResult,
        PutObjectGoal,
        PutObjectResult,
    )
else:
    from src.actions.communication_msgs import (
        MoveToFeedback,
        MoveToGoal,
        MoveToResult,
        PickupObjectGoal,
        PickupObjectResult,
        PutObjectGoal,
        PutObjectResult,
    )

from omni.isaac.core import World
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.kit import SimulationApp
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)
from omni.usd import get_world_transform_matrix

from src.actions.base_server import ActionServer
from src.config import Config
from src.controllers.pick_place import PickPlaceController
from src.sensors.cameras import setup_cameras
from src.sensors.imu import setup_imu_graph
from src.sensors.lidar import setup_lidar_graph
from src.sensors.tf import setup_tf_graph
from src.tasks.pick_place import PickPlace





class HuskyController:
    def __init__(self, cfg: Config, world: World, simulation_app: SimulationApp) -> None:
        """
        A class that initializes and controls the Husky robot in the simulated environment.

        Args:
            cfg (Config): The configuration object for the simulation.
            world (World): The Isaac simulation world object.
            simulation_app (SimulationApp): The Isaac simulation application object.
        """
        # Initialize attributes
        self.odom_subscriber = rospy.Subscriber('/cartographer/tracked_global_odometry', Odometry, self.odom_callback)
        self._husky_position = [0, 0]
        self._stop_distance = 0.05
        self._start_distance = 0
        self.task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=5)
        self._cfg = cfg
        self.world = world
        self.simulation_app = simulation_app
        self._pick_task = PickPlace(
            name="denso_pick_place",
            cfg=cfg,
        )
        self.world.add_task(self._pick_task)
        self.world.reset()
        self._denso = self.world.scene.get_object("my_ur5")
        self.world.reset()
        self._husky = self._pick_task.robots["husky"]
        self.controller = CmdVelDiffController('difcontrol', self._cfg)
        self._ur5 = self._pick_task.robots["ur5"]
        self._cube = self._pick_task.obj["cube"]
        self._scene = {}
        self._obj_on_hand = ""
        self.world.reset()
        self._prim_trans_point = self._pick_task.robots["trans_point"]
        self._husky_controller = WheelBasePoseController(
            name="cool_controller",
            open_loop_wheel_controller=DifferentialController(
                name="simple_control", wheel_radius=cfg.wheel_radius, wheel_base=cfg.wheel_base
            ),
            is_holonomic=False,
        )
        self._pick_place_controller = PickPlaceController(
            name="controller",
            robot_articulation=self._denso,
            gripper=self._denso.gripper,
            cfg=cfg,
        )
        self._task_params = self.world.get_task("denso_pick_place").get_params()
        self._articulation_controller = self._denso.get_articulation_controller()

        # Set up sensors if enabled
        if self._cfg.use_sensors:
            self._setup_sensors()


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self._husky_position[0], self._husky_position[1] = self.current_pose.position.x, self.current_pose.position.y

    def _setup_sensors(self):
        """
        Sets up the sensors for the simulation.
        """
        our_stage = get_current_stage()

        self.simulation_app.update()
        setup_cameras(
            self._cfg, self.simulation_app, our_stage
        )  # , ["zed", "realsense_rear", "realsense_front"]

        setup_tf_graph(self._cfg, self.simulation_app, our_stage)
        setup_lidar_graph(self._cfg, self.simulation_app, our_stage)
        setup_imu_graph(self._cfg, self.simulation_app, our_stage)

    def move_to_location(self, task: MoveToGoal, action_server: ActionServer) -> None:
        """
        Moves the Husky robot to a specified location.

        Args:
            task: The task object containing the location to move to.
            action_server: The action server object for the move to location action.
        """
        stage = get_current_stage()

        _location = task.location.replace(" ", "_")
        _object = task.object.replace(" ", "_")

        obj_move_str = ""
        if _location != "unspecified":
            obj_move_str = _location
        elif _object != "unspecified":
            obj_move_str = _object
        else:
            return False
        print(f"obj_move_str: {str(obj_move_str)}")
        prim_obj_move = stage.GetPrimAtPath(f"/background/{obj_move_str}")
        pose_obj_move = get_world_transform_matrix(prim_obj_move)
        position_obj_move = pose_obj_move.ExtractTranslation()

        print(f"Now move to {obj_move_str}, position {position_obj_move.__repr__()}\n")

        task = Float32MultiArray()
        task.layout.data_offset = 0
        task.layout.dim.append(MultiArrayDimension())
        task.layout.dim[0].label = "width"
        task.layout.dim[0].size = 5
        task.layout.dim[0].stride = 5

        position, orientation = self._husky.get_world_pose() # be carefull, this is world frame odometry

        _, __, target_theta = tf.transformations.euler_from_quaternion(orientation)

        

        task.data = [position[0], position[1], position_obj_move[0], position_obj_move[1], 0]
        self.task_publisher.publish(task)

    def move_to_location_by_coordinates(self, goal: list, action_server: ActionServer) -> None:
        """
        Moves the Husky robot to a specified location.

        Args:
            task: The task object containing the location to move to.
            action_server: The action server object for the move to location action.
        """
        stage = get_current_stage()

        # BE CAREFULL THIS IS WORLD TOPIC ODOMETRY
        # position, orientation = self._husky.get_world_pose()

        self._start_distance = math.sqrt((goal[0] - self._husky_position[0]) ** 2 + (goal[1] - self._husky_position[1]) ** 2)

        while (self._stop_distance * self._start_distance < math.sqrt((goal[0] - self._husky_position[0]) ** 2 + (goal[1] - self._husky_position[1]) ** 2)):
            print("MIN_DIST : " + str(self._stop_distance * self._start_distance) + ' ' + "ACTUAL_DIST: " + str(math.sqrt((goal[0] - self._husky_position[0]) ** 2 + (goal[1] - self._husky_position[1]) ** 2)))
            
            task = Float32MultiArray()
            task.layout.data_offset = 0
            task.layout.dim.append(MultiArrayDimension())
            task.layout.dim[0].label = "width"
            task.layout.dim[0].size = 5
            task.layout.dim[0].stride = 5
        
            task.data = [self._husky_position[0], self._husky_position[1], goal[0], goal[1], 0]

            print([self._husky_position[0], self._husky_position[1], goal[0], goal[1], 0])

            self.task_publisher.publish(task)
            self.world.step(render=True)
            self.move_by_cmd_vel_msgs()


    def move_to_location_by_trajectory(self, task: list, action_server: ActionServer) -> None:
        """
        Moves the Husky robot to a specified location.

        Args:
            task: The task object containing the trajectory to move on (as coordinate sequense).
            action_server: The action server object for the move to location action.
        """

        stage = get_current_stage()

        distance = 100

        stop_distance = 1.2


        # task = list(list(obj: x, y))

        for goal in task:
            self.move_to_location_by_coordinates([goal.x, goal.y], action_server)

        # Sending result
        result = MoveToResult()
        result.result = f"Done!"
        action_server.put_result(result)
        print(f"Result sent: {result}")
    

    def move_by_cmd_vel_msgs(self):
        self._husky.apply_wheel_actions(self.controller.forward())

    def move_by_keyboard(self, vel):
        #print("KEY ACTION", vel)
        self._husky.apply_wheel_actions(vel)


    def pickup_object(self, task: PickupObjectGoal, action_server: ActionServer) -> None:
        """
        Picks up an object with the UR5 manipulator.

        Args:
            task: The task object containing the object to pick up.
            action_server: The action server, responsible for this task types.
                            Needed to send feedback and result back to planner.
        """
        _object = task.object.replace(" ", "_")

        obj_pick_str = ""
        if _object != "unspecified":
            obj_pick_str = _object
        else:
            return False

        stage = get_current_stage()
        prim_obj_pick = stage.GetPrimAtPath(f"/background/{obj_pick_str}")
        pose_obj_pick = get_world_transform_matrix(prim_obj_pick)
        position_obj_pick = pose_obj_pick.ExtractTranslation()
        print(f"Now move to {obj_pick_str}, position {position_obj_pick}\n")

        position, orientation = self._husky.get_world_pose()

        for _ in range(10):
            wheel_actions = self._husky_controller.forward(
                start_position=position,
                start_orientation=orientation,
                goal_position=position[:2],
                lateral_velocity=self._cfg.lateral_velocity,
                yaw_velocity=self._cfg.yaw_velocity,
                position_tol=self._cfg.position_tol,
            )
            wheel_actions.joint_velocities = np.tile(wheel_actions.joint_velocities, 2)
            self._husky.apply_wheel_actions(wheel_actions)
            self.world.step(render=True)

        position, orientation = self._husky.get_world_pose()
        self._pick_place_controller._cspace_controller.reset()

        while not self._pick_place_controller.pick_done():
            # wheel_actions=self._husky_controller.forward(start_position=position,
            #                                         start_orientation=orientation,
            #                                         goal_position=position[:2],
            #                                         lateral_velocity=self._cfg.lateral_velocity,
            #                                         yaw_velocity=self._cfg.yaw_velocity,
            #                                         position_tol = self._cfg.position_tol,)
            # wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
            # self._husky.apply_wheel_actions(wheel_actions)
            self.world.step(render=True)

            observations = self.world.get_observations()
            end_effector_offset = np.array(self._cfg.end_effector_offset) - np.array([0, 0, 0.0])  # For cup
            # end_effector_orientation =

            actions = self._pick_place_controller.forward(
                # picking_position=cube.get_world_pose()[0],
                picking_position=position_obj_pick,
                placing_position=observations[self._task_params["cube_name"]["value"]][
                    "target_position"
                ],  # # the placing position is not matter here, because only pick
                current_joint_positions=observations[self._task_params["robot_name"]["value"]][
                    "joint_positions"
                ],
                # end_effector_offset=np.array([0, 0, 0.25]),
                end_effector_offset=end_effector_offset,
                end_effector_orientation=euler_angles_to_quat(np.array([0, np.pi, 0.5 * np.pi])),
                prim_trans_point=self._prim_trans_point,
            )
            self._articulation_controller.apply_action(actions)
        # ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"]

        self._pick_place_controller.pause()
        print("pick done!")
        result = PickupObjectResult()
        result.result = f"Done! Object {obj_pick_str} is on hand"
        action_server.put_result(result)
        print(f"Result sent: {result}")

    def put_object(self, task: PutObjectGoal, action_server: ActionServer) -> None:
        _location = task.location.replace(" ", "_")
        _object = task.object.replace(" ", "_")

        obj_put_str = ""
        obj_loc_str = ""
        if _object != "unspecified":
            obj_put_str = _object
        else:
            return False
        if _location != "unspecified":
            obj_loc_str = _location
        else:
            return False
        print(f"Now put {obj_put_str} on {obj_loc_str}\n")
        stage = get_current_stage()
        prim_obj_put_2 = stage.GetPrimAtPath(f"/background/{obj_loc_str}")
        pose_obj_2 = get_world_transform_matrix(prim_obj_put_2)
        position_obj_2 = pose_obj_2.ExtractTranslation()
        if (obj_loc_str == "table") or (obj_loc_str == "drawer"):
            prim_obj_put_2 = stage.GetPrimAtPath(f"/World/{obj_loc_str}/point")
            pose_obj_2 = get_world_transform_matrix(prim_obj_put_2)
            position_obj_2 = pose_obj_2.ExtractTranslation()
            position_obj_2[-1] = (
                position_obj_2[-1] + 0.036 if obj_loc_str == "table" else position_obj_2[-1] + 0.06
            )  # offset of place
            print(f"position_obj_loc: {position_obj_2}\n")
        else:
            position_obj_2[-1] = position_obj_2[-1] + 0.4

        position, orientation = self._husky.get_world_pose()

        for _ in range(10):
            wheel_actions = self._husky_controller.forward(
                start_position=position,
                start_orientation=orientation,
                goal_position=position[:2],
                lateral_velocity=self._cfg.lateral_velocity,
                yaw_velocity=self._cfg.yaw_velocity,
                position_tol=self._cfg.position_tol,
            )
            wheel_actions.joint_velocities = np.tile(wheel_actions.joint_velocities, 2)
            self._husky.apply_wheel_actions(wheel_actions)
            self.world.step(render=True)

        position, orientation = self._husky.get_world_pose()
        self._pick_place_controller._cspace_controller.reset()

        self._pick_place_controller.resume()
        print(f"is done: {self._pick_place_controller.is_done()}")
        while not self._pick_place_controller.is_done():
            self.world.step(render=True)

            observations = self.world.get_observations()
            end_effector_orientation = (
                euler_angles_to_quat(np.array([0, np.pi, 0.5 * np.pi]))
                if obj_loc_str == "table"
                else euler_angles_to_quat(np.array([0, np.pi, 0.8 * np.pi]))
            )

            actions = self._pick_place_controller.forward(
                picking_position=self._cube.get_world_pose()[0],
                # placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
                placing_position=position_obj_2,
                current_joint_positions=observations[self._task_params["robot_name"]["value"]][
                    "joint_positions"
                ],
                # end_effector_offset=np.array([0, 0, 0.25]),
                end_effector_offset=np.array(self._cfg.end_effector_offset),
                end_effector_orientation=end_effector_orientation,
                prim_trans_point=self._prim_trans_point,
                ee_pose=observations[self._task_params["robot_name"]["value"]]["end_effector_position"],
            )

            self._articulation_controller.apply_action(actions)
        self._pick_place_controller.reset()
        result = PutObjectResult()
        result.result = f"Done! Object {obj_put_str} is on hand"
        action_server.put_result(result)
        print(f"Result sent: {result}")
