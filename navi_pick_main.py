#!/usr/bin/env python3
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# This import should be there according to examples
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})

import traceback
import queue
import threading
import time
import carb
from omni.isaac.core import World
from omni.isaac.core.utils import extensions

from src.actions.base_server import ActionServer
from src.actions.controller import HuskyController
from src.actions.scenario import add_scenario_to_queue, setup_scene_for_scenario
from src.config import get_config
from src.scene_setup.environment import setup_scene_background

# from omni.isaac.core.utils.stage import get_current_stage


# Control ROBOT with KEYBOARD
from carb.input import KeyboardEventType

import omni.appwindow
import carb.input
import numpy as np



import carb
import omni


from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)

class DifferentialController4(DifferentialController):
    def forward(self, command: np.ndarray):
        rs = super().forward(command)
        rs.joint_velocities = np.tile(rs.joint_velocities, 2)
        return rs

action = np.array([0.0, 0.0])


def keyboard_events(event):
    global action
    if (event.type == carb.input.KeyboardEventType.KEY_PRESS
        or event.type == carb.input.KeyboardEventType.KEY_REPEAT):
        print("KEY PRESSED")
        if event.input == carb.input.KeyboardInput.W:  #NUMPAD_8
            base_controller.initialVelocity = 0.3
            base_controller.initialAccel = 0.2
            action = action*0.2 + 0.8*np.array([20, 0.0])
        if event.input == carb.input.KeyboardInput.S: # NUMPAD_2
            base_controller.initialVelocity = 0.3
            base_controller.initialAccel = 0.2
            action = action*0.2 + 0.8*np.array([-20, 0.0])
        if event.input == carb.input.KeyboardInput.A: #NUMPAD_4
            base_controller.initialVelocity = 0.5
            base_controller.initialAccel = 0.2
            action =action*0.5 + 1.5*np.array([0.0, np.pi / 1])
        if event.input == carb.input.KeyboardInput.D: # NUMPAD_6
            base_controller.initialVelocity = 0.5
            base_controller.initialAccel = 0.2
            action = action*0.5 + 1.5*np.array([0.0, -np.pi / 1])
        print(action)
    if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        action = np.array([0.0, 0.0])

app_window = omni.appwindow.get_default_app_window()

keyboard = app_window.get_keyboard()
input = carb.input.acquire_input_interface()

keyboard_sub_id = input.subscribe_to_keyboard_events(keyboard, keyboard_events)

appwindow = omni.appwindow.get_default_app_window()
keyboard = app_window.get_keyboard()
input = carb.input.acquire_input_interface()

keyboard_sub_id = input.subscribe_to_keyboard_events(keyboard, keyboard_events)
base_controller = DifferentialController4('husky_base_controller', 0.3, 0.5)



# END of control ROBOT KEYBOARD


cfg = get_config()

# enable ROS bridge extension
extensions.enable_extension(cfg.ros_cfg[cfg.ros].ros_bridge_extension)

if cfg.ros_cfg[cfg.ros].ros_v == 1:
    import rosgraph
    import rospy
    from rosnode import get_node_names

    rospy.init_node('navi_pick_main_node')
    if not rosgraph.is_master_online():
        carb.log_error("Please run roscore before executing this script")
        simulation_app.close()
        exit()

if cfg.mode == "online":
    from src.actions.action_servers import (
        MoveToActionServer,
        OpenSeedActionServer,
        PickupObjectActionServer,
        PutObjectActionServer,
    )

    if cfg.ros == "noetic":
        import rospy


my_world = World(stage_units_in_meters=1.0)

setup_scene_background(simulation_app=simulation_app, cfg=cfg)

controller = HuskyController(cfg=cfg, world=my_world, simulation_app=simulation_app)

# Need to initialize physics getting any articulation..etc
my_world.initialize_physics()

task_queue = queue.Queue()
action_servers = {}

if cfg.mode == "idle":
    pass
elif cfg.mode == "online":

    def server_thread():
        rospy.spin()  # This will block and process ROS callbacks

    rospy.init_node("HuskyActionServer", anonymous=True)

    action_servers["move_to_location"] = MoveToActionServer(
        server_name="move_to_location",
        task_queue=task_queue,
    )
    action_servers["pick_up_object"] = PickupObjectActionServer(
        server_name="pick_up_object",
        task_queue=task_queue,
    )
    action_servers["put_object"] = PutObjectActionServer(
        server_name="put_object",
        task_queue=task_queue,
    )

    action_servers["open_seed_setter"] = OpenSeedActionServer(
        server_name="text_query_generation",
        task_queue=task_queue,
    )

    t = threading.Thread(target=server_thread)
    t.start()

elif cfg.mode == "offline":
    # Fake servers to adhere to a single payline
    action_servers["move_to_location"] = ActionServer(
        server_name="move_to_location",
    )
    action_servers["move_to_location_by_coordinates"] = ActionServer(
        server_name="move_to_location_by_coordinates",
    )
    action_servers["move_to_location_by_trajectory"] = ActionServer(
        server_name="move_to_location_by_trajectory",
    )
    action_servers["pick_up_object"] = ActionServer(
        server_name="pick_up_object",
    )
    action_servers["put_object"] = ActionServer(
        server_name="put_object",
    )
    action_servers["open_seed_setter"] = ActionServer(
        server_name="text_query_generation",
    )

    # Add scenario to queue
    task_queue.put(("move_to_location_by_coordinates", [1.9440345764160156, -2.3873443603515625])) # data: [-0.01564454473555088, -0.1015431135892868, 1.9440345764160156, -2.3873443603515625, 0.0]
    add_scenario_to_queue(cfg, task_queue)
    setup_scene_for_scenario(cfg, my_world)

else:
    raise ValueError(f"Unknown mode: {cfg.mode}")

########################################## server-client connection ##########################################

import rospy
# import actionlib

# from track_pkg.msg import TrackCoordinateAction, TrackCoordinateActionGoal, TrackCoordinateActionResult, TrackCoordinateActionFeedback
# from track_pkg.msg import TrackTrajectoryAction, TrackTrajectoryActionGoal, TrackTrajectoryActionResult, TrackTrajectoryActionFeedback

# def track_client_by_coordinate():
#     position, orientation = controller._husky.get_world_pose()
#     client = actionlib.SimpleActionClient('move_to_location', TrackCoordinateAction)
#     timeout = rospy.Duration(10)
#     if not client.wait_for_server(timeout=timeout):
#         rospy.logerr("Server unreachable during {} seconds".format(timeout.to_sec()))
#         return 0
#     goal = TrackCoordinateActionGoal()
#     goal.goal.x, goal.goal.y = position[:2]
#     client.send_goal(goal.goal)
#     client.wait_for_result()
#     result = client.get_result()
#     rospy.loginfo(f'Server response: x: {result.x} y: {result.y}')
#     if not (result.x, result.y) == (goal.goal.x, goal.goal.y):
#         task_queue.put(("move_to_location_by_coordinates", (result.x, result.y)))
#         rospy.loginfo(f'Task: x: {result.x}, y: {result.y} pushed to task queue')
#     else:
#         rospy.loginfo(f'Goal position is now position, task done by trivial algorithm')
#     return result

# def track_client_by_trajectory():
#     position, orientation = controller._husky.get_world_pose()
#     client = actionlib.SimpleActionClient('move_to_location', TrackTrajectoryAction)
#     timeout = rospy.Duration(10)
#     if not client.wait_for_server(timeout=timeout):
#         rospy.logerr("Server unreachable during {} seconds".format(timeout.to_sec()))
#         return 0
#     goal = TrackTrajectoryActionGoal()
#     goal.goal.x, goal.goal.y = position[:2]
#     client.send_goal(goal.goal)
#     client.wait_for_result()
#     result = client.get_result().result.points
#     rospy.loginfo(f'Server response: {result}')
#     if not len(result) == 0:
#         task_queue.put(("move_to_location_by_trajectory", result))
#         rospy.loginfo(f'Task: {result} pushed to task queue')
#     else:
#         rospy.loginfo(f'No result recieved')
#     return result

# TRACK_PIPELINE = 'TRAJECTORY' # can be switched on COORDINATE

# ######################################### server-client connection ##########################################    

base_controller.reset()

while simulation_app.is_running():
    my_world.step(render=True)

    TELEOP = False
    TASK_BY_GOAL = False
    THETA_STAR_NODENAME = '/action_server' # in code from_rviz.py it's name is tx2_action_server
    if TELEOP:
        controller.move_by_keyboard(base_controller.forward(action))
    elif TASK_BY_GOAL:
        controller.move_by_cmd_vel_msgs()
    else:
        if not task_queue.empty():
            try:
                get_node_names().index(THETA_STAR_NODENAME)
            except Exception:
                print('Theta star node is not activated (Waiting for the planner...)')
                continue
            if my_world.is_playing():
                server_name, task = task_queue.get()
                print(f"Main thread processing: task: {task.__str__()} server_id: {server_name}")

                if server_name == "move_to_location":
                    controller.move_to_location(task, action_servers[server_name])
                    time.sleep(2)
                    task_queue.task_done()
                elif server_name == "move_to_location_by_coordinates":
                    controller.move_to_location_by_coordinates(task, action_servers[server_name])
                    time.sleep(2)
                    task_queue.task_done()
                elif server_name == 'move_to_location_by_trajectory':
                    controller.move_to_location_by_trajectory(task, action_servers[server_name])
                    time.sleep(2)
                    task_queue.task_done()
                elif server_name == "pick_up_object":
                    controller.pickup_object(task, action_servers[server_name])
                    time.sleep(2)
                    task_queue.task_done()
                elif server_name == "put_object":
                    controller.put_object(task, action_servers[server_name])
                    time.sleep(2)
                    task_queue.task_done()
                elif server_name == "text_query_generation":
                    pass
                else:
                    print(f"Unknown server name: {server_name}")
                    print(f"Task: {task.__str__()}")
                    continue
        else:
            print("Waiting for tasks... 1, 2, 3... 10")
            for step in range(10):
                my_world.step(render=True)
                time.sleep(0.05)


simulation_app.close()  # close Isaac Sim
