#!/usr/bin/env python3

from __future__ import print_function
from builtins import str
from builtins import range
from builtins import object
from nav_scripts.gazebo_master import MultiMasterCoordinator
from nav_scripts.testing_scenarios import TestingScenario, TestingScenarios, GeneralScenario
from nav_scripts.controller_launcher import ControllerLauncher
from nav_scripts.ros_launcher_helper import RosEnv, GazeboLauncher, RoscoreLauncher
from nav_scripts.gazebo_driver import GazeboDriver
from nav_scripts.movebase_driver import run_test
import nav_quadrotor.scenarios
import nav_quadrotor.quadrotor_impl
import rospy
from geometry_msgs.msg import PoseStamped
import time
import csv
import math
from pathlib import Path
import rospkg
rospack = rospkg.RosPack()

quadrotor = rospack.get_path("nav_quadrotor") + "/launch/spawn_mav_hanging_cylinder.launch"
#TODO: make a proper class encapsulating other robot-specific parameters, etc, such as odometry topic

ControllerLauncher.registerController(name="aerial_pips", filepath=rospack.get_path("nav_quadrotor") + "/launch/aerial_pips.launch")


def add_rosbag_key(task):
    import random
    import datetime
    import os

    #for key in ['scenario', 'controller', 'seed',]
    task_str = str(datetime.datetime.now()) + '_' + str('%010x' % random.randrange(16 ** 10)) + ".bag"

    rosbag_path = "/tmp/simulation_data/rosbags/"
    Path(rosbag_path).mkdir(parents=True, exist_ok=True)
    rosbag_path = os.path.join(rosbag_path, task_str)
    rosbag_path = "'" + os.path.expanduser(rosbag_path) + "'"

    if 'controller_args' not in task:
        task['controller_args'] = {}

    task['controller_args']['rosbag_file'] = rosbag_path



def multi_test_runner(tasks, num_masters=1, save_results=False, use_existing_roscore=False):

    start_time = time.time()
    master = MultiMasterCoordinator(num_masters=num_masters,save_results=save_results, use_existing_roscore=use_existing_roscore)
    master.fieldnames.extend(['repeat', 'rosbag_file'])
    master.start()

    master.add_tasks(tasks=tasks)
    master.wait_to_finish()
    master.shutdown()
    end_time = time.time()
    print("Total time: " + str(end_time - start_time))



def get_scenario_tasks(scenarios, num, show_gazebo, record_rosbag, extra_args=None):
    if not isinstance(scenarios, list):
        scenarios = [scenarios]


    def getTasks():
        controller_freq = 5
        for scenario in scenarios:
            for repeat in range(num):
                task = {'controller': 'aerial_pips', 'repeat': repeat, 'scenario': scenario, 'robot': quadrotor,
                     'record': False, 'timeout': 360, 'world_args': {"gazebo_gui":show_gazebo},
                        'controller_args': {'controller_freq': controller_freq}, 'robot_impl': "quadrotor"}
                if record_rosbag:
                    add_rosbag_key(task=task)
                if extra_args is not None:
                    task.update(extra_args)
                yield task

    return getTasks




if __name__ == "__main__":
    record_rosbag = True
    show_gazebo = True
    num = 5
    scenarios = ['hall_obstacle_course', 'demo_gap']
    runner = multi_test_runner(tasks=get_scenario_tasks(scenarios=scenarios, num=num, show_gazebo=show_gazebo, record_rosbag=record_rosbag)(), num_masters=1, save_results=True, use_existing_roscore=False)



