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
import rospkg
rospack = rospkg.RosPack()

quadrotor = rospack.get_path("nav_quadrotor") + "/launch/spawn_mav_hanging_cylinder.launch"
#TODO: make a proper class encapsulating other robot-specific parameters, etc, such as odometry topic

ControllerLauncher.registerController(name="aerial_pips", filepath=rospack.get_path("nav_quadrotor") + "/launch/aerial_pips.launch")


def run_auto_tests(show_gazebo=False):

    start_time = time.time()
    master = MultiMasterCoordinator(num_masters=1,save_results=True)
    master.start()

    def getTasks():
        controller_freq = 5
        world_num = 0
        for scenario in ['forest_gen']:
            for seed in range(0, 1000):
                for controller in ['aerial_pips']:
                        task = {'controller': controller, 'seed': seed, 'world_num':world_num, 'scenario': scenario, 'robot': quadrotor,
                             'record': False, 'timeout': 120, 'world_args': {"gazebo_gui":show_gazebo},
                                'controller_args': {'controller_freq': controller_freq}}

                        yield task


    master.add_tasks(tasks=getTasks())
    master.wait_to_finish()
    master.shutdown()
    end_time = time.time()
    print( "Total time: " + str(end_time - start_time))


def run_single_task(task, launch_controller=False):
    rospy.init_node('test_driver', anonymous=True)
    rospy.Rate(1).sleep()

    task["robot_impl"] = "quadrotor"
    task["action_server_wait_time"]=0

    scenarios = TestingScenarios()
    scenario = scenarios.getScenario(task)

    rospy.Rate(1).sleep()

    start_time = time.time()
    scenario.setupScenario()
    end_time = time.time()

    print(str(end_time - start_time))

    if launch_controller:
        RosEnv.init(use_existing_roscore=True)
        controller = ControllerLauncher()
        controller.launch(robot=task['robot'], controller_name=task['controller'],
                          controller_args=task['controller_args'])

    try:
        result = run_test(task=task, goal_pose=scenario.getGoalMsg(), record=False)
        print(result)
        if isinstance(result, dict):
            task.update(result)
        else:
            task["result"] = result
        return task
    except rospy.exceptions.ROSInterruptException as e:
        pass
    return None


def run_single_demo_test(launch_controller=False):
    task = {'controller': 'aerial_pips', 'scenario': 'demo_gap', 'robot': quadrotor,
            'record': False, 'world_args': {"gazebo_gui": False},
            'controller_args': {}}
    run_single_task(task=task, launch_controller=launch_controller)

def run_single_clinic_test():
    task = {'controller': 'aerial_pips', 'scenario': 'clinic', 'robot': quadrotor,
            'record': False, 'world_args': {"gazebo_gui": False},
            'controller_args': {}}
    run_single_task(task=task)

def run_hall_tests(show_gazebo=True):

    start_time = time.time()
    master = MultiMasterCoordinator(num_masters=1,save_results=True,use_existing_roscore=True)
    master.start()

    def getTasks():
        controller_freq = 5
        for scenario in ['long_hall']:
            for seed in range(0, 100):
                for controller in ['aerial_pips']:
                        task = {'controller': controller, 'seed': seed, 'scenario': scenario, 'robot': quadrotor,
                             'record': False, 'timeout': 300, 'world_args': {"gazebo_gui":show_gazebo},
                                'controller_args': {'controller_freq': controller_freq}}

                        yield task


    master.add_tasks(tasks=getTasks())
    master.wait_to_finish()
    master.shutdown()
    end_time = time.time()
    print("Total time: " + str(end_time - start_time))


def run_single_hall_test(seed=0, num_obstacles=0):
    task = {'controller': 'aerial_pips', 'seed': seed, 'scenario': 'industrial_plant', 'num_obstacles':num_obstacles, 'robot': quadrotor,
            'record': False, 'timeout': 360, 'world_args': {"gazebo_gui":False}, 'controller_args': {'controller_freq': 5}}
    run_single_task(task=task)


def add_rosbag_key(task):
    import random
    import datetime
    import os

    #for key in ['scenario', 'controller', 'seed',]
    task_str = str(datetime.datetime.now()) + '_' + str('%010x' % random.randrange(16 ** 10)) + ".bag"

    rosbag_path = "~/simulation_data/rosbags/"
    rosbag_path = os.path.join(rosbag_path, task_str)
    rosbag_path = "'" + os.path.expanduser(rosbag_path) + "'"

    if 'controller_args' not in task:
        task['controller_args'] = {}

    task['controller_args']['rosbag_file'] = rosbag_path


def run_new_tests(show_gazebo=True, record=True):

    start_time = time.time()
    master = MultiMasterCoordinator(num_masters=1,save_results=record,use_existing_roscore=True)
    master.fieldnames.extend(['repeat', 'rosbag_file'])
    master.start()

    def getTasks():
        controller_freq = 5
        inner_loop = 10
        outer_loop = 15
        for outer in range(0, outer_loop):
            for scenario in ['industrial_plant', 'clinic', 'demo_gap']: #'industrial_plant', 'clinic', 'demo_gap']:
                for repeat in range(0, inner_loop):

                    for controller in ['aerial_pips']:
                            task = {'controller': controller, 'repeat': outer*inner_loop+repeat, 'scenario': scenario, 'robot': quadrotor,
                                 'record': False, 'timeout': 360, 'world_args': {"gazebo_gui":show_gazebo},
                                    'controller_args': {'controller_freq': controller_freq}}
                            if record:
                                add_rosbag_key(task=task)
                            yield task


    master.add_tasks(tasks=getTasks())
    master.wait_to_finish()
    master.shutdown()
    end_time = time.time()
    print("Total time: " + str(end_time - start_time))



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




def run_other_tests(show_gazebo=True):

    start_time = time.time()
    master = MultiMasterCoordinator(num_masters=1,save_results=True,use_existing_roscore=False)
    master.fieldnames.extend(['repeat', 'rosbag_file'])
    master.start()

    def getTasks():
        controller_freq = 5
        inner_loop = 2
        outer_loop = 10
        for outer in range(0, outer_loop):
            for scenario in ['demo_gap', 'industrial_plant', 'clinic']: #'industrial_plant', 'clinic', 'demo_gap']:
                for repeat in range(0, inner_loop):

                    for controller in ['aerial_pips']:
                            task = {'controller': controller, 'repeat': outer*inner_loop+repeat, 'scenario': scenario, 'robot': quadrotor,
                                 'record': False, 'timeout': 360, 'world_args': {"gazebo_gui":show_gazebo},
                                    'controller_args': {'controller_freq': controller_freq}}
                            yield task


    master.add_tasks(tasks=getTasks())
    master.wait_to_finish()
    master.shutdown()
    end_time = time.time()
    print("Total time: " + str(end_time - start_time))



def run_controller_test():
    rospy.init_node('test_controller', anonymous=True)

    task = {'controller': 'aerial_pips', 'scenario': 'clinic',
            'robot': quadrotor,
            'record': False, 'world_args': {"gazebo_gui": False},
            'controller_args': {}}

    controller = ControllerLauncher(ros_port=11311, use_mp=True)
    controller.launch(robot=task['robot'], controller_name=task['controller'], controller_args=task['controller_args'])

    while not rospy.is_shutdown():
        time.sleep(1)
    #rospy.spin()




def run_gazebo_test():
    port = 11311
    core = RoscoreLauncher(ros_port=port, use_mp=False)
    core.launch()

    rospy.init_node('test_gazebo', anonymous=True)


    scenes = ['demo_gap', 'industrial_plant', 'clinic']

    task = {'controller': 'aerial_pips', 'scenario': 'clinic',
            'robot': quadrotor,
            'record': False, 'world_args': {"gazebo_gui": True},
            'controller_args': {}}


    scenarios = TestingScenarios()

    while True:
        if rospy.is_shutdown():
            break
        for scene in scenes:
            if rospy.is_shutdown():
                break
            task['scenario'] = scene

            scenario = scenarios.getScenario(task)


            gazebo = GazeboLauncher(ros_port=port, gazebo_port=port+100, gazebo_launch_mutex=None, use_mp=False)
            gazebo.launch(world=scenario.getGazeboLaunchFile(), world_args=task['world_args'])
            time.sleep(20)

            gazebo.shutdown()

    time.sleep(10)
    core.shutdown()

    #while not rospy.is_shutdown():
    #    time.sleep(1)
    time.sleep(5)

    # rospy.spin()



def get_scenario_tasks(scenarios, num, show_gazebo, record_rosbag, extra_args=None):
    if not isinstance(scenarios, list):
        scenarios = [scenarios]


    def getTasks():
        controller_freq = 5
        for round in range(2):
            for scenario in scenarios:
                for repeat in range(round * num, (round + 1) * num):
                    for controller in ['aerial_pips']:
                        task = {'controller': controller, 'repeat': repeat, 'scenario': scenario, 'robot': quadrotor,
                             'record': False, 'timeout': 360, 'world_args': {"gazebo_gui":show_gazebo},
                                'controller_args': {'controller_freq': controller_freq}, 'robot_impl': "quadrotor"}
                        if record_rosbag:
                            add_rosbag_key(task=task)
                        if extra_args is not None:
                            task.update(extra_args)
                        yield task

    return getTasks




if __name__ == "__main__":

    #run_auto_tests(show_gazebo=False)

    #for seed in range(2):
    #    run_single_forest_test(seed=seed)

    if False:
        while not rospy.is_shutdown():
            run_single_demo_test(launch_controller=False)
    elif False:
        tasks = get_scenario_tasks(scenarios="demo_gap", num=100, show_gazebo=True)
        multi_test_runner(tasks=tasks(), num_masters=1, save_results=True, use_existing_roscore=True)
    elif False:
        run_single_hall_test()
    else:
        pass

    #run_single_clinic_test(launch_controller=True)
    #run_gazebo_test()
    #run_hall_tests()
    if True:
        #run_single_clinic_test()
        pass
    else:
        #run_controller_test()
        pass
    #run_single_hall_test(num_obstacles=500)
    #run_hall_tests()

    #run_single_demo_test()
    #exit()

    just_one = True
    scenario_type = "hall"

    if scenario_type == "forest":

        forest_task = {'controller': 'aerial_pips', 'seed': 0, 'scenario': 'forest_gen',
                       'robot': quadrotor, 'world_num': 1,
                       'record': False, 'timeout': 360, 'world_args': {"gazebo_gui": True, },
                       'controller_args': {'controller_freq': 5}}

        task = forest_task

        if just_one:
            run_single_task(task=task)
        else:
            show_gazebo = True
            record_rosbag = True
            worlds = [0]
            seeds = [3]
            num_repeats = 4
            runner = multi_test_runner(tasks=get_forest_tasks(world_nums=worlds, repeats=num_repeats, seed_nums=seeds, show_gazebo=show_gazebo,
                                                                record_rosbag=record_rosbag)(), num_masters=1,
                                       save_results=True, use_existing_roscore=True)

    elif scenario_type=="hall":
        is_hard = False
        hall_task = {'controller': 'aerial_pips', 'seed': 0, 'scenario': 'hall_obstacle_course', 'num_obstacles': 0,
                'robot': quadrotor, 'target_z':2,
                'record': False, 'timeout': 360, 'world_args': {"gazebo_gui": False, 'hard': is_hard},
                'controller_args': {'controller_freq': 5}}

        task = hall_task

        if just_one:
            run_single_task(task=task)
        else:
            record_rosbag = True
            show_gazebo = True
            num = 100
            scenarios = ['hall_obstacle_course', 'demo_gap']
            runner = multi_test_runner(tasks=get_scenario_tasks(scenarios=scenarios, num=num, show_gazebo=show_gazebo, record_rosbag=record_rosbag)(), num_masters=1, save_results=True, use_existing_roscore=True)



    #run_single_hall_test()
    #run_new_tests(show_gazebo=True)
    #run_other_tests(show_gazebo=True)
    #run_single_clinic_test()
    #run_manual_test('demo_gap', 0)
