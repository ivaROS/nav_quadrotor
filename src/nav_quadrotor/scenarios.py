from __future__ import print_function
from builtins import str
from builtins import range
from builtins import object

import time
import csv
import math
import rospkg
import rospy

from geometry_msgs.msg import PoseStamped

from nav_scripts.testing_scenarios import TestingScenario, TestingScenarios, GeneralScenario, TestingScenarioError
from nav_scripts.controller_launcher import ControllerLauncher
from nav_scripts.ros_launcher_helper import RosEnv, GazeboLauncher
from nav_scripts.gazebo_driver import GazeboDriver
from nav_scripts.interruptible import Rate as InterruptibleRate, InterruptedSleepException


def convert_string(value):
    try:
        flt_num = float(value)
        int_num = int(flt_num)

        return int_num if flt_num == int_num else flt_num
    except ValueError:
        return str(value)

#TODO: separate truly general functionality of gazebo_driver from robot-specific
#Also, separate scenario-specific behavior from robot-specific behavior
class QuadrotorGazeboDriver(GazeboDriver):
    def __init__(self, as_node = True, seed=None):
        super(QuadrotorGazeboDriver, self).__init__(as_node, seed)

        self.desired_pose_pub = rospy.Publisher(
            '/hummingbird/command/pose', PoseStamped, queue_size=1)
        self.robotName = 'hummingbird'

    @staticmethod
    def addPosePublisher(instance):
        try:
            a = instance.desired_pose_pub
        except AttributeError:
            instance.desired_pose_pub = rospy.Publisher('/hummingbird/command/pose', PoseStamped, queue_size=1, latch=True )

    def sendDesiredPose(self, pose_stamped):
        self.desired_pose_pub.publish(pose_stamped)


class QuadrotorScenario(GeneralScenario):
    name = "quadrotor"
    nav_frame_id = "world"
    robot_name = "hummingbird"

    def __init__(self, task):
        super(QuadrotorScenario, self).__init__(task=task)
        self.task = task

        robot_args = task['robot_args'] if 'robot_args' in task else {}
        robot_args.update({'odom_topic': '/hummingbird/odometry_sensor1/odometry'})
        task['robot_args'] = robot_args

        self.stability_dist = 0.1
        self.stability_time = rospy.Duration(1)
        self.max_stability_attempts = 5

        #Add publisher
        QuadrotorGazeboDriver.addPosePublisher(instance=self)

    def addInitSpawnPose(self, pose):
        robot_args = self.task['robot_args']  # if 'robot_args' in task else {}
        robot_args.update({'init_x': pose[0], 'init_y': pose[1], 'init_z': pose[2]})

    def setupEnvironment(self):
        pass

    #Override this to for full control over scenario setup
    def setupScenario(self):

        pose_stamped = PoseStamped(pose=self.getStartingPoseMsg())
        pose_stamped.header.frame_id = self.nav_frame_id
        pose_stamped.header.stamp = rospy.Time.now()

        def getRobotPose():
            models = self.gazebo_driver.models
            try:
                ind = models.name.index(self.robot_name)
            except ValueError as e:
                rospy.logerr("Unable to find pose for " + self.robot_name + " in current gazebo environment!")
                raise TestingScenarioError("Unable to find pose for " + self.robot_name + " in current gazebo environment!", task=self.task) from e
            else:
                return models.pose[ind]

        def getDistance(pose1, pose2):
            dx = pose1.position.x - pose2.position.x
            dy = pose1.position.y - pose2.position.y
            dz = pose1.position.z - pose2.position.z

            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            return dist

        def moveRobot():
            self.gazebo_driver.setPose(self.robot_name, pose_stamped.pose)

        def sendDesiredPose():
            self.desired_pose_pub.publish(pose_stamped)

        self.gazebo_driver.checkServicesTopics(10)

        self.gazebo_driver.pause()
        sendDesiredPose()
        moveRobot()
        self.gazebo_driver.reset(self.seed)
        self.setupEnvironment()
        self.gazebo_driver.unpause()

        stability_start_time = None
        stable = False
        rate = 0.2
        #rate = InterruptibleRate(hz=rate, timeout=1.0/(3*rate))
        rate = rospy.Rate(rate)
        num_attempts = 1

        while not stable:
            try:
                rate.sleep()
            except InterruptedSleepException as e:
                raise GazeboLauncher.exc_type_runtime(msg="Spent too long sleeping!") from e

            if num_attempts == self.max_stability_attempts:
                rospy.loginfo("Too many attempts to stabilize robot! [" + str(num_attempts) + "]")
                raise GazeboLauncher.exc_type_runtime(msg="Too many attempts to stabilize robot! [" + str(num_attempts) + "]")

            num_attempts+=1

            self.gazebo_driver.checkServicesTopics(10)

            cur_pose = getRobotPose()
            cur_time = rospy.Time.now()

            #TODO: include 2 metrics: absolute position error, and diminishing velocity
            error = getDistance(pose_stamped.pose, cur_pose)
            if error < self.stability_dist:
                rospy.loginfo("Current pose error is acceptable: " + str(error) + "m")
                if stability_start_time is not None:
                    if cur_time - stability_start_time > self.stability_time:
                        stable = True
                        rospy.loginfo("Stable conditions maintained long enough")
                    else:
                        rospy.loginfo("Stable conditions have not been maintained long enough...")
                else:
                    stability_start_time = cur_time
            else:
                stability_start_time = None
                rospy.loginfo("Pose error too high (" + str(error) + "), resetting robot")
                self.gazebo_driver.pause()
                sendDesiredPose()
                moveRobot()
                self.gazebo_driver.unpause()


class DemoGapScenario(QuadrotorScenario):
    name = "demo_gap"
    world = "demo_gap"

    def __init__(self, task):
        super(DemoGapScenario, self).__init__(task=task)

        self.init_pose = [4.5, -2.7, 1.4, math.pi]
        self.target_pose = [-0.28, -2.2, 2, math.pi]
        self.addInitSpawnPose(pose=self.init_pose)


    def setupEnvironment(self):
        pass

    def getGazeboLaunchFile(self):
        return self.rospack.get_path("nav_quadrotor") + "/launch/gazebo_demo.launch"

    @staticmethod
    def getUniqueFieldNames():
        return []

TestingScenarios.registerScenario(DemoGapScenario)


class HallObstacleCourse(QuadrotorScenario):
    name = "hall_obstacle_course"
    world = "hallway_obtacle_course4"

    def __init__(self, task):
        super(HallObstacleCourse, self).__init__(task=task)

        target_z = task["target_z"] if "target_z" in task else 2

        self.init_pose = [0, 0, target_z, 0]
        self.target_pose = [26, 0, target_z, 0]
        self.addInitSpawnPose(pose=self.init_pose)

    def getGazeboLaunchFile(self):
        return self.rospack.get_path("nav_quadrotor") + "/launch/gazebo_hall_obstacles.launch"

    @staticmethod
    def getUniqueFieldNames():
        return ["target_z"]

TestingScenarios.registerScenario(HallObstacleCourse)

