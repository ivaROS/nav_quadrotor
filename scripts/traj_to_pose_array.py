#!/usr/bin/env python3

from builtins import str
from builtins import object
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_quadrotor.msg import TrajectoryArray
#from nav_quadrotor.state_conversions import StateToMsg
from geometry_msgs.msg import PoseArray, Pose, Point


def traj_to_pose_array(trajectory):
    pa = PoseArray()
    pa.header = trajectory.header

    pa.poses = [Pose(position=Point(x=p.transforms[0].translation.x, y=p.transforms[0].translation.y, z=p.transforms[0].translation.z), orientation=p.transforms[0].rotation) for p in trajectory.points]
    return pa

class PoseGoalPublisher(object):

    def traj_cb(self, traj):
        rospy.logdebug("Trajectory callback")
        self.traj_pose_array_pub.publish(traj_to_pose_array(traj))

    def __init__(self):
        self.traj_pose_array_pub = rospy.Publisher("/hummingbird/command/trajectory_pose_array", PoseArray, queue_size=1)
        self.traj_sub = rospy.Subscriber("/hummingbird/command/trajectory", MultiDOFJointTrajectory, self.traj_cb, queue_size=1)

if __name__ == '__main__':
    try:
        rospy.init_node('trajectory_pose_array_publisher')
        node = PoseGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo("exception: " + str(e))

