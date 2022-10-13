#!/usr/bin/env python3

from builtins import str
from builtins import object
import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Vector3

class PoseGoalPublisher(object):

    def goalCB(self, goal):
        rospy.logdebug("Goal callback")
        goal.pose.position.z += 2
        self.goal_pub.publish(goal)

    def __init__(self):
        self.goal_pub = rospy.Publisher("/hummingbird/command/pose", PoseStamped, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCB, queue_size=1)

if __name__ == '__main__':
    try:
        rospy.init_node('rotors_pose_command_relay')
        PoseGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo("exception: " + str(e))

