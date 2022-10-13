#!/usr/bin/env python3

from builtins import str
from builtins import object
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
import tf.transformations

class ProjectedPosePublisher(object):

    def odomCB(self, odom):
        rospy.logdebug("Odometry callback")
        pose = PoseStamped(header=odom.header, pose=odom.pose.pose)
        q = odom.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,yaw))
        pose.pose.position.z = 0

        self.pose_pub.publish(pose)

    def __init__(self):
        self.pose_pub = rospy.Publisher("projected_pose", PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCB, queue_size=1)

if __name__ == '__main__':
    try:
        rospy.init_node('rotors_projected_pose_publisher')
        ProjectedPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo("exception: " + str(e))

