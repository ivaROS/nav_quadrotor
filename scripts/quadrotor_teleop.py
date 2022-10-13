#!/usr/bin/env python3

#quadrotor_teleop.py: based heavily on turtlebot_teleop_key

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
from __future__ import division
from past.utils import old_div
from builtins import object
import rospy

from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys, select, termios, tty, math

msg = """
Control Your Quadrotor!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
space/; : up/down

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .2
turn = 0.27


def GetQuaternion(yaw):
    roll = pitch = 0
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*quaternion)

def GetYaw(q):
    quat = (q.x, q.y, q.z, q.w)
    _, _, yaw = euler_from_quaternion(quaternion=quat)
    return yaw



def getCommandedPose(cur_pose, control_speed, control_turn, control_z, delta_t):
    yaw = GetYaw(cur_pose.pose.orientation)
    dx = math.cos(yaw)*control_speed*delta_t
    dy = math.sin(yaw)*control_speed*delta_t
    dyaw = control_turn*delta_t
    dz = control_z*delta_t

    pos = Point()
    pos.x = cur_pose.pose.position.x + dx
    pos.y = cur_pose.pose.position.y + dy
    pos.z = cur_pose.pose.position.z + dz

    new_yaw = yaw + dyaw
    rotation = GetQuaternion(new_yaw)
    pose = PoseStamped(pose=Pose(position=pos, orientation=rotation))
    pose.header = cur_pose.header
    return pose

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


class PoseGetter(object):

    def __init__(self, delta_t):
        self.cur_pose = None
        self.cmd_pose = None
        self.delta_t = delta_t
        self.sub = rospy.Subscriber('~cur_pose', Odometry, callback=self.update_pose, queue_size=5)
        rospy.wait_for_message('~cur_pose', Odometry)
        rospy.loginfo("Received first pose message!")

    def update_pose(self, odom):
        self.cur_pose = PoseStamped(pose=odom.pose.pose, header=odom.header)
        rospy.logdebug("Updated pose!")

    def getCommandedPose(self, control_speed, control_turn, control_z):
        delta_t = self.delta_t

        cur_pose = self.cmd_pose if self.cmd_pose is not None else self.cur_pose

        yaw = GetYaw(cur_pose.pose.orientation)
        dx = math.cos(yaw) * control_speed * delta_t
        dy = math.sin(yaw) * control_speed * delta_t
        dyaw = control_turn * delta_t
        dz = control_z * delta_t

        pos = Point()
        pos.x = cur_pose.pose.position.x + dx
        pos.y = cur_pose.pose.position.y + dy
        pos.z = cur_pose.pose.position.z + dz

        new_yaw = yaw + dyaw
        rotation = GetQuaternion(new_yaw)
        pose = PoseStamped(pose=Pose(position=pos, orientation=rotation))
        pose.header = cur_pose.header
        self.cmd_pose = pose
        return pose

    def hasPose(self):
        return self.cur_pose is not None

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

    pose_pub = rospy.Publisher('~cmd_pose', PoseStamped, queue_size=5)


    x = 0
    th = 0
    z = 0
    status = 0
    count = 0
    lin_acc = 0.1
    ang_acc = 0.1
    z_acc = 0.1
    target_speed = 0
    target_turn = 0
    target_z = 0
    control_speed = 0
    control_turn = 0
    control_z = 0

    des_rate = 10

    r = rospy.Rate(hz=des_rate)
    delta_t = 1.0/des_rate

    pose_getter = PoseGetter(delta_t=delta_t)


    #rospy.spin()

    try:
        print(msg)
        print(vels(speed, turn))
        while (1):
            if not pose_getter.hasPose():
                rospy.logwarn_throttle(1, "No pose received yet...")
                r.sleep()
                continue

            key = getKey()
            if key in list(moveBindings.keys()):
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
                z = 0
            elif key in list(speedBindings.keys()):
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == 'k':
                x = 0
                th = 0
                z = 0
                control_speed = 0
                control_turn = 0
            elif key == ' ':
                x = 0
                th = 0
                z = 1
            elif key == ';':
                x = 0
                th = 0
                z = -1
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                    z = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th
            target_z = old_div(z * speed, 2)

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + lin_acc)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - lin_acc)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + ang_acc)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - ang_acc)
            else:
                control_turn = target_turn

            if target_z > control_z:
                control_z = min(target_z, control_z + z_acc)
            elif target_turn < control_turn:
                control_z = max(target_z, control_z - z_acc)
            else:
                control_z = target_z

            cmd_pose = pose_getter.getCommandedPose(control_speed=control_speed, control_turn=control_turn, control_z=control_z)

            twist = Twist()
            twist.linear.x = control_speed
            twist.linear.y = 0
            twist.linear.z = control_z
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = control_turn
            pub.publish(twist)

            pose_pub.publish(cmd_pose)

            r.sleep()

            # print("loop: {0}".format(count))
            # print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            # print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
