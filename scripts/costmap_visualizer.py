#!/usr/bin/env python3

import rospy
from nav_quadrotor.endpoint_sampling import CostmapVisualizer


if __name__ == '__main__':
    try:
        rospy.init_node('costmap_visualizer_node')
        rospy.loginfo("costmap_visualizer_node started")

        crop_image = rospy.get_param("~crop_image", True)
        point_sampler = CostmapVisualizer(crop_image=crop_image)

        rospy.spin()
    except rospy.ROSInterruptException as ex:
        rospy.logwarn("Exception: " + str(ex))
