#!/usr/bin/env python3

import rospy
from nav_quadrotor.endpoint_sampling import PointSamplingNode
import cv2


if __name__ == '__main__':
    try:
        rospy.init_node('endpoint_sampling_test_node')
        rospy.loginfo("endpoint_sampling_test_node started")

        viz = rospy.get_param("~viz", False)
        outsource_convolutions = True
        profile = False

        profiler = None
        if profile:
            import cProfile as profile
            profiler = profile.Profile()
            profiler.disable()
        point_sampler = PointSamplingNode(viz=viz, timing=True, profiler=profiler)

        rospy.spin()
    except rospy.ROSInterruptException as ex:
        rospy.logwarn("Exception: " + str(ex))
    finally:
        cv2.destroyAllWindows()
        if profile:
            profiler.disable()
            profiler.dump_stats('/tmp/profile.pstat')
