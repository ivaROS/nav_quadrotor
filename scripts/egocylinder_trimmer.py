#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from rospy.topics import SubscribeListener

class ImageCleaner(SubscribeListener):

    def __init__(self):
        self.img_pub = rospy.Publisher(name="image_out", data_class=Image, subscriber_listener=self, queue_size=5)
        self.im_sub = None
        #self.im_sub = rospy.Subscriber("image_in", Image, self.image_cb, queue_size=5)
        self.bridge = CvBridge()
        rospy.loginfo("Image cleaner is ready")

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if self.img_pub.get_num_connections()>0:
            if self.im_sub is None:
                self.im_sub = rospy.Subscriber("image_in", Image, self.image_cb, queue_size=5)

    def peer_unsubscribe(self, topic_name, num_peers):
        if self.img_pub.get_num_connections()==0:
            self.im_sub.unregister()
            self.im_sub = None

    def image_cb(self, image_msg):
        rospy.logdebug("Hallucinated image callback")

        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough").copy()
            w = image_msg.width
            num_divisions = 6
            c = int(w/num_divisions)
            image[:,0:c]=np.nan
            image[:,(num_divisions-1)*c:w]=np.nan
            cleaned_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            cleaned_image_msg.header = image_msg.header
            self.img_pub.publish(cleaned_image_msg)

        except CvBridgeError as e:
            rospy.logerr(e)




if __name__ == '__main__':
    try:
        rospy.init_node('image_cleaner_publisher')
        rospy.loginfo("Image cleaner node starting up")
        ImageCleaner()
        rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.logerr(str(e))
