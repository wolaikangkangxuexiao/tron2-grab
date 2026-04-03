#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    rospy.init_node('caminit_node')
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=30)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Can not find the image !!!")
            break

        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        image_pub.publish(ros_image)



