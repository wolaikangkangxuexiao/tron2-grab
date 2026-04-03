#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray

move_flag = 0

t1 = 0
t2 = 0

### x轴运动函数（末端保持水平） ###
#x_in是向前移动的值，arr_in是输出列表
def move_x(x_in,arr_in):
    arr_in[7] -= x_in
    arr_in[10] += x_in
    return arr_in

### y轴运动函数（末端保持水平） ###
#y_in是水平移动的值，arr_in是输出列表
def move_y(y_in,arr_in):
    arr_in[8]-= y_in
    arr_in[11] -= y_in
    arr_in[13] = y_in / 2
    return arr_in

def Camcallback(msg):
    global move_flag,t1,t2
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    if not move_flag:
        t1 = time.time()
        move_flag = 1
    img = cv_image[200:480, 0:550]
    img_cvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100, 43, 46])
    upper_blue = np.array([124, 255, 255])

    mask_blue = cv2.inRange(img_cvt, lower_blue, upper_blue)

    contours,h= cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        # 筛选去除噪声
        if area > 1400:
            lenth = cv2.arcLength(contour, True)
            conpoly = cv2.approxPolyDP(contour, 0.02 * lenth, True)
            cv2.drawContours(img, conpoly, -1, (0, 255, 0), 2)
            boundrect = cv2.boundingRect(conpoly)

            cv2.rectangle(img, (boundrect[0], boundrect[1]), (boundrect[0] + boundrect[2], boundrect[1] + boundrect[3]),
                          (0, 0, 255), 2)

            rospy.logwarn("x : %d, y : %d", (boundrect[0] + boundrect[2])/2, (boundrect[1] + boundrect[3])/2)


            arm_pub = rospy.Publisher('/Tron2_JointCommand', Float32MultiArray, queue_size=20)
            movex = (236 - (boundrect[0] + boundrect[2])/2)*(0.1/50)
            array = Float32MultiArray()
            array.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          -0.4, 0, 0.0, -1.3, -0.2, 0.0, 0,50,10,0,60,5,40]

            array.data = move_x(0.3, array.data)
            array.data = move_y(0.3, array.data)
            array.data = move_x(movex + 0.3,array.data)


            # 向右移动 #
            t2 = time.time()
            rospy.logwarn("t2 - t1 : %f", t2 - t1)

            arm_pub.publish(array)



    cv2.imshow("Image window", cv_image)
    cv2.imshow("mask", mask_blue)
    cv2.waitKey(1)



if __name__=="__main__":
    rospy.init_node('move_node')
    sub_cam = rospy.Subscriber('/camera/image_raw', Image, Camcallback)
    arm_pub = rospy.Publisher('/Tron2_JointCommand', Float32MultiArray, queue_size=20)

    rospy.spin()