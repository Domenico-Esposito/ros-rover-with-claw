#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def publisher():
    camera_frame = rospy.Publisher('/camera_frame', Image, queue_size=None)
    cam = cv2.VideoCapture(0)
    # rate = rospy.Rate(15)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    bridge = CvBridge()

    if not cam.isOpened():
        return -1

    try:
        while not rospy.is_shutdown():
            ret, frame = cam.read()

            if ret == True:
                try:
                    frame = bridge.cv2_to_imgmsg(frame, 'bgr8')
                    camera_frame.publish(frame)
                    # rate.sleep()
                except CvBridgeError as e:
                    print(e)

    except Exception as e:
        rospy.loginfo("Termino...")
        print(e)
        cam.release()

if __name__ == '__main__':
    rospy.init_node("camera_frame")
    publisher()