#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

_target_type = "yellow"

def find_yellow(hsv):

    upper_range_yellow = np.array([81, 255, 255])
    lower_range_yellow = np.array([49, 116, 160])

    mask = cv2.inRange(hsv, lower_range_yellow, upper_range_yellow)
    edges = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    return edges

def find_blue(hsv):
    upper_range_blue = np.array([127,255,255])
    lower_range_blue = np.array([89,101,69])

    mask = cv2.inRange(hsv, lower_range_blue, upper_range_blue)
    edges = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    return edges

def find_green(hsv):
    upper_range_blue = np.array([80,255,255])
    lower_range_blue = np.array([37,145,73])

    mask = cv2.inRange(hsv, lower_range_blue, upper_range_blue)
    edges = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    return edges

def find_purple(hsv):
    lower_range_blue = np.array([139,126,0])
    upper_range_blue = np.array([170,255,255])

    mask = cv2.inRange(hsv, lower_range_blue, upper_range_blue)
    edges = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    return edges

def subscriber():
    rospy.Subscriber('/camera_frame', Image, detection, queue_size=1)
    rospy.Subscriber('/target_type', String, change_target_type, queue_size=1)
    rospy.spin()

def change_target_type(target):
    global _target_type
    _target_type = target.data

def detection(frame):
    position = String()

    frame = bridge.imgmsg_to_cv2(frame)

    height, width, _ = frame.shape
    centerWidth = width / 2
    padding = width / 5

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    edges = find_purple(hsv)

    # if _target_type == "yellow":
    #     edges = find_green(hsv)
    # elif _target_type == "blue":
    #     edges = find_purple(hsv)

    position.data = "stop"
    
    target_radius = Float32()
    target_radius.data = 0

    if len(edges) > 0:
        c = max(edges, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        
        target_radius.data = radius

        if radius > 25:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

            if(x >= centerWidth - padding and x <= centerWidth + padding):
                position.data = "forward"
                rospy.loginfo("forward")
            elif( x < centerWidth ):
                position.data = "left"
                rospy.loginfo("Sinistra: Muovi motore destro")
            elif( x > centerWidth ):
                position.data = "right"
                rospy.loginfo("Destra: Muovi motore sinistro")
            else:
                rospy.loginfo("Stop")
        else:
            rospy.loginfo("Stop")
    else:
        rospy.loginfo("Stop")
    
    target_size.publish(target_radius)
    target_position.publish(position)

    try:
        frame = bridge.cv2_to_imgmsg(frame, 'bgr8')
        camera_frame_with_detection.publish(frame)
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    camera_frame_with_detection = rospy.Publisher('/camera_frame_with_detection', Image, queue_size=None)
    target_position = rospy.Publisher('/target_position', String, queue_size=None)
    target_size = rospy.Publisher('/target_size', Float32, queue_size=None)

    bridge = CvBridge()

    rospy.init_node("color_detection")
    subscriber()