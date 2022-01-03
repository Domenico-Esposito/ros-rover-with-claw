#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

_target_type = "green"

def find_yellow(hsv):

    upper_range_yellow = np.array([81, 255, 255])
    lower_range_yellow = np.array([49, 116, 160])

    mask = cv2.inRange(hsv, lower_range_yellow, upper_range_yellow)
    return mask

def find_blue(hsv):
    upper_range_blue = np.array([127,255,255])
    lower_range_blue = np.array([89,101,69])

    mask = cv2.inRange(hsv, lower_range_blue, upper_range_blue)
    return mask

def find_green(hsv):
    upper_range_blue = np.array([88,255,255])
    lower_range_blue = np.array([46,110,57])

    mask = cv2.inRange(hsv, lower_range_blue, upper_range_blue)
    return mask

def find_purple(hsv):
    lower_range_blue = np.array([139,126,0])
    upper_range_blue = np.array([170,255,255])

    mask = cv2.inRange(hsv, lower_range_blue, upper_range_blue)
    return mask

def find_orage(hsv):
    lower_range_blue = np.array([0,79,204])
    upper_range_blue = np.array([23,255,255])

    mask = cv2.inRange(hsv, lower_range_blue, upper_range_blue)
    return mask

def publisher():
    cam = cv2.VideoCapture(0)

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cam.isOpened():
        return -1

    position = String()
    velocity = String()
    target_radius = Float32()

    bridge = CvBridge()
    rate = rospy.Rate(30)

    queue_operation = []
    
    try:
        while not rospy.is_shutdown():
            ret, frame = cam.read()
            # frame = frame[0:140, :]
            position.data = "stop"

            if ret == True:
                height, width, _ = frame.shape
                centerWidth = width / 2
                padding = width / 5

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                if _target_type == "green":
                    mask = find_green(hsv)
                elif _target_type == "purple":
                    mask = find_purple(hsv)

                percent_color = mask.mean()
                edges = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                # edges = find_purple(hsv)
            
                target_radius.data = percent_color
                # target_size.publish(target_radius)

                if len(edges) > 0:
                    c = max(edges, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)

                    # rospy.loginfo("[radius] %f"%radius)
                    
                    if radius > 90:
                        # SLOW
                        velocity.data = "slow"
                    else:
                        velocity.data = "normal"
                    
                    target_size.publish(target_radius)

                    if radius > 10:
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        if(x >= centerWidth - padding and x <= centerWidth + padding):
                            position.data = "forward"
                            #rospy.loginfo("forward")
                        elif( x < centerWidth ):
                            position.data = "left"
                            #rospy.loginfo("left: Muovi motore destro")
                        elif( x > centerWidth ):
                            position.data = "right"
                            #rospy.loginfo("right: Muovi motore sinistro")
                #         else:
                #             #rospy.loginfo("Stop")
                #     else:
                #         #rospy.loginfo("Stop")
                # else:
                #     #rospy.loginfo("Stop")
                # else:
                #     if len(queue_operation) > 0:
                #         position.data = queue_operation.pop()

                try:
                    frame = bridge.cv2_to_imgmsg(frame, 'bgr8')
                    camera_frame_with_detection.publish(frame)
                except CvBridgeError as e:
                    print(e)
 
            target_position.publish(position)
            rate.sleep()

    except Exception as e:
        #rospy.loginfo("Termino...")
        print(e)
        cam.release()

def change_target_type(target):
    global _target_type
    _target_type = target.data

if __name__ == '__main__':
    rospy.init_node("publisher_video")
    velocity = rospy.Publisher('/velocity', String, queue_size=10)
    target_position = rospy.Publisher('/target_position', String, queue_size=10)
    target_size = rospy.Publisher('/target_size', Float32, queue_size=1)
    camera_frame_with_detection = rospy.Publisher('/camera_frame_with_detection', Image, queue_size=1)
    rospy.Subscriber('/target_type', String, change_target_type, queue_size=1)

    publisher()