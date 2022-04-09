#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class ManageCamera:

    def __init__(self) -> None:
        self._target_type = "green"
        self.setupCamera()

    def getCamera(self):
        return self.cam

    def setupCamera(self):
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    @property
    def target_type(self):
        return self._target_type

    @target_type.setter
    def target_type(self, value):
        if value == "purple":
            self.setPurpleTarget()
        elif value == "green":
            self.setGreenTarget()
        else:
            raise ValueError("Target type unsupported")

    def setPurpleTarget(self):
        self._target_type = "purple"
        self._upper_range_hsv = np.array([170,255,255])
        self._lower_range_hsv = np.array([139,126,0])

    def setGreenTarget(self):
        self._target_type = "green"
        self._upper_range_hsv = np.array([88,255,255])
        self._lower_range_hsv = np.array([46,110,57])

    def getEdge(self):
        return cv2.findContours(self._mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    def analyzeFrame(self, frame):
        self._hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self._mask = cv2.inRange(self._hsv, self._lower_range_hsv, self._upper_range_hsv)

        return self.getEdge(), self._mask.mean()

def publisher():

    cam = manageCamera.getCamera()

    if not cam.isOpened():
        return -1

    position = String()
    velocity = String()
    target_size = Float32()
    
    bridge = CvBridge()
    rate = rospy.Rate(30)
    
    try:
        while not rospy.is_shutdown():
            ret, frame = cam.read()
            position.data = "stop"

            if ret == True:
                height, width, _ = frame.shape
                centerFrameWidth = width / 2
                sectionPadding = width / 5
                
                edges, size = manageCamera.analyzeFrame(frame)

                target_size.data = size

                if len(edges) > 0:
                    maxContournAreaColorTarget = max(edges, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(maxContournAreaColorTarget)
                    
                    if radius > 90:
                        velocity.data = "slow"
                    else:
                        velocity.data = "normal"
                    
                    target_size.publish(target_size)

                    if radius > 10:
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        if(x >= centerFrameWidth - sectionPadding and x <= centerFrameWidth + sectionPadding):
                            position.data = "forward"
                        elif( x < centerFrameWidth ):
                            position.data = "left"
                        elif( x > centerFrameWidth ):
                            position.data = "right"
                try:
                    frame = bridge.cv2_to_imgmsg(frame, 'bgr8')
                    camera_frame_with_detection.publish(frame)
                except CvBridgeError as e:
                    print(e)
 
            target_position.publish(position)
            rate.sleep()

    except Exception as e:
        cam.release()

def change_target_type(target):
    manageCamera.target_type = target.data

if __name__ == '__main__':
    manageCamera = ManageCamera()

    rospy.init_node("main_camera")

    velocity = rospy.Publisher('/velocity', String, queue_size=10)
    target_position = rospy.Publisher('/target_position', String, queue_size=10)
    target_size = rospy.Publisher('/target_size', Float32, queue_size=1)
    camera_frame_with_detection = rospy.Publisher('/camera_frame_with_detection', Image, queue_size=1)
    
    rospy.Subscriber('/target_type', String, change_target_type, queue_size=1)

    publisher()