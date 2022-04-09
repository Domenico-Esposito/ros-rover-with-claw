#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from gpio_element.Ultrasound import Ultrasound

def publisher():
    pub = rospy.Publisher('ultrasound_distance', Float32, queue_size=1)
    rate = rospy.Rate(5)

    radar = Ultrasound(24, 22)
    
    while not rospy.is_shutdown():
        distanceInCm = radar.getDistanceInCm()    
        rospy.loginfo("[ultrasound_distance] distance: %f"%distanceInCm)

        if ( distanceInCm > 3 and distanceInCm < 15 ):
            messageToPublisher = Float32()
            messageToPublisher.data = distanceInCm

            pub.publish(messageToPublisher)
        
        rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node("ultrasound_sensor")
    publisher()