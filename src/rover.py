#!/usr/bin/env python3

import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, String
from gpio_element.Claw import Claw
from gpio_element.Motors import Motors

class Rover():

    def __init__(self):
        self._motors = Motors()    
        self._claw = Claw(12, 16)

        self._ultrasound_distance = 0
        self._target_size = 0
        self._target_type = "green"
        self._has_catch = "no"
        self._manual_rotation = "no"
        self._openClose = 0
        self._manual_open = "no"
        self._manual_stop = "no"
        self._claw_operation = "no"
        self._rover_velocity = "normal"
        self._number_of_stop = 0
        self._motors_action = "stop"
        self._avgDistance = 0
        self._index = 1
        self._autoOpenIndex = 0

        self._pub_target_type = rospy.Publisher('/target_type', String, queue_size=1)

    def subscribers(self):
        rospy.Subscriber('/ultrasound_distance', Float32, self.ultrasound_distance_callback, queue_size=1)
        rospy.Subscriber('/target_position', String, self.target_position_callback, queue_size=1)
        rospy.Subscriber('/target_size', Float32, self.target_size_callback, queue_size=1)
        rospy.Subscriber('/velocity', String, self.rover_velocity, queue_size=1)
        rospy.spin()

    def rover_velocity(self, velocity):
        self._rover_velocity = velocity.data
        rospy.loginfo("[_rover_velocity]: %s"%self._rover_velocity)

    def target_size_callback(self, size):
        self._target_size = size.data
        rospy.loginfo("[target_size]: %f"%self._target_size)

        if self._target_size > 160 and self._manual_stop == "no" and self._manual_rotation == "no":
            rospy.loginfo("[STOP]: Stop _target_size")
            self._manual_stop = "ok"
            self._motors.reset()
        else:
            self._manual_stop = "no"

        target_type_message = String()
        target_type_message.data = self._target_type

        if self._target_type == "purple" and self._manual_rotation == "no" and self._has_catch == "ok":
            if self._target_size > 99:
                self._manual_rotation = "ok"
                rospy.loginfo("[target_size]: Rilascio")
                self._claw.openMax()
                self._target_type = "green"
                self._pub_target_type.publish(target_type_message)
                time.sleep(0.3)
                self._motors.backward()
                time.sleep(0.3)
                self._motors.right()
                time.sleep(1.2)
                self._motors.reset()
                self._manual_rotation = "no"
                self._has_catch = "no"

    def target_search(self):
        rospy.loginfo("[target_position]: Target search")
        self._motors.left()
        time.sleep(0.01)

    def target_forward(self):
        self._motors.forward()
        if self._rover_velocity == "normal":
            time.sleep(0.01)
        elif self._rover_velocity == "slow":
            time.sleep(0.009)
        self._motors.reset()

    def target_left(self):
        self._motors.left()
        if self._rover_velocity == "normal":
            time.sleep(0.009)
        elif self._rover_velocity == "slow":
            time.sleep(0.0045)
        self._motors.reset()

    def target_right(self):
        self._motors.right()
        if self._rover_velocity == "normal":
            time.sleep(0.009)
        elif self._rover_velocity == "slow":
            time.sleep(0.0045)
        self._motors.reset()

    def target_position_callback(self, position): 
        self._motors_action = position.data

        if self._manual_stop == "ok" or self._manual_rotation == "ok" or self._target_size > 160:
            return

        if self._motors_action == "stop":
            if self._number_of_stop >= 10:
                self.target_search()
            else:
                self._number_of_stop = self._number_of_stop + 1
                self._motors.reset()

        self._motors.reset()

        if self._motors_action == "forward":
            self.target_forward()

        if self._motors_action == "left":
            self.target_left()

        if self._motors_action == "right":
            self.target_right()

    def ultrasound_distance_callback(self, distance):
        if self._manual_open == "ok":
            return

        if self._target_type == "purple":
            return

        if self._has_catch == "ok":
            return

        self._ultrasound_distance = distance.data
        
        self._avgDistance = (self._avgDistance + self._ultrasound_distance) / self._index
        self._index = self._index + 1

        rospy.loginfo("[ultrasound_distance] distance: %f"%self._ultrasound_distance)
        rospy.loginfo("[ultrasound_distance] meanDistance: %f"%self._avgDistance)
        
        if self._index == 3:
            if self._avgDistance < 5:
                self.manageCatch()
            else:
                if not self._claw.isOpen():
                    self._claw.openMax()
                else:
                    self.strategyRetryCatch()
            rospy.loginfo("[ultrasound_distance] Reset")
            self._avgDistance = 0
            self._index = 1

    def manageCatch(self):
        self._has_catch = "ok"
        self._manual_rotation = "ok"
        self._target_type = "purple"

        target_type_message = String()
        target_type_message.data = self._target_type
        self._pub_target_type.publish(target_type_message)

        self._claw.openMin()
        self._claw.keepHard()
        time.sleep(0.3)
        self._motors.backward()
        time.sleep(1)
        self._motors.right()
        time.sleep(1.2)
        self._motors.reset()
        self._manual_rotation = "no"
        self._manual_stop = "no"

    def strategyRetryCatch(self):
        if self._manual_stop == "ok":
            self._autoOpenIndex = self._autoOpenIndex + 1
            if self._autoOpenIndex >= 3:
                self._manual_rotation = "ok"
                self._autoOpenIndex = 0
                self._motors.backward()
                time.sleep(1.2)        
                self._manual_rotation = "no"                    
            else:
                self._claw.openAndClose()
                self._motors.forward()
                time.sleep(0.009)

if __name__ == "__main__":
    rospy.init_node("rover")    

    rover = Rover()
    rover.subscribers()