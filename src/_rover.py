#!/usr/bin/env python3

import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, String
from gpio_element.Claw import Claw
from gpio_element.Motors import Motors

motors = Motors()
_ultrasound_distance = 0
_target_size = 0
_target_type = "green"
_has_catch = "no"
_manual_rotation = "no"
_openClose = 0
_manual_open = "no"
_manual_stop = "no"
_claw_operation = "no"
_rover_velocity = "normal"

def subscribers():
    rospy.Subscriber('/ultrasound_distance', Float32, ultrasound_distance_callback, queue_size=1)
    rospy.Subscriber('/target_position', String, target_position_callback, queue_size=1)
    rospy.Subscriber('/target_size', Float32, target_size_callback, queue_size=1)
    rospy.Subscriber('/velocity', String, rover_velocity, queue_size=1)
    rospy.spin()

def rover_velocity(velocity):
    global _rover_velocity
    _rover_velocity = velocity.data
    rospy.loginfo("[_rover_velocity]: %s"%_rover_velocity)


def target_size_callback(size):
    global _manual_rotation
    global _target_type
    global _has_catch
    global _openClose
    global _manual_open
    global _manual_stop
    global _claw_operation
    global _target_size

    _target_size = size.data
    rospy.loginfo("[target_size]: %f"%_target_size)

    # 195
    if _target_size > 160 and _manual_stop == "no" and _manual_rotation == "no":
        rospy.loginfo("[STOP]: Stop _target_size")
        _manual_stop = "ok"
        motors.reset()
    else:
        _manual_stop = "no"

    # if _manual_rotation == "no" and _has_catch == "no":
    #     if _target_size > 195:
    #         _openClose = _openClose + 1
    #         _manual_open = "ok"
    #         _manual_stop = "ok"
    #         claw.openAndClose()

    if _target_type == "purple" and _manual_rotation == "no" and _has_catch == "ok":
        if _target_size > 99:
            _manual_rotation = "ok"
            rospy.loginfo("[target_size]: Rilascio")
            claw.openMax()
            _target_type = "green"
            target_type_message = String()
            target_type_message.data = _target_type
            target_type.publish(target_type_message)
            _has_catch = "no"
            time.sleep(0.3)
            motors.backward()
            time.sleep(0.3)
            motors.right()
            time.sleep(1.2)
            motors.reset()
            _manual_rotation = "no"

_number_of_stop = 0

def target_position_callback(position): 
    global _manual_rotation
    global _openClose
    global _manual_stop
    global _motors_action
    global _number_of_stop

    # if _openClose > 3 and _has_catch == "no":
    #     _manual_rotation = "ok"
    #     motors.reset()
    #     motors.backward()
    #     time.sleep(3)
    #     _manual_rotation = "no"
    #     _openClose = 0
    #     return
    _motors_action = position.data
    if _manual_stop == "ok" or _manual_rotation == "ok" or _target_size > 160:
        return

    if _number_of_stop >= 10 and _motors_action == "stop":
        rospy.loginfo("[target_position]: Avvio ricerca target")
        motors.left()
        time.sleep(0.01)

    motors.reset()

    if _motors_action == "forward":
        motors.forward()
        if _rover_velocity == "normal":
            time.sleep(0.01)
        elif _rover_velocity == "slow":
            time.sleep(0.009)
        motors.reset()

    if _motors_action == "left":
        motors.left()
        if _rover_velocity == "normal":
            time.sleep(0.009)
        elif _rover_velocity == "slow":
            time.sleep(0.0045)
        motors.reset()

    if _motors_action == "right":
        motors.right()
        if _rover_velocity == "normal":
            time.sleep(0.009)
        elif _rover_velocity == "slow":
            time.sleep(0.0045)
        motors.reset()

    if _motors_action == "stop":
        _number_of_stop = _number_of_stop + 1
        motors.reset()

    # rospy.loginfo("[target_position_callback]: %s"%motors_action)

    # rospy.loginfo("Ricerco target: %s"%_target_type)
    # rospy.loginfo("Azione motori: %s"%motors_action)

meanDistance = 0
index = 0
_block = "no"

_autoOpenIndex = 0

def ultrasound_distance_callback(distance):
    global index
    global meanDistance
    global _target_type
    global _has_catch
    global _manual_rotation
    global _claw_operation
    global _block
    global _manual_open
    global _autoOpenIndex
    global _motors_action

    if _manual_open == "ok":
        return

    if _target_type == "purple":
        return

    if _has_catch == "ok":
        return

    _ultrasound_distance = distance.data

    if index > 0:
        meanDistance = (meanDistance + _ultrasound_distance) / index
    else:
        meanDistance = _ultrasound_distance;

    index = index + 1

    rospy.loginfo("[ultrasound_distance] Distance: %f"%_ultrasound_distance)
    rospy.loginfo("[ultrasound_distance] meanDistance: %f"%meanDistance)
    
    if index == 3:
        if meanDistance < 5:
            _has_catch = "ok"
            _manual_rotation = "ok"
            _target_type = "purple"
            target_type_message = String()
            target_type_message.data = _target_type
            target_type.publish(target_type_message)
            claw.openMin()
            claw.keepHard()
            time.sleep(0.3)
            motors.backward()
            time.sleep(1)
            motors.right()
            time.sleep(1.2)
            motors.reset()
            _manual_rotation = "no"
        else:
            if not claw.isOpen():
                claw.openMax()
            else:
                if _manual_stop == "ok":
                    _autoOpenIndex = _autoOpenIndex + 1
                    if _autoOpenIndex >= 3:
                        _manual_rotation = "ok"
                        _autoOpenIndex = 0
                        motors.backward()
                        time.sleep(1.2)        
                        _manual_rotation = "no"                    
                    else:
                        claw.openAndClose()
                        motors.forward()
                        time.sleep(0.009)
        rospy.loginfo("[ultrasound_distance] Reset")
        meanDistance = 0
        index = 1

if __name__ == "__main__":
    rospy.init_node("rover")    
    target_type = rospy.Publisher('/target_type', String, queue_size=1)
    
    claw = Claw(12, 16)
    subscribers()
