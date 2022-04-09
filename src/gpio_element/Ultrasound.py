#!/usr/bin/env python3
import time
import rospy

import RPi.GPIO as GPIO

class Ultrasound():

    def __init__(self, gpio_trigger, gpio_echo):
        self.__gpio_trigger = gpio_trigger
        self.__gpio_echo = gpio_echo

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(gpio_trigger, GPIO.OUT)
        GPIO.setup(gpio_echo, GPIO.IN)

    def getDistanceInCm(self):
        GPIO.output(self.__gpio_trigger, True)
        time.sleep(0.00001)
        GPIO.output(self.__gpio_trigger, False)

        StartTime = time.time()
        StopTime = time.time()

        counter = 0
        while GPIO.input(self.__gpio_echo) == 0:
            StartTime = time.time()
            counter += 1
            if counter == 150:
                counter = 0
                break
        
        if counter == 0:
            return 0

        counter = 0
        while GPIO.input(self.__gpio_echo) == 1:
            StopTime = time.time()
            counter += 1
            if counter == 150:
                counter = 0
                break
        
        if counter == 0:
            return 0

        TimeElapsed = StopTime - StartTime
        distanceInCm = (TimeElapsed * 34300) / 2

        return distanceInCm