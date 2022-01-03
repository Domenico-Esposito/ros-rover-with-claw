#!/usr/bin/env python3

import rospy
import time
import RPi.GPIO as GPIO

class Motors:

    PIN_B_1A = 5
    PIN_B_1B = 3

    PIN_A_1A = 7
    PIN_A_1B =  11

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.PIN_B_1A, GPIO.OUT)
        GPIO.setup(self.PIN_B_1B, GPIO.OUT)
        GPIO.setup(self.PIN_A_1A, GPIO.OUT)
        GPIO.setup(self.PIN_A_1B, GPIO.OUT)

        self.reset()

    def reset(self):
        GPIO.output(self.PIN_B_1A, GPIO.LOW)
        GPIO.output(self.PIN_B_1B, GPIO.LOW)
        GPIO.output(self.PIN_A_1A, GPIO.LOW)
        GPIO.output(self.PIN_A_1B, GPIO.LOW)

    def backward(self):
        GPIO.output(self.PIN_B_1A, GPIO.LOW)
        GPIO.output(self.PIN_B_1B, GPIO.HIGH)
        GPIO.output(self.PIN_A_1A, GPIO.LOW)
        GPIO.output(self.PIN_A_1B, GPIO.HIGH)

    def forward(self):
        GPIO.output(self.PIN_B_1A, GPIO.HIGH)
        GPIO.output(self.PIN_B_1B, GPIO.LOW)
        GPIO.output(self.PIN_A_1A, GPIO.HIGH)
        GPIO.output(self.PIN_A_1B, GPIO.LOW)

    def left(self):
        GPIO.output(self.PIN_B_1A, GPIO.HIGH)
        GPIO.output(self.PIN_B_1B, GPIO.LOW)
        GPIO.output(self.PIN_A_1A, GPIO.HIGH)
        GPIO.output(self.PIN_A_1B, GPIO.HIGH)

    def right(self):
        GPIO.output(self.PIN_B_1A, GPIO.HIGH)
        GPIO.output(self.PIN_B_1B, GPIO.HIGH)
        GPIO.output(self.PIN_A_1A, GPIO.HIGH)
        GPIO.output(self.PIN_A_1B, GPIO.LOW)