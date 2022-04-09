#!/usr/bin/env python3

import time
import RPi.GPIO as GPIO

class Claw():
    
    def __init__(self, gpio_servo_des, gpio_servo_sin):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(gpio_servo_des, GPIO.OUT)
        GPIO.setup(gpio_servo_sin, GPIO.OUT)

        self.__state = 1
        self.__pwm_des = GPIO.PWM(gpio_servo_des, 50)
        self.__pwm_des.start(0)

        self.__pwm_sin = GPIO.PWM(gpio_servo_sin, 50)
        self.__pwm_sin.start(0)
        self.reset()

    def reset(self):
        self.__pwm_des.ChangeDutyCycle(0)
        self.__pwm_sin.ChangeDutyCycle(0)

    def openMax(self):
        self.__state = 1
        self.__pwm_des.ChangeDutyCycle(1)
        self.__pwm_sin.ChangeDutyCycle(12)
        time.sleep(1)
        self.reset()

    def openMid(self):
        self.__state = 0.5
        self.__pwm_des.ChangeDutyCycle(4.5)
        self.__pwm_sin.ChangeDutyCycle(9)
        time.sleep(1)
        self.reset()

    def openMin(self):
        self.__state = 0
        self.__pwm_des.ChangeDutyCycle(6)
        self.__pwm_sin.ChangeDutyCycle(7.5)
        time.sleep(1)
        self.reset()

    def keepHard(self):
        self.__pwm_des.ChangeDutyCycle(6)
        self.__pwm_sin.ChangeDutyCycle(7.5)

    def openAndClose(self):
        self.__pwm_des.ChangeDutyCycle(6)
        self.__pwm_sin.ChangeDutyCycle(7.5)
        time.sleep(0.3)
        self.__pwm_des.ChangeDutyCycle(1)
        self.__pwm_sin.ChangeDutyCycle(12)
        time.sleep(0.3)
        self.reset()
        
    def isOpen(self):
        return self.__state > 0.5