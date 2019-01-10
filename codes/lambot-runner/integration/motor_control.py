#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import sys

class MotorControl:
    """
        MotorControl provides functionalities for controlling the motors and provides functions:
        - stop
        - move_forward
        - move_backwards
        - turn_right
        - turn_left
        - cleanup
        - private get_pin_status
    """
    __turn_sleep_time = 0.05#.50 #0.40

    def __init__(self):
        print('Initializing motor pin setup...')

        # MOTOR A - left side of the car
        self.forward_motor_A_pin_20  = 20
        self.backward_motor_A_pin_26 = 26

        # MOTOR B - right side of the car
        self.forward_motor_B_pin_16  = 16
        self.backward_motor_B_pin_19 = 19

        #Enable PIN, gpio 13 used for PWM to control speed
        self.enable_motor_B_pin_13   = 13

        # Setup GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.forward_motor_A_pin_20, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.backward_motor_A_pin_26, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.forward_motor_B_pin_16, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.backward_motor_B_pin_19, GPIO.OUT, initial=GPIO.LOW)

        #PWM Setup 13
        print('Setting up pwm for speed control')
        GPIO.setup(self.enable_motor_B_pin_13, GPIO.OUT)
        self.turn_duty_cycle = 55
        self.forward_duty_cycle = 50
        
        self.pwm13 = GPIO.PWM(self.enable_motor_B_pin_13, 1000)
        self.pwm13.start(self.forward_duty_cycle)
        
        print('Finished pin setup successfully')
        
        self.pin20_status = False
        self.pin16_status = False
        self.pin26_status = False
        self.pin19_status = False

    def __get_pin_status(self):
        self.pin20_status = GPIO.input(self.forward_motor_A_pin_20)
        self.pin16_status = GPIO.input(self.forward_motor_B_pin_16)
        self.pin26_status = GPIO.input(self.backward_motor_A_pin_26)
        self.pin19_status = GPIO.input(self.backward_motor_B_pin_19)


    def stop(self):
        print('stopping motors')
        self.__get_pin_status()

        if self.pin20_status:
            GPIO.output(self.forward_motor_A_pin_20, GPIO.LOW)
        if self.pin16_status:
            GPIO.output(self.forward_motor_B_pin_16, GPIO.LOW)
        if self.pin26_status:
            GPIO.output(self.backward_motor_A_pin_26, GPIO.LOW)
        if self.pin19_status:
            GPIO.output(self.backward_motor_B_pin_19, GPIO.LOW)

    def move_forward(self):
        print('forward')
        self.__get_pin_status()

        if not self.pin20_status:
            GPIO.output(self.forward_motor_A_pin_20, GPIO.HIGH)
        if not self.pin16_status:
            GPIO.output(self.forward_motor_B_pin_16, GPIO.HIGH)
        if self.pin26_status:
            GPIO.output(self.backward_motor_A_pin_26, GPIO.LOW)
        if self.pin19_status:
            GPIO.output(self.backward_motor_B_pin_19, GPIO.LOW)

    def move_backwards(self):
        print('backwards')
        self.__get_pin_status()
        
        if self.pin20_status:
            GPIO.output(self.forward_motor_A_pin_20, GPIO.LOW)
        if self.pin16_status:
            GPIO.output(self.forward_motor_B_pin_16, GPIO.LOW)
        if not self.pin26_status:
            GPIO.output(self.backward_motor_A_pin_26, GPIO.HIGH)
        if not self.pin19_status:
            GPIO.output(self.backward_motor_B_pin_19, GPIO.HIGH)

    def turn_right(self):
        print('right')
        self.__get_pin_status()
        
        if not self.pin20_status:
            GPIO.output(self.forward_motor_A_pin_20, GPIO.HIGH)
        if self.pin16_status:
            GPIO.output(self.forward_motor_B_pin_16, GPIO.LOW)
        if self.pin26_status:
            GPIO.output(self.backward_motor_A_pin_26, GPIO.LOW)
        if not self.pin19_status:
            GPIO.output(self.backward_motor_B_pin_19, GPIO.HIGH)

        time.sleep(self.__turn_sleep_time)
        GPIO.output(self.forward_motor_A_pin_20, GPIO.LOW)
        GPIO.output(self.backward_motor_B_pin_19, GPIO.LOW)

    def turn_left(self):
        print('left')
        self.__get_pin_status()
        
        if self.pin20_status:
            GPIO.output(self.forward_motor_A_pin_20, GPIO.LOW)
        if not self.pin16_status:
            GPIO.output(self.forward_motor_B_pin_16, GPIO.HIGH)
        if not self.pin26_status:
            GPIO.output(self.backward_motor_A_pin_26, GPIO.HIGH)
        if self.pin19_status:
            GPIO.output(self.backward_motor_B_pin_19, GPIO.LOW)
        
        time.sleep(self.__turn_sleep_time)
        GPIO.output(self.forward_motor_B_pin_16, GPIO.HIGH)
        GPIO.output(self.backward_motor_A_pin_26, GPIO.HIGH)

    def change_speed(self, duty):
        self.pwm13.ChangeDutyCycle(duty)

    def cleanup(self):
        print('Cleanup')
        self.stop()	
        time.sleep(0.5)
