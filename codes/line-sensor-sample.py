#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import sys

print 'Initializing pin setup...'

# MOTOR A - left side of the car
linesensor_A_pin_2  = 2
linesensor_pin_3 = 3
linesensor_A_pin_4 = 4


#Enable PIN, gpio 13 used for PWM to control speed
enable_motor_B_pin_13   = 13

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(forward_motor_A_pin_20, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(backward_motor_A_pin_26, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(forward_motor_B_pin_16, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(backward_motor_B_pin_19, GPIO.OUT, initial=GPIO.LOW)

#PWM Setup 13
print 'Setting up pwm for speed control'
GPIO.setup(enable_motor_B_pin_13, GPIO.OUT)
pwm13 = GPIO.PWM(enable_motor_B_pin_13, 1000)
pwm13.start(100) #less than 100 duty cycle = sound beeping (not enough)
print 'Finished pin setup successfully'

# Time properties
time_interval_straight = 2
time_interval_side = 0.70

def low_motors():
	print 'LOW pins'
	GPIO.output(forward_motor_A_pin_20, GPIO.LOW)
	GPIO.output(forward_motor_B_pin_16, GPIO.LOW)
	GPIO.output(backward_motor_A_pin_26, GPIO.LOW)
	GPIO.output(backward_motor_B_pin_19, GPIO.LOW)
	time.sleep(1)

def move_forward(interval):
	low_motors()
	print 'forward'
	GPIO.output(forward_motor_A_pin_20, GPIO.HIGH)
	GPIO.output(forward_motor_B_pin_16, GPIO.HIGH)
	time.sleep(interval)

def move_backwards(interval):
	low_motors()
	print 'backwards'
	GPIO.output(backward_motor_A_pin_26, GPIO.HIGH)
	GPIO.output(backward_motor_B_pin_19, GPIO.HIGH)
	time.sleep(interval)

def turn_right(interval):
	low_motors()
	print 'right'
	GPIO.output(forward_motor_A_pin_20, GPIO.HIGH)
	GPIO.output(backward_motor_B_pin_19, GPIO.HIGH)
	time.sleep(interval)

def turn_left(interval):
	low_motors()
	print 'left'
	GPIO.output(backward_motor_A_pin_26, GPIO.HIGH)
	GPIO.output(forward_motor_B_pin_16, GPIO.HIGH)
	time.sleep(interval)

try:
# Movements
	time.sleep(2) # wait 2 seconds before moving
	#move_forward(time_interval_straight)
	#move_backwards(time_interval_straight)
	#move_forward(time_interval_straight)
	#turn_right(time_interval_side)
	#move_forward(time_interval_straight)
	#turn_left(time_interval_side)
	#move_forward(time_interval_straight)
	
	move_forward(5)
	turn_left(5)
	turn_right(5)
	
	
except KeyboardInterrupt:
    pass	
finally:
	low_motors()
	# Free/Clean resources before exit code
	GPIO.cleanup()	
	time.sleep(2)
	print 'Cleansed the ports...'
sys.exit(0)
