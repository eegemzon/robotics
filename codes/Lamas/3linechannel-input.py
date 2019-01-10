#!/usr/bin/env python

import RPi.GPIO as GPIO        
import time                           # calling for time to provide delays in program
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
    __turn_sleep_time = 0.5

    def __init__(self):
        print 'Initializing motor pin setup...'

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
        print 'Setting up pwm for speed control'
        GPIO.setup(self.enable_motor_B_pin_13, GPIO.OUT)
        self.pwm13 = GPIO.PWM(self.enable_motor_B_pin_13, 1000)
        self.pwm13.start(100) #less than 100 duty cycle = sound beeping (not enough)
        print 'Finished pin setup successfully'
        
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
    	print 'stopping motors'
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
        print 'forward'
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
        print 'backwards'
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
        print 'right'
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
        print 'left'
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
        GPIO.output(self.forward_motor_B_pin_16, GPIO.LOW)
        GPIO.output(self.backward_motor_A_pin_26, GPIO.LOW)

    def cleanup(self):
        print 'Cleanup'
        self.stop()	
        time.sleep(0.5)

print 'Setting up pin outs for LDR  start'
#ALREADY DEFINED 
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)

#1st sensor
GPIO.setup(2,GPIO.IN) # left sensor on pin2
GPIO.setup(3,GPIO.IN) # middle sensor on pin3
GPIO.setup(4,GPIO.IN) # right sensor on pin4 -- this is not working


#2nd sensor
GPIO.setup(17,GPIO.IN) # left sensor on pin17
GPIO.setup(27,GPIO.IN) # middle sensor on pin27
GPIO.setup(22,GPIO.IN) # right sensor on pin22 



detect_dark_color = GPIO.HIGH #true
detect_light_color = GPIO.LOW #false

print 'Setting up pin outs for LDR  end'

test = MotorControl()  


try:
	while True:
		# left sensor
		#if ( GPIO.input(2) == detect_dark_color):
		#	print "left dark 2"
        
        # middle sensor 
		# if GPIO.input(3) == detect_dark_color : 
		#	print "middle dark 3"

        # right sensor NOT WORKING ON HARDWARE
		#if GPIO.input(4) == detect_dark_color : 
		# 	print "dark 4"
      
        # 1st sensor
		if ((GPIO.input(2) == detect_light_color) and (GPIO.input(3) == detect_dark_color)): 
			print "detected light color - move left"
			test.turn_left()      

        # 2nd sensor
		if ((GPIO.input(3) == detect_light_color) and (GPIO.input(2) == detect_dark_color)): 
			print "detected light color - 22"
			test.turn_right()      

		#if ((GPIO.input(22) == detect_light_color)):# or (GPIO.input(2) == detect_light_color) or (GPIO.input(4) == detect_light_color)): 
		#	print "detected light color - 22"


		#if ((GPIO.input(17) == detect_light_color)):# or (GPIO.input(2) == detect_light_color) or (GPIO.input(4) == detect_light_color)): 
		#	print "detected light color - 17"
      
        # left sensor
		#if ( GPIO.input(2) == detect_light_color) and (GPIO.input(3) == detect_dark_color):
		#	print "detected light color of left sensor pin 2 - move right"
		#	test.turn_right()
        
        # middle sensor
		#if ((GPIO.input(3) == detect_light_color) and (GPIO.input(2) == detect_dark_color)): 
		#	print "detected light color of middle sensor pin 3 - move left"
		#	test.turn_left()
		
		#if (GPIO.input(3) == detect_dark_color and GPIO.input(2)  == detect_dark_color):
		#	 print "both dark - forward"
		#	 test.move_forward()
		
		#if (GPIO.input(3) == detect_light_color and GPIO.input(2)  == detect_light_color):
		#	 print "both dark - forward"
		#	 test.move_forward()
		
			 
		#if ((GPIO.input(3) == detect_light_color and GPIO.input(2)  == detect_light_color) or (GPIO.input(3) == detect_dark_color and GPIO.input(2)  == detect_dark_color)):
		#	 print "both light and dark- forward"
		#	 test.move_forward()
			 

		
        # right sensor  NOT WORKING ON HARDWARE
		 #if GPIO.input(4) == detect_light_color : 
		 #	print "light 4"

		

except KeyboardInterrupt:
    pass	
finally:	
	# Free/Clean resources before exit code
	print 'cleaning up'
	GPIO.cleanup()	
	time.sleep(2)


sys.exit(0)

# dark ldr reading nets more than 800 counts
# light ldr reading nets less than 700 counts
# further calibration needed
