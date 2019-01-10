import RPi.GPIO as GPIO
import time
import sys

print 'Initializing pin setup...'
print GPIO.getmode()

# MOTOR A - left side of the car
forward_motor_A_pin_20  = 20
#backward_motor_A_pin_26 = 26

# MOTOR B - right side of the car
forward_motor_B_pin_16  = 16
#backward_motor_B_pin_19 = 19
enable_motor_B_pin_13   = 13

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)

GPIO.setup(forward_motor_A_pin_20, GPIO.OUT, initial=GPIO.LOW)
#GPIO.setup(backward_motor_A_pin_26, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(forward_motor_B_pin_16, GPIO.OUT, initial=GPIO.LOW)
#GPIO.setup(backward_motor_B_pin_19, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(enable_motor_B_pin_13, GPIO.OUT)
pwm13 = GPIO.PWM(enable_motor_B_pin_13, 1000)
pwm13.start(100) #less than 100 duty cycle = sound beeping (not enough)


time_interval_straight = 10
time_interval_side = 2.5

def low_motors():
	print 'LOW pins'
	GPIO.output(forward_motor_A_pin_20, GPIO.LOW)
	GPIO.output(forward_motor_B_pin_16, GPIO.LOW)
#	GPIO.output(backward_motor_A_pin_26, GPIO.LOW)
#	GPIO.output(backward_motor_B_pin_19, GPIO.LOW)
	time.sleep(1)

def move_forward(interval):
	low_motors()
	print 'forward'
	GPIO.output(forward_motor_A_pin_20, GPIO.HIGH)
	GPIO.output(forward_motor_B_pin_16, GPIO.HIGH)
	time.sleep(interval)

#Movements
time.sleep(2)
move_forward(time_interval_straight)

low_motors()

# Free/Clean resources before exit code
GPIO.cleanup()	
time.sleep(2)
print 'Cleansed the ports...'
sys.exit(0)
