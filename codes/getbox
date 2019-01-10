#!/usr/bin/env python


import RPi.GPIO as IO        
import time                           # calling for time to provide delays in program
import sys


print 'Initializing pin setup...'

#baseSM = 19
rightSM=19
leftSM=20

IO.setwarnings(False)          # do not show any warnings
IO.setmode (IO.BCM)            
#IO.setup(baseSM,IO.OUT)             # initialize GPIO19 as an output
IO.setup(rightSM,IO.OUT)            
IO.setup(leftSM,IO.OUT)             
#p = IO.PWM(baseSM,50)              # GPIO19 as PWM output, with 50Hz frequency
r = IO.PWM(rightSM,50)
l = IO.PWM(leftSM,50)
r.start(7.5)                             # generate PWM signal with 7.5% duty cycle

print 'Preparing arms movements...'

def testing():
	r.ChangeDutyCycle(2.5)       # base 0 degrees           
	time.sleep(2)                                     # sleep for 1 second
	r.ChangeDutyCycle(7.5)       # base 90 degrees  our initial position 
	time.sleep(2)                                      # sleep for 1 second
	r.ChangeDutyCycle(12.5)      # base 180 degrees            
	time.sleep(2)              
	r.ChangeDutyCycle(2.5)       # base 0 degrees           
	time.sleep(2)                                     # sleep for 1 second
	r.ChangeDutyCycle(7.5)       # base 90 degrees  our initial position 
	time.sleep(2)                                     # sleep for 1 second
	r.ChangeDutyCycle(12.5)      # base 180 degrees            
	time.sleep(2)            

#testing()

dutyCycle = 3.0


print 'returning to original position'

#while (dutyCycle != 2.5) :
#	p.ChangeDutyCycle(dutyCycle)
#	dutyCycle = dutyCycle - 0.5
#	time.sleep(0.05)

while (dutyCycle != 12.5):
	r.ChangeDutyCycle(dutyCycle)
	dutyCycle = dutyCycle + 0.5
	time.sleep(0.05)

print 'Cleaning the ports...'

# Free/Clean resources before exit code
IO.cleanup()	
time.sleep(2)

sys.exit(0)
