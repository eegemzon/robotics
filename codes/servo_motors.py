#!/usr/bin/env python3


import RPi.GPIO as IO        
import time       
import sys


print('Initializing pin setup...')


IO.setwarnings(False)	
IO.setmode (IO.BCM)   

clip = 12

IO.setup(clip,IO.OUT)     
pwm = IO.PWM(clip,50)	
pwm.start(2.5)

def init():
	print('Init')
	pwm.ChangeDutyCycle(2.5)

def clipped():
	print('Clipping')
	pwm.ChangeDutyCycle(12.5)


try:
	#while(True):
		#init()
		#time.sleep(1)
		#clipped()
		#time.sleep(2)
	
	init()
	time.sleep(1)
	clipped()
	time.sleep(2)
	while(True):
		print('') #cleanup after this will cause the arm to open up a little bit
	
except KeyboardInterrupt:
	pass
finally:
	# Free/Clean resources before exit code
	IO.cleanup()	
	time.sleep(2)

sys.exit(0)
