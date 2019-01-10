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

class LDRReading:
	def __init__(self):
		self.__ldr_pin = 6
		self.threshold_low = 320
		self.threshold_high = 3700

	def read(self):
		count = 0
		IO.setup(self.__ldr_pin, IO.OUT)
		IO.output(self.__ldr_pin, IO.LOW)
		time.sleep(0.1)
		IO.setup(self.__ldr_pin, IO.IN)
		while(IO.input(self.__ldr_pin) == IO.LOW):
			count += 1
		print("Reading: ", count)
		return count


try:
	#while(True):
		#init()
		#time.sleep(1)
		#clipped()
		#time.sleep(2)
	reader = LDRReading()
	while True:
		count = reader.read()
		if count > reader.threshold_low and count < reader.threshold_high:
			init()
			clipped()
			time.sleep(0.5)
			while(True):
				time.time()

	
except KeyboardInterrupt:
	pass
finally:
	# Free/Clean resources before exit code
	IO.cleanup()	
	time.sleep(2)

sys.exit(0)
