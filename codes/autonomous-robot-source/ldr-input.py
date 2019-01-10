#!/usr/bin/env python

import RPi.GPIO as IO        
import time                           # calling for time to provide delays in program
import sys

print 'Setting up pin outs'

IO.setwarnings(False) 
IO.setmode(IO.BCM)

right_ldr_pin = 5
left_ldr_pin = 6

right_black_threshold = 32000 # more than 30000
left_black_threshold = 22000 # more than 22000

def rightLDR (right):
	count = 0
	IO.setup(right, IO.OUT)
	IO.output(right, IO.LOW)
	time.sleep(0.1)

	IO.setup(right, IO.IN)
  
	while (IO.input(right) == IO.LOW):
		count += 10
	#print "RIGHT READING: ", count
	return count


def leftLDR (left):
	count = 0
	IO.setup(left, IO.OUT)
	IO.output(left, IO.LOW)
	time.sleep(0.1)

	IO.setup(left, IO.IN)
  
	while (IO.input(left) == IO.LOW):
		count += 10
	#print "LEFT READING: ", count
	return count


try:
	while True:
		# both are in light color
		if ( 
            rightLDR(right_ldr_pin) < right_black_threshold 
            and leftLDR(left_ldr_pin) < left_black_threshold 
        ):
			print "Move backwards"
        
        # right is in light color
		elif rightLDR(right_ldr_pin) < right_black_threshold : 
			print "Move LEFT"

        # left is in light color
		elif leftLDR(left_ldr_pin) < left_black_threshold:
			print "Move RIGHT"
        
        # left and right is in black color
		else:
			print "FORWARD"
		

except KeyboardInterrupt:
    pass	
finally:	
	# Free/Clean resources before exit code
	print 'cleaning up'
	IO.cleanup()	
	time.sleep(2)


sys.exit(0)

# dark ldr reading nets more than 800 counts
# light ldr reading nets less than 700 counts
# further calibration needed
