import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

TRIG = 3 
ECHO = 2

print "Distance Measurement In Progress"

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

try: 

	GPIO.output(TRIG,False)
	print "Waiting For Sensor To Settle"
	time.sleep(2)
	hasReceived = 0
	
	while True:
		
		pulse_start = 0
		pulse_duration = 0
		distance = 0
		hasReceived = 0		
		
		GPIO.output(TRIG,True)
		pulse_start = time.time()
		#print "pulse_start :", pulse_start
		hasReceived = GPIO.input(ECHO)
		
		while hasReceived == 0:
			time.sleep(0.00001)
			hasReceived = GPIO.input(ECHO)
			if (time.time() - pulse_start) >= 1:
				#hasReceived = -1
				#print "Timeout"
				break
				
		#print "ed :", hasReceived
		if hasReceived == 1:
			#print "if2"
			pulse_duration = time.time() - pulse_start
			distance = (pulse_duration * 34300) / 2
			#distance = round(distance, 2)
			print "Distance:",distance,"cm , Pulse:" , pulse_duration, " , hasReceived :", hasReceived
			#time.sleep(2)
		else:
			print "Infinity"
		
		GPIO.output(TRIG,False)
		pulse_start = 0
		pulse_duration = 0
		distance = 0
		hasReceived = 0
		time.sleep(1)#(.05)
		
	##GPIO.output(TRIG,True)
	##time.sleep(0.00001)
	##GPIO.output(TRIG,False)
	##print "testing"
	
	#while GPIO.input(ECHO)==0:
		#pulse_start = time.time()
		#print "testing IN WHILE"
	#while GPIO.input(ECHO)==1:
		#pulse_end = time.time()
		#print "testing IN WHILE2"
	#pulse_duration = pulse_end - pulse_start

	#distance = pulse_duration * 17150

	#distance = round(distance, 2)

	#print "Distance:",distance,"cm"




except Exception as e:
    print(str(e))    
finally:
	
	# Free/Clean resources before exit code
	GPIO.cleanup()	
	time.sleep(2)
	print 'Cleansed the ports...'


