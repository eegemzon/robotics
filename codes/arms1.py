import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(7,GPIO.OUT)

try:
	while True:
		GPIO.output(7,1)
		print('move')
		time.sleep(0.0015)
		GPIO.output(7,0)
		print('stop')
		time.sleep(2)

except KeyboardInterrupt:
	GPIO.cleanup()
