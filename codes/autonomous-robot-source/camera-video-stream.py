#!/usr/bin/env python

from picamera import PiCamera
from picamera.array import PiRGBArray

import cv2
import time
import sys

try:
	print 'Initializing camera'
	
	camera = PiCamera()
	camera.resolution = (1024, 768)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(1024, 768))
	
	print 'Capturing raw video stream'
	
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		# grab the image from the frame object, and array properties that returns numpy array
		# then convert to gray scale	
		image = frame.array
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		cv2.imshow("Video Stream", image)
		
		# wait for a keypress, for q quiting the application
		key = cv2.waitKey(1) & 0xFF

		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)

		if key == ord("q"):
			break
			
except KeyboardInterrupt:
	pass
finally:
	print 'closing application'
	time.sleep(1)
	sys.exit(0)
