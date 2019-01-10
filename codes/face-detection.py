#!/usr/bin/env python

from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import cv2
import sys
import time

try:
	print 'Initializing camera'
	
	camera = PiCamera()
	camera.resolution = (250, 250)
	camera.framerate = 32
	camera.vflip = True
	camera.hflip = True
	
	rawCapture = PiRGBArray(camera, size=(250, 250))
	
	face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
	
	print 'Capturing raw video stream'
	
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		# grab the image from the frame object, and array properties that returns numpy array
		# then convert to gray scale	
		image = frame.array
		#image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		
		faces = face_cascade.detectMultiScale(image, 1.3, 5) 
		
		for (x,y,w,h) in faces:
			print 'found a face'
			image = cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
			
		cv2.imshow("Face Detection", image)
		
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
