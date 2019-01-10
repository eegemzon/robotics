#!/usr/bin/env python

from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import cv2
import sys
import time

try:
	print 'Initializing camera'
	
	face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
	cap = cv2.VideoCapture(-1)
		
	#while True:
	ret, img = cap.read()
	#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#faces = face_cascade.detectMultiScale(gray, 1.3, 5)

	#for (x,y,w,h) in faces:
	#	cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
	
	image = cv2.imread(img, 0)
	cv2.imshow('img',img)
   		
	# wait for a keypress, for q quiting the application
	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		sys.exit(0)
			
except KeyboardInterrupt:
	pass
finally:
	print 'closing application'
	time.sleep(1)
	sys.exit(0)

