#!/usr/bin/env python

from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import cv2
import sys
import time

try:

	threshold = 150

	face_cascade = cv2.CascadeClassifier('traffic_light.xml')
	image = cv2.imread('./images/traffic3.jpg', cv2.IMREAD_COLOR)

	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	
	print("1----")
	
	faces = face_cascade.detectMultiScale(image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)) 
	print("2----")
	
	for (x,y,w,h) in faces:
		print('found a traffic light')
		image = cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
		roi_traffic_light = image[y:y+h, x:x+w]

		#find in roi_traffic_light colors red and green
		#roi_traffic_light = cv2.cvtColor(roi_traffic_light, cv2.COLOR_BGR2GRAY)
		mask = cv2.GaussianBlur(roi_traffic_light, (25, 25), 0)
		minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(mask)
			
		# check if light is on
		if (maxVal - minVal) > threshold:
			cv2.circle(roi_traffic_light, maxLoc, 5, (255,0,0), 2)
			# Red light
			if 1.0/8*(h-30) < maxLoc[1] < 4.0/8*(h-30):
				print('found red')
				cv2.putText(image, 'Red', (x+5, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				#self.red_light = True
			# Green light
			elif 5.5/8*(h-30) < maxLoc[1] < h-30:
				print('Found green')
				cv2.putText(image, 'Green', (x+5, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				#self.green_light = True

	cv2.imshow("Traffic Light Detection", image)
		
	# wait for a keypress, for q quiting the application
	cv2.waitKey(0)
	cv2.destroyAllWindows()
			
except KeyboardInterrupt:
	pass
finally:
	print('closing application')
	time.sleep(1)
	sys.exit(0)
