#!/usr/bin/env python

import numpy as np
import cv2
import sys
import time

#OKAY IMAGES:
# stop2_with_hand.jpg, 
# stop_sign_ped.jpg,
# stop3.jpg
# stop4.jpg

while True:
	stop_sign_cascade = cv2.CascadeClassifier('stop_sign.xml')
		
	image = cv2.imread('./images/stop2_with_hand.jpg', 1)
	image = cv2.resize(image, (256,256))
	grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			
	stop_signs = stop_sign_cascade.detectMultiScale(grey_image, 1.3, 5) 
			
	for (x,y,w,h) in stop_signs:
		print 'found a stop sign'
		image = cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)

	cv2.imshow("Stop Sign Detection", image)
	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		print "Exiting Application"
		cv2.destroyAllWindows()
		break

