#!/usr/bin/env python

from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import cv2
import sys
import time
import pytesseract

try:
    print 'Initializing camera'
	
    camera = PiCamera()
    camera.resolution = (256, 256)
    camera.framerate = 32
    camera.vflip = True
    camera.hflip = True

    rawCapture = PiRGBArray(camera, size=(256, 256))
	
    print 'Capturing raw video stream'
	
	# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		# grab the image from the frame object, and array properties that returns numpy array
		# then convert to gray scale	
        image = frame.array
        grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        print "running time: ", time.time()
        # Apply dilation and erosion to remove some noise
        kernel = np.ones((1, 1), np.uint8)
        grey_image = cv2.dilate(grey_image, kernel, iterations=1)
        grey_image = cv2.erode(grey_image, kernel, iterations=1)
        
        result = pytesseract.image_to_string(grey_image)
        
        if result.lower() == "cargo":
            print "Found Cargo"

        cv2.imshow("Cargo", image)
		
		# wait for a keypress, for q quiting the application
        key = cv2.waitKey(1) & 0xFF

		# clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        if key == ord("q"):
            print "Exiting Application"
            cv2.destroyAllWindows()
            break

except KeyboardInterrupt:
	pass
except Exception as e:
    print str(e)
finally:
	print 'closing application'
	time.sleep(1)
	sys.exit(0)
