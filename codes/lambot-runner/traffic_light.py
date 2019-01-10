#!/usr/bin/env python

import cv2

class TrafficLight:
    """
        TrafficLight class provides detectColorInTrafficLight method
        The method detects if traffic light is red for stop and green for go
    """
    
    def __init__(self):
        self.red_light = False
        self.green_light = False
        # minimum value to proceed traffic light state validation
        self.threshold = 120
    
    def detectColorInTrafficLight(self, image, roi_traffic_light, x, y, w, h):
        #reset
        self.red_light = False
        self.green_light = False

    	#find in roi_traffic_light colors red and green
        mask = cv2.GaussianBlur(roi_traffic_light, (25, 25), 0)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(mask)
		
	    # check if light is on
        if (maxVal - minVal) > self.threshold:
            cv2.circle(roi_traffic_light, maxLoc, 5, (255,0,0), 2)
                
            # Red light
            if 1.0/8*(h-30) < maxLoc[1] < 4.0/8*(h-30):
                cv2.putText(image, 'Red', (x+5, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                self.red_light = True
            # Green light
            elif 5.5/8*(h-30) < maxLoc[1] < h-30:
                print 'FOund green'
                cv2.putText(image, 'Green', (x+5, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.green_light = True
                
        return image

