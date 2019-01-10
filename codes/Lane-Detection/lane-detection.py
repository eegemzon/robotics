#!/usr/bin/env python

# Reference Link:
# https://medium.com/@ldesegur/a-lane-detection-approach-for-self-driving-vehicles-c5ae1679f7ee

from picamera import PiCamera
from picamera.array import PiRGBArray
import matplotlib.pyplot as plt
import RPi.GPIO as IO
import numpy as np
import cv2
import sys
import time


class CameraStream:
    def __init__(self):
        print 'Initializing camera'
        self.camera = PiCamera()
        self.camera.resolution = (256, 256)
        self.camera.framerate = 40
        self.camera.vflip = True
        self.camera.hflip = True
        self.RAW_CAPTURE = PiRGBArray(self.camera, size=(256, 256))

    def cameraStreamCapture(self):
        print 'Capturing raw video stream'

        # capture frames from the camera
        for frame in self.camera.capture_continuous(self.RAW_CAPTURE, format="bgr", use_video_port=True):
            print "--------------------"
            image = frame.array

            gaussian_image = cv2.GaussianBlur(image, (21,21), 0) ## 25,25
            gray_gaussian_image = cv2.cvtColor(gaussian_image, cv2.COLOR_BGR2GRAY)
            canny_edge_image = cv2.Canny(gray_gaussian_image, 50, 150) #low threshold=50, high threshold=150
            
            xsize = canny_edge_image.shape[0] # get rows in an image
            ysize = canny_edge_image.shape[1] # get columns in an image
            
            # adjust dx1 and dx2 value to readjust the polygonial shape's size
            dx1 = int(0.0425 * xsize) #0.0425
            dx2 = int(0.125 * xsize) #0.125
            dy = int(0.6 * ysize)
            
            # calculate vertices for region of interest
            vertices = np.array([[(dx1, ysize), (dx2, dy), (xsize - dx2, dy), (xsize - dx1, ysize)]], dtype=np.int32)
            
            region_of_interest_image = self.region_of_interest(canny_edge_image, vertices)
            
            # yanyan - trial added code
            rho = 0.8 #1 #6
            theta = np.pi/180 #np.pi/60
            threshold = 25 #15 #160
            min_line_len = 50 #10 #40
            max_line_gap = 200 #20 #25
            lines = lines = cv2.HoughLinesP(region_of_interest_image, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
            print "lines after hough Lines: ", lines
            #self.draw_lines(region_of_interest_image, lines)
			
            right_lines, left_lines = self.separate_lines(lines)
            right = ""
            left = ""
            
            if right_lines is not None and left_lines is not None:
                if right_lines and left_lines:
                    right = self.reject_outliers(right_lines,  cutoff=(0.45, 0.75))
                    left = self.reject_outliers(left_lines, cutoff=(-0.85, -0.6))
                    print "rejected right lines: ", right
                    print "rejected left lines: ", left

                x1, y1, m1, c1 = self.lines_linreg(right_lines)
                x2, y2, m2, c2 = self.lines_linreg(left_lines)
				
                if all(value is not None for value in (x1, y1, m1, c1, x2, y2, m2, c2)):
                    # This variable represents the top-most point in the image where we can reasonable draw a line to.
                    min_y1 = np.min(y1)
				    # Calculate the top point using the slopes and intercepts we got from linear regression.
                    top_point1 = np.array([(min_y1 - c1) / m1, min_y1], dtype=int)
                    # Repeat this process to find the bottom left point.
                    max_y1 = np.max(y1)
                    bot_point1 = np.array([(max_y1 - c1) / m1, max_y1], dtype=int)

                    min_y2 = np.min(y2)
                    # Calculate the top point using the slopes and intercepts we got from linear regression.
                    top_point2 = np.array([(min_y2 - c2) / m2, min_y2], dtype=int)
                    # Repeat this process to find the bottom left point.
                    max_y2 = np.max(y2)
                    bot_point2 = np.array([(max_y2 - c2) / m2, max_y2], dtype=int)
            
                    x1e, y1e = self.extend_point(bot_point1[0],bot_point1[1],top_point1[0],top_point1[1], -1000) # bottom point
                    x2e, y2e = self.extend_point(bot_point2[0],bot_point2[1],top_point2[0],top_point2[1],  1000) # top point
 
                    line = np.array([[x1e,y1e,x2e,y2e]])
                    lines = np.array([line], dtype=np.int32)
                    #self.draw_lines(region_of_interest_image, lines)
            
                    line_image = np.copy((image)*0)
                    self.draw_lines(line_image, lines, thickness=3)
                    line_image = self.region_of_interest(line_image, vertices)
                    final_image = self.weighted_image(line_image, image)
                    cv2.imshow("6Final Image", final_image)
                    cv2.imshow("7lineImage", line_image)
            
            cv2.imshow("1Original Image", image)
            cv2.imshow("2Gaussian", gaussian_image)
            cv2.imshow("3Gray Image", gray_gaussian_image)
            cv2.imshow("4Canny Edge", canny_edge_image)
            cv2.imshow("5Get Region of Interest", region_of_interest_image)
            

            key = cv2.waitKey(1) & 0xFF
            self.RAW_CAPTURE.truncate(0)

            if key == ord("q"):
                print "Exiting Application"
                break

    def region_of_interest(self, canny_edge_image, vertices):
        # defining a blank mask to start with 
        mask = np.zeros_like(canny_edge_image)
        
        if len(canny_edge_image.shape) > 2:
            channel_count = canny_edge_image.shape[2]  # i.e. 3 or 4 depending on your image
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        # filling pixels inside the polygon defined by "vertices" with the fill color
        cv2.fillPoly(mask, vertices, ignore_mask_color)
  
        # returning the image only where mask pixels are non-zero
        masked_image = cv2.bitwise_and(canny_edge_image, mask)
        return masked_image


    def draw_lines(self, image, lines, color=[255, 0, 0], thickness=2):
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(image, (x1, y1), (x2, y2), color, thickness)


    def slope(self, x1, y1, x2, y2):
        return (y1 - y2) / (x1 - x2)

	
    def separate_lines(self, lines):
        right = []
        left = []
        if lines is not None:
            for x1,y1,x2,y2 in lines[:, 0]:
                m = self.slope(x1,y1,x2,y2)
                if m >= 0:
                    print "right lanes: (", x1, "," , y1, ")", "- (", x2, ",", y2, ")"
                    right.append([x1,y1,x2,y2,m])
                else:
                    print "left lanes: (", x1, "," , y1, ")", "- (", x2, ",", y2, ")"
                    left.append([x1,y1,x2,y2,m])
            return right, left
        else:
            print "separate_lines - returning none"
            return None, None

 
    def reject_outliers(self, data, cutoff, threshold=0.08):
        data = np.array(data)
        data = data[(data[:, 4] >= cutoff[0]) & (data[:, 4] <= cutoff[1])]
        m = np.mean(data[:, 4], axis=0)
        return data[(data[:, 4] <= m+threshold) & (data[:, 4] >= m-threshold)]


    def lines_linreg(self, lines_array):
        lines_array = np.asarray(lines_array)
        if len(lines_array) > 0:
			x = np.reshape(lines_array[:, [0, 2]], (1, int(len(lines_array) * 2)))[0]
			y = np.reshape(lines_array[:, [1, 3]], (1, int(len(lines_array) * 2)))[0]
			A = np.vstack([x, np.ones(len(x))]).T
			m, c = np.linalg.lstsq(A, y)[0]
			x = np.array(x)
			y = np.array(x * m + c)
			return x, y, m, c
        else:
			print "lines_linreg - returning none"
			return None, None, None, None


    def extend_point(self, x1, y1, x2, y2, length):
        line_len = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        x = x2 + (x2 - x1) / line_len * length
        y = y2 + (y2 - y1) / line_len * length
        return x, y


    def weighted_image(self, image, initial_image, a=0.8, b=1., c=0.):
        return cv2.addWeighted(initial_image, a, image, b, c)

cameraStream = CameraStream()
cameraStream.cameraStreamCapture()
