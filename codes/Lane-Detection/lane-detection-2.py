#!/usr/bin/env python

# Reference Link:
# https://medium.com/@galen.ballew/opencv-lanedetection-419361364fc0
from __future__ import division # enables quotient with floating point
from picamera import PiCamera
from picamera.array import PiRGBArray
from mpmath import *
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

        self.left_line_x2_thresh_coord = 73 #adjust by increasing value 11 initially
        self.right_line_x1_thresh_coord = 160 #adjust by decreasing value 185 initially
        self.thresh_theta_for_removal = 15
        self.thresh_too_large_theta_for_removal = 60

    def cameraStreamCapture(self):
        print 'Capturing raw video stream'

        # capture frames from the camera
        for frame in self.camera.capture_continuous(self.RAW_CAPTURE, format="bgr", use_video_port=True):
            print "--------------------"
            image = frame.array

            gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            
            #hsv = [hue, saturation, value]
            #more accurate range for yellow since it is not strictly black, white, r, g, or b
            lower_yellow = np.array([20, 100, 100], dtype = "uint8")
            upper_yellow = np.array([30, 255, 255], dtype="uint8")
            
            mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
            mask_white = cv2.inRange(gray_image, 200, 255)
            mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
            mask_yw_image = cv2.bitwise_and(gray_image, mask_yw)
            
            kernel_size = 25
            gauss_gray = cv2.GaussianBlur(mask_yw_image, (kernel_size, kernel_size), 0)
            
            #same as quiz values
            low_threshold = 50
            high_threshold = 150
            canny_edges = cv2.Canny(gauss_gray,low_threshold,high_threshold)
            
            vertices = self.calculate_vertices(canny_edges)
            roi_image = self.region_of_interest(canny_edges, vertices)
            
            #rho and theta are the distance and angular resolution of the grid in Hough space
            #same values as quiz
            rho = 1
            theta = np.pi/180
            #threshold is minimum number of intersections in a grid for candidate line to go to output
            threshold = 20
            min_line_len = 50
            max_line_gap = 200
            lines, line_image = self.hough_lines(roi_image, rho, theta, threshold, min_line_len, max_line_gap)
            result = self.weighted_img(line_image, image)
            
            # start computation - direction determination to be computed here
            right_lines, left_lines = self.separate_lines(lines) #used just to log line coordinates 
            #print "right: ", right_lines
            #print "left: ", left_lines
            
            # filter or remove 0-15 degree thetas, andGet dominant line for right and left by getting largest degree/theta
            dominant_right = []
            dominant_left = []
            
            if right_lines is not None:
                #print "filtering right lines..."
                right_lines = self.remove_unwanted_lines(right_lines)
                dominant_right = self.get_dominant_line(right_lines)
            if left_lines is not None:
                #print "filtering left lines..."
                left_lines = self.remove_unwanted_lines(left_lines)
                dominant_left = self.get_dominant_line(left_lines)             
            
            print "dominant right: ", dominant_right
            print "dominant left: ", dominant_left       
            
            # Comparison
            self.direction_determination(dominant_right, dominant_left)
            
            cv2.imshow("1Original Image", image)
            #cv2.imshow("2Gray Image", gray_image)
            #cv2.imshow("3HSV", img_hsv)
            #cv2.imshow("4MaskImage", mask_yw_image)
            #cv2.imshow("5Gaussian", gauss_gray)
            
            #cv2.imshow("6Canny Edge", canny_edges)
            cv2.imshow("7Get Region of Interest", roi_image)
            cv2.imshow("lines in image", line_image)
            if result is not None:
                cv2.imshow("8Result", result)

            key = cv2.waitKey(1) & 0xFF
            self.RAW_CAPTURE.truncate(0)

            if key == ord("q"):
                print "Exiting Application"
                break


    def direction_determination(self, dominant_right, dominant_left) :
        if len(dominant_right) > 0 and len(dominant_left) > 0 :
            # only right thresh is hit
            if dominant_right[0] < self.right_line_x1_thresh_coord and dominant_left[2] <= self.left_line_x2_thresh_coord :
                print "go left please"
                    
                # only left thresh is hit
            elif dominant_right[0] >= self.right_line_x1_thresh_coord and dominant_left[2] > self.left_line_x2_thresh_coord :
                print "go right please"
            else :
                print "move forward please"
            # use ranges here, like minus 10 or plus 10 if both hits the threshold
            #elif dominant_right[0] < self.right_line_x1_thresh_coord and dominant_left[2] > self.left_line_x2_thresh_coord :
                #print "left and right hit both"              
                    
        elif len(dominant_right) > 0 and len(dominant_left) == 0  :
            # use for comparison the dominant right's dominant_right[0] to get x1 value
            if dominant_right[0] < self.right_line_x1_thresh_coord :
                print "go left please"
            else:
                print "Move forward"
        elif len(dominant_right) == 0 and len(dominant_left) > 0 :
            # use for comparison the dominant left's dominant_left[2] to get x2 value
            if dominant_left[2] > self.left_line_x2_thresh_coord :
                print "go right please"
            else:
                print "move forward"
        else :
            print "move forward only"


    def calculate_vertices(self,canny_edges):
        xsize = canny_edges.shape[0] # get rows in an image
        ysize = canny_edges.shape[1] # get columns in an image
        # adjust dx1 and dx2 value to readjust the polygonial shape's size
        dx1 = int(0.0325 * xsize) #0.0425
        dx2 = int(0.050 * xsize) #0.125
        dy = int(0.6 * ysize)
        # calculate vertices for region of interest
        vertices = np.array([[(dx1, ysize), (dx2, dy), (xsize - dx2, dy), (xsize - dx1, ysize)]], dtype=np.int32)
        return vertices


    def region_of_interest(self, img, vertices):
        #defining a blank mask to start with
        mask = np.zeros_like(img)

        #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
        if len(img.shape) > 2:
            channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        #filling pixels inside the polygon defined by "vertices" with the fill color
        cv2.fillPoly(mask, vertices, ignore_mask_color)

        #returning the image only where mask pixels are nonzero
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image


    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=5):
        if lines is not None:
            for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)


    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        self.draw_lines(line_img, lines)
        return lines, line_img


    def weighted_img(self, img, initial_img, a=0.8, b=1., c=0):
        if img is not None:
            return cv2.addWeighted(initial_img, a, img, b, c)
        else:
            return None


    def slope(self, x1, y1, x2, y2):
        return (y1 - y2) / (x1 - x2)

	
    def separate_lines(self, lines):
        right = []
        left = []
        if lines is not None:
            for x1,y1,x2,y2 in lines[:, 0]:
                m = self.slope(x1,y1,x2,y2)
                theta = self.get_theta(m)
                if m >= 0:
                    #print "0: ", theta ,", slope: ", m ,", right lanes: (", x1, "," , y1, ")", "- (", x2, ",", y2, ")"
                    right.append([x1,y1,x2,y2,m,theta])
                else:
                    #print "0: ", theta ,", slope: ", m ,", left lanes: (", x1, "," , y1, ")", "- (", x2, ",", y2, ")"
                    left.append([x1,y1,x2,y2,m,theta])
            return right, left
        else:
            #print "separate_lines - returning none"
            return None, None

    def get_theta(self, m_slope):
        rad = mp.atan(m_slope)
        deg = mp.degrees(rad)
        if deg < 0 :
			deg = abs(deg)
        return deg
        
    
    def remove_unwanted_lines(self, lines):
        filtered_lines = []
        for line in lines:
            #print "degree: ", line[5], ", remove_unwanted_lines: ", line
            if line[5] > self.thresh_theta_for_removal and line[5] < self.thresh_too_large_theta_for_removal:
                filtered_lines.append(line)
        return filtered_lines


    def get_dominant_line(self, lines):
        #print "lines: ", lines
        if lines is None or len(lines) == 0:
            return [] #empty list
        # sort via last element of inner array, which is the degree value
		# sort from highest to lowest    
        sorted_line = sorted(lines, key=lambda x: x[5], reverse=True)
        #print "sorted line: ", sorted_line
        return sorted_line[0] #returns highest
            

cameraStream = CameraStream()
cameraStream.cameraStreamCapture()
