#!/usr/bin/env python3

# Use TAB INDENTATION ONLY on this file!!!!!!!!!!!!!!
# This file is for sample integration of lambot-runner with the lane-detection-2.py

from threading import Thread
from picamera import PiCamera
from picamera.array import PiRGBArray
from motor_control import MotorControl
from traffic_light import TrafficLight
from lane_and_direction import LaneAndDirection

import RPi.GPIO as IO
import numpy as np
import cv2
import sys
import time

# Global variables to be shared by some threads

global quitApp
global red_traffic_light
global stop_sign_found
global pass_thru_gate_found
global direction

quitApp = False
red_traffic_light = False 
stop_sign_found = False
pass_thru_gate_found = False
direction = 0
# Camera Streaming

class CameraStream:
    def __init__(self):
        self.traffic_light_cascade = cv2.CascadeClassifier('traffic_light.xml')
        self.stop_sign_cascade = cv2.CascadeClassifier('stop_sign.xml')
        #self.pass_thru_gate_cascade = cv2.CascadeClassifier('cascade.xml')
        print('Initializing camera')
        self.camera = PiCamera()
        self.trafficLight = TrafficLight()
        self.laneAndDirection = LaneAndDirection()
        self.camera.resolution = (256, 256)
        self.camera.framerate = 32
        self.camera.vflip = True
        self.camera.hflip = True
        self.RAW_CAPTURE = PiRGBArray(self.camera, size=(256,256))

    def cameraStreamCapture(self):
        print('Capturing raw video stream')
        global quitApp
        global red_traffic_light
        global stop_sign_found
        global pass_thru_gate_found
        global direction

        # capture frames from the camera
        for frame in self.camera.capture_continuous(self.RAW_CAPTURE, format="bgr", use_video_port=True):
            
            # grab the image from the frame object, and array properties that returns numpy array
            # then convert to gray scale
            image = frame.array
            
            display_image = image
            gray_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2GRAY)

            traffic_lights = self.traffic_light_cascade.detectMultiScale(gray_image, 1.3, 5)
            stop_signs = self.stop_sign_cascade.detectMultiScale(gray_image, 1.3, 5)
            #pass_thru_gates = self.pass_thru_gate_cascade.detectMultiScale(gray_image, 1.3, 5)

            # Traffic Light detection
            for (x, y, w, h) in traffic_lights:
                print('Found traffic light')
                display_image = cv2.rectangle(display_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                roi_traffic_light = gray_image[y:y+h, x:x+w]
                display_image = self.trafficLight.detectColorInTrafficLight(display_image, roi_traffic_light, x, y, w, h)

                if self.trafficLight.red_light:
                    print('Red light found - camera stream thread')
                    red_traffic_light = self.trafficLight.red_light

            # Stop Sign detection
            for(x, y, w, h) in stop_signs:
                print('Found stop sign')
                display_image = cv2.rectangle(display_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                stop_sign_found = True

            if red_traffic_light == False and stop_sign_found == False :
                roi_image, line_image, result, direction = self.laneAndDirection.process_image_direction(image)
                cv2.imshow("2ROI ", roi_image)
                cv2.imshow("3Lines", line_image)
                cv2.imshow("4Result", result)
                       
            cv2.imshow("1Camera View", display_image)
            
            
            # wait for a keypress, for q quiting the application
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            self.RAW_CAPTURE.truncate(0)

            if key == ord("q"):
                print("Exiting Application")
                quitApp = True
                break


# Main program

try:
    print("Opencv Version: ", cv2.__version__)
    motorControl = MotorControl()
    cameraStream = CameraStream()

    cameraThread = Thread(target=cameraStream.cameraStreamCapture)
    cameraThread.start()
   
    while quitApp == False:
        time.sleep(3)

        if cameraThread.isAlive():
            if red_traffic_light:
                print('Found red traffic light - main thread')
                time.sleep(1)
                motorControl.stop()
                time.sleep(3)
                red_traffic_light = False
            elif stop_sign_found:
                print('Found stop sign - main thread')
                time.sleep(6)
                motorControl.stop()
                time.sleep(3)
                motorControl.move_forward()
                time.sleep(3)
                stop_sign_found = False
            else:
				# sleep time below will be for blindness
                adjustment_time_forward = 0.75
                adjustment_time_turn = 0.3 #0.67 #try 0.5 with 70 turn cylce
                
                if direction == -1 :
                    print("go left -- main thread")
                    time.sleep(adjustment_time_turn)
                    motorControl.change_speed(motorControl.turn_duty_cycle)
                    motorControl.turn_left()
                elif direction == 1 :
                    print("go right -- main thread")
                    time.sleep(adjustment_time_turn)
                    motorControl.change_speed(motorControl.turn_duty_cycle)
                    motorControl.turn_right()
                elif direction == 0:
                    print("move forward -- main thread")
                    time.sleep(adjustment_time_forward)
                    motorControl.change_speed(motorControl.forward_duty_cycle)
                    motorControl.move_forward()

except KeyboardInterrupt:
    #print "Interrupted processing..."
    pass
except Exception as e:
    print(str(e))
finally:
    motorControl.cleanup()
    IO.cleanup()
    sys.exit(0)
