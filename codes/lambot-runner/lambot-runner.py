#!/usr/bin/env python

# Use TAB INDENTATION ONLY on this file!!!!!!!!!!!!!!

from threading import Thread
from picamera import PiCamera
from picamera.array import PiRGBArray
from motor_control import MotorControl
from traffic_light import TrafficLight

import RPi.GPIO as IO
import numpy as np
import cv2
import sys
import time

# Global variables to be shared by some threads

global quit
global red_traffic_light
global stop_sign_found
global pass_thru_gate_found

quit = False
red_traffic_light = False 
stop_sign_found = False
pass_thru_gate_found = False

# Camera Streaming

class CameraStream:
    def __init__(self):
        self.traffic_light_cascade = cv2.CascadeClassifier('traffic_light.xml')
        self.stop_sign_cascade = cv2.CascadeClassifier('stop_sign.xml')
        #self.pass_thru_gate_cascade = cv2.CascadeClassifier('cascade.xml')

        print 'Initializing camera'
        self.camera = PiCamera()
        self.trafficLight = TrafficLight()
        self.camera.resolution = (256, 256)
        self.camera.framerate = 32
        self.camera.vflip = True
        self.camera.hflip = True
        self.RAW_CAPTURE = PiRGBArray(self.camera, size=(256, 256))

    def cameraStreamCapture(self):
        print 'Capturing raw video stream'
        global quit
        global red_traffic_light
        global stop_sign_found
        global pass_thru_gate_found

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
                print 'Found traffic light'
                display_image = cv2.rectangle(display_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                roi_traffic_light = gray_image[y:y+h, x:x+w]
                display_image = self.trafficLight.detectColorInTrafficLight(display_image, roi_traffic_light, x, y, w, h)

                if self.trafficLight.red_light:
                    print 'Red light found - camera stream thread'
                    red_traffic_light = self.trafficLight.red_light

            # Stop Sign detection
            for(x, y, w, h) in stop_signs:
                print 'Found stop sign'
                display_image = cv2.rectangle(display_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                stop_sign_found = True

            # Pass Thru Gate detection
            #for(x, y, w, h) in pass_thru_gates:
            #    print 'Found pass thru gate'
             #   image = cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
             #   pass_thru_gate_found = True

            cv2.imshow("Camera View", display_image)

            # wait for a keypress, for q quiting the application
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            self.RAW_CAPTURE.truncate(0)

            if key == ord("q"):
                print "Exiting Applicating"
                quit = True
                break


# Main program

try:
    motorControl = MotorControl()
    cameraStream = CameraStream()

    cameraThread = Thread(target=cameraStream.cameraStreamCapture)
    cameraThread.start()

    while quit == False:
        #print "running app... time: ", time.time()
        time.sleep(0.5)

        if cameraThread.isAlive():
            if red_traffic_light:
                print 'Found red traffic light - main thread'
                time.sleep(1)
                motorControl.stop()
                time.sleep(3)
                red_traffic_light = False
            elif stop_sign_found:
                print 'Found stop sign - main thread'
                time.sleep(1)
                motorControl.stop()
                time.sleep(3)
                motorControl.move_forward()
                time.sleep(3)
                stop_sign_found = False
            #elif pass_thru_gate_found:
                #print 'Found pass thru gate - main thread'
                #time.sleep(1)
                #motorControl.stop()
                #time.sleep(3)
                #pass_thru_gate_found = False
            else:
                motorControl.move_forward()

except KeyboardInterrupt:
    print "Interrupted processing..."
    pass
except Exception as e:
    print str(e)
finally:
    motorControl.cleanup()
    IO.cleanup()
    sys.exit(0)
