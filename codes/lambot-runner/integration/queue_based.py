#!/usr/bin/env python3

from picamera import PiCamera
from picamera.array import PiRGBArray
from motor_control import MotorControl
from traffic_light import TrafficLight
from lane_and_direction import LaneAndDirection
from multiprocessing import Queue, Process
import RPi.GPIO as IO
import numpy as np
import cv2
import sys
import time

# Camera Streaming

class CameraStream:
    def __init__(self):
        self.traffic_light_cascade = cv2.CascadeClassifier('traffic_light.xml')
        self.stop_sign_cascade = cv2.CascadeClassifier('stop_sign.xml')
        

    def cameraStreamCapture(self,quitApp,red_traffic_light,stop_sign_found,direction):
        print('Initializing camera')
        self.camera = PiCamera()
        self.trafficLight = TrafficLight()
        self.laneAndDirection = LaneAndDirection()
        self.camera.resolution = (256, 256)
        self.camera.framerate = 32
        #self.camera.vflip = True
        #self.camera.hflip = True
        self.RAW_CAPTURE = PiRGBArray(self.camera, size=(256,256))
        
        # capture frames from the camera
        try:
            for frame in self.camera.capture_continuous(self.RAW_CAPTURE, format="bgr", use_video_port=True):
                image = frame.array
                cv2.rectangle(image,(65,240),(195,256),(0,0,0),thickness=cv2.FILLED)
                
                display_image = image
                gray_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2GRAY)

                traffic_lights = self.traffic_light_cascade.detectMultiScale(gray_image, 1.3, 5, minSize=(15,15))
                stop_signs = self.stop_sign_cascade.detectMultiScale(gray_image, 1.3, 5, minSize=(25,25))

                #display_image = self.trafficLight.detectColorInTrafficLight2(display_image)
                #if self.trafficLight.red_light:
                    #print('Red light found - camera stream thread')
                    #if red_traffic_light.empty():
                        #red_traffic_light.put(self.trafficLight.red_light)

                # Traffic Light detection
                for (x, y, w, h) in traffic_lights:
                    print('Found traffic light')
                    display_image = cv2.rectangle(display_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    roi_traffic_light = display_image[y:y+h, x:x+w]
                    display_image = self.trafficLight.detectColorInTrafficLight(display_image, roi_traffic_light, x, y, w, h)

                    if self.trafficLight.red_light:
                        print('Red light found - camera stream thread')
                        if red_traffic_light.empty():
                            red_traffic_light.put(self.trafficLight.red_light)

                # Stop Sign detection
                for(x, y, w, h) in stop_signs:
                    print('Found stop sign')
                    display_image = cv2.rectangle(display_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    if stop_sign_found.empty():
                        stop_sign_found.put(1)

                if red_traffic_light.empty() and stop_sign_found.empty() :
                    roi_image, line_image, result, movement = self.laneAndDirection.process_image_direction(image)
                    #cv2.imshow("2ROI ", roi_image)
                    #cv2.imshow("3Lines", line_image)
                    cv2.imshow("4Result", result)
                    if not direction.empty():
                        print("", end="")
                    else:
                        direction.put(movement)
                    
                cv2.imshow("1Camera View", display_image)
          
                # wait for a keypress, for q quiting the application
                key = cv2.waitKey(1) & 0xFF
                # clear the stream in preparation for the next frame
                self.RAW_CAPTURE.truncate(0)

                if key == ord("q"):
                    print("Exiting Application")
                    quitApp.put(1)
                    break
        except Exception as e:
            print("Exception in camerastream: ", str(e))
            quitApp.put(1)

# For clipping the cargo
class LDRReading:
    def __init__(self):
        IO.setmode (IO.BCM)
        clip = 12
        IO.setup(clip,IO.OUT)
        self.pwm = IO.PWM(clip,50)
        self.pwm.start(2.5)
        self.__ldr_pin = 6
        self.threshold_low = 360
        self.threshold_high = 3700

    def read(self):
        count = 0
        IO.setup(self.__ldr_pin, IO.OUT)
        IO.output(self.__ldr_pin, IO.LOW)
        time.sleep(0.1)
        IO.setup(self.__ldr_pin, IO.IN)
        while(IO.input(self.__ldr_pin) == IO.LOW):
            count += 1
        #print("Reading: ", count)
        return count

    def initial(self):
        #print('Init')
        self.pwm.ChangeDutyCycle(2.5)

    def clipped(self):
        #print('Clipping')
        self.pwm.ChangeDutyCycle(12.5)

def ldr_reading_clipper():
    reader = LDRReading()
    while True:
        count = reader.read()
        #if count > reader.threshold_low and count < reader.threshold_high:
        if count < 550 and count > 460:
            reader.initial()
            time.sleep(1.5)
            reader.clipped()
            time.sleep(0.5)
            while(True):
                time.time()

# Main program
if __name__ == "__main__":
    try:
        print("Opencv Version: ", cv2.__version__)
        IO.setmode(IO.BCM)
        right_light_sensor_pin = 2
        left_light_sensor_pin = 3
        IO.setup(right_light_sensor_pin,IO.IN)
        IO.setup(left_light_sensor_pin,IO.IN)

        detect_dark_color = IO.HIGH #true
        detect_light_color = IO.LOW #false
        
        quitApp = Queue()
        red_traffic_light = Queue() 
        stop_sign_found = Queue()
        direction = Queue()
    
        motorControl = MotorControl()
        cameraStream = CameraStream()

        cameraProcess = Process(target=cameraStream.cameraStreamCapture, args=(quitApp,red_traffic_light,stop_sign_found,direction))
        cameraProcess.start()
        
        ldrProcess = Process(target=ldr_reading_clipper)
        #ldrProcess.start()
        
        traffic_light_flg = False
        stop_sign_flg = False
        
        #motorControl.change_speed(60)
        #motorControl.move_forward()
        #time.sleep(2)
        time.sleep(5)
        while quitApp.empty():     
            if not red_traffic_light.empty() and not traffic_light_flg:
                print('Found red traffic light - main thread')
                time.sleep(1)
                motorControl.stop()
                time.sleep(3)
                red_traffic_light.get()
                traffic_light_flg = True
            elif not stop_sign_found.empty() and not stop_sign_flg:
                print('Found stop sign - main thread')
                time.sleep(4)
                motorControl.stop()
                time.sleep(3)
                motorControl.move_forward()
                time.sleep(3)
                stop_sign_found.get()
                stop_sign_flg = True
            elif not direction.empty():
				# sleep time below will be for blindness
                adjustment_time_forward = 0.3#0.5
                adjustment_time_turn = 0.4#0.3 
                movement = direction.get()
                
                if movement == -1 :
                    print("go left -- main thread")
                    time.sleep(adjustment_time_turn)
                    motorControl.change_speed(motorControl.turn_duty_cycle)
                    motorControl.turn_left()
                elif movement == 1 :
                    print("go right -- main thread")
                    time.sleep(adjustment_time_turn)
                    motorControl.change_speed(motorControl.turn_duty_cycle)
                    motorControl.turn_right()
                elif movement == 0:
                    print("move forward -- main thread")
                    time.sleep(adjustment_time_forward)
                    motorControl.change_speed(motorControl.forward_duty_cycle)
                    motorControl.move_forward()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(str(e))
    finally:
        print("Cleanup")
        IO.cleanup()
        sys.exit(0)
