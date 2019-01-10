#!/usr/bin/env python

from motor_control import MotorControl
import sys
import time

motorControl = MotorControl()

try:
    motorControl.move_forward()
    time.sleep(3)
    motorControl.turn_right()
    time.sleep(3)
    motorControl.move_backwards()
    time.sleep(3)
    motorControl.turn_left()
    time.sleep(3)
    motorControl.move_forward()
    time.sleep(3)
    motorControl.stop()

except KeyboardInterrupt:
    pass
finally:
    motorControl.cleanup()
    sys.exit(0)