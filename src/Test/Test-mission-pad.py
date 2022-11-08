"""
    This is a direct replecation to the maker of the djitellopy 
    module and is used for testing purpoese to double check if 
    I understand
"""

from djitellopy import Tello
import cv2


# camera feed
frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(1)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

# create and connect
tello = Tello()
tello.connect()

# configure drone
tello.enable_mission_pads()
tello.set_mission_pad_detection_direction(1)
tello.takeoff()

pad = tello.get_mission_pad_id()

# detect and react to pads until we see pad #1
while pad != 1:
    if pad == 3:
        tello.rotate_clockwise(90)

    if pad == 4:
        tello.flip_forward()

    pad = tello.get_mission_pad_id()

# graceful termination
tello.disable_mission_pads()
tello.land()
tello.end()