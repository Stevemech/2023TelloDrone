#-- This is the python code of the _____ for the beginner level of the ARC Competition --#
from time import sleep
from djitellopy import tello
from modules import keyPressModule as kpm
from time import sleep
from getBattery import baReminder
import cv2 as cv
from cv2 import aruco
import numpy as np
import itertools
import math

#-- test results
# 11 9 18 - results: 5, 4, 9, 5
# 6 9 14
#-- User input for three numbers

userIDS = input("Enter wanted ids separated by a space: ")
userIDS = userIDS.split()

for i in range(len(userIDS)):
    userIDS[i] = int(userIDS[i])

targetList = userIDS
print("targetList: ", targetList)

#-- Marker IDs and Marker Corners from ArUco markers for target balloons
cidlist = []
cidxlist = []

#--
count = 0
detected = False


#-- Define the camera feed resolution
w = 360
d = 240

#--
check_timer = 30
#--
balNo = 9

#--
kpm.init()

#-- Initialize the drone
drone = tello.Tello()
drone.connect()

#-- Check battery level
baReminder()

#-- Start camera feed from drone's main camera
drone.streamon()
#cap = cv.VideoCapture(1)

#-- Define ArUco dictionary, parameters, and size
marker_length = 10  # 10 cm
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

#-- Camera calibration matrix
camera_matrix = np.array([[2.13243514e+03, 0.00000000e+00, 1.33300826e+03],
                          [0.00000000e+00, 2.15409995e+03, 4.95966668e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coefficients = np.array([[0.20388981, -0.66295284, -0.0584427, 0.00772092, 1.17854164]])

axis = np.array([[marker_length, 0, 0], [0, marker_length, 0], [0, 0, marker_length * -1]]).reshape(-1, 3)

#-- Keyboard controls for drone for use during debugging (not for autonomous)
def getKeyBoardInput():
    # lr is left & right
    # fb is front & back
    # ud is up & down
    # yv is yaw & velocity
    lr, fb, ud,  yv = 0, 0, 0, 0
    speed = 50

    if kpm.getKey("LEFT"): lr  = -speed
    elif kpm.getKey("RIGHT"): lr  = speed

    if kpm.getKey("UP"): fb  = speed

    elif kpm.getKey("DOWN"): fb  = -speed

    if kpm.getKey("w"): ud  = speed
    elif kpm.getKey("s"): ud  = -speed

    if kpm.getKey("a"): yv  = speed
    elif kpm.getKey("d"): yv  = -speed

    if kpm.getKey("q"): drone.land()
    if kpm.getKey("t"): drone.takeoff()
    return [lr, fb, ud, yv]

#-- Drone autonomous driving using ArUco track function
#-- CenterX and CenterY are the center coordinates of the detected ArUco marker
#-- LengthX and LengthY are the lengths of the detected ArUco marker as they appear on the camera feed
def trackARuCOXY(centerX, centerY, lengthX, lengthY, totalbals):
    #-- The difference between the center of the video feed and the center of the detected ArUco marker
    x_delta = centerX - w/2
    y_delta = centerY - d/3


    if (lengthX >=40):
        steplength = 30
    else:
        steplength = 40
    print("lengthX, lengthY, area pixel, x_delta, y_delta", lengthX, lengthY, lengthX * lengthY, x_delta, y_delta)
    #-- Gauge to determine when the drone is close enough to the ArUco marker
    #-- Takes the lengthX and lengthY of the detected ArUco marker and calculates area
    if (lengthX * lengthY < int((w*d)/32)):
        # -- If the area of the detected ArUco marker is 1/32 of the total area of the video feed,
        # -- the drone is close enough
        # -- If the drone is not close enough, the drone will realign
        # -- using the centers of the detected ArUco marker and
        # -- the video feed and continue moving forward
        if (abs(x_delta) > lengthX/6 and abs(y_delta) > lengthY/4):
                drone.go_xyz_speed(steplength, int(x_delta/lengthX * (-10)), int(y_delta/lengthY * (-10))+5, 40)
                sleep(0.5)
        elif (abs(y_delta) <= lengthY/4):
            drone.go_xyz_speed(steplength, int(x_delta/lengthX * (-10)), 5, 40)
            sleep(0.5)
        elif (abs(x_delta) <= lengthX/6):
            drone.go_xyz_speed(steplength, 0, int(y_delta/lengthY * (-10))+5, 40)
            sleep(0.5)
        #-- If the drone is close enough, the drone will move forward to pop the balloon
        #-- and move backwards,  looking for the next ArUco marker
    else:
        print("height: ", drone.get_distance_tof())
        if (lengthX * lengthY < 9500):
            totalbals -= 1
            print("ready to poke")
            print("flight time: ", drone.get_flight_time())
            drone.go_xyz_speed(45, 0, 0, 100)
            detected = True
        # move back 100cm
        drone.go_xyz_speed(-117, 0, 0, 80)
    return int(totalbals)



#-- Drone takes off
drone.takeoff()
sleep(2)
drone.go_xyz_speed(0, 0, 100, 100)
sleep(1)
drone.go_xyz_speed(150, 0, 0, 100)
sleep(1)
print("flight time: ", drone.get_flight_time())

while True:

    #-- Initialize all variables for every new frame
    cidlist = []
    cidxlist = []
#    tvec = []
#    rvec = []
    x_center = 0
    y_center = 0
    x_length = 0
    y_length = 0
    check_timer -= 1
    LM_tag00 = []
    LM_tag01 = []
    LM_tag10 = []
    LM_tag11 = []


    #-- Check for keyboard input
    #-- This is a debugging feature and is not used for autonomous driving
    #-- It is an emergency stop for the drone
    vals = getKeyBoardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    print("balNo: ", balNo)
    if (balNo > 0):
        #-- Get the frame from the drone's camera feed and set the size
        frame = drone.get_frame_read().frame
        frame = cv.resize(frame, (w, d))

        #-- Detect as many ArUco markers as possible in the frame
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        if marker_IDs is not None:
            #-- last less than 3 back off found the marker, reset the back off step to 0
            count = 0
            #-- Draw detected ArUco markers on the video feed
            img_aruco = aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs, (0, 255, 0))

            #-- Find the wanted ArUco markers specified in "targetList"
            for ids1, marker_corners1 in zip(marker_IDs, marker_corners):
                print(ids1, " ", marker_corners1.astype(np.int32))
                if ids1[0] in targetList:
                    cidlist.append(ids1[0])
                    cidxlist.append(marker_corners1[0])
                    if len(LM_tag00) != 0:
                        if marker_corners1[0][0][0] < LM_tag00[0]:
                            LM_tag00 = marker_corners1[0][0]
                            LM_tag01 = marker_corners1[0][1]
                            LM_tag10 = marker_corners1[0][2]
                            LM_tag11 = marker_corners1[0][3]
                    else:
                        LM_tag00 = marker_corners1[0][0]
                        LM_tag01 = marker_corners1[0][1]
                        LM_tag10 = marker_corners1[0][2]
                        LM_tag11 = marker_corners1[0][3]
            print("cidlist:", cidlist)
            print("cidxlist: ", cidxlist)
            print("lM_tag00, LM_tag01, LM_tag10, LM_tag11 ", LM_tag00, LM_tag01, LM_tag10, LM_tag11)
            #-- Post estimation of the ArUco marker's position
            #-- Note: This is currently only used to find the xyz coordinates of the ArUco marker
            #-- In the future, this will be utilized for more sophisticated ArUco marker navigation
            #-- tvec is the center of the marker in the camera's world
     #       rvec, tvec,  _ = aruco.estimatePoseSingleMarkers(cidxlist, marker_length, camera_matrix,
    #                                                         distortion_coefficients)

     #       print("tvec ", tvec)
     #       print("rvec ", rvec)
     #       if rvec is not None and tvec is not None:
     #           # In case there are multiple markers
     #           #           for i in range(len(cidlist)):
     #           img_aruco = cv.drawFrameAxes(img_aruco, camera_matrix, distortion_coefficients, rvec[0][0], tvec[0][0],
     #                                        marker_length, 3)

            #-- Use the corners of the ArUco markers to calculate the center and lengths of the ArUco marker
            #-- This always picks the first ArUco marker in the list
    #        if len(cidxlist) != 0:
    #            x_center = (cidxlist[0][0][0] + cidxlist[0][1][0] + cidxlist[0][2][0] + cidxlist[0][3][0]) / 4
    #            y_center = (cidxlist[0][0][1] + cidxlist[0][1][1] + cidxlist[0][2][1] + cidxlist[0][3][1]) / 4
    #            print("x_center, y_center ", x_center, y_center)
    #            x_length = cidxlist[0][1][0] - cidxlist[0][0][0]
    #            y_length = cidxlist[0][2][1] - cidxlist[0][1][1]
    #            x_length = max(cidxlist[0][0][0], cidxlist[0][1][0], cidxlist[0][2][0], cidxlist[0][3][0]) - min(cidxlist[0][0][0], cidxlist[0][1][0], cidxlist[0][2][0], cidxlist[0][3][0])
    #            y_length = max(cidxlist[0][0][1], cidxlist[0][1][1], cidxlist[0][2][1], cidxlist[0][3][1]) - min(cidxlist[0][0][1], cidxlist[0][1][1], cidxlist[0][2][1], cidxlist[0][3][1])
            if len(LM_tag00) != 0:
                x_center = (LM_tag00[0] + LM_tag01[0] + LM_tag10[0] + LM_tag11[0]) / 4
                y_center = (LM_tag00[1] + LM_tag01[1] + LM_tag10[1] + LM_tag11[1]) / 4
                print("x_center, y_center ", x_center, y_center)
                x_length = LM_tag01[0] - LM_tag00[0]
                y_length = LM_tag10[1] - LM_tag01[1]

                print("x_length, y_length ", x_length, y_length)
                #-- Navigate the drone using the ArUco marker's center and length
                #-- Tries to align the center of the camera feed to the center of the ArUco marker
                balNo = trackARuCOXY(x_center, y_center, x_length, y_length, balNo)
            else:
                #-- If no ArUco markers are detected, the drone will do something to try to find
                #-- the ArUco marker from a different position
                if check_timer <= 0:
                    check_timer = 30
                    if count < 2:
                        #drone.move_back(20)
                        drone.go_xyz_speed(-67, 0, 0, 70)
                        count += 1
                        sleep(0.5)
                    else:
                        count = 0
                        #drone.move_forward(40)
                        print("Height: ", drone.get_height())
                        if drone.get_height() > 200:
                            drone.go_xyz_speed(40, 0, -20, 40)
                        else:
                            drone.go_xyz_speed(40, 0, 20, 40)
                        sleep(0.5)

    ## YOU ARE ENTERING THE CONSTRUCTION ZONE ##
        else:
            #-- Work in progress...
            if check_timer <= 0:
                check_timer = 30
                if count < 2:
                    # drone.move_back(20)
                    drone.go_xyz_speed(-67, 0, 0, 70)
                    count += 1
                    sleep(0.5)
                elif detected is True:
                    # drone.move_back(60)
                    drone.go_xyz_speed(-67, 0, 0, 70)
                    detected = False
                else:
                    count = 0
                    # drone.move_forward(40)
                    print("Height: ", drone.get_height())
                    if drone.get_height() > 200:
                        drone.go_xyz_speed(40, 0, -20, 40)
                    else:
                        drone.go_xyz_speed(40, 0, 20, 40)
                    sleep(0.5)
        print("check_timer, count:", check_timer, count)
        ## END OF CONSTRUCTION ZONE ##

        #-- Display the video feed on the screen
        cv.imshow("frame", frame)
        cv.waitKey(1)
    else:
        print("Task completed! Yahoooo!")
        drone.land()
        print("flight time: ", drone.get_flight_time())
        break

cv.destroyAllWindows()





