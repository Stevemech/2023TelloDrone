"""
    Here, the main route for the general movement for this
    drone will be done here and all needed modules will
    be imported here.

"""
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

# wid1 = input("Enter wanted id1:")
# wid2 = input("Enter wanted id2:")
# wid3 = input("Enter wanted id3:")


targetList = [9, 6, 30]

cidlist = []
cidxlist = []
count = 0
detected = False
w=360
d=240
check_timer = 60


kpm.init()
drone = tello.Tello()
drone.connect()
baReminder()
drone.streamoff()
drone.streamon()
#cap = cv.VideoCapture(1)
marker_length = 10  # 10 cm
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

camera_matrix = np.array([[2.13243514e+03, 0.00000000e+00, 1.33300826e+03],
                          [0.00000000e+00, 2.15409995e+03, 4.95966668e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distortion_coefficients = np.array([[0.20388981, -0.66295284, -0.0584427, 0.00772092, 1.17854164]])

axis = np.array([[marker_length, 0, 0], [0, marker_length, 0], [0, 0, marker_length * -1]]).reshape(-1, 3)

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

def trackARuCOXY(centerX, centerY, lengthX, lengthY):
    x_delta = centerX - w/2
    y_delta = centerY - d/3

    print("lengthX, lengthY, area pixel", lengthX, lengthY, lengthX * lengthY)
    if (lengthX * lengthY < int((w*d)/32)):
        if (abs(x_delta) > lengthX/2 and abs(y_delta) > lengthY/2):
            drone.go_xyz_speed(40, int(x_delta/lengthX * (-10)), int(y_delta/lengthY * (-10)), 30)
        elif (abs(y_delta) <= lengthY/2):
            drone.go_xyz_speed(40, int(x_delta/lengthX * (-10)), 0, 30)
        elif (abs(x_delta) <= lengthX/2):
            drone.go_xyz_speed(40, 0, int(y_delta/lengthY * (-10)), 30)
    else:
        drone.go_xyz_speed(40, 0, 0, 80)
        drone.move_back(200)


drone.takeoff()
drone.move_up(40)
drone.move_forward(30)
sleep(1)

while True:
    cidlist = []
    cidxlist = []
    tvec = []
    rvec = []
    x_center = 0
    y_center = 0
    x_length = 0
    y_length = 0
    check_timer -= 1


    vals = getKeyBoardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    frame = drone.get_frame_read().frame
    frame = cv.resize(frame, (w, d))

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_IDs is not None:
        img_aruco = aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs, (0, 255, 0))


        for ids1, marker_corners1 in zip(marker_IDs, marker_corners):
            print(ids1, " ", marker_corners1.astype(np.int32))
            if ids1[0] in targetList:
                detected = True
                cidlist.append(ids1[0])
                count = len(cidlist)
                cidxlist.append(marker_corners1[0])
        print("cidlist:", cidlist)
        print("cidxlist: ", cidxlist)
        # tvec is the center of the marker in the camera's world
        rvec, tvec,  _ = aruco.estimatePoseSingleMarkers(cidxlist, marker_length, camera_matrix,
                                                        distortion_coefficients)

        print("tvec ", tvec)
        print("rvec ", rvec)
        if rvec is not None and tvec is not None:
            # In case there are multiple markers
            #           for i in range(len(cidlist)):
            img_aruco = cv.drawFrameAxes(img_aruco, camera_matrix, distortion_coefficients, rvec[0][0], tvec[0][0],
                                         marker_length, 3)
        if len(cidxlist) != 0:
            x_center = (cidxlist[0][0][0] + cidxlist[0][1][0] + cidxlist[0][2][0] + cidxlist[0][3][0]) / 4
            y_center = (cidxlist[0][0][1] + cidxlist[0][1][1] + cidxlist[0][2][1] + cidxlist[0][3][1]) / 4
            print("x_center, y_center ", x_center, y_center)
            x_length = max(cidxlist[0][0][0], cidxlist[0][1][0], cidxlist[0][2][0], cidxlist[0][3][0]) - min(cidxlist[0][0][0], cidxlist[0][1][0], cidxlist[0][2][0], cidxlist[0][3][0])
            y_length = max(cidxlist[0][0][1], cidxlist[0][1][1], cidxlist[0][2][1], cidxlist[0][3][1]) - min(cidxlist[0][0][1], cidxlist[0][1][1], cidxlist[0][2][1], cidxlist[0][3][1])
            print("x_length, y_length ", x_length, y_length)
            trackARuCOXY(x_center, y_center, x_length, y_length)
        else:
            if check_timer == 0:
                check_timer = 60
                if count < 5:
                    drone.move_back(20)
                    count += 1
                    sleep(0.5)
                else:
                    count = 0
                    drone.move_forward(40)
                    sleep(0.5)
    else:
        if check_timer == 0:
            check_timer = 60
            if count < 5:
                drone.move_back(20)
                count += 1
                sleep(0.5)
            else:
                count = 0
                drone.move_forward(40)
                sleep(0.5)

    print("check_timer, count:", check_timer, count)
    cv.imshow("frame", frame)
    cv.waitKey(1)

cv.destroyAllWindows()





