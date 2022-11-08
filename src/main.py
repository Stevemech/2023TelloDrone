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

wid1 = 2
wid2 = 27
wid3 = 19
targetList = [2, 27, 19]

cidlist = []
cidxlist = []
count = 0
tvec_x_prev = 0
tvec_y_prev = 0
tvec_z_prev = 0
detected = False
w=360
d=240

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

#-- Check if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

#--Calculate the roation matrix to euler angles
#--The result is the same as MATLAB except the order
#--of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

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

#-- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] = -1.0
R_flip[2,2] = -1.0

def trackARuCOXY(centerX, centerY, lengthX, lengthY):
    x_delta = centerX - w/2
    y_delta = centerY - d/3

    print("lengthX, lengthY, area pixel", lengthX, lengthY, lengthX * lengthY)
    if (lengthX * lengthY < int((w*d)/32)):
        if (abs(x_delta) > lengthX/2 and abs(y_delta) > lengthY/2):
            drone.go_xyz_speed(40, int(x_delta/lengthX * (-10)), int(y_delta/lengthY * (-10)), 20)
        elif (abs(y_delta) <= lengthY/2):
            drone.go_xyz_speed(40, int(x_delta/lengthX * (-10)), 0, 20)
        elif (abs(x_delta) <= lengthX/2):
            drone.go_xyz_speed(40, 0, int(y_delta/lengthY * (-10)), 20)
    else:
        drone.move_forward(30)
        sleep(1)
        drone.move_back(30)
#        print("lengthX, lengthY, area pixel", lengthX, lengthY, lengthX*lengthY)

    sleep(1)

def trackARuCOZ(z):
    if (z>600):
        drone.move_forward(25)
        sleep(1)

def trackARuCO(x, y, z, x_prev, y_prev, z_prev):

    print("x, y, z, x_prev, y_prev, z_prev ", x, y, z, x_prev, y_prev, z_prev)
    if abs(x-x_prev) > 50 and x<(-600*z/1200):
        drone.move_left(20)
        sleep(1)
    elif abs(x-x_prev) > 50 and x>(600*z/1200):
        drone.move_right(20)
        sleep(1)
    if abs(y-y_prev)>50 and y<(-200*z/1200):
        drone.move_down(20)
        sleep(1)
    elif abs(y-y_prev) >50 and y>(200*z/1200):
        drone.move_up(20)
        sleep(1)
    if z>700:
        drone.move_forward(25)
        sleep(1)


#    if area > marker_range[0] and area < marker_range[1]:
#        fb = 0
#    elif area > marker_range[1]:
#        fb = -20
#    elif area < marker_range[0] and area !=0:
#        fb = 20


#    if z < 400 and z > 0:
#        fb = -15
#    elif z > 400:
#        fb = 15
#    else:
#        fb = 0

#    if x > 400:
#        lr = -15
#    elif x < -400:
#        lr = 15
#    else:
#        lr = 0

#    if y > 400:
#        ud = 15
#    elif y < -400:
#        ud = -15
#    else:
#        ud = 0


#    drone.send_rc_control(lr, fb, ud, 0)
#    sleep(1.5)
#    drone.send_rc_control(0,0,0,0)
#   sleep(0.5)

def convert3to2(inputList):
    outputList = list(itertools.chain.from_iterable(inputList))
#    outputList = [sub[0] for sub in inputList]
    return outputList

def Sort(inputList):
    inputList.sort(key = lambda x: x[0])
    return inputList

drone.takeoff()
drone.move_up(40)
drone.move_forward(20)

while True:
    tvec_x = 0
    tvec_y = 0
    tvec_z = 0
    cidlist = []
    cidxlist = []
    tvec_2 = []
    tvec = []
    rvec = []
    tvec_sorted = []
    rvecCurrent = []
    tvecCurrent = []

    x_center = 0
    y_center = 0
    x_length = 0
    y_length = 0


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
                cidxlist.append(marker_corners1[0])
        print("cidlist:", cidlist)
        print("cidxlist: ", cidxlist)
        # tvec is the center of the marker in the camera's world
        rvec, tvec,  _ = aruco.estimatePoseSingleMarkers(cidxlist, marker_length, camera_matrix,
                                                        distortion_coefficients)

        print("tvec ", tvec)
        print("rvec ", rvec)
        if len(cidxlist) != 0:
            x_center = (cidxlist[0][0][0] + cidxlist[0][1][0] + cidxlist[0][2][0] + cidxlist[0][3][0]) / 4
            y_center = (cidxlist[0][0][1] + cidxlist[0][1][1] + cidxlist[0][2][1] + cidxlist[0][3][1]) / 4
            print("x_center, y_center ", x_center, y_center)
            x_length = max(cidxlist[0][0][0], cidxlist[0][1][0], cidxlist[0][2][0], cidxlist[0][3][0]) - min(cidxlist[0][0][0], cidxlist[0][1][0], cidxlist[0][2][0], cidxlist[0][3][0])
            y_length = max(cidxlist[0][0][1], cidxlist[0][1][1], cidxlist[0][2][1], cidxlist[0][3][1]) - min(cidxlist[0][0][1], cidxlist[0][1][1], cidxlist[0][2][1], cidxlist[0][3][1])
            print("x_length, y_length ", x_length, y_length)
            trackARuCOXY(x_center, y_center, x_length, y_length)
        if rvec is not None and tvec is not None:
            rvecCurrent = rvec[0][0]
            tvecCurrent = tvec[0][0]

            # In case there are multiple markers
#           for i in range(len(cidlist)):
            img_aruco = cv.drawFrameAxes(img_aruco, camera_matrix, distortion_coefficients, rvecCurrent, tvecCurrent,
                                     marker_length, 3)


            #-- Obtain the rotation matrix tag->camera
            R_ct = np.matrix(cv.Rodrigues(rvecCurrent)[0])
            R_tc = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

            #-- Print the marker's attitude respect to camera frame
            print("Roll: %4.0f, Pitch %4.0f, Yaw %4.0f" % (roll_marker, pitch_marker, yaw_marker))

            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc*np.matrix(tvecCurrent).T
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_ct)

            #-- Print the camera's attitude respect to the marker frame
            print("Camera Position in frame marker: %4.0f, %4.0f, %4.0f" % (pos_camera[0], pos_camera[1], pos_camera[2]))

            tvec_2 = convert3to2(tvec)
            print("tvec_2: ", tvec_2)
            tvec_sorted = Sort(tvec_2)
            print("sorted tvec_2 ", tvec_sorted)
            tvec_x = tvec_sorted[0][0]
            tvec_y = tvec_sorted[0][1]
            tvec_z = tvec_sorted[0][2]
#            trackARuCOZ(tvec_z)
            tvec_x_prev = tvec_x
            tvec_y_prev = tvec_y
            tvec_z_prev = tvec_z
 #       elif detected == True:
            #-- lost target search again
#            drone.move_back(40)


    cv.imshow("frame", frame)
    cv.waitKey(1)


#drone.land()
cv.destroyAllWindows()





