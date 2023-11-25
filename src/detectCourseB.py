from djitellopy import Tello
import cv2
import time

tello = Tello()

upper_pink, lower_pink = (175, 151, 187), (142, 133, 174)
upper_lg, lower_lg = (133, 175, 125), (88, 134, 109)
upper_purple, lower_purple = (194, 160, 166), (175, 106, 140)

color_dict = {"P": 0, "purple" : 0, "lg": 0}
w, d = 360, 240

def find_balloon(tello):
    while True:
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (w, d))
        # Apply color filtering to isolate the balloon
        mask_pink = cv2.inRange(frame, lower_pink, upper_pink)
        mask_lg = cv2.inRange(frame, lower_lg, upper_lg)
        mask_purple = cv2.inRange(frame, lower_purple, upper_purple)

        cv2.imshow('Frame', frame)
        cv2.waitKey(1)
        contour_pink, _ = cv2.findContours(mask_pink, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_lg, _ = cv2.findContours(mask_lg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_purple, _ = cv2.findContours(mask_purple, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contour_pink:
            print("Pink Balloon detected!")
            color_dict["P"] += 1
        if contour_lg:
            print("Light Green Balloon detected!")
            color_dict["lg"] += 1
        if contour_purple:
            print("Purple Balloon detected!")
            color_dict["purple"] += 1

        if contour_pink or contour_lg or contour_purple:
            break

        #else:
        #    print("Not detect a single one, try another frame")
    #time.sleep(15)
    #cv2.destroyAllWindows()

try:
    tello.connect()
    tello.streamon()
    tello.takeoff()
    # Perform the mission

    # Move Up 80
    tello.go_xyz_speed(0, 0, 80, 100)
    find_balloon(tello)
    tello.rotate_clockwise(90)
    time.sleep(5)
    find_balloon(tello)
    tello.rotate_clockwise(90)
    time.sleep(5)
    find_balloon(tello)
    tello.rotate_clockwise(90)
    time.sleep(5)
    find_balloon(tello)
    print(color_dict)

except Exception as e:
    print("")
finally:
    tello.end()