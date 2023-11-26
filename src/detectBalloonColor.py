from djitellopy import Tello
import cv2
import time

tello = Tello()

upper_blue, lower_blue  = (187, 157, 63), (138, 100, 35)
upper_red, lower_red = (59, 18, 173), (10, 4, 124)
upper_green, lower_green = (103, 181, 96), (47, 152, 54)
upper_pink, lower_pink = (175, 151, 187), (152, 143, 184)
upper_lg, lower_lg = (123, 165, 115), (98, 144, 119)
upper_purple, lower_purple = (166, 96, 118), (133, 74, 102)
upper_lb, lower_lb = (197, 147, 73), (118, 100, 35)
upper_yellow, lower_yellow = (14, 78, 77), (0, 62, 69)
upper_orange, lower_orange = (39, 138, 221), (0, 103, 180)

color_dict = {"R" : 0, "G": 0, "B" : 0, "P" : 0, "lg": 0, "purple" : 0, "lb" : 0, "yellow": 0, "orange" : 0}
w, d = 360, 240

def find_balloon(tello):
    while True:
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (w, d))
        # Apply color filtering to isolate the balloon
        mask_blue = cv2.inRange(frame, lower_blue, upper_blue)
        mask_red = cv2.inRange(frame, lower_red , upper_red)
        mask_green = cv2.inRange(frame, lower_green, upper_green)
        mask_pink = cv2.inRange(frame, lower_pink, upper_pink)
        mask_lg = cv2.inRange(frame, lower_lg, upper_lg)
        mask_purple = cv2.inRange(frame, lower_purple, upper_purple)
        mask_lb = cv2.inRange(frame, lower_lb, upper_lb)
        mask_yellow = cv2.inRange(frame, lower_yellow, upper_yellow)
        mask_orange = cv2.inRange(frame, lower_orange, upper_orange)

        cv2.imshow('Frame', frame)
        cv2.waitKey(1)
        contour_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_pink, _ = cv2.findContours(mask_pink, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_lg, _ = cv2.findContours(mask_lg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_purple, _ = cv2.findContours(mask_purple, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_lb, _ = cv2.findContours(mask_lb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_orange, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contour_blue:
            print("Blue Balloon detected!")
            color_dict["B"] += 1
        if contour_red:
            print("Red Balloon detected!")
            color_dict["R"] += 1
        if contour_green:
            print("Green Balloon detected!")
            color_dict["G"] += 1
        if contour_pink:
            print("Pink Balloon detected!")
            color_dict["P"] += 1
        if contour_lg:
            print("Light Green Balloon detected!")
            color_dict["lg"] += 1
        if contour_purple:
            print("Purple Balloon detected!")
            color_dict["purple"] += 1
        if contour_lb:
            print("Light Blue Balloon detected!")
            color_dict["lb"] += 1
        if contour_yellow:
            print("Yellow Balloon detected!")
            color_dict["yellow"] += 1
        if contour_orange:
            print("Orange detected!")
            color_dict["orange"] += 1

        if contour_blue or contour_red or contour_green or contour_pink or contour_lg or contour_purple or contour_lb or contour_yellow or contour_orange:
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