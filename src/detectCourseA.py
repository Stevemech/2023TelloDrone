from djitellopy import Tello
import cv2
import time

tello = Tello()

upper_blue, lower_blue  = (197, 147, 73), (128, 90, 25)
upper_red, lower_red = (59, 18, 173), (10, 4, 124)
upper_green, lower_green = (117, 159, 47), (78, 120,33)

color_dict = {"R" : 0, "G": 0, "B" : 0}
w, d = 360, 240

def find_balloon(tello):
    while True:
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (w, d))
        # Apply color filtering to isolate the balloon
        mask_blue = cv2.inRange(frame, lower_blue, upper_blue)
        mask_red = cv2.inRange(frame, lower_red , upper_red)
        mask_green = cv2.inRange(frame, lower_green, upper_green)

        cv2.imshow('Frame', frame)
        cv2.waitKey(1)
        contour_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contour_blue:
            print("Blue Balloon detected!")
            color_dict["B"] += 1
        if contour_red:
            print("Red Balloon detected!")
            color_dict["R"] += 1
        if contour_green:
            print("Green Balloon detected!")
            color_dict["G"] += 1

        if contour_blue or contour_red or contour_green:
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
    #print("Find!!!")
    #print("Find!!!")
    tello.rotate_clockwise(90)
    find_balloon(tello)
    tello.rotate_clockwise(90)
    find_balloon(tello)
    tello.rotate_clockwise(90)
    find_balloon(tello)
    print(color_dict)


except Exception as e:
    print("")
finally:
    tello.end()