from djitellopy import Tello
import cv2
import time

def takeoff_and_land(tello):
    tello.takeoff()
    time.sleep(5)
    tello.land()

# uses OpenCV to capture the video stream from the default camera. It converts each frame
# to grayscale, then uses the ArUco dictionary and a detector to find any ArUco markers in the frame.
# If a marker is detected, it prints a message, lands the drone, and exits the loop.
def detect_aruco(tello):
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Load the ArUco dictionary and detect markers
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If markers are found, land the drone and break out of the loop
        if ids is not None:
            print("ArUco tag detected! Landing...")
            takeoff_and_land(tello)
            break

        # Display the frame with ArUco marker
        cv2.imshow('Frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def main():
    tello = Tello()

    try:
        tello.connect()

        # Perform the mission
        tello.takeoff()
       # takeoff_and_land(tello)
        detect_aruco(tello)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        tello.end()

if __name__ == "__main__":
    main()