from djitellopy import Tello
import cv2
import time

def takeoff_and_land(tello):
    tello.takeoff()
    time.sleep(5)
    tello.land()

#uses OpenCV to capture the video stream from the default camera.
# It applies color filtering to isolate the balloon (in this example,
# the color range is set to red). It then finds contours in the filtered image.
# If any contours are found, it considers it as the balloon.
def detect_balloon_color():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        # Define the color range for your balloon (in this example, red)
        lower_red = (0, 0, 100)
        upper_red = (100, 100, 255)

        # Apply color filtering to isolate the balloon
        mask = cv2.inRange(frame, lower_red, upper_red)

        # Find contours in the filtered image
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If any contours are found, consider it as the balloon
        if contours:
            print("Red Balloon detected!")
            break

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
        detect_balloon_color()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        tello.end()

if __name__ == "__main__":
    main()