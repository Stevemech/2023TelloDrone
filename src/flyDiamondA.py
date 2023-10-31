from djitellopy import Tello
import time


def takeoff_and_land(tello):
    tello.takeoff()
    time.sleep(5)
    tello.land()

#command the drone to move forward for the specified side length
# and then rotate 45 degrees four times to complete a diamond pattern.
def fly_diamond(tello, side_length_cm):
    tello.rotate_clockwise(45)
    tello.move_forward(100)
    time.sleep(2)
    tello.rotate_clockwise(135)
    tello.move_forward(100)
    time.sleep(2)
    tello.rotate_clockwise(45)
    tello.move_forward(100)
    time.sleep(2)
    tello.rotate_clockwise(135)
    tello.move_forward(100)
    time.sleep(2)


def main():
    tello = Tello()

    try:
        tello.connect()

        # Perform the mission
        #takeoff_and_land(tello)
        tello.takeoff()
        fly_diamond(tello, 100)
        takeoff_and_land(tello)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        tello.end()


if __name__ == "__main__":
    main()