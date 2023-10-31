from djitellopy import Tello
import time
import math


def takeoff_and_land(tello):
    tello.takeoff()
    time.sleep(5)
    tello.land()

# approximates a circle by breaking it down into small segments. It calculates the circumference
# of the circle based on the provided radius, then determines the number of segments
# to use (in this case, 36 for a relatively smooth circle).
# It calculates the distance to move in each segment (segment_length) and the angle to
# turn (angle). It then executes the movements and rotations in a loop.
def fly_circle(tello, radius_cm):
    # Calculate the circumference of the circle
    print("coming to fly circle")
    circumference = 2 * math.pi * radius_cm
    print("circumference")
    # Calculate the number of segments to approximate the circle
    num_segments = 36  # You can adjust this value for smoother circles

    # Calculate the distance to move in each segment
    segment_length = circumference / num_segments
    print("segment_length")
    # Calculate the angle to turn in each segment
    angle = 360 / num_segments

    for _ in range(num_segments):
        print("coming into loop")
        tello.move_forward(segment_length)
        tello.rotate_clockwise(angle)
        time.sleep(2)


def main():
    tello = Tello()

    try:
        tello.connect()

        # Perform the mission
     #   takeoff_and_land(tello)
        tello.takeoff()
        fly_circle(tello, 100)  # Adjust the radius as needed
        takeoff_and_land(tello)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        tello.end()


if __name__ == "__main__":
    main()

