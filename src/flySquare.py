from djitellopy import tello
import time

# Connect to the Tello drone
me = tello.Tello()
me.connect()


# Function to fly a square pattern
def fly_square(side_length):
    for _ in range(4):
        me.move_forward(side_length)
        me.rotate_clockwise(90)


# Take off
me.takeoff()

time.sleep(2)  # Allow the drone to stabilize

# Fly a square pattern with 100 cm sides
side_length = 100  # in cm
fly_square(side_length)

# Land
me.land()
time.sleep(2)  # Allow time for landing

# Disconnect from the drone
me.end()
