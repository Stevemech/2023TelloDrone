from djitellopy import tello

from time import sleep

me = tello.Tello()

me.connect()

print(me.get_battery())
print("taking off")
me.takeoff()
sleep(15)
me.land()
print("landing")