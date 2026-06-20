from djitellopy import Tello
import time

tello = Tello()
tello.connect()

while True:
    time.sleep(0.5)
    print(f"agx: {tello.get_acceleration_x()} " +
          f"agy: {tello.get_acceleration_y()} " +
          f"agz: {tello.get_acceleration_z()}")

tello.end()
