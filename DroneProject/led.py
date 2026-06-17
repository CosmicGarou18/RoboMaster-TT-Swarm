from djitellopy import Tello
import time

tello = Tello('192.168.10.1')
tello.connect()
print(f"Battery: {tello.get_battery()}%")

tello.end()
