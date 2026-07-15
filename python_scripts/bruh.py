from djitellopy import Tello
import time

t = Tello()
t.connect()

try:
    while True:
        print(t.send_command_with_return("tof?"))
except KeyboardInterrupt:
    t.land()
