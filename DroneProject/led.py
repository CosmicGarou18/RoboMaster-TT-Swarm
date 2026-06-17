from djitellopy import Tello
import time

tello = Tello()
tello.connect()

tello.takeoff()

tello.send_rc_control(40, 0, 0, 0)
time.sleep(2000)
tello.land()

tello.end()
